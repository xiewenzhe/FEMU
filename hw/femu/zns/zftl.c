#include "zns.h"

//#define FEMU_DEBUG_ZFTL

static void *ftl_thread(void *arg);

/*
 * 一条 host read 会被拆成多个 NAND read 子请求。
 *
 * 这个结构体保存其中一个子请求的信息：
 *   - 没有 est_ 前缀的字段是真实执行后观测到的结果；
 *   - 有est_ 前缀：estimated，表示“估计值”，是在真正执行前用 shadow
 *     plane 时间线模拟出来的结果。
 *
 * 这里的在线估计只用于观察 estimator 是否准确，不改变真实调度顺序。
 */
struct zns_read_slack_sample {
    uint64_t lpn;          /* 这个子请求对应的 4KiB 逻辑页号。 */
    struct ppa ppa;        /* LPN 查表得到的真实物理页地址。 */
    uint64_t sublat;       /* 真实执行后观测到的子请求延迟。 */
    uint64_t est_start;    /* 估计的 NAND read 开始时间。 */
    uint64_t est_finish;   /* 估计的 NAND read 完成时间。 */
    uint64_t est_sublat;   /* 估计的子请求延迟：est_finish - req_stime。 */
    uint64_t est_slack;    /* 估计的 slack：est_maxlat - est_sublat。 */
    uint16_t ch;           /* 目标 channel。 */
    uint16_t lun;          /* 目标 LUN/chip。 */
    uint16_t pl;           /* 目标 plane。 */
};

/*
 * 估计阶段使用的“草稿纸”plane 时间线。
 *
 * 真实调度会更新 pl->next_plane_avail_time；估计阶段不能提前修改真实
 * 时间线，所以把目标 plane 当前的 next_plane_avail_time 抄到 avail，
 * 然后只在这个 shadow 结构里模拟排队和完成时间。
 */
struct zns_read_shadow_plane {
    uint16_t ch;
    uint16_t lun;
    uint16_t pl;
    uint64_t avail;        /* shadow 里的 plane 下一次可用时间。 */
};

/* 只用于把多行子操作日志关联到同一条主机读取请求。 */
static uint64_t zns_read_slack_reqid;

static inline uint64_t zns_absdiff_u64(uint64_t a, uint64_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/* 在当前 host read 的 shadow plane 表里查找某个 PPA 对应的 plane。 */
static struct zns_read_shadow_plane *zns_find_read_shadow_plane(
    struct zns_read_shadow_plane *planes, uint64_t nr_planes, struct ppa *ppa)
{
    uint64_t i;

    for (i = 0; i < nr_planes; i++) {
        if (planes[i].ch == ppa->g.ch &&
            planes[i].lun == ppa->g.fc &&
            planes[i].pl == ppa->g.pl) {
            return &planes[i];
        }
    }

    return NULL;
}

/*
 * 读这个文件先记住几组名词：
 * req 是一条 NVMe 请求；LBA 是主机看到的逻辑块；LPN 是 FTL 内部 4KiB 逻辑页；
 * PPA 是物理地址，拆成 ch/fc(这里相当于 LUN)/pl/blk/pg/spg；
 * next_plane_avail_time 是当前 ZNS timing 里真正表示 plane 忙闲的时间戳。
 */

/* L2P 查询：输入 LPN，返回当前映射到的 PPA。 */
static inline struct ppa get_maptbl_ent(struct zns_ssd *zns, uint64_t lpn)
{
    return zns->maptbl[lpn];
}

/* L2P 更新：把某个 LPN 的映射改成新的 PPA。 */
static inline void set_maptbl_ent(struct zns_ssd *zns, uint64_t lpn, struct ppa *ppa)
{
    ftl_assert(lpn < zns->l2p_sz);
    zns->maptbl[lpn] = *ppa;
}

/* ZNS FTL 初始化：创建后台 FTL 线程处理 to_ftl 队列中的请求。 */
void zftl_init(FemuCtrl *n)
{
    struct zns_ssd *ssd = n->zns;

    qemu_thread_create(&ssd->ftl_thread, "FEMU-FTL-Thread", ftl_thread, n,
                       QEMU_THREAD_JOINABLE);
}

/* 根据 PPA 中的 ch 字段定位 channel。 */
static inline struct zns_ch *get_ch(struct zns_ssd *zns, struct ppa *ppa)
{
    return &(zns->ch[ppa->g.ch]);
}

/* 根据 PPA 中的 fc 字段定位 flash chip/LUN。 */
static inline struct zns_fc *get_fc(struct zns_ssd *zns, struct ppa *ppa)
{
    struct zns_ch *ch = get_ch(zns, ppa);
    return &(ch->fc[ppa->g.fc]);
}

/* 根据 PPA 中的 pl 字段定位 plane；ZNS timing 当前主要在这里建模并行度。 */
static inline struct zns_plane *get_plane(struct zns_ssd *zns, struct ppa *ppa)
{
    struct zns_fc *fc = get_fc(zns, ppa);
    return &(fc->plane[ppa->g.pl]);
}

/* 根据 PPA 中的 blk 字段定位 block。 */
static inline struct zns_blk *get_blk(struct zns_ssd *zns, struct ppa *ppa)
{
    struct zns_plane *pl = get_plane(zns, ppa);
    return &(pl->blk[ppa->g.blk]);
}

/* 检查地址编号是否在合法范围内。 */
static inline void check_addr(int a, int max)
{
   assert(a >= 0 && a < max);
}

/* 推进全局写指针：一轮 plane 写完后切到下一个 channel/lun。 */
static void zns_advance_write_pointer(struct zns_ssd *zns)
{
    struct write_pointer *wpp = &zns->wp;

    check_addr(wpp->ch, zns->num_ch);
    wpp->ch++;
    if (wpp->ch == zns->num_ch) {
        wpp->ch = 0;
        check_addr(wpp->lun, zns->num_lun);
        wpp->lun++;
        /* channel 已经轮完，这里推进到下一个 LUN。 */
        if (wpp->lun == zns->num_lun) {
            wpp->lun = 0;
        }
    }
}

/* 推进一次 NAND 子操作的 plane 时间戳，并返回这次子操作的可见时延。 */
static uint64_t zns_advance_status(struct zns_ssd *zns, struct ppa *ppa,struct nand_cmd *ncmd)
{
    /*
     * ncmd 描述的是一个 NAND 层子操作，不是一整条 NVMe 请求。
     * ncmd->cmd 可能是 NAND_READ / NAND_WRITE / NAND_ERASE。
     * ncmd->stime 是调用方传入的请求到达时间基准。
     */
    int c = ncmd->cmd;

    /*
     * nand_stime 是本次 NAND 操作在目标 plane 上真正开始执行的时间。
     * 如果 plane 在 req_stime 前已经空闲，则 nand_stime=req_stime；
     * 如果 plane 还忙，则 nand_stime=pl->next_plane_avail_time。
     */
    uint64_t nand_stime;

    /*
     * req_stime 是做 latency 统计时使用的“请求到达时间”。
     * 前台 I/O 一般传 req->stime；如果调用方传 0，就使用当前 QEMU 时间。
     */
    uint64_t req_stime = (ncmd->stime == 0) ? \
        qemu_clock_get_ns(QEMU_CLOCK_REALTIME) : ncmd->stime;

    /*
     * 当前 ZNS timing 真正使用的是 plane 级并行：
     * 每个 plane 自己维护 next_plane_avail_time。
     */
    struct zns_plane *pl = get_plane(zns, ppa);

    /*
     * lat 是这次子操作对调用方可见的时延：
     *   lat = finish_time - req_stime
     * 它包含两部分：在同一 plane 上排队等待的时间 + NAND 操作自身时延。
     */
    uint64_t lat = 0;

    /*
     * block 记录当前块的 nand_type，用它选择 SLC/TLC/QLC 等不同 timing 参数。
     */
    int nand_type = get_blk(zns,ppa)->nand_type;

    /* 根据 nand_type 取出本次 read/write/erase 的基础时延。 */
    uint64_t read_delay = zns->timing.pg_rd_lat[nand_type];
    uint64_t write_delay = zns->timing.pg_wr_lat[nand_type];
    uint64_t erase_delay = zns->timing.blk_er_lat[nand_type];

    //增加代码
    //old_avail：这个 plane 原来什么时候空闲
    uint64_t old_avail = pl->next_plane_avail_time; 
    //op_delay：本次操作本身要花多久
    uint64_t op_delay = 0;
    const char *cmd_name = "UNKNOWN";

    //
    switch (c) {
    case NAND_READ:
        // nand_stime = (pl->next_plane_avail_time < req_stime) ? req_stime : \ pl->next_plane_avail_time;
        // pl->next_plane_avail_time = nand_stime + read_delay;
        // lat = pl->next_plane_avail_time - req_stime;
        op_delay = read_delay;
        cmd_name = "READ";
	    break;

    case NAND_WRITE:
	    // nand_stime = (pl->next_plane_avail_time < req_stime) ? req_stime : \ pl->next_plane_avail_time;
	    // pl->next_plane_avail_time = nand_stime + write_delay;
	    // lat = pl->next_plane_avail_time - req_stime;
        op_delay = write_delay;
        cmd_name = "WRITE";
	    break;

    case NAND_ERASE:
        // nand_stime = (pl->next_plane_avail_time < req_stime) ? req_stime : \pl->next_plane_avail_time;
        // pl->next_plane_avail_time = nand_stime + erase_delay;
        // lat = pl->next_plane_avail_time - req_stime;
        op_delay = erase_delay;
        cmd_name = "ERASE";
        break;

    default:
        /* 未识别的 NAND 命令不处理，保留分支避免编译告警。 */
        ;
    }

     /*
     * nand_stime：本次 NAND 操作真正开始的时间
     * 如果 plane 已经空闲，就从 req_stime 开始；
     * 如果 plane 还忙，就等到 old_avail。
     */
    nand_stime = (old_avail < req_stime) ? req_stime : old_avail;

    /*
     * 推进当前 plane 的时间线。
     */
    pl->next_plane_avail_time = nand_stime + op_delay;

    /*
     * 对请求可见的延迟 = 完成时间 - 请求到达时间。
     */
    lat = pl->next_plane_avail_time - req_stime;

    /*
     * 用于分析并行单元忙闲情况。
     */
    uint64_t finish_time = pl->next_plane_avail_time;
    uint64_t queue_wait = (old_avail > req_stime) ? (old_avail - req_stime) : 0;
    uint64_t idle_gap = (req_stime > old_avail) ? (req_stime - old_avail) : 0;

    /*
     * 每一行代表一次 NAND 子操作占用一个 plane 的时间段。
     */
    femu_log("[PU] cmd=%s req=%lu old_avail=%lu start=%lu finish=%lu delay=%lu lat=%lu wait=%lu idle=%lu "
             "ch=%u lun=%u pl=%u blk=%u pg=%u spg=%u\n",
             cmd_name, //读or写or擦除
             req_stime, //请求到达时间
             old_avail, //这个 plane 上一次操作在什么时刻结束
             nand_stime, //操作的开始时间
             finish_time, //操作的结束时间
             op_delay, //操作的自身执行延迟
             lat, //完成时间 - 请求到达时间
             queue_wait, //排队等待时间
             idle_gap, //这个plane的空闲时间
             ppa->g.ch, //channel
             ppa->g.fc, //flash chip
             ppa->g.pl, //plane
             ppa->g.blk, //block
             ppa->g.pg, //page
             ppa->g.spg); //subpage
    return lat;
}

/* 检查 PPA 是否落在当前 ZNS 几何范围内。 */
static inline bool valid_ppa(struct zns_ssd *zns, struct ppa *ppa)
{
    int ch = ppa->g.ch;
    int lun = ppa->g.fc;
    int pl = ppa->g.pl;
    int blk = ppa->g.blk;
    int pg = ppa->g.pg;
    int sub_pg = ppa->g.spg;

    if (ch >= 0 && ch < zns->num_ch && lun >= 0 && lun < zns->num_lun && pl >=
        0 && pl < zns->num_plane && blk >= 0 && blk < zns->num_blk && pg>=0 && pg < zns->num_page && sub_pg >= 0 && sub_pg < ZNS_PAGE_SIZE/LOGICAL_PAGE_SIZE)
        return true;

    return false;
}

/* 判断 PPA 是否已经映射；UNMAPPED_PPA 表示还没有物理位置。 */
static inline bool mapped_ppa(struct ppa *ppa)
{
    return !(ppa->ppa == UNMAPPED_PPA);
}

/* 根据当前写指针生成新 PPA 的基础坐标，plane/page/subpage 后续再补。 */
static struct ppa get_new_page(struct zns_ssd *zns)
{
    struct write_pointer *wpp = &zns->wp;
    struct ppa ppa;
    ppa.ppa = 0;
    ppa.g.ch = wpp->ch;
    ppa.g.fc = wpp->lun;
    ppa.g.blk = zns->active_zone;
    ppa.g.V = 1; // 1 表示这不是 padding page
    if(!valid_ppa(zns,&ppa))
    {
        ftl_err("[Misao] invalid ppa: ch %u lun %u pl %u blk %u pg %u subpg  %u \n",ppa.g.ch,ppa.g.fc,ppa.g.pl,ppa.g.blk,ppa.g.pg,ppa.g.spg);
        ppa.ppa = UNMAPPED_PPA;
    }
    return ppa;
}

/* 查找当前 active zone 已绑定的 write cache 槽位；找不到返回 -1。 */
static int zns_get_wcidx(struct zns_ssd* zns)
{
    int i;
    for(i = 0;i < zns->cache.num_wc;i++)
    {
        if(zns->cache.write_cache[i].sblk==zns->active_zone)
        {
            return i;
        }
    }
    return -1;
}

/* 处理一条 ZNS read：LBA -> LPN -> PPA，然后对每个有效 PPA 推进 NAND_READ。 */
static uint64_t zns_read(struct zns_ssd *zns, NvmeRequest *req)
{
    /*
     * req->slba 是这条 NVMe read 的起始 LBA。
     * req->nlb 是这条请求覆盖的 LBA 数量。
     */
    uint64_t lba = req->slba;
    uint32_t nlb = req->nlb;

    /*
     * secs_per_pg 表示一个 4KiB = 4*1024B FTL 逻辑页包含多少个 NVMe LBA。
     * 例如 zns->lbasz=512B 时，secs_per_pg=4096/512=8。
     */
    uint64_t secs_per_pg = LOGICAL_PAGE_SIZE/zns->lbasz;
    uint64_t start_lpn = lba / secs_per_pg;
    uint64_t end_lpn = (lba + nlb - 1) / secs_per_pg;
    //int wcidx = zns_get_wcidx(zns);

    /* 当前 LPN 通过 L2P 查到的物理位置。 */
    struct ppa ppa;

    /* 循环处理本请求覆盖到的每个 4KiB 逻辑页。 */
    uint64_t lpn;

    /*
     * sublat 是当前 LPN 对应的一个 NAND_READ 子操作时延。
     * maxlat 是整条 NVMe read 的时延：多个子读都从 req->stime 出发，
     * 请求完成时间由最慢的那个子读决定。
     */
    uint64_t sublat, maxlat = 0;
    /*
     * est 是 estimated 的缩写，表示“在线估计值”。
     * 这些值在真实调度前用 shadow plane 时间线算出，后面会和真实观测值对比。
     */
    uint64_t est_maxlat = 0;
    uint64_t est_minlat = ~0ULL;
    uint64_t est_sumlat = 0;
    uint64_t read_stime = (req->stime == 0) ? \
        qemu_clock_get_ns(QEMU_CLOCK_REALTIME) : req->stime;
    /*
     * 当前读取请求的动机实验数据：
     *   sublat：单个 NAND 读取子操作的延迟
     *   maxlat：整条主机读取请求的延迟，由最慢的子操作决定
     *   松弛时间：maxlat - sublat
     */
    uint64_t nr_lpn = end_lpn - start_lpn + 1; //计算这条请求一共覆盖了多少个 LPN。
    uint64_t sample_cnt = 0; //子请求数量
    uint64_t minlat = ~0ULL; //表示一个 uint64_t 能表示的最大值
    uint64_t sumlat = 0; 
    struct zns_read_slack_sample *samples;

    samples = g_malloc0(sizeof(*samples) * nr_lpn);

    /*
     * 第一步：只收集这条 host read 的所有有效 NAND read 子请求。
     *
     * 原来的代码是一边查 LPN，一边立刻调用 zns_advance_status()。
     * 现在为了在真实执行前估计 slack，先只把 LPN/PPA/plane 记下来，
     * 暂时不更新任何 plane 的 next_plane_avail_time。
     */
    for (lpn = start_lpn; lpn <= end_lpn; lpn++) {
        /* LPN -> PPA：先查逻辑页当前映射到哪个物理页。 */
        ppa = get_maptbl_ent(zns, lpn);
        if (!mapped_ppa(&ppa) || !valid_ppa(zns, &ppa)) {
            /*
             * 没写过或映射非法的 LPN 不产生 NAND timing。
             * 当前实现直接跳过，不返回读错误。
             */
            continue;
        }

        samples[sample_cnt].lpn = lpn;
        samples[sample_cnt].ppa = ppa;
        samples[sample_cnt].ch = ppa.g.ch;
        samples[sample_cnt].lun = ppa.g.fc;
        samples[sample_cnt].pl = ppa.g.pl;
        sample_cnt++;
    }

    if (sample_cnt) {
        /*
         * 第二步：在 shadow plane timeline 上“假装执行”这些 read 子请求。
         *
         * 对每个子请求：
         *   est_start  = max(req到达时间, shadow plane 可用时间)
         *   est_finish = est_start + read_delay
         *   est_sublat = est_finish - req到达时间
         *
         * 然后只更新 shadow->avail，不更新真实的 pl->next_plane_avail_time。
         * 这样可以提前得到估计 slack，又不会改变当前 FEMU baseline 行为。
         */
        struct zns_read_shadow_plane *shadow_planes;
        uint64_t shadow_cnt = 0;
        uint64_t i;

        shadow_planes = g_malloc0(sizeof(*shadow_planes) * sample_cnt);

        for (i = 0; i < sample_cnt; i++) {
            struct ppa *sppa = &samples[i].ppa;
            struct zns_read_shadow_plane *shadow;
            uint64_t read_delay;

            shadow = zns_find_read_shadow_plane(shadow_planes, shadow_cnt, sppa);
            if (!shadow) {
                /*
                 * 第一次遇到这个 plane，把真实 next_plane_avail_time 抄到
                 * shadow->avail。之后同一个 host read 内再访问这个 plane，
                 * 就沿用 shadow->avail 模拟该 plane 上的排队。
                 */
                shadow = &shadow_planes[shadow_cnt++];
                shadow->ch = sppa->g.ch;
                shadow->lun = sppa->g.fc;
                shadow->pl = sppa->g.pl;
                shadow->avail = get_plane(zns, sppa)->next_plane_avail_time;
            }

            read_delay = zns->timing.pg_rd_lat[get_blk(zns, sppa)->nand_type];

            /*
             * 这几行就是在线估计的核心：
             * 如果 shadow plane 已经空闲，就从 read_stime 开始；
             * 如果 shadow plane 还忙，就等到 shadow->avail 再开始。
             */
            samples[i].est_start = (shadow->avail < read_stime) ? \
                read_stime : shadow->avail;
            samples[i].est_finish = samples[i].est_start + read_delay;
            samples[i].est_sublat = samples[i].est_finish - read_stime;

            /*
             * 只推进 shadow 时间线。真实 plane 时间线稍后仍由
             * zns_advance_status() 推进。
             */
            shadow->avail = samples[i].est_finish;

            est_minlat = (samples[i].est_sublat < est_minlat) ? \
                samples[i].est_sublat : est_minlat;
            est_sumlat += samples[i].est_sublat;
            est_maxlat = (samples[i].est_sublat > est_maxlat) ? \
                samples[i].est_sublat : est_maxlat;
        }

        for (i = 0; i < sample_cnt; i++) {
            /*
             * host read 的完成时间由最慢的子请求决定。
             * 子请求估计 slack = 估计最慢延迟 - 当前子请求估计延迟。
             */
            samples[i].est_slack = est_maxlat - samples[i].est_sublat;
        }

        g_free(shadow_planes);
    }

    /*
     * 第三步：按原来的方式真实发起 NAND_READ。
     *
     * 前面的估计只碰 shadow，所以这里仍然由 zns_advance_status() 推进真实
     * plane 时间线，并返回真实观测到的 sublat。也就是说，本次改动只增加
     * 估计和日志，不改变 read 请求的真实完成时间。
     */
    for (uint64_t i = 0; i < sample_cnt; i++) {
        ppa = samples[i].ppa;

        /*
         * 为当前 LPN 构造一个 NAND page read 子命令。
         * USER_IO 表示这是前台主机 I/O，不是 GC 或后台任务。
         */
        struct nand_cmd srd;
        srd.type = USER_IO;
        srd.cmd = NAND_READ;

        /*
         * 同一条 NVMe 请求内的所有子读都使用 req->stime 作为到达时间。
         * 如果它们落到不同 plane，就可以在模型里并行推进。
         */
        srd.stime = read_stime;

        sublat = zns_advance_status(zns, &ppa, &srd);
        femu_log("[R] lpn:\t%lu\t<--ch:\t%u\tlun:\t%u\tpl:\t%u\tblk:\t%u\tpg:\t%u\tsubpg:\t%u\tlat\t%lu\n",samples[i].lpn,ppa.g.ch,ppa.g.fc,ppa.g.pl,ppa.g.blk,ppa.g.pg,ppa.g.spg,sublat);

        /*
         * 保存真实观测到的子请求延迟。这里还不能立刻计算真实 slack，
         * 因为只有所有子请求都执行完后，才知道真实 maxlat。
         */
        samples[i].sublat = sublat;
        minlat = (sublat < minlat) ? sublat : minlat;
        sumlat += sublat;
        maxlat = (sublat > maxlat) ? sublat : maxlat;
    }

    if (sample_cnt) {
        /*
         * 整条主机读取请求在 maxlat 时刻完成。任何 sublat < maxlat 的子操作
         * 都存在请求内部松弛时间，理论上可以被 Slacker 类调度器利用。
         */
        uint64_t total_slack = 0;
        uint64_t max_slack = 0;
        uint64_t est_total_slack = 0;
        uint64_t est_max_slack = 0;
        uint64_t total_sublat_err = 0;
        uint64_t total_slack_err = 0;
        uint64_t reqid = zns_read_slack_reqid++; //为这条读请求分配一个 reqid
        uint64_t unique_planes = 0; //这条读请求覆盖了多少个不同的 flash plane
        uint64_t i, j;

        for (i = 0; i < sample_cnt; i++) { //遍历每个子请求
            uint64_t slack = maxlat - samples[i].sublat;
            /*
             * slack 是真实观测值；est_slack 是前面 shadow 模拟出来的估计值。
             * 两者的误差用于判断 estimator 准不准。
             */
            uint64_t est_slack = samples[i].est_slack;
            uint64_t sublat_err = zns_absdiff_u64(samples[i].est_sublat,
                                                  samples[i].sublat);
            uint64_t slack_err = zns_absdiff_u64(est_slack, slack);
            bool seen = false; //表示当前这个 plane 之前是否已经出现过

            total_slack += slack;
            max_slack = (slack > max_slack) ? slack : max_slack;
            est_total_slack += est_slack;
            est_max_slack = (est_slack > est_max_slack) ? est_slack : est_max_slack;
            total_sublat_err += sublat_err;
            total_slack_err += slack_err;

            for (j = 0; j < i; j++) {
                if (samples[i].ch == samples[j].ch &&
                    samples[i].lun == samples[j].lun &&
                    samples[i].pl == samples[j].pl) {
                    seen = true;
                    break;
                }
            }
            if (!seen) {
                unique_planes++;
            }

            /*
             * 子操作级日志：后续可以用 slack_ns 字段画“松弛时间”的累计分布图，
             * 对应 Slacker 论文动机实验中的松弛时间分布。
             */
            ftl_log("ZNS_READ_SUBSLACK,reqid=%lu,subidx=%lu,lpn=%lu,"
                    "ch=%u,lun=%u,pl=%u,sublat_ns=%lu,slack_ns=%lu\n",
                    reqid, i, samples[i].lpn, samples[i].ch, samples[i].lun,
                    samples[i].pl, samples[i].sublat, slack);

            /*
             * 子操作级在线估计日志：
             *   est_*：estimated，真实调度前用 shadow timeline 算出的估计值；
             *   obs_*：observed，zns_advance_status() 真实执行后的观测值；
             *   *_err：估计值和观测值的绝对误差。
             */
            ftl_log("ZNS_READ_SUBESTSLACK,reqid=%lu,subidx=%lu,lpn=%lu,"
                    "ch=%u,lun=%u,pl=%u,est_start_ns=%lu,est_finish_ns=%lu,"
                    "est_sublat_ns=%lu,est_slack_ns=%lu,obs_sublat_ns=%lu,"
                    "obs_slack_ns=%lu,sublat_err_ns=%lu,slack_err_ns=%lu\n",
                    reqid, i, samples[i].lpn, samples[i].ch, samples[i].lun,
                    samples[i].pl, samples[i].est_start, samples[i].est_finish,
                    samples[i].est_sublat, est_slack, samples[i].sublat,
                    slack, sublat_err, slack_err);
        }

        /*
         * 请求级汇总日志：unique_planes 是 FEMU/ZNS 里的等价指标，用来表示
         * 一条读取请求实际覆盖了多少个内部闪存平面资源。
         */
        ftl_log("ZNS_READ_SLACK,reqid=%lu,slba=%lu,nlb=%u,subops=%lu,"
                "unique_planes=%lu,minlat_ns=%lu,maxlat_ns=%lu,"
                "avg_sublat_ns=%lu,total_slack_ns=%lu,avg_slack_ns=%lu,"
                "max_slack_ns=%lu\n",
                reqid, lba, nlb, sample_cnt, unique_planes, minlat, maxlat,
                sumlat / sample_cnt, total_slack, total_slack / sample_cnt,
                max_slack);

        /*
         * 请求级在线估计日志。
         *
         * 这行把一整条 host read 的估计结果和真实观测结果放在一起。
         * 初版没有改变调度顺序，因此在当前 deterministic timing 模型下，
         * maxlat_err_ns / avg_sublat_err_ns / avg_slack_err_ns 理论上应该很小。
         */
        ftl_log("ZNS_READ_ESTSLACK,reqid=%lu,slba=%lu,nlb=%u,subops=%lu,"
                "unique_planes=%lu,est_minlat_ns=%lu,est_maxlat_ns=%lu,"
                "est_avg_sublat_ns=%lu,est_total_slack_ns=%lu,"
                "est_avg_slack_ns=%lu,est_max_slack_ns=%lu,"
                "obs_maxlat_ns=%lu,obs_max_slack_ns=%lu,maxlat_err_ns=%lu,"
                "avg_sublat_err_ns=%lu,avg_slack_err_ns=%lu\n",
                reqid, lba, nlb, sample_cnt, unique_planes, est_minlat,
                est_maxlat, est_sumlat / sample_cnt, est_total_slack,
                est_total_slack / sample_cnt, est_max_slack, maxlat,
                max_slack, zns_absdiff_u64(est_maxlat, maxlat),
                total_sublat_err / sample_cnt, total_slack_err / sample_cnt);
    }

    g_free(samples);

    return maxlat;
}

/* 把一个 write cache 槽位里的 LPN 刷到 NAND；这里才真正产生 NAND_WRITE。 */
static uint64_t zns_wc_flush(struct zns_ssd* zns, int wcidx, int type,uint64_t stime)
{
    /*
     * 这些循环变量很重要：
     *   i       指向 write_cache[wcidx] 里下一个待刷的 LPN；
     *   p       遍历当前 (channel, LUN) 下的 plane；
     *   j       按 flash_type 决定每个 plane 连续写几个物理 page；
     *           本代码约定 SLC=1, TLC=3, QLC=4；
     *   subpage 遍历一个物理 ZNS page 内部的 4KiB 子页位置。
     */
    int i,j,p,subpage;

    /* 当前正在填充并准备 program 的物理地址。 */
    struct ppa ppa;

    /*
     * oldppa 是被覆盖 LPN 的旧物理地址。
     * 当前代码只检查旧映射是否存在，还没有维护 invalid page 计数。
     */
    struct ppa oldppa;

    /* 当前从 write cache 里取出的逻辑页号。 */
    uint64_t lpn;

    /*
     * flash_type 决定每个 plane 在这一轮里写多少个物理 page。
     * 比如 QLC=4，则 j 循环 4 次。
     */
    int flash_type = zns->flash_type;

    /*
     * sublat 是当前一次 NAND_WRITE 的时延；
     * maxlat 是本次 flush 返回给调用方的可见时延。
     */
    uint64_t sublat = 0, maxlat = 0;

    /* i 指向当前还未刷入阵列的第一个 LPN。 */
    i = 0;
    while(i < zns->cache.write_cache[wcidx].used)
    {
        /*
         * 在当前 (channel, lun) 位置上，依次向各个 plane 分配页。
         * 同一轮会把尽可能多的 LPN 条带化写到不同 plane 上。
         */
        for(p = 0;p<zns->num_plane;p++){
            /* 为当前 plane 生成一个新的物理写入位置，block 由 active_zone 决定。 */
            ppa = get_new_page(zns);

            /*
             * get_new_page() 只填 channel/LUN/block。
             * plane 由当前 p 循环决定，因此这层循环是在跨 plane 条带化写入。
             */
            ppa.g.pl = p;
            for(j = 0; j < flash_type ;j++)
            {
                /* 取出该 block 在当前 plane 上的下一可写页。 */
                ppa.g.pg = get_blk(zns,&ppa)->page_wp;

                /*
                 * page_wp 是这个 block 在这个 plane 内的 next-free-page 指针。
                 * 每分配一个物理 page 就立刻递增，避免后续写覆盖同一页。
                 */
                get_blk(zns,&ppa)->page_wp++;
                //遍历一个物理 page 内部的 subpage
                for(subpage = 0;subpage < ZNS_PAGE_SIZE/LOGICAL_PAGE_SIZE;subpage++)
                {
                    if(i+subpage >= zns->cache.write_cache[wcidx].used)
                    {
                        /* 缓存中的有效 LPN 已经用完，不再填充无效子页。 */
                        break;
                    }
                    /* 从写缓存中取出一个 LPN，准备映射到当前物理页的一个 subpage。 */
                    lpn = zns->cache.write_cache[wcidx].lpns[i+subpage];

                    /*
                     * 如果这个 LPN 以前已经有映射，说明这是覆盖写。
                     * 下面 set_maptbl_ent() 会用新 PPA 替换旧 PPA。
                     */
                    oldppa = get_maptbl_ent(zns, lpn);
                    if (mapped_ppa(&oldppa)) {
                        /* FIXME: 旧映射失效后的元数据暂未维护。 */
                    }
                    /* subpage 编号决定该 LPN 落在当前物理页中的哪个 4 KiB 片段。 */
                    ppa.g.spg = subpage;
                    /* 更新 L2P 映射：记录这个 LPN 现在对应的新 PPA。 */
                    set_maptbl_ent(zns, lpn, &ppa);
                    //femu_log("[F] lpn:\t%lu\t-->ch:\t%u\tlun:\t%u\tpl:\t%u\tblk:\t%u\tpg:\t%u\tsubpg:\t%u\tlat\t%lu\n",lpn,ppa.g.ch,ppa.g.fc,ppa.g.pl,ppa.g.blk,ppa.g.pg,ppa.g.spg,sublat);
                }
                /* 一个 16 KiB 物理页最多承载 4 个 4 KiB LPN。 */
                i+=ZNS_PAGE_SIZE/LOGICAL_PAGE_SIZE;
            }
            /* FIXME: 这里默认按有效页统计，尚未单独区分 padding page。 */
            //判断 PPA 是否有效
            if(ppa.g.V)
            {
                struct nand_cmd swr;
                swr.type = type;
                swr.cmd = NAND_WRITE;

                /*
                 * stime 是触发这次 flush 的主机请求时间。
                 * 本次 flush 内的多个 NAND_WRITE 都用这个时间作到达时间，
                 * 是否排队由各自目标 plane 的 next_plane_avail_time 决定。
                 */
                swr.stime = stime;
                /* 按当前 plane 的可用时间推进一次 NAND program 时延。 */
                sublat = zns_advance_status(zns, &ppa, &swr);
                maxlat = (sublat > maxlat) ? sublat : maxlat;
            }
        }
        /*
         * 当前 (channel, lun) 这一轮的各个 plane 都写完后，
         * 再把全局写指针推进到下一个 channel；channel 用完后推进 lun。
         */
        zns_advance_write_pointer(zns);
    }
    /* 刷写完成后，清空该写缓存槽位中的有效条目数。 */
    zns->cache.write_cache[wcidx].used = 0;
    /* 返回本次 flush 中所有物理写的最大完成时延。 */
    return maxlat;
}

/* 处理一条 ZNS write：先写 SRAM write cache，必要时再 flush 到 NAND。 */
static uint64_t zns_write(struct zns_ssd *zns, NvmeRequest *req)
{
    /* 主机写请求的起始 LBA 和长度。 */
    uint64_t lba = req->slba;
    uint32_t nlb = req->nlb;

    /* 将请求的 LBA 范围换算成 4 KiB 粒度的逻辑页号 LPN。 */
    uint64_t secs_per_pg = LOGICAL_PAGE_SIZE/zns->lbasz;
    uint64_t start_lpn = lba / secs_per_pg;
    uint64_t end_lpn = (lba + nlb - 1) / secs_per_pg;

    /* 当前准备追加到 SRAM write cache 的 LPN。 */
    uint64_t lpn;

    /*
     * sublat 累加连续写 SRAM cache 的时延；
     * maxlat 是最终返回给 NVMe 请求的可见时延。
     */
    uint64_t sublat = 0, maxlat = 0;

    /* 扫描 write cache 槽位时使用。 */
    int i;

    /* 当前 zone 优先复用已经绑定到自己的写缓存槽位。 */
    int wcidx = zns_get_wcidx(zns);

    if(wcidx==-1)
    {
        /*
         * 当前没有缓存槽位属于这个 zone。
         * 优先找空闲槽位；如果没有空闲槽位，就复用一个已有槽位，
         * 并在复用前先把其中暂存的 LPN 刷到 NAND 阵列。
         */
        wcidx = 0;

        /*
         * t_used 记录候选 cache 槽位里已经缓存了多少 LPN。
         * 槽位选择策略：
         *   1. 优先使用空槽位；
         *   2. 如果没有空槽位，选择 used 最大的槽位，先 flush 再复用。
         */
        uint64_t t_used = zns->cache.write_cache[wcidx].used;
        for(i = 1;i < zns->cache.num_wc;i++)
        {
            if(zns->cache.write_cache[i].used==0)//有空闲槽
            {
                t_used = 0;
                wcidx = i; // 找到空闲 write cache 槽位
                break;
            }
            //没有遇到空闲槽位的情况下，选择当前缓存数据最多的槽位 
            if(zns->cache.write_cache[i].used > t_used)
            {
                t_used = zns->cache.write_cache[i].used;
                wcidx = i;
            }
        }

        /*
         * 复用非空槽位前必须先 flush。
         * 这是 host write 真正开始占用 NAND plane 的时刻之一。
         */
        if(t_used) maxlat = zns_wc_flush(zns,wcidx,USER_IO,req->stime);
        /* 将选中的缓存槽位绑定到当前请求所在的 zone。 */
        zns->cache.write_cache[wcidx].sblk = zns->active_zone;
    }

    for (lpn = start_lpn; lpn <= end_lpn; lpn++) {
        //如果当前 write cache 满了，先 flush
        if(zns->cache.write_cache[wcidx].used==zns->cache.write_cache[wcidx].cap)
        {
            //打印 flush 日志，写缓存的使用量
            femu_log("[W] flush wc %d (%u/%u)\n",wcidx,(int)zns->cache.write_cache[wcidx].used,(int)zns->cache.write_cache[wcidx].cap);
            sublat = zns_wc_flush(zns,wcidx,USER_IO,req->stime);
            femu_log("[W] flush lat: %u\n", (int)sublat);//打印这次 flush 产生的延迟
            maxlat = (sublat > maxlat) ? sublat : maxlat;
            sublat = 0;
        }
        /* 将一个逻辑页先写入当前 zone 对应的 SRAM 写缓存。 */  
        zns->cache.write_cache[wcidx].lpns[zns->cache.write_cache[wcidx].used++]=lpn;

        /*
         * 这里只模拟写 SRAM cache 的开销。
         * 它不会更新任何 plane 的 next_plane_avail_time；
         * 真正的 NAND timing 从 zns_wc_flush() 开始。
         */
        sublat += SRAM_WRITE_LATENCY_NS; // 简化模拟 SRAM 写入时延
        maxlat = (sublat > maxlat) ? sublat : maxlat;
        femu_log("[W] lpn:\t%lu\t-->wc cache:%u, used:%u\n",lpn,(int)wcidx,(int)zns->cache.write_cache[wcidx].used);
    }
    /*
     * 返回这次请求看到的完成时延：
     * 可能只是写入 SRAM 缓存的时延，也可能包含触发 flush 后更大的阵列写时延。
     */
    return maxlat;
}

/* ZNS FTL 后台线程：从 to_ftl 取请求，计算时延，再放回 to_poller。 */
static void *ftl_thread(void *arg)
{
    FemuCtrl *n = (FemuCtrl *)arg;    // 解析线程入参：FEMU控制结构体
    struct zns_ssd *zns = n->zns;     // 提取ZNS SSD模拟对象
    NvmeRequest *req = NULL;          // 存储当前处理的NVMe请求
    uint64_t lat = 0;                 // 请求处理时延，单位是纳秒
    int rc;                           // 存储函数返回码（校验操作结果）
    int i;                            // 队列遍历计数器

    while (!*(zns->dataplane_started_ptr)) { //确保FTL线程只在SSD处理IO请求的核心链路完全初始化后才工作
        usleep(100000);
    }

    /* FIXME: 这里直接绑定队列指针，后续可以改成更稳妥的生命周期管理。 */
    zns->to_ftl = n->to_ftl;    // 绑定“待FTL处理”的请求队列数组
    zns->to_poller = n->to_poller;  // 绑定“待响应主机”的请求队列数组

    while (1) {
        for (i = 1; i <= n->nr_pollers; i++) {
            // 跳过“空队列指针”或“无请求的队列”
            if (!zns->to_ftl[i] || !femu_ring_count(zns->to_ftl[i]))
                continue;
            // 从to_ftl[i]队列中取出1个请求
            rc = femu_ring_dequeue(zns->to_ftl[i], (void *)&req, 1);
            if (rc != 1) {
                printf("FEMU: FTL to_ftl dequeue failed\n");
            }

            ftl_assert(req);
            /*
             * req->cmd.opcode 是 NVMe 命令类型：
             *   WRITE / ZONE_APPEND 走 zns_write()
             *   READ                走 zns_read()
             *   DSM                 当前不模拟 NAND 时延
             */
            switch (req->cmd.opcode) {
                // 修复：zone append 也需要走配置好的 ZNS 写入时延。
                case NVME_CMD_ZONE_APPEND:
                    /* zone append 和普通 write 走同一条写路径。 */
                case NVME_CMD_WRITE:
                    lat = zns_write(zns, req);
                    break;
                case NVME_CMD_READ:
                    lat = zns_read(zns, req);
                    break;
                case NVME_CMD_DSM:
                    lat = 0;
                    break;
                default:
                    //ftl_err("FTL 收到未知请求类型\n");
                    ;
            }

            // 记录请求的处理延迟
            req->reqlat = lat;

            /*
             * req->expire_time 在 nvme-io.c 里初始化为 req->stime。
             * 这里加上 lat，表示这条请求最早可以完成的时间；
             * NVMe poller 后面会等当前时间到达 expire_time 后再发 CQE。
             */
            req->expire_time += lat;// 更新请求过期时间（加延迟）

            // 将处理完毕的请求，入队到对应的to_poller[i]队列
            rc = femu_ring_enqueue(zns->to_poller[i], (void *)&req, 1);
            if (rc != 1) {
                ftl_err("FTL to_poller enqueue failed\n");
            }

        }
    }

    return NULL;
}
