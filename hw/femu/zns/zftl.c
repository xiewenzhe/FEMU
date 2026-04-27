#include "zns.h"

//#define FEMU_DEBUG_ZFTL

static void *ftl_thread(void *arg);

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

    switch (c) {
    case NAND_READ:
        /*
         * 这里是后续插桩观察 slack 的核心位置：
         *   old_avail     = 修改前的 pl->next_plane_avail_time
         *   arrival_slack = max(0, req_stime - old_avail)
         *   queue_wait    = max(0, old_avail - req_stime)
         *
         * 如果 req_stime 比 old_avail 晚，说明请求到达前 plane 已经空闲了一段时间；
         * 如果 old_avail 比 req_stime 晚，说明请求需要等这个 plane。
         */
        nand_stime = (pl->next_plane_avail_time < req_stime) ? req_stime : \
                     pl->next_plane_avail_time;
        pl->next_plane_avail_time = nand_stime + read_delay;
        lat = pl->next_plane_avail_time - req_stime;
	    break;

    case NAND_WRITE:
        /* 写和读使用同一套排队规则，只是推进 page program 时延。 */
	    nand_stime = (pl->next_plane_avail_time < req_stime) ? req_stime : \
		            pl->next_plane_avail_time;
	    pl->next_plane_avail_time = nand_stime + write_delay;
	    lat = pl->next_plane_avail_time - req_stime;
	    break;

    case NAND_ERASE:
        /* 擦除也会占用目标 plane，通常比 read/write 更久。 */
        nand_stime = (pl->next_plane_avail_time < req_stime) ? req_stime : \
                        pl->next_plane_avail_time;
        pl->next_plane_avail_time = nand_stime + erase_delay;
        lat = pl->next_plane_avail_time - req_stime;
        break;

    default:
        /* 未识别的 NAND 命令不处理，保留分支避免编译告警。 */
        ;
    }

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
     * secs_per_pg 表示一个 4KiB FTL 逻辑页包含多少个 NVMe LBA。
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

    /* 普通读路径：逐个 LPN 查表并发起 NAND_READ。 */
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
        srd.stime = req->stime;

        sublat = zns_advance_status(zns, &ppa, &srd);
        femu_log("[R] lpn:\t%lu\t<--ch:\t%u\tlun:\t%u\tpl:\t%u\tblk:\t%u\tpg:\t%u\tsubpg:\t%u\tlat\t%lu\n",lpn,ppa.g.ch,ppa.g.fc,ppa.g.pl,ppa.g.blk,ppa.g.pg,ppa.g.spg,sublat);
        maxlat = (sublat > maxlat) ? sublat : maxlat;
    }

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
