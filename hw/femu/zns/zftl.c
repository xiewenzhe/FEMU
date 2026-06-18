#include "zns.h"

//#define FEMU_DEBUG_ZFTL

static void *ftl_thread(void *arg);
static inline struct zns_plane *get_plane(struct zns_ssd *zns, struct ppa *ppa);

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
    uint64_t sublat;       /* 当前调度下真实返回给 host read 的子请求延迟。 */
    uint64_t read_delay;   /* 这个 NAND read 自身的服务时间。 */
    uint64_t est_start;    /* 估计的 NAND read 开始时间。 */
    uint64_t est_finish;   /* 估计的 NAND read 完成时间。 */
    uint64_t est_sublat;   /* 估计的子请求延迟：est_finish - req_stime。 */
    uint64_t est_slack;    /* 估计的 slack：est_maxlat - est_sublat。 */
    uint64_t pred_critical_gap; /* est_maxlat - est_sublat，越小越接近关键路径。 */
    uint64_t exec_start;   /* execute 原型真正安排的 NAND read 开始时间。 */
    uint64_t exec_finish;  /* execute 原型真正安排的 NAND read 完成时间。 */
    uint64_t exec_hops;    /* 当前子请求真正绕过了多少个 waiting read。 */
    uint64_t exec_benefit; /* 相对 baseline est_finish 提前了多少时间。 */
    uint64_t exec_consumed_victim_slack; /* 当前子请求绕过 victim 时消耗的 slack 总量。 */
    uint64_t exec_blocked_by_pred_noncritical_slack; /* 被非关键绕行先消耗掉的 victim slack。 */
    bool exec_candidate;   /* 当前子请求是否满足进入 bypass 判断的前置条件。 */
    bool pred_critical;    /* 在线估计认为它是否接近当前 host read 的关键路径。 */
    bool exec_reject_slack_insufficient; /* 扫描时是否因 victim slack 不足而停止。 */
    bool exec_reject_after_pred_noncritical; /* slack 不足前，victim 是否被预测非关键请求消耗过。 */
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

/*
 * read bypass 队列中的一个 read 子请求。
 *
 * dry-run 阶段它只是 baseline 队列快照；进入 execute 原型后，它表示当前
 * plane 上“已经安排、但还没有开始”的 read 队列。每个字段的含义如下：
 *   reqid/subidx/lpn：唯一定位这个 read 子请求来自哪条 host read、哪一个子请求；
 *   req_stime：它所属 host read 到达 FTL 的时间；
 *   start/finish：当前 read 队列顺序下，这个子请求在目标 plane 的开始/结束时间；
 *   deadline：它所属 host read 已经承诺给主机的完成时间；
 *   slack：在不超过 deadline 的前提下，这个子请求当前还能容忍多少额外延迟；
 *   read_delay：这个子请求本身占用 plane 的服务时间；
 *   ch/lun/pl：它落到哪个内部 plane，主要用于日志核对。
 */
//记录一个 read 子请求做绕行判断时需要知道的全部信息
struct zns_read_bypass_entry {
    uint64_t reqid;
    uint64_t subidx;
    uint64_t lpn;
    uint64_t req_stime;
    uint64_t start;
    uint64_t finish;
    uint64_t deadline;
    uint64_t slack;
    /*
     * 诊断字段：记录这个 victim 的 slack 曾经被谁消耗过。
     * 后续如果一个 predicted-critical read 因 slack 不足被挡住，
     * 就能判断是不是之前 predicted-noncritical read 先吃掉了 slack。
     */
    uint64_t slack_consumed_by_pred_critical;
    uint64_t slack_consumed_by_pred_noncritical;
    uint64_t read_delay;
    uint16_t ch;
    uint16_t lun;
    uint16_t pl;
};

/*
 * 每个 plane 对应一个 read bypass 队列。
 *
 * entries 按当前服务顺序保存仍未开始的 read 子请求：
 *   entries[0]      是离队头最近、最早会被服务的 waiting read；
 *   entries[len-1]  是队尾，也是新 read 做 Slacker 式前移时首先检查的对象。
 */
//每个 plane 自己的 read bypass 队列
struct zns_read_bypass_queue {
    struct zns_read_bypass_entry *entries;
    uint64_t len;
    uint64_t cap;
};

#define ZNS_BYPASS_MAX_CH   (1 << CH_BITS)
#define ZNS_BYPASS_MAX_LUN  (1 << FC_BITS)
#define ZNS_BYPASS_MAX_PL   (1 << PL_BITS)
#define ZNS_BYPASS_MAX_PLANES \
    (ZNS_BYPASS_MAX_CH * ZNS_BYPASS_MAX_LUN * ZNS_BYPASS_MAX_PL)

/* 只用于把多行子操作日志关联到同一条主机读取请求。 */
static uint64_t zns_read_slack_reqid;
static struct zns_read_bypass_queue zns_read_bypass_queues[ZNS_BYPASS_MAX_PLANES];
static bool zns_read_bypass_exec_flag_loaded;
static bool zns_read_bypass_exec_flag;

static inline uint64_t zns_absdiff_u64(uint64_t a, uint64_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/*
 * 用环境变量切换 baseline / execute，便于同一份代码做干净对照：
 *   unset 或 0：保留 baseline read 调度，只输出 dry-run 机会分析；
 *   1/true/on：真正执行 read-read bypass。
 */
static bool zns_read_bypass_exec_enabled(void)
{
    const char *value;

    if (!zns_read_bypass_exec_flag_loaded) {
        value = g_getenv("FEMU_ZNS_READ_BYPASS_EXEC");
        zns_read_bypass_exec_flag =
            value &&
            (!g_ascii_strcasecmp(value, "1") ||
             !g_ascii_strcasecmp(value, "true") ||
             !g_ascii_strcasecmp(value, "on"));
        zns_read_bypass_exec_flag_loaded = true;
    }

    return zns_read_bypass_exec_flag;
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

//把 PPA 映射到队列下标
static int zns_read_bypass_plane_idx(struct ppa *ppa)
{
    if (ppa->g.ch >= ZNS_BYPASS_MAX_CH ||
        ppa->g.fc >= ZNS_BYPASS_MAX_LUN ||
        ppa->g.pl >= ZNS_BYPASS_MAX_PL) {
        //如果字段越界，就返回-1
        return -1;
    }

    //idx = ch * LUN数 * PL数 + lun * PL数 + pl
    return ppa->g.ch * ZNS_BYPASS_MAX_LUN * ZNS_BYPASS_MAX_PL +
           ppa->g.fc * ZNS_BYPASS_MAX_PL + ppa->g.pl;
}

//根据 PPA 找到对应 plane 队列
static struct zns_read_bypass_queue *zns_read_bypass_get_queue(struct ppa *ppa)
{
    //算出队列下标
    int idx = zns_read_bypass_plane_idx(ppa);

    if (idx < 0) {
        return NULL;
    }

    //下标合法返回这个 PPA 所属 plane 的 read bypass 队列
    return &zns_read_bypass_queues[idx];
}

//清空某个 plane 上记录的 pending read 队列。
static void zns_read_bypass_clear_queue(struct ppa *ppa)
{
    struct zns_read_bypass_queue *queue = zns_read_bypass_get_queue(ppa);

    if (queue) {
        queue->len = 0;
    }
}

/*
 * 已经开始执行的 read 不能再被后来请求绕过，所以它们不应继续留在 dry-run
 * 队列里。队列按 baseline start 时间有序，前缀一旦开始就可以整体删掉。
 */
//删除已经开始执行的 read。prune:删除
static void zns_read_bypass_prune_started(struct zns_read_bypass_queue *queue,
                                          uint64_t read_stime)
{
    uint64_t drop = 0;
    uint64_t i;

    while (drop < queue->len && queue->entries[drop].start <= read_stime) {
        drop++;
    }

    if (!drop) {
        return;
    }

    //把剩下的 entry 前移，维持队列连续
    for (i = drop; i < queue->len; i++) {
        queue->entries[i - drop] = queue->entries[i];
    }
    queue->len -= drop;
}

//确保队列至少还能多放一个 read entry
static void zns_read_bypass_queue_reserve_one(struct zns_read_bypass_queue *queue)
{
    if (queue->len == queue->cap) {
        uint64_t new_cap = queue->cap ? queue->cap * 2 : 8;

        queue->entries = g_realloc(queue->entries,
                                   sizeof(*queue->entries) * new_cap);
        queue->cap = new_cap;
    }
}

//往队列尾部加入一个 read entry
static void zns_read_bypass_queue_push(struct zns_read_bypass_queue *queue,
                                       struct zns_read_bypass_entry *entry)
{
    zns_read_bypass_queue_reserve_one(queue);
    queue->entries[queue->len++] = *entry;
}

//构造一个可放入 bypass 队列的 read entry。
static struct zns_read_bypass_entry zns_read_bypass_make_entry(
    uint64_t reqid, uint64_t subidx, struct zns_read_slack_sample *sample,
    uint64_t read_stime, uint64_t deadline, uint64_t start, uint64_t finish)
{
    struct zns_read_bypass_entry entry;

    entry.reqid = reqid;
    entry.subidx = subidx;
    entry.lpn = sample->lpn;
    entry.req_stime = read_stime;
    entry.start = start;
    entry.finish = finish;
    entry.deadline = deadline;
    entry.slack = (entry.deadline > entry.finish) ?
        (entry.deadline - entry.finish) : 0;
    entry.slack_consumed_by_pred_critical = 0;
    entry.slack_consumed_by_pred_noncritical = 0;
    entry.read_delay = sample->read_delay;
    entry.ch = sample->ch;
    entry.lun = sample->lun;
    entry.pl = sample->pl;

    return entry;
}

/*
 * execute 模式里，当前 host read 的真实完成时间要等它所有子请求都安排完
 * 才能知道。因此刚把 entry 插入队列时，只能先写一个临时 deadline。
 *
 * 由于 FTL 线程串行处理 host request，下一条 host read 到来前，当前请求一定
 * 已经走到这里。我们在返回给上层之前，把当前请求所有 queued subop 的
 * deadline 统一改成“真正会写入 req->expire_time 的 host completion”。
 *
 * 这个动作非常关键：后续 read 只能消耗 victim 在这个 deadline 之前的 slack。
 * 只要不把 victim 推过 deadline，它所属旧 host request 的 expire_time 就不需要
 * 被回头修改，主机看到的完成时间仍然正确。
 */
static void zns_read_bypass_finalize_deadline(uint64_t reqid,
                                              struct zns_read_slack_sample *samples,
                                              uint64_t sample_cnt,
                                              uint64_t read_stime,
                                              uint64_t req_maxlat)
{
    uint64_t deadline = read_stime + req_maxlat;
    uint64_t i;

    for (i = 0; i < sample_cnt; i++) {
        struct zns_read_bypass_queue *queue =
            zns_read_bypass_get_queue(&samples[i].ppa);
        uint64_t pos;

        if (!queue) {
            continue;
        }

        for (pos = 0; pos < queue->len; pos++) {
            struct zns_read_bypass_entry *entry = &queue->entries[pos];

            if (entry->reqid == reqid && entry->subidx == i) {
                entry->deadline = deadline;
                entry->slack = (deadline > entry->finish) ?
                    (deadline - entry->finish) : 0;
                break;
            }
        }
    }
}

//把一个 baseline read 子请求记录到队列，供后续 dry-run 使用。
static void zns_read_bypass_remember_baseline_read(
    uint64_t reqid, uint64_t subidx, struct zns_read_slack_sample *sample,
    uint64_t read_stime, uint64_t req_maxlat)
{
    struct zns_read_bypass_queue *queue =
        zns_read_bypass_get_queue(&sample->ppa);
    uint64_t finish;
    uint64_t start;
    uint64_t deadline;
    struct zns_read_bypass_entry entry;

    if (!queue) {
        return;
    }

    finish = read_stime + sample->sublat;
    start = finish - sample->read_delay;
    deadline = read_stime + req_maxlat;
    entry = zns_read_bypass_make_entry(reqid, subidx, sample, read_stime,
                                       deadline, start, finish);
    zns_read_bypass_queue_push(queue, &entry);
}

/*
 * 按 Slacker 的方式做多次前移 dry-run：
 * incoming read 从 plane 队尾向队头扫描，只要 waiting read 的 slack 足以
 * 容纳 incoming read 的服务时间，就假设可以继续向前跨过它。
 *
 * 注意：dry-run 不会真的改 entries 顺序，也不会把 victim->slack 写回扣减；
 * 它只回答“在当前 baseline 队列上，这个 read 最多能向前跨过多少个 read”。
 */
//对当前 host read 里的每个 read 子请求，在不改变真实调度的前提下，判断它能不能在对应 plane 队列
//中向前绕过前面的 waiting read。
static void zns_read_bypass_dryrun(uint64_t reqid,
                                   struct zns_read_slack_sample *samples,
                                   uint64_t sample_cnt,
                                   uint64_t read_stime)
{
    uint64_t candidate_subops = 0; //有机会参与 bypass 判断的子请求数
    uint64_t bypassable_subops = 0; //实际可以绕过至少一个 waiting read 的子请求数
    uint64_t total_hops = 0; //所有子请求一共能绕过多少个 waiting read
    uint64_t total_benefit = 0; //总共理论上能提前多少时间
    uint64_t max_hops = 0; //单个子请求最多能绕过几个 waiting read
    uint64_t max_benefit = 0; //单个子请求最大理论收益
    uint64_t i;

    //逐个子请求找对应 plane 队列
    for (i = 0; i < sample_cnt; i++) {
        struct zns_read_bypass_queue *queue =
            zns_read_bypass_get_queue(&samples[i].ppa);
        uint64_t hops = 0;
        uint64_t bypass_start = 0;
        uint64_t bypass_finish = 0;
        uint64_t benefit = 0;
        uint64_t pos;

        if (!queue) {
            continue;
        }

        //删掉已经开始执行的 waiting read。
        zns_read_bypass_prune_started(queue, read_stime);

        //如果队列为空：说明当前 plane 上没有可绕过的 pending read，直接跳过
        if (!queue->len) {
            continue;
        }

        /*
         * 如果队尾 read 的 finish 不是当前 read 的 baseline start，说明两者
         * 中间还夹了别的真实操作；保守起见不跨越这个空档或非 read 操作。
         */
        if (queue->entries[queue->len - 1].finish != samples[i].est_start) {
            continue;
        }

        /*
          1. 有对应 plane 队列；
          2. 队列中有 waiting read；
          3. 队尾 waiting read 与当前 read 在 baseline 时间线上连续；
        */
        // 因此这个子请求是一个 bypass 候选
        candidate_subops++;

        //从队尾向队头扫描，multi-hop bypass
        for (pos = queue->len; pos > 0; pos--) {
            struct zns_read_bypass_entry *victim = &queue->entries[pos - 1];
            uint64_t victim_new_slack;

            /*
             * 论文中的条件：waiting sub-request 的 slack 必须不小于
             * incoming sub-request 的预计服务时间。
             */
            if (victim->slack < samples[i].read_delay) {
                break;
            }

            victim_new_slack = victim->slack - samples[i].read_delay;
            hops++;
            bypass_start = victim->start;

            //当前 incoming read 成功绕过了哪个 victim，以及 victim 被扣减 slack 后还剩多少。
            ftl_log("ZNS_READ_BYPASS_DRYRUN_HOP,reqid=%lu,subidx=%lu,lpn=%lu,"
                    "ch=%u,lun=%u,pl=%u,hop=%lu,victim_reqid=%lu,"
                    "victim_subidx=%lu,victim_lpn=%lu,victim_start_ns=%lu,"
                    "victim_finish_ns=%lu,victim_deadline_ns=%lu,"
                    "victim_slack_ns=%lu,victim_new_slack_ns=%lu,"
                    "slack_used_ns=%lu\n",
                    reqid, i, samples[i].lpn, samples[i].ch, samples[i].lun,
                    samples[i].pl, hops, victim->reqid, victim->subidx,
                    victim->lpn, victim->start, victim->finish,
                    victim->deadline, victim->slack, victim_new_slack,
                    samples[i].read_delay);
        }

        //没有绕过任何 waiting read，就跳过
        if (!hops) {
            continue;
        }

        bypass_finish = bypass_start + samples[i].read_delay;
        benefit = (samples[i].est_finish > bypass_finish) ?
            (samples[i].est_finish - bypass_finish) : 0;
        bypassable_subops++;
        total_hops += hops;
        total_benefit += benefit;
        max_hops = (hops > max_hops) ? hops : max_hops;
        max_benefit = (benefit > max_benefit) ? benefit : max_benefit;

        //当前 subop 能绕过几个 waiting read
        //baseline 下什么时候开始/结束
        //dry-run 下什么时候开始/结束
        //理论收益是多少
        ftl_log("ZNS_READ_BYPASS_DRYRUN,reqid=%lu,subidx=%lu,lpn=%lu,"
                "ch=%u,lun=%u,pl=%u,queue_len=%lu,hops=%lu,"
                "baseline_start_ns=%lu,baseline_finish_ns=%lu,"
                "bypass_start_ns=%lu,bypass_finish_ns=%lu,benefit_ns=%lu\n",
                reqid, i, samples[i].lpn, samples[i].ch, samples[i].lun,
                samples[i].pl, queue->len, hops, samples[i].est_start,
                samples[i].est_finish, bypass_start, bypass_finish, benefit);
    }
    //这条 host read 内部有多少子请求有 bypass 判断条件，
    //其中多少真的能 bypass，最多能前移几步，总理论收益是多少。
    if (candidate_subops) {
        ftl_log("ZNS_READ_BYPASS_DRYRUN_SUMMARY,reqid=%lu,subops=%lu,"
                "candidate_subops=%lu,bypassable_subops=%lu,total_hops=%lu,"
                "max_hops=%lu,total_benefit_ns=%lu,max_benefit_ns=%lu\n",
                reqid, sample_cnt, candidate_subops, bypassable_subops,
                total_hops, max_hops, total_benefit, max_benefit);
    }
}

/*
 * 真正执行 read-read bypass。
 *
 * 当前 plane 队列只保存“已经安排、但还没有开始”的 read。incoming read
 * 默认会接在队尾；如果队尾开始向前的一段 waiting read 都有足够 slack，
 * 就把 incoming read 插到它们前面，并把被跨过的 victim 各自后移一个
 * incoming read 的服务时间。
 *
 * 这版原型仍然保持保守：
 *   1. 只允许 read 绕过 read；
 *   2. 不跨过同一条 host read 内更早的子请求，避免在一个请求内部再重排；
 *   3. 不跨过 write/erase，非 read 到来时队列会被清空；
 *   4. 每个 victim 被后移后都必须仍然不晚于它所属 host read 已经承诺的
 *      completion deadline。只要这个不变量成立，victim 的 host request
 *      expire_time 就无需回头修改。
 *
 * 注意：entry 插入时的 req_deadline 只是当前请求的临时 deadline；等当前
 * host read 的真实 maxlat 算出后，zns_read_bypass_finalize_deadline() 会在
 * zns_read() 返回前把它收紧成真正承诺给主机的 completion time。
 */
static uint64_t zns_read_bypass_execute(uint64_t reqid, uint64_t subidx,
                                        struct zns_ssd *zns,
                                        struct zns_read_slack_sample *sample,
                                        uint64_t read_stime,
                                        uint64_t req_deadline)
{
    struct zns_read_bypass_queue *queue =
        zns_read_bypass_get_queue(&sample->ppa);
    struct zns_plane *pl = get_plane(zns, &sample->ppa);
    struct zns_read_bypass_entry entry;
    uint64_t old_avail = pl->next_plane_avail_time;
    uint64_t baseline_start = (old_avail < read_stime) ? read_stime : old_avail;
    uint64_t baseline_finish = baseline_start + sample->read_delay;
    uint64_t start = baseline_start;
    uint64_t finish = baseline_finish;
    uint64_t hops = 0;
    uint64_t insertion_pos = 0;
    uint64_t pos;

    if (!queue) {
        /*
         * 理论上合法 PPA 都应能映射到队列；如果映射失败，就退化成普通
         * baseline read，保证功能正确。
         */
        sample->exec_start = baseline_start;
        sample->exec_finish = baseline_finish;
        sample->exec_hops = 0;
        sample->exec_benefit = 0;
        sample->exec_candidate = false;
        pl->next_plane_avail_time = baseline_finish;
        return baseline_finish - read_stime;
    }

    zns_read_bypass_prune_started(queue, read_stime);

    /*
     * 只有队尾 read 与当前 read 在时间线上连续，且它来自更早的 host read，
     * incoming read 才算 candidate。这样既避免跨过空档/非 read，也避免把
     * 同一条 host read 内本来就禁止互相绕行的 sibling subop 误计为候选。
     */
    if (queue->len &&
        queue->entries[queue->len - 1].finish == baseline_start &&
        queue->entries[queue->len - 1].reqid != reqid) {
        sample->exec_candidate = true;

        /*
         * 从队尾向队头扫描。只要 victim 仍有足够 slack，就继续前移。
         * 如果遇到同一条 host read 内更早的子请求，也停止，避免 sibling
         * subop 在 execute 原型里互相穿插。
         */
        for (pos = queue->len; pos > 0; pos--) {
            struct zns_read_bypass_entry *victim = &queue->entries[pos - 1];
            uint64_t required_slack = sample->read_delay;

            if (victim->reqid == reqid) {
                break;
            }

            if (victim->slack < required_slack) {
                sample->exec_reject_slack_insufficient = true;
                if (sample->pred_critical &&
                    victim->slack_consumed_by_pred_noncritical) {
                    sample->exec_reject_after_pred_noncritical = true;
                    sample->exec_blocked_by_pred_noncritical_slack =
                        victim->slack_consumed_by_pred_noncritical;
                }
                break;
            }

            hops++;
        }
    }

    if (hops) {
        uint64_t queue_len_before = queue->len;

        insertion_pos = queue->len - hops;
        start = queue->entries[insertion_pos].start;
        finish = start + sample->read_delay;

        zns_read_bypass_queue_reserve_one(queue);

        /*
         * 从队尾向后搬移被跨过的 victim。每个 victim 都被 incoming read
         * 多占用的一个 read slot 推迟一次，因此扣掉同样长度的 residual slack。
         */
        for (pos = queue->len; pos > insertion_pos; pos--) {
            struct zns_read_bypass_entry victim = queue->entries[pos - 1];
            uint64_t old_start = victim.start;
            uint64_t old_finish = victim.finish;
            uint64_t old_slack = victim.slack;

            victim.start += sample->read_delay;
            victim.finish += sample->read_delay;
            victim.slack -= sample->read_delay;
            if (sample->pred_critical) {
                victim.slack_consumed_by_pred_critical += sample->read_delay;
            } else {
                victim.slack_consumed_by_pred_noncritical += sample->read_delay;
            }
            sample->exec_consumed_victim_slack += sample->read_delay;
            queue->entries[pos] = victim;

            ftl_log("ZNS_READ_BYPASS_EXEC_HOP,reqid=%lu,subidx=%lu,lpn=%lu,"
                    "ch=%u,lun=%u,pl=%u,hop=%lu,victim_reqid=%lu,"
                    "victim_subidx=%lu,victim_lpn=%lu,victim_old_start_ns=%lu,"
                    "victim_old_finish_ns=%lu,victim_new_start_ns=%lu,"
                    "victim_new_finish_ns=%lu,victim_deadline_ns=%lu,"
                    "victim_old_slack_ns=%lu,victim_new_slack_ns=%lu,"
                    "slack_used_ns=%lu,incoming_pred_critical=%u,"
                    "victim_consumed_by_pred_critical_ns=%lu,"
                    "victim_consumed_by_pred_noncritical_ns=%lu\n",
                    reqid, subidx, sample->lpn, sample->ch, sample->lun,
                    sample->pl, queue->len - pos + 1, victim.reqid,
                    victim.subidx, victim.lpn, old_start, old_finish,
                    victim.start, victim.finish, victim.deadline, old_slack,
                    victim.slack, sample->read_delay,
                    sample->pred_critical ? 1 : 0,
                    victim.slack_consumed_by_pred_critical,
                    victim.slack_consumed_by_pred_noncritical);
        }

        entry = zns_read_bypass_make_entry(reqid, subidx, sample, read_stime,
                                           req_deadline, start, finish);
        queue->entries[insertion_pos] = entry;
        queue->len++;

        sample->exec_hops = hops;
        sample->exec_benefit =
            (baseline_finish > finish) ? (baseline_finish - finish) : 0;

        ftl_log("ZNS_READ_BYPASS_EXEC,reqid=%lu,subidx=%lu,lpn=%lu,"
                "ch=%u,lun=%u,pl=%u,queue_len_before=%lu,hops=%lu,"
                "baseline_start_ns=%lu,baseline_finish_ns=%lu,"
                "exec_start_ns=%lu,exec_finish_ns=%lu,benefit_ns=%lu,"
                "pred_critical=%u,pred_critical_gap_ns=%lu,"
                "consumed_victim_slack_ns=%lu\n",
                reqid, subidx, sample->lpn, sample->ch, sample->lun,
                sample->pl, queue_len_before, hops, baseline_start,
                baseline_finish, start, finish, sample->exec_benefit,
                sample->pred_critical ? 1 : 0,
                sample->pred_critical_gap,
                sample->exec_consumed_victim_slack);
    } else {
        /*
         * 没有可绕行 victim，就按当前真实时间线接到队尾。
         */
        entry = zns_read_bypass_make_entry(reqid, subidx, sample, read_stime,
                                           req_deadline, start, finish);
        zns_read_bypass_queue_push(queue, &entry);
        sample->exec_hops = 0;
        sample->exec_benefit = 0;
    }

    sample->exec_start = start;
    sample->exec_finish = finish;

    /*
     * 队尾 finish 就是当前 plane 在 execute 调度下真正的下一次空闲时间。
     */
    pl->next_plane_avail_time = queue->entries[queue->len - 1].finish;

    /*
     * 保留原有 [PU] 日志形式，方便继续观察 plane 占用。若之后发生 bypass，
     * 被后移 victim 的修正会额外体现在 ZNS_READ_BYPASS_EXEC_HOP 中。
     */
    femu_log("[PU] cmd=READ req=%lu old_avail=%lu start=%lu finish=%lu delay=%lu "
             "lat=%lu wait=%lu idle=%lu ch=%u lun=%u pl=%u blk=%u pg=%u spg=%u\n",
             read_stime, old_avail, start, finish, sample->read_delay,
             finish - read_stime,
             (start > read_stime) ? (start - read_stime) : 0,
             (read_stime > old_avail) ? (read_stime - old_avail) : 0,
             sample->ppa.g.ch, sample->ppa.g.fc, sample->ppa.g.pl,
             sample->ppa.g.blk, sample->ppa.g.pg, sample->ppa.g.spg);

    return finish - read_stime;
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

    /*
     * 当前 bypass 原型只讨论 read 绕过 read。
     * 某个 plane 一旦出现 write/erase，就把它的 read bypass 队列清空，
     * 保守地避免后续 read 跨过这些非读操作。
     */
    if (ncmd->cmd != NAND_READ) {
        zns_read_bypass_clear_queue(ppa);
    }

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
    uint64_t reqid = 0;
    bool exec_enabled = zns_read_bypass_exec_enabled();
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
         * dry-run 在真实执行前就要打日志，所以 reqid 需要提前分配。
         * 后面的 slack / estimator 日志继续复用同一个 reqid。
         */
        reqid = zns_read_slack_reqid++;
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
            samples[i].read_delay = read_delay;

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
             *
             * pred_critical 是在线可见的“预测关键路径”标签：
             * 如果某个 subop 距离估计 maxlat 不超过一个 NAND read_delay，
             * 就认为它接近关键路径。后续诊断会检查这个预测是否真的命中
             * baseline/execute 的关键 subop。
             */
            samples[i].est_slack = est_maxlat - samples[i].est_sublat;
            samples[i].pred_critical_gap = est_maxlat - samples[i].est_sublat;
            samples[i].pred_critical =
                samples[i].pred_critical_gap <= samples[i].read_delay;
        }

        g_free(shadow_planes);
    }

    if (sample_cnt) {
        /*
         * 只做机会统计，不改真实调度：按论文方式从 plane 队尾向前扫描，
         * 看当前 read 子请求最多能跨过多少个 waiting read。
         */
        zns_read_bypass_dryrun(reqid, samples, sample_cnt, read_stime);
    }

    /*
     * 第三步：根据运行模式选择真实调度路径。
     *
     * baseline 模式继续走原来的 zns_advance_status()；
     * execute 模式则真正使用 read-read bypass 队列调度。
     */
    for (uint64_t i = 0; i < sample_cnt; i++) {
        ppa = samples[i].ppa;
        if (exec_enabled) {
            /*
             * est_maxlat 是“不做当前 read 绕行时”的 baseline host 完成时间。
             * execute 原型可以让 incoming read 更早完成，但前序 victim 最晚
             * 只能被推迟到各自原本的 baseline deadline。
             */
            sublat = zns_read_bypass_execute(reqid, i, zns, &samples[i],
                                             read_stime,
                                             read_stime + est_maxlat);
        } else {
            struct nand_cmd srd;

            srd.type = USER_IO;
            srd.cmd = NAND_READ;
            srd.stime = read_stime;
            sublat = zns_advance_status(zns, &ppa, &srd);
        }
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
        uint64_t exec_candidate_subops = 0;
        uint64_t exec_bypass_subops = 0;
        uint64_t exec_total_hops = 0;
        uint64_t exec_max_hops = 0;
        uint64_t exec_total_benefit = 0;
        uint64_t exec_max_benefit = 0;
        uint64_t exec_baseline_maxlat_subops = 0;
        uint64_t exec_bypass_critical_subops = 0;
        uint64_t exec_host_improvement = 0;
        uint64_t pred_critical_subops = 0;
        uint64_t true_baseline_critical_subops = 0;
        uint64_t pred_true_baseline_critical_subops = 0;
        uint64_t obs_critical_subops = 0;
        uint64_t pred_obs_critical_subops = 0;
        uint64_t pred_critical_candidate_subops = 0;
        uint64_t pred_noncritical_candidate_subops = 0;
        uint64_t pred_critical_executed_subops = 0;
        uint64_t pred_noncritical_executed_subops = 0;
        uint64_t pred_critical_consumed_slack = 0;
        uint64_t pred_noncritical_consumed_slack = 0;
        uint64_t pred_critical_reject_slack_insufficient = 0;
        uint64_t pred_noncritical_reject_slack_insufficient = 0;
        uint64_t pred_critical_reject_after_noncritical = 0;
        uint64_t pred_critical_blocked_by_noncritical_slack = 0;
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
            bool true_baseline_critical = samples[i].est_sublat == est_maxlat;
            bool obs_critical = samples[i].sublat == maxlat;
            bool seen = false; //表示当前这个 plane 之前是否已经出现过

            total_slack += slack;
            max_slack = (slack > max_slack) ? slack : max_slack;
            est_total_slack += est_slack;
            est_max_slack = (est_slack > est_max_slack) ? est_slack : est_max_slack;
            total_sublat_err += sublat_err;
            total_slack_err += slack_err;
            pred_critical_subops += samples[i].pred_critical ? 1 : 0;
            true_baseline_critical_subops += true_baseline_critical ? 1 : 0;
            pred_true_baseline_critical_subops +=
                (samples[i].pred_critical && true_baseline_critical) ? 1 : 0;
            obs_critical_subops += obs_critical ? 1 : 0;
            pred_obs_critical_subops +=
                (samples[i].pred_critical && obs_critical) ? 1 : 0;
            exec_candidate_subops += samples[i].exec_candidate ? 1 : 0;
            if (samples[i].exec_candidate) {
                if (samples[i].pred_critical) {
                    pred_critical_candidate_subops++;
                } else {
                    pred_noncritical_candidate_subops++;
                }
            }
            if (true_baseline_critical) {
                exec_baseline_maxlat_subops++;
            }
            if (samples[i].exec_hops) {
                exec_bypass_subops++;
                if (true_baseline_critical) {
                    exec_bypass_critical_subops++;
                }
                if (samples[i].pred_critical) {
                    pred_critical_executed_subops++;
                    pred_critical_consumed_slack +=
                        samples[i].exec_consumed_victim_slack;
                } else {
                    pred_noncritical_executed_subops++;
                    pred_noncritical_consumed_slack +=
                        samples[i].exec_consumed_victim_slack;
                }
                exec_total_hops += samples[i].exec_hops;
                exec_max_hops = (samples[i].exec_hops > exec_max_hops) ?
                    samples[i].exec_hops : exec_max_hops;
                exec_total_benefit += samples[i].exec_benefit;
                exec_max_benefit =
                    (samples[i].exec_benefit > exec_max_benefit) ?
                    samples[i].exec_benefit : exec_max_benefit;
            }
            if (samples[i].exec_reject_slack_insufficient) {
                if (samples[i].pred_critical) {
                    pred_critical_reject_slack_insufficient++;
                } else {
                    pred_noncritical_reject_slack_insufficient++;
                }
            }
            if (samples[i].exec_reject_after_pred_noncritical) {
                pred_critical_reject_after_noncritical++;
                pred_critical_blocked_by_noncritical_slack +=
                    samples[i].exec_blocked_by_pred_noncritical_slack;
            }

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

        exec_host_improvement = (est_maxlat > maxlat) ?
            (est_maxlat - maxlat) : 0;

        if (exec_enabled && exec_candidate_subops) {
            ftl_log("ZNS_READ_BYPASS_EXEC_SUMMARY,reqid=%lu,subops=%lu,"
                    "candidate_subops=%lu,bypass_subops=%lu,total_hops=%lu,"
                    "max_hops=%lu,total_benefit_ns=%lu,max_benefit_ns=%lu,"
                    "baseline_maxlat_subops=%lu,bypass_critical_subops=%lu,"
                    "baseline_maxlat_ns=%lu,exec_maxlat_ns=%lu,"
                    "host_improved=%u,host_improvement_ns=%lu,"
                    "pred_critical_subops=%lu,true_baseline_critical_subops=%lu,"
                    "pred_true_baseline_critical_subops=%lu,"
                    "obs_critical_subops=%lu,pred_obs_critical_subops=%lu,"
                    "pred_critical_candidate_subops=%lu,"
                    "pred_noncritical_candidate_subops=%lu,"
                    "pred_critical_executed_subops=%lu,"
                    "pred_noncritical_executed_subops=%lu,"
                    "pred_critical_consumed_slack_ns=%lu,"
                    "pred_noncritical_consumed_slack_ns=%lu,"
                    "pred_critical_reject_slack_insufficient=%lu,"
                    "pred_noncritical_reject_slack_insufficient=%lu,"
                    "pred_critical_reject_after_noncritical=%lu,"
                    "pred_critical_blocked_by_noncritical_slack_ns=%lu\n",
                    reqid, sample_cnt, exec_candidate_subops,
                    exec_bypass_subops, exec_total_hops, exec_max_hops,
                    exec_total_benefit, exec_max_benefit,
                    exec_baseline_maxlat_subops, exec_bypass_critical_subops,
                    est_maxlat, maxlat, exec_host_improvement ? 1 : 0,
                    exec_host_improvement, pred_critical_subops,
                    true_baseline_critical_subops,
                    pred_true_baseline_critical_subops,
                    obs_critical_subops, pred_obs_critical_subops,
                    pred_critical_candidate_subops,
                    pred_noncritical_candidate_subops,
                    pred_critical_executed_subops,
                    pred_noncritical_executed_subops,
                    pred_critical_consumed_slack,
                    pred_noncritical_consumed_slack,
                    pred_critical_reject_slack_insufficient,
                    pred_noncritical_reject_slack_insufficient,
                    pred_critical_reject_after_noncritical,
                    pred_critical_blocked_by_noncritical_slack);
        }

        if (!exec_enabled) {
            /*
             * baseline 模式下，真实 read 已经走完原始调度，再把当前 read
             * 记入队列，继续支持后续请求的 dry-run opportunity analysis。
             */
            for (i = 0; i < sample_cnt; i++) {
                zns_read_bypass_remember_baseline_read(reqid, i, &samples[i],
                                                       read_stime, maxlat);
            }
        } else {
            /*
             * execute 模式下，当前请求刚排入队列时只拿到了临时 deadline。
             * 现在 maxlat 已经确定，必须在 zns_read() 返回前把当前请求所有
             * entry 的 deadline 收紧成真正会承诺给 host 的完成时间。
             */
            zns_read_bypass_finalize_deadline(reqid, samples, sample_cnt,
                                              read_stime, maxlat);
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
         * 这行把一整条 host read 的 baseline 估计结果和当前 execute 调度
         * 结果放在一起。进入 bypass execute 后，*_err 字段不再只表示
         * estimator 误差，也会包含当前 read 因绕行而获得的调度收益。
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
/*处理一条主机写请求：把主机写入的 LBA 范围转换成 FTL 内部的 LPN，然后先写入 SRAM write cache；
**如果 write cache 满了，或者需要复用别的 cache 槽位，就触发 zns_wc_flush把缓存里的数据真正写入NAND。
**最后返回这条写请求看到的延迟 maxlat。*/
static uint64_t zns_write(struct zns_ssd *zns, NvmeRequest *req)
{
    /* 主机写请求的起始 LBA 和长度。 */
    uint64_t lba = req->slba; //起始逻辑块地址
    uint32_t nlb = req->nlb; //写请求覆盖的逻辑块数量

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
    /*如果找到了，比如：wcidx = 2。说明当前 zone 已经在使用 write_cache[2]，后面直接往这个槽位追加 LPN。
      如果找不到：wcidx == -1
    */
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
            //打印这次 flush 产生的延迟
            femu_log("[W] flush lat: %u\n", (int)sublat);
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
