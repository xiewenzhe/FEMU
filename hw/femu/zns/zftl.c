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
    uint64_t blocking_write_flush_id;
    uint64_t blocking_write_subidx;
    uint64_t blocking_write_start;
    uint64_t blocking_write_finish;
    uint64_t blocking_write_slack;
    uint64_t blocking_write_wait;
    bool exec_candidate;   /* 当前子请求是否满足进入 bypass 判断的前置条件。 */
    bool pred_critical;    /* 在线估计认为它是否接近当前 host read 的关键路径。 */
    bool blocked_by_write;
    bool blocking_write_not_started;
    bool blocking_write_opportunity;
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

enum zns_read_bypass_policy {
    ZNS_READ_BYPASS_POLICY_NAIVE = 0,
    ZNS_READ_BYPASS_POLICY_DYNAMIC,
};

enum zns_read_bypass_dynamic_block_mode {
    ZNS_READ_BYPASS_DYNAMIC_BLOCK_PREFIX = 0,
    ZNS_READ_BYPASS_DYNAMIC_BLOCK_VARIANTS,
    ZNS_READ_BYPASS_DYNAMIC_BLOCK_SINGLE,
};

/*
 * dynamic block probe：围绕一个关键/近关键 target subop，尝试把同一条
 * host read 中排在它前面的同 plane sibling 一起组成 block 前移。
 *
 * 这样可以处理 A B C 场景：C 是关键，B slack 小导致 C 单独绕不过，
 * 但 [B C] 可以整体绕过 slack 充足的 A。
 */
struct zns_read_bypass_block_probe {
    bool candidate;
    bool valid;
    bool reject_slack_insufficient;
    bool reject_no_queue;
    bool reject_empty_queue;
    bool reject_tail_not_ready;
    bool reject_same_req_tail;
    bool reject_same_req_blocker;
    uint64_t queue_len;
    uint64_t victim_count;
    uint64_t insertion_pos;
    uint64_t block_start_idx;
    uint64_t target_idx;
    uint64_t block_count;
    uint64_t block_delay;
    uint64_t baseline_start;
    uint64_t baseline_finish;
    uint64_t start;
    uint64_t finish;
    uint64_t critset_size;
    uint64_t critset_covered;
    uint64_t critset_gain;
    uint64_t critset_remaining_max;
    uint64_t victim_cost;
};

/*
 * dynamic opportunity funnel：记录每条 host read 在 dynamic probe 流程中
 * 掉在哪一步。它只做诊断，不改变调度选择。
 */
struct zns_read_bypass_dynamic_funnel {
    uint64_t target_subops;
    uint64_t reject_noncritical_gap;
    uint64_t near_critical_targets;
    uint64_t reject_no_queue;
    uint64_t reject_empty_queue;
    uint64_t reject_tail_not_ready;
    uint64_t reject_same_req_tail;
    uint64_t queue_candidate_targets;
    uint64_t reject_same_req_blocker;
    uint64_t reject_slack_insufficient;
    uint64_t valid_block_targets;
    uint64_t reject_no_host_gain;
    uint64_t positive_gain_targets;
};

/*
 * 只用于诊断 read-over-write 机会，不改变真实 NAND 调度。
 * timing_sample 记录一次 NAND 子操作在 plane 时间线上的实际位置。
 */
struct zns_nand_timing_sample {
    uint64_t req_stime;
    uint64_t old_avail;
    uint64_t start;
    uint64_t finish;
    uint64_t delay;
    uint64_t lat;
    uint64_t wait;
    uint64_t idle;
};

#define ZNS_ROW_MAX_WRITE_LPNS (4 * (ZNS_PAGE_SIZE / LOGICAL_PAGE_SIZE))

struct zns_wc_write_diag {
    uint64_t flush_id;
    uint64_t subidx;
    struct ppa ppa;
    struct zns_nand_timing_sample timing;
    uint64_t slack;
    uint64_t lpns[ZNS_ROW_MAX_WRITE_LPNS];
    uint32_t nr_lpns;
};

struct zns_plane_last_op_diag {
    int cmd;
    uint64_t start;
    uint64_t finish;
    uint64_t delay;
    uint64_t flush_id;
    uint64_t write_subidx;
    uint64_t write_slack;
};

enum zns_mixed_diag_cmd {
    ZNS_MIXED_DIAG_READ = 0,
    ZNS_MIXED_DIAG_WRITE = 1,
};

struct zns_mixed_diag_entry {
    enum zns_mixed_diag_cmd cmd;
    uint64_t start;
    uint64_t finish;
    uint64_t delay;
    uint64_t deadline;
    uint64_t slack;
    uint64_t reqid;
    uint64_t subidx;
    uint64_t lpn;
    uint64_t flush_id;
    uint64_t write_subidx;
    uint64_t lpns[ZNS_ROW_MAX_WRITE_LPNS];
    uint32_t nr_lpns;
    uint16_t ch;
    uint16_t lun;
    uint16_t pl;
};

struct zns_mixed_diag_queue {
    struct zns_mixed_diag_entry *entries;
    uint64_t len;
    uint64_t cap;
};

struct zns_mixed_bypass_probe {
    bool valid;
    bool reject_no_queue;
    bool reject_empty_queue;
    bool reject_tail_mismatch;
    bool tail_mismatch_tail_read;
    bool tail_mismatch_tail_write;
    bool tail_mismatch_tail_before;
    bool tail_mismatch_tail_after;
    bool tail_idle_gap_allowed;
    bool reject_same_req_read;
    bool reject_started_write;
    bool reject_read_slack;
    bool reject_write_slack;
    bool reject_raw;
    uint64_t tail_mismatch_gap;
    uint64_t tail_idle_gap;
    uint64_t block_start_idx;
    uint64_t target_idx;
    uint64_t block_count;
    uint64_t block_delay;
    uint64_t insertion_pos;
    uint64_t victim_count;
    uint64_t victim_reads;
    uint64_t victim_writes;
    uint64_t bypass_start;
    uint64_t new_maxlat;
    uint64_t host_gain;
    uint64_t critset_covered;
    uint64_t critset_gain;
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
static bool zns_read_bypass_summary_log_flag_loaded;
static bool zns_read_bypass_summary_log_flag = true;
static bool zns_read_bypass_policy_loaded;
static enum zns_read_bypass_policy zns_read_bypass_policy;
static bool zns_read_bypass_dynamic_max_rounds_loaded;
static uint64_t zns_read_bypass_dynamic_max_rounds = 1;
static bool zns_read_bypass_max_hops_loaded;
static uint64_t zns_read_bypass_max_hops;
static bool zns_read_bypass_dynamic_block_mode_loaded;
static enum zns_read_bypass_dynamic_block_mode
    zns_read_bypass_dynamic_block_mode =
        ZNS_READ_BYPASS_DYNAMIC_BLOCK_PREFIX;
static bool zns_read_bypass_dynamic_critset_policy_loaded;
static bool zns_read_bypass_dynamic_critset_policy;
static bool zns_read_bypass_dynamic_critset_neartie_loaded;
static uint64_t zns_read_bypass_dynamic_critset_neartie_reads;
static bool zns_read_bypass_dynamic_target_critical_only_loaded;
static bool zns_read_bypass_dynamic_target_critical_only;
static bool zns_row_diag_flag_loaded;
static bool zns_row_diag_flag;
static bool zns_row_exec_flag_loaded;
static bool zns_row_exec_flag;
static bool zns_mixed_bypass_exec_flag_loaded;
static bool zns_mixed_bypass_exec_flag;
static bool zns_mixed_bypass_allow_idle_gap_flag_loaded;
static bool zns_mixed_bypass_allow_idle_gap_flag;
static bool zns_detail_log_flag_loaded;
static bool zns_detail_log_flag;
static bool zns_mixed_bypass_summary_every_loaded;
static uint64_t zns_mixed_bypass_summary_every = 1;
static uint64_t zns_wc_flush_next_id;
static bool zns_wc_diag_active;
static uint64_t zns_wc_diag_flush_id;
static uint64_t zns_wc_diag_write_subidx;
static bool zns_read_global_baseline_avail_valid[ZNS_BYPASS_MAX_PLANES];
static uint64_t zns_read_global_baseline_avail[ZNS_BYPASS_MAX_PLANES];
static bool zns_read_paired_model_flag_loaded;
static bool zns_read_paired_model_flag;
static bool zns_read_paired_baseline_avail_valid[ZNS_BYPASS_MAX_PLANES];
static uint64_t zns_read_paired_baseline_avail[ZNS_BYPASS_MAX_PLANES];
static bool zns_read_paired_bypass_avail_valid[ZNS_BYPASS_MAX_PLANES];
static uint64_t zns_read_paired_bypass_avail[ZNS_BYPASS_MAX_PLANES];
static struct zns_read_bypass_queue
    zns_read_paired_bypass_queues[ZNS_BYPASS_MAX_PLANES];
static struct zns_plane_last_op_diag
    zns_plane_last_op_diag[ZNS_BYPASS_MAX_PLANES];
static struct zns_mixed_diag_queue
    zns_mixed_diag_queues[ZNS_BYPASS_MAX_PLANES];

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

/*
 * Naive execute summary is useful for debugging, but it makes the bypass run
 * write substantially more log data than baseline. Keep it switchable so
 * latency experiments can use the lighter per-request timeline log instead.
 */
static bool zns_read_bypass_summary_log_enabled(void)
{
    const char *value;

    if (!zns_read_bypass_summary_log_flag_loaded) {
        value = g_getenv("FEMU_ZNS_READ_BYPASS_SUMMARY_LOG");
        if (value &&
            (!g_ascii_strcasecmp(value, "0") ||
             !g_ascii_strcasecmp(value, "false") ||
             !g_ascii_strcasecmp(value, "off"))) {
            zns_read_bypass_summary_log_flag = false;
        }
        zns_read_bypass_summary_log_flag_loaded = true;
    }

    return zns_read_bypass_summary_log_flag;
}

/*
 * 调度策略开关：
 *   unset/naive：保持最初的 subop 局部贪心 execute；
 *   dynamic：按整条 host read 的当前关键路径动态选择要绕过的 subop。
 */
static enum zns_read_bypass_policy zns_read_bypass_get_policy(void)
{
    const char *value;

    if (!zns_read_bypass_policy_loaded) {
        value = g_getenv("FEMU_ZNS_READ_BYPASS_POLICY");
        if (value && !g_ascii_strcasecmp(value, "dynamic")) {
            zns_read_bypass_policy = ZNS_READ_BYPASS_POLICY_DYNAMIC;
        } else {
            zns_read_bypass_policy = ZNS_READ_BYPASS_POLICY_NAIVE;
        }
        zns_read_bypass_policy_loaded = true;
    }

    return zns_read_bypass_policy;
}

/*
 * dynamic 每条 host read 最多提交多少轮 block move。
 * 默认 1，保持 t2 的 critical-block dynamic 语义；实验时可设为 2/3 做
 * bounded multi-round 对照。
 */
static uint64_t zns_read_bypass_get_dynamic_max_rounds(void)
{
    const char *value;
    char *end = NULL;
    uint64_t parsed;

    if (!zns_read_bypass_dynamic_max_rounds_loaded) {
        value = g_getenv("FEMU_ZNS_READ_BYPASS_DYNAMIC_MAX_ROUNDS");
        if (value && value[0]) {
            parsed = g_ascii_strtoull(value, &end, 10);
            if (end != value && parsed > 0) {
                zns_read_bypass_dynamic_max_rounds = parsed;
            }
        }
        zns_read_bypass_dynamic_max_rounds_loaded = true;
    }

    return zns_read_bypass_dynamic_max_rounds;
}

/*
 * naive read-read bypass 每个 incoming subop 最多允许跨过多少个 victim。
 * unset/0 表示保持原始 naive 行为：只要 victim slack 足够就一直向前绕。
 */
static uint64_t zns_read_bypass_get_max_hops(void)
{
    const char *value;
    char *end = NULL;
    uint64_t parsed;

    if (!zns_read_bypass_max_hops_loaded) {
        value = g_getenv("FEMU_ZNS_READ_BYPASS_MAX_HOPS");
        if (value && value[0]) {
            parsed = g_ascii_strtoull(value, &end, 10);
            if (end != value) {
                zns_read_bypass_max_hops = parsed;
            }
        }
        zns_read_bypass_max_hops_loaded = true;
    }

    return zns_read_bypass_max_hops;
}

/*
 * paired model 只做影子模拟，不改变真实调度。
 *
 * 它解决 fio 两轮实验 arrival time 不一致的问题：同一条到达 FEMU 的 read
 * stream，会同时推进一条 no-bypass baseline 时间线和一条 naive-bypass 时间线。
 */
static bool zns_read_paired_model_enabled(void)
{
    const char *value;

    if (!zns_read_paired_model_flag_loaded) {
        value = g_getenv("FEMU_ZNS_READ_PAIRED_MODEL");
        zns_read_paired_model_flag =
            value &&
            (!g_ascii_strcasecmp(value, "1") ||
             !g_ascii_strcasecmp(value, "true") ||
             !g_ascii_strcasecmp(value, "on"));
        zns_read_paired_model_flag_loaded = true;
    }

    return zns_read_paired_model_flag;
}

static enum zns_read_bypass_dynamic_block_mode
zns_read_bypass_get_dynamic_block_mode(void)
{
    const char *value;

    if (!zns_read_bypass_dynamic_block_mode_loaded) {
        value = g_getenv("FEMU_ZNS_READ_BYPASS_DYNAMIC_BLOCK_MODE");
        if (value && !g_ascii_strcasecmp(value, "variants")) {
            zns_read_bypass_dynamic_block_mode =
                ZNS_READ_BYPASS_DYNAMIC_BLOCK_VARIANTS;
        } else if (value && (!g_ascii_strcasecmp(value, "single") ||
                             !g_ascii_strcasecmp(value, "target"))) {
            zns_read_bypass_dynamic_block_mode =
                ZNS_READ_BYPASS_DYNAMIC_BLOCK_SINGLE;
        } else {
            zns_read_bypass_dynamic_block_mode =
                ZNS_READ_BYPASS_DYNAMIC_BLOCK_PREFIX;
        }
        zns_read_bypass_dynamic_block_mode_loaded = true;
    }

    return zns_read_bypass_dynamic_block_mode;
}

static bool zns_read_bypass_dynamic_critset_policy_enabled(void)
{
    const char *value;

    if (!zns_read_bypass_dynamic_critset_policy_loaded) {
        value = g_getenv("FEMU_ZNS_READ_BYPASS_DYNAMIC_CRITSET_POLICY");
        zns_read_bypass_dynamic_critset_policy =
            value &&
            (!g_ascii_strcasecmp(value, "1") ||
             !g_ascii_strcasecmp(value, "true") ||
             !g_ascii_strcasecmp(value, "on"));
        zns_read_bypass_dynamic_critset_policy_loaded = true;
    }

    return zns_read_bypass_dynamic_critset_policy;
}

static uint64_t zns_read_bypass_get_dynamic_critset_neartie_reads(void)
{
    const char *value;
    char *end = NULL;
    uint64_t parsed;

    if (!zns_read_bypass_dynamic_critset_neartie_loaded) {
        value = g_getenv("FEMU_ZNS_READ_BYPASS_DYNAMIC_CRITSET_NEARTIE_READS");
        if (value && value[0]) {
            parsed = g_ascii_strtoull(value, &end, 10);
            if (end != value) {
                zns_read_bypass_dynamic_critset_neartie_reads = parsed;
            }
        }
        zns_read_bypass_dynamic_critset_neartie_loaded = true;
    }

    return zns_read_bypass_dynamic_critset_neartie_reads;
}

static bool zns_read_bypass_dynamic_target_critical_only_enabled(void)
{
    const char *value;

    if (!zns_read_bypass_dynamic_target_critical_only_loaded) {
        value = g_getenv("FEMU_ZNS_READ_BYPASS_DYNAMIC_TARGET_CRITICAL_ONLY");
        zns_read_bypass_dynamic_target_critical_only =
            value &&
            (!g_ascii_strcasecmp(value, "1") ||
             !g_ascii_strcasecmp(value, "true") ||
             !g_ascii_strcasecmp(value, "on"));
        zns_read_bypass_dynamic_target_critical_only_loaded = true;
    }

    return zns_read_bypass_dynamic_target_critical_only;
}

static bool zns_row_diag_enabled(void)
{
    const char *value;

    if (!zns_row_diag_flag_loaded) {
        value = g_getenv("FEMU_ZNS_ROW_DIAG");
        zns_row_diag_flag =
            value &&
            (!g_ascii_strcasecmp(value, "1") ||
             !g_ascii_strcasecmp(value, "true") ||
             !g_ascii_strcasecmp(value, "on"));
        zns_row_diag_flag_loaded = true;
    }

    return zns_row_diag_flag;
}

static bool zns_row_exec_enabled(void)
{
    const char *value;

    if (!zns_row_exec_flag_loaded) {
        value = g_getenv("FEMU_ZNS_ROW_EXEC");
        zns_row_exec_flag =
            value &&
            (!g_ascii_strcasecmp(value, "1") ||
             !g_ascii_strcasecmp(value, "true") ||
             !g_ascii_strcasecmp(value, "on"));
        zns_row_exec_flag_loaded = true;
    }

    return zns_row_exec_flag;
}

static bool zns_mixed_bypass_exec_enabled(void)
{
    const char *value;

    if (!zns_mixed_bypass_exec_flag_loaded) {
        value = g_getenv("FEMU_ZNS_MIXED_BYPASS_EXEC");
        zns_mixed_bypass_exec_flag =
            value &&
            (!g_ascii_strcasecmp(value, "1") ||
             !g_ascii_strcasecmp(value, "true") ||
             !g_ascii_strcasecmp(value, "on"));
        zns_mixed_bypass_exec_flag_loaded = true;
    }

    return zns_mixed_bypass_exec_flag;
}

static bool zns_mixed_bypass_allow_idle_gap_enabled(void)
{
    const char *value;

    if (!zns_mixed_bypass_allow_idle_gap_flag_loaded) {
        value = g_getenv("FEMU_ZNS_MIXED_BYPASS_ALLOW_IDLE_GAP");
        zns_mixed_bypass_allow_idle_gap_flag =
            value &&
            (!g_ascii_strcasecmp(value, "1") ||
             !g_ascii_strcasecmp(value, "true") ||
             !g_ascii_strcasecmp(value, "on"));
        zns_mixed_bypass_allow_idle_gap_flag_loaded = true;
    }

    return zns_mixed_bypass_allow_idle_gap_flag;
}

static bool zns_detail_log_enabled(void)
{
    const char *value;

    if (!zns_detail_log_flag_loaded) {
        value = g_getenv("FEMU_ZNS_DETAIL_LOG");
        zns_detail_log_flag =
            value &&
            (!g_ascii_strcasecmp(value, "1") ||
             !g_ascii_strcasecmp(value, "true") ||
             !g_ascii_strcasecmp(value, "on"));
        zns_detail_log_flag_loaded = true;
    }

    return zns_detail_log_flag;
}

static uint64_t zns_mixed_bypass_get_summary_every(void)
{
    const char *value;
    char *end = NULL;
    uint64_t parsed;

    if (!zns_mixed_bypass_summary_every_loaded) {
        value = g_getenv("FEMU_ZNS_MIXED_BYPASS_SUMMARY_EVERY");
        if (value && value[0]) {
            parsed = g_ascii_strtoull(value, &end, 10);
            if (end != value) {
                zns_mixed_bypass_summary_every = parsed;
            }
        }
        zns_mixed_bypass_summary_every_loaded = true;
    }

    return zns_mixed_bypass_summary_every;
}

static bool zns_mixed_bypass_should_log_summary(uint64_t reqid,
                                                uint64_t committed_blocks,
                                                uint64_t valid_blocks)
{
    uint64_t every = zns_mixed_bypass_get_summary_every();

    if (committed_blocks || valid_blocks) {
        return true;
    }
    if (!every) {
        return false;
    }

    return (reqid % every) == 0;
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

static struct zns_plane_last_op_diag *zns_get_plane_last_op_diag(struct ppa *ppa)
{
    int idx = zns_read_bypass_plane_idx(ppa);

    if (idx < 0) {
        return NULL;
    }

    return &zns_plane_last_op_diag[idx];
}

static void zns_set_plane_last_read_diag(struct ppa *ppa, uint64_t start,
                                         uint64_t finish, uint64_t delay)
{
    struct zns_plane_last_op_diag *last_op =
        zns_get_plane_last_op_diag(ppa);

    if (!last_op) {
        return;
    }

    last_op->cmd = NAND_READ;
    last_op->start = start;
    last_op->finish = finish;
    last_op->delay = delay;
    last_op->flush_id = 0;
    last_op->write_subidx = 0;
    last_op->write_slack = 0;
}

static bool zns_read_bypass_tail_has_no_intervening_op(
    struct ppa *ppa, struct zns_read_bypass_entry *tail,
    uint64_t baseline_start)
{
    struct zns_plane_last_op_diag *last_op =
        zns_get_plane_last_op_diag(ppa);

    if (tail->finish > baseline_start) {
        return false;
    }

    /*
     * tail.finish < baseline_start is allowed only when tail is still the
     * last operation seen on this plane. Then the gap is idle time, not an
     * intervening read/write/erase that the read-bypass queue failed to model.
     */
    if (!last_op || last_op->cmd != NAND_READ) {
        return false;
    }

    return last_op->start == tail->start && last_op->finish == tail->finish;
}

static struct zns_mixed_diag_queue *zns_mixed_diag_get_queue(struct ppa *ppa)
{
    int idx = zns_read_bypass_plane_idx(ppa);

    if (idx < 0) {
        return NULL;
    }

    return &zns_mixed_diag_queues[idx];
}

static void zns_mixed_diag_queue_reserve_one(
    struct zns_mixed_diag_queue *queue)
{
    if (queue->len == queue->cap) {
        uint64_t new_cap = queue->cap ? queue->cap * 2 : 16;

        queue->entries = g_realloc(queue->entries,
                                   sizeof(*queue->entries) * new_cap);
        queue->cap = new_cap;
    }
}

static void zns_mixed_diag_queue_reserve(struct zns_mixed_diag_queue *queue,
                                         uint64_t extra)
{
    uint64_t need = queue->len + extra;
    uint64_t new_cap;

    if (need <= queue->cap) {
        return;
    }

    new_cap = queue->cap ? queue->cap : 16;
    while (new_cap < need) {
        new_cap *= 2;
    }

    queue->entries = g_realloc(queue->entries,
                               sizeof(*queue->entries) * new_cap);
    queue->cap = new_cap;
}

static void zns_mixed_diag_queue_push(struct zns_mixed_diag_queue *queue,
                                      struct zns_mixed_diag_entry *entry)
{
    zns_mixed_diag_queue_reserve_one(queue);
    queue->entries[queue->len++] = *entry;
}

static void zns_mixed_diag_prune_started(struct zns_mixed_diag_queue *queue,
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

    for (i = drop; i < queue->len; i++) {
        queue->entries[i - drop] = queue->entries[i];
    }
    queue->len -= drop;
}

static void zns_mixed_diag_remember_write(
    struct ppa *ppa, uint64_t flush_id, uint64_t write_subidx,
    struct zns_nand_timing_sample *timing, uint64_t flush_max_finish,
    uint64_t write_slack, uint64_t *lpns, uint32_t nr_lpns)
{
    struct zns_mixed_diag_queue *queue = zns_mixed_diag_get_queue(ppa);
    struct zns_mixed_diag_entry entry;
    uint32_t i;

    if (!queue) {
        return;
    }

    entry = (struct zns_mixed_diag_entry) { 0 };
    entry.cmd = ZNS_MIXED_DIAG_WRITE;
    entry.start = timing->start;
    entry.finish = timing->finish;
    entry.delay = timing->delay;
    entry.deadline = flush_max_finish;
    entry.slack = write_slack;
    entry.flush_id = flush_id;
    entry.write_subidx = write_subidx;
    entry.nr_lpns = nr_lpns;
    for (i = 0; i < nr_lpns && i < G_N_ELEMENTS(entry.lpns); i++) {
        entry.lpns[i] = lpns[i];
    }
    entry.ch = ppa->g.ch;
    entry.lun = ppa->g.fc;
    entry.pl = ppa->g.pl;

    zns_mixed_diag_queue_push(queue, &entry);
}

static void zns_mixed_diag_remember_read(
    uint64_t reqid, uint64_t subidx, struct zns_read_slack_sample *sample,
    uint64_t read_stime, uint64_t req_maxlat)
{
    struct zns_mixed_diag_queue *queue =
        zns_mixed_diag_get_queue(&sample->ppa);
    struct zns_mixed_diag_entry entry;
    uint64_t finish;

    if (!queue) {
        return;
    }

    finish = read_stime + sample->sublat;

    entry = (struct zns_mixed_diag_entry) { 0 };
    entry.cmd = ZNS_MIXED_DIAG_READ;
    entry.start = finish - sample->read_delay;
    entry.finish = finish;
    entry.delay = sample->read_delay;
    entry.deadline = read_stime + req_maxlat;
    entry.slack = (entry.deadline > entry.finish) ?
        (entry.deadline - entry.finish) : 0;
    entry.reqid = reqid;
    entry.subidx = subidx;
    entry.lpn = sample->lpn;
    entry.ch = sample->ch;
    entry.lun = sample->lun;
    entry.pl = sample->pl;

    zns_mixed_diag_queue_push(queue, &entry);
}

static void zns_mixed_diag_finalize_read_deadline(
    uint64_t reqid, struct zns_read_slack_sample *samples,
    uint64_t sample_cnt, uint64_t read_stime, uint64_t req_maxlat)
{
    uint64_t deadline = read_stime + req_maxlat;
    uint64_t i;

    for (i = 0; i < sample_cnt; i++) {
        struct zns_mixed_diag_queue *queue =
            zns_mixed_diag_get_queue(&samples[i].ppa);
        uint64_t pos;

        if (!queue) {
            continue;
        }

        for (pos = 0; pos < queue->len; pos++) {
            struct zns_mixed_diag_entry *entry = &queue->entries[pos];

            if (entry->cmd == ZNS_MIXED_DIAG_READ &&
                entry->reqid == reqid && entry->subidx == i) {
                entry->deadline = deadline;
                entry->slack = (deadline > entry->finish) ?
                    (deadline - entry->finish) : 0;
                break;
            }
        }
    }
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

static void zns_read_bypass_queue_reserve(struct zns_read_bypass_queue *queue,
                                          uint64_t extra)
{
    uint64_t need = queue->len + extra;
    uint64_t new_cap;

    if (need <= queue->cap) {
        return;
    }

    new_cap = queue->cap ? queue->cap : 8;
    while (new_cap < need) {
        new_cap *= 2;
    }

    queue->entries = g_realloc(queue->entries,
                               sizeof(*queue->entries) * new_cap);
    queue->cap = new_cap;
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
 * 独立的全局 no-bypass shadow timeline。
 *
 * zns_read() 里的 est_maxlat 是“当前真实 timeline 上，如果当前请求自己不绕”
 * 的局部 baseline；execute 模式下，当前真实 timeline 已经可能被前面请求的
 * bypass 改写。
 *
 * 这个函数维护另一条完全不参与真实调度的 plane availability，从 replay 开始
 * 按原始 read 顺序推进，用来回答更强的问题：
 *   在同一条 arrival trace 下，actual bypass 是否比“全程不绕过”更快/更慢？
 */
static uint64_t zns_read_global_baseline_model(struct zns_ssd *zns,
                                               struct zns_read_slack_sample *samples,
                                               uint64_t sample_cnt,
                                               uint64_t read_stime)
{
    uint64_t maxlat = 0;
    uint64_t i;

    for (i = 0; i < sample_cnt; i++) {
        int idx = zns_read_bypass_plane_idx(&samples[i].ppa);
        uint64_t start;
        uint64_t finish;

        if (idx < 0) {
            continue;
        }

        if (!zns_read_global_baseline_avail_valid[idx]) {
            zns_read_global_baseline_avail[idx] =
                get_plane(zns, &samples[i].ppa)->next_plane_avail_time;
            zns_read_global_baseline_avail_valid[idx] = true;
        }

        start = (zns_read_global_baseline_avail[idx] < read_stime) ?
            read_stime : zns_read_global_baseline_avail[idx];
        finish = start + samples[i].read_delay;
        zns_read_global_baseline_avail[idx] = finish;

        maxlat = ((finish - read_stime) > maxlat) ?
            (finish - read_stime) : maxlat;
    }

    return maxlat;
}

static uint64_t zns_read_paired_baseline_model(
    struct zns_ssd *zns, struct zns_read_slack_sample *samples,
    uint64_t sample_cnt, uint64_t read_stime)
{
    uint64_t maxlat = 0;
    uint64_t i;

    for (i = 0; i < sample_cnt; i++) {
        int idx = zns_read_bypass_plane_idx(&samples[i].ppa);
        uint64_t start;
        uint64_t finish;

        if (idx < 0) {
            continue;
        }

        if (!zns_read_paired_baseline_avail_valid[idx]) {
            zns_read_paired_baseline_avail[idx] =
                get_plane(zns, &samples[i].ppa)->next_plane_avail_time;
            zns_read_paired_baseline_avail_valid[idx] = true;
        }

        start = (zns_read_paired_baseline_avail[idx] < read_stime) ?
            read_stime : zns_read_paired_baseline_avail[idx];
        finish = start + samples[i].read_delay;
        zns_read_paired_baseline_avail[idx] = finish;

        maxlat = ((finish - read_stime) > maxlat) ?
            (finish - read_stime) : maxlat;
    }

    return maxlat;
}

static void zns_read_paired_bypass_finalize_deadline(
    uint64_t reqid, struct zns_read_slack_sample *samples,
    uint64_t sample_cnt, uint64_t read_stime, uint64_t req_maxlat)
{
    uint64_t deadline = read_stime + req_maxlat;
    uint64_t i;

    for (i = 0; i < sample_cnt; i++) {
        int idx = zns_read_bypass_plane_idx(&samples[i].ppa);
        struct zns_read_bypass_queue *queue;
        uint64_t pos;

        if (idx < 0) {
            continue;
        }

        queue = &zns_read_paired_bypass_queues[idx];
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

static uint64_t zns_read_paired_bypass_append(
    uint64_t reqid, uint64_t subidx, struct zns_ssd *zns,
    struct zns_read_slack_sample *sample, uint64_t read_stime,
    uint64_t req_deadline, uint64_t *hops_out)
{
    int idx = zns_read_bypass_plane_idx(&sample->ppa);
    struct zns_read_bypass_queue *queue;
    struct zns_read_bypass_entry entry;
    uint64_t baseline_start;
    uint64_t baseline_finish;
    uint64_t start;
    uint64_t finish;
    uint64_t hops = 0;
    uint64_t max_hops = zns_read_bypass_get_max_hops();
    uint64_t insertion_pos = 0;
    uint64_t pos;

    *hops_out = 0;

    if (idx < 0) {
        return sample->read_delay;
    }

    if (!zns_read_paired_bypass_avail_valid[idx]) {
        zns_read_paired_bypass_avail[idx] =
            get_plane(zns, &sample->ppa)->next_plane_avail_time;
        zns_read_paired_bypass_avail_valid[idx] = true;
    }

    queue = &zns_read_paired_bypass_queues[idx];
    baseline_start = (zns_read_paired_bypass_avail[idx] < read_stime) ?
        read_stime : zns_read_paired_bypass_avail[idx];
    baseline_finish = baseline_start + sample->read_delay;
    start = baseline_start;
    finish = baseline_finish;

    zns_read_bypass_prune_started(queue, read_stime);

    /*
     * paired bypass 队列只模拟 read-only 时间线，因此判断条件可以直接用
     * tail.finish <= baseline_start；没有真实 write/erase 插入的问题。
     */
    if (queue->len &&
        queue->entries[queue->len - 1].finish <= baseline_start &&
        queue->entries[queue->len - 1].reqid != reqid) {
        for (pos = queue->len; pos > 0; pos--) {
            struct zns_read_bypass_entry *victim = &queue->entries[pos - 1];

            if (max_hops && hops >= max_hops) {
                break;
            }

            if (victim->reqid == reqid) {
                break;
            }

            if (victim->slack < sample->read_delay) {
                break;
            }

            hops++;
        }
    }

    if (hops) {
        insertion_pos = queue->len - hops;
        start = queue->entries[insertion_pos].start;
        finish = start + sample->read_delay;

        zns_read_bypass_queue_reserve_one(queue);
        for (pos = queue->len; pos > insertion_pos; pos--) {
            struct zns_read_bypass_entry victim = queue->entries[pos - 1];

            victim.start += sample->read_delay;
            victim.finish += sample->read_delay;
            victim.slack -= sample->read_delay;
            queue->entries[pos] = victim;
        }

        entry = zns_read_bypass_make_entry(reqid, subidx, sample, read_stime,
                                           req_deadline, start, finish);
        queue->entries[insertion_pos] = entry;
        queue->len++;
    } else {
        entry = zns_read_bypass_make_entry(reqid, subidx, sample, read_stime,
                                           req_deadline, start, finish);
        zns_read_bypass_queue_push(queue, &entry);
    }

    zns_read_paired_bypass_avail[idx] =
        queue->entries[queue->len - 1].finish;
    *hops_out = hops;

    return finish - read_stime;
}

static void zns_read_paired_model_run(struct zns_ssd *zns, uint64_t reqid,
                                      uint64_t lba, uint32_t nlb,
                                      struct zns_read_slack_sample *samples,
                                      uint64_t sample_cnt,
                                      uint64_t read_stime)
{
    uint64_t baseline_maxlat;
    uint64_t bypass_maxlat = 0;
    uint64_t bypass_subops = 0;
    uint64_t total_hops = 0;
    uint64_t max_hops = 0;
    uint64_t i;

    if (!zns_read_paired_model_enabled()) {
        return;
    }

    baseline_maxlat =
        zns_read_paired_baseline_model(zns, samples, sample_cnt, read_stime);

    for (i = 0; i < sample_cnt; i++) {
        uint64_t hops = 0;
        uint64_t sublat =
            zns_read_paired_bypass_append(reqid, i, zns, &samples[i],
                                          read_stime,
                                          read_stime + baseline_maxlat,
                                          &hops);

        bypass_maxlat = (sublat > bypass_maxlat) ? sublat : bypass_maxlat;
        if (hops) {
            bypass_subops++;
            total_hops += hops;
            max_hops = (hops > max_hops) ? hops : max_hops;
        }
    }

    zns_read_paired_bypass_finalize_deadline(reqid, samples, sample_cnt,
                                             read_stime, bypass_maxlat);

    ftl_log("ZNS_READ_PAIRED_MODEL,reqid=%lu,slba=%lu,nlb=%u,"
            "read_stime_ns=%lu,subops=%lu,"
            "baseline_maxlat_ns=%lu,bypass_maxlat_ns=%lu,"
            "bypass_subops=%lu,total_hops=%lu,max_hops=%lu\n",
            reqid, lba, nlb, read_stime, sample_cnt, baseline_maxlat,
            bypass_maxlat, bypass_subops, total_hops, max_hops);
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
         * 队尾 read 和当前 read 之间不能夹其他 NAND op；允许中间存在
         * idle gap，因为 gap 本身不是安全风险。
         */
        if (!zns_read_bypass_tail_has_no_intervening_op(
                &samples[i].ppa, &queue->entries[queue->len - 1],
                samples[i].est_start)) {
            continue;
        }

        /*
          1. 有对应 plane 队列；
          2. 队列中有 waiting read；
          3. 队尾 waiting read 与当前 read 之间没有其他 NAND op；
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
            if (zns_detail_log_enabled()) {
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
        if (zns_detail_log_enabled()) {
            ftl_log("ZNS_READ_BYPASS_DRYRUN,reqid=%lu,subidx=%lu,lpn=%lu,"
                    "ch=%u,lun=%u,pl=%u,queue_len=%lu,hops=%lu,"
                    "baseline_start_ns=%lu,baseline_finish_ns=%lu,"
                    "bypass_start_ns=%lu,bypass_finish_ns=%lu,benefit_ns=%lu\n",
                    reqid, i, samples[i].lpn, samples[i].ch, samples[i].lun,
                    samples[i].pl, queue->len, hops, samples[i].est_start,
                    samples[i].est_finish, bypass_start, bypass_finish, benefit);
        }
    }
    //这条 host read 内部有多少子请求有 bypass 判断条件，
    //其中多少真的能 bypass，最多能前移几步，总理论收益是多少。
    if (candidate_subops && zns_detail_log_enabled()) {
        ftl_log("ZNS_READ_BYPASS_DRYRUN_SUMMARY,reqid=%lu,subops=%lu,"
                "candidate_subops=%lu,bypassable_subops=%lu,total_hops=%lu,"
                "max_hops=%lu,total_benefit_ns=%lu,max_benefit_ns=%lu\n",
                reqid, sample_cnt, candidate_subops, bypassable_subops,
                total_hops, max_hops, total_benefit, max_benefit);
    }
}

static uint64_t zns_read_bypass_model_maxlat(uint64_t *finish,
                                             uint64_t sample_cnt,
                                             uint64_t read_stime)
{
    uint64_t i;
    uint64_t maxlat = 0;

    for (i = 0; i < sample_cnt; i++) {
        uint64_t sublat = finish[i] - read_stime;
        maxlat = (sublat > maxlat) ? sublat : maxlat;
    }

    return maxlat;
}

static uint64_t zns_read_bypass_model_critical_idx(uint64_t *finish,
                                                   uint64_t sample_cnt,
                                                   uint64_t read_stime)
{
    uint64_t critical_idx = 0;
    uint64_t maxlat = 0;
    uint64_t i;

    for (i = 0; i < sample_cnt; i++) {
        uint64_t sublat = finish[i] - read_stime;

        if (sublat > maxlat) {
            maxlat = sublat;
            critical_idx = i;
        }
    }

    return critical_idx;
}

static bool zns_read_bypass_block_contains(
    struct zns_read_slack_sample *samples, bool *scheduled,
    uint64_t block_start_idx, uint64_t target_idx, uint64_t idx);

static void zns_read_bypass_compute_critset_metrics(
    struct zns_read_slack_sample *samples, bool *scheduled,
    uint64_t sample_cnt, uint64_t read_stime, uint64_t old_maxlat,
    uint64_t *old_finish, uint64_t *new_finish,
    struct zns_read_bypass_block_probe *probe,
    uint64_t *critset_size, uint64_t *critset_covered,
    uint64_t *critset_gain, uint64_t *critset_remaining_max)
{
    uint64_t i;

    *critset_size = 0;
    *critset_covered = 0;
    *critset_gain = 0;
    *critset_remaining_max = 0;

    for (i = 0; i < sample_cnt; i++) {
        uint64_t old_sublat = old_finish[i] - read_stime;
        uint64_t new_sublat = new_finish[i] - read_stime;
        uint64_t gap = (old_sublat < old_maxlat) ?
            (old_maxlat - old_sublat) : 0;

        if (gap > samples[i].read_delay) {
            continue;
        }

        (*critset_size)++;
        *critset_remaining_max =
            (new_sublat > *critset_remaining_max) ?
            new_sublat : *critset_remaining_max;

        if (zns_read_bypass_block_contains(samples, scheduled,
                                           probe->block_start_idx,
                                           probe->target_idx, i)) {
            (*critset_covered)++;
            if (old_sublat > new_sublat) {
                *critset_gain += old_sublat - new_sublat;
            }
        }
    }
}

/*
 * 根据当前真实 plane 队列，刷新“如果剩余 subop 不再 bypass”的完成时间。
 * 已经 commit 的 subop 使用真实 exec_finish；未 commit 的 subop 按原始顺序
 * 接到各自 plane 队尾，用 shadow plane 时间线避免提前改真实状态。
 */
static void zns_read_bypass_refresh_dynamic_model(
    struct zns_ssd *zns, struct zns_read_slack_sample *samples,
    uint64_t sample_cnt, bool *scheduled, uint64_t read_stime,
    uint64_t *model_finish)
{
    struct zns_read_shadow_plane *shadow_planes;
    uint64_t shadow_cnt = 0;
    uint64_t i;

    shadow_planes = g_malloc0(sizeof(*shadow_planes) * sample_cnt);

    for (i = 0; i < sample_cnt; i++) {
        struct zns_read_shadow_plane *shadow;
        struct ppa *sppa = &samples[i].ppa;

        if (scheduled[i]) {
            model_finish[i] = samples[i].exec_finish;
            continue;
        }

        shadow = zns_find_read_shadow_plane(shadow_planes, shadow_cnt, sppa);
        if (!shadow) {
            shadow = &shadow_planes[shadow_cnt++];
            shadow->ch = sppa->g.ch;
            shadow->lun = sppa->g.fc;
            shadow->pl = sppa->g.pl;
            shadow->avail = get_plane(zns, sppa)->next_plane_avail_time;
        }

        model_finish[i] =
            ((shadow->avail < read_stime) ? read_stime : shadow->avail) +
            samples[i].read_delay;
        shadow->avail = model_finish[i];
    }

    g_free(shadow_planes);
}

static bool zns_read_bypass_same_plane(struct zns_read_slack_sample *a,
                                       struct zns_read_slack_sample *b)
{
    return a->ppa.g.ch == b->ppa.g.ch &&
           a->ppa.g.fc == b->ppa.g.fc &&
           a->ppa.g.pl == b->ppa.g.pl;
}

static bool zns_read_bypass_block_contains(
    struct zns_read_slack_sample *samples, bool *scheduled,
    uint64_t block_start_idx, uint64_t target_idx, uint64_t idx)
{
    if (idx < block_start_idx || idx > target_idx || scheduled[idx]) {
        return false;
    }

    return zns_read_bypass_same_plane(&samples[idx], &samples[target_idx]);
}

static uint64_t zns_read_bypass_block_prefix_start(
    struct zns_read_slack_sample *samples, uint64_t sample_cnt,
    bool *scheduled, uint64_t target_idx)
{
    uint64_t start = target_idx;
    uint64_t i;

    for (i = 0; i < sample_cnt; i++) {
        if (i > target_idx || scheduled[i]) {
            continue;
        }
        if (zns_read_bypass_same_plane(&samples[i], &samples[target_idx])) {
            start = i;
            break;
        }
    }

    return start;
}

static uint64_t zns_mixed_diag_block_delay(
    struct zns_read_slack_sample *samples, bool *scheduled,
    uint64_t block_start_idx, uint64_t target_idx)
{
    uint64_t delay = 0;
    uint64_t i;

    for (i = block_start_idx; i <= target_idx; i++) {
        if (zns_read_bypass_block_contains(samples, scheduled,
                                           block_start_idx, target_idx, i)) {
            delay += samples[i].read_delay;
        }
    }

    return delay;
}

static uint64_t zns_mixed_diag_block_count(
    struct zns_read_slack_sample *samples, bool *scheduled,
    uint64_t block_start_idx, uint64_t target_idx)
{
    uint64_t count = 0;
    uint64_t i;

    for (i = block_start_idx; i <= target_idx; i++) {
        if (zns_read_bypass_block_contains(samples, scheduled,
                                           block_start_idx, target_idx, i)) {
            count++;
        }
    }

    return count;
}

static uint64_t zns_mixed_diag_block_target_finish(
    struct zns_read_slack_sample *samples, bool *scheduled,
    uint64_t block_start_idx, uint64_t target_idx, uint64_t block_start)
{
    uint64_t finish = block_start;
    uint64_t i;

    for (i = block_start_idx; i <= target_idx; i++) {
        if (zns_read_bypass_block_contains(samples, scheduled,
                                           block_start_idx, target_idx, i)) {
            finish += samples[i].read_delay;
            if (i == target_idx) {
                return finish;
            }
        }
    }

    return finish;
}

static uint64_t zns_mixed_diag_model_maxlat_after_block(
    struct zns_read_slack_sample *samples, bool *scheduled,
    uint64_t sample_cnt, uint64_t read_stime, uint64_t old_maxlat,
    uint64_t block_start_idx, uint64_t target_idx, uint64_t bypass_start,
    uint64_t *critset_size, uint64_t *critset_covered,
    uint64_t *critset_gain)
{
    uint64_t maxlat = 0;
    uint64_t block_next = bypass_start;
    uint64_t i;

    *critset_size = 0;
    *critset_covered = 0;
    *critset_gain = 0;

    for (i = 0; i < sample_cnt; i++) {
        uint64_t old_finish = samples[i].est_finish;
        uint64_t new_finish = old_finish;
        uint64_t old_sublat = old_finish - read_stime;
        uint64_t gap = (old_sublat < old_maxlat) ?
            (old_maxlat - old_sublat) : 0;
        bool in_critset = gap <= samples[i].read_delay;

        if (zns_read_bypass_block_contains(samples, scheduled,
                                           block_start_idx, target_idx, i)) {
            block_next += samples[i].read_delay;
            new_finish = block_next;
        }

        if (in_critset) {
            (*critset_size)++;
            if (zns_read_bypass_block_contains(samples, scheduled,
                                               block_start_idx, target_idx,
                                               i)) {
                (*critset_covered)++;
                if (old_finish > new_finish) {
                    *critset_gain += old_finish - new_finish;
                }
            }
        }

        maxlat = ((new_finish - read_stime) > maxlat) ?
            (new_finish - read_stime) : maxlat;
    }

    return maxlat;
}

static bool zns_mixed_row_has_raw_conflict(
    struct zns_read_slack_sample *samples, bool *scheduled,
    uint64_t block_start_idx, uint64_t target_idx,
    struct zns_mixed_diag_entry *write)
{
    uint64_t i;
    uint32_t j;

    for (i = block_start_idx; i <= target_idx; i++) {
        if (!zns_read_bypass_block_contains(samples, scheduled,
                                           block_start_idx, target_idx, i)) {
            continue;
        }

        for (j = 0; j < write->nr_lpns; j++) {
            if (samples[i].lpn == write->lpns[j]) {
                return true;
            }
        }
    }

    return false;
}

static bool zns_mixed_row_probe_suffix(
    struct zns_read_slack_sample *samples, uint64_t sample_cnt,
    bool *scheduled, uint64_t read_stime, uint64_t est_maxlat,
    uint64_t block_start_idx, uint64_t target_idx,
    uint64_t *block_count, uint64_t *block_delay,
    uint64_t *victim_writes, uint64_t *insertion_pos,
    uint64_t *bypass_start, uint64_t *new_maxlat,
    uint64_t *host_gain, uint64_t *critset_covered,
    uint64_t *critset_gain, uint64_t *stop_started,
    uint64_t *stop_slack, uint64_t *stop_raw, uint64_t *stop_read)
{
    struct zns_mixed_diag_queue *queue =
        zns_mixed_diag_get_queue(&samples[target_idx].ppa);
    uint64_t pos;
    uint64_t critset_size;
    uint64_t i;

    *block_count = 0;
    *block_delay = 0;
    *victim_writes = 0;
    *insertion_pos = 0;
    *bypass_start = 0;
    *new_maxlat = est_maxlat;
    *host_gain = 0;
    *critset_covered = 0;
    *critset_gain = 0;

    for (i = block_start_idx; i <= target_idx; i++) {
        if (zns_read_bypass_block_contains(samples, scheduled,
                                           block_start_idx, target_idx, i)) {
            (*block_count)++;
            *block_delay += samples[i].read_delay;
        }
    }

    if (!*block_delay || !queue) {
        return false;
    }

    zns_mixed_diag_prune_started(queue, read_stime);
    if (!queue->len) {
        return false;
    }

    if (queue->entries[queue->len - 1].finish !=
        samples[block_start_idx].est_start) {
        return false;
    }

    for (pos = queue->len; pos > 0; pos--) {
        struct zns_mixed_diag_entry *victim = &queue->entries[pos - 1];

        if (victim->cmd != ZNS_MIXED_DIAG_WRITE) {
            (*stop_read)++;
            break;
        }
        if (victim->start < read_stime) {
            (*stop_started)++;
            break;
        }
        if (victim->slack < *block_delay) {
            (*stop_slack)++;
            break;
        }
        if (zns_mixed_row_has_raw_conflict(samples, scheduled,
                                           block_start_idx, target_idx,
                                           victim)) {
            (*stop_raw)++;
            break;
        }

        (*victim_writes)++;
    }

    if (!*victim_writes) {
        return false;
    }

    *insertion_pos = queue->len - *victim_writes;
    *bypass_start = queue->entries[*insertion_pos].start;
    *new_maxlat = zns_mixed_diag_model_maxlat_after_block(
        samples, scheduled, sample_cnt, read_stime, est_maxlat,
        block_start_idx, target_idx, *bypass_start, &critset_size,
        critset_covered, critset_gain);
    *host_gain = (est_maxlat > *new_maxlat) ? (est_maxlat - *new_maxlat) : 0;

    return *host_gain > 0;
}

static bool zns_mixed_row_execute_suffix(
    uint64_t reqid, struct zns_ssd *zns,
    struct zns_read_slack_sample *samples, uint64_t sample_cnt,
    uint64_t read_stime, uint64_t est_maxlat, bool *row_scheduled,
    bool row_diag_enabled)
{
    bool *scheduled;
    uint64_t target_subops = 0;
    uint64_t near_critical_targets = 0;
    uint64_t candidate_blocks = 0;
    uint64_t valid_blocks = 0;
    uint64_t committed_blocks = 0;
    uint64_t total_bypassed_writes = 0;
    uint64_t stop_started = 0;
    uint64_t stop_slack = 0;
    uint64_t stop_raw = 0;
    uint64_t stop_read = 0;
    uint64_t best_target = 0;
    uint64_t best_block_start = 0;
    uint64_t best_block_count = 0;
    uint64_t best_block_delay = 0;
    uint64_t best_victim_writes = 0;
    uint64_t best_insertion_pos = 0;
    uint64_t best_bypass_start = 0;
    uint64_t best_new_maxlat = est_maxlat;
    uint64_t best_host_gain = 0;
    uint64_t best_critset_covered = 0;
    uint64_t best_critset_gain = 0;
    uint64_t i;

    if (!sample_cnt) {
        return false;
    }

    scheduled = g_malloc0(sizeof(*scheduled) * sample_cnt);

    for (i = 0; i < sample_cnt; i++) {
        uint64_t gap = (samples[i].est_sublat < est_maxlat) ?
            (est_maxlat - samples[i].est_sublat) : 0;
        uint64_t start_limit;
        uint64_t block_start_idx;
        bool keep_going = true;

        target_subops++;
        if (gap > samples[i].read_delay) {
            continue;
        }
        near_critical_targets++;

        start_limit = zns_read_bypass_block_prefix_start(
            samples, sample_cnt, scheduled, i);
        block_start_idx = i;

        while (keep_going) {
            uint64_t block_count;
            uint64_t block_delay;
            uint64_t victim_writes;
            uint64_t insertion_pos;
            uint64_t bypass_start;
            uint64_t new_maxlat;
            uint64_t host_gain;
            uint64_t critset_covered;
            uint64_t critset_gain;
            uint64_t old_stop_started = stop_started;
            uint64_t old_stop_slack = stop_slack;
            uint64_t old_stop_raw = stop_raw;
            uint64_t old_stop_read = stop_read;

            candidate_blocks++;
            if (zns_mixed_row_probe_suffix(
                    samples, sample_cnt, scheduled, read_stime, est_maxlat,
                    block_start_idx, i, &block_count, &block_delay,
                    &victim_writes, &insertion_pos, &bypass_start,
                    &new_maxlat, &host_gain, &critset_covered, &critset_gain,
                    &stop_started, &stop_slack, &stop_raw, &stop_read)) {
                uint64_t tie_window = samples[i].read_delay;

                valid_blocks++;
                if (!best_host_gain ||
                    host_gain > best_host_gain + tie_window ||
                    (zns_absdiff_u64(host_gain, best_host_gain) <= tie_window &&
                     (critset_covered > best_critset_covered ||
                      (critset_covered == best_critset_covered &&
                       critset_gain > best_critset_gain)))) {
                    best_target = i;
                    best_block_start = block_start_idx;
                    best_block_count = block_count;
                    best_block_delay = block_delay;
                    best_victim_writes = victim_writes;
                    best_insertion_pos = insertion_pos;
                    best_bypass_start = bypass_start;
                    best_new_maxlat = new_maxlat;
                    best_host_gain = host_gain;
                    best_critset_covered = critset_covered;
                    best_critset_gain = critset_gain;
                }
            } else if (row_diag_enabled &&
                       (old_stop_started != stop_started ||
                        old_stop_slack != stop_slack ||
                        old_stop_raw != stop_raw ||
                        old_stop_read != stop_read)) {
                ftl_log("ZNS_ROW_PROBE_STOP,reqid=%lu,target_subidx=%lu,"
                        "block_start_subidx=%lu,stop_started=%lu,"
                        "stop_slack=%lu,stop_raw=%lu,stop_read=%lu\n",
                        reqid, i, block_start_idx,
                        stop_started - old_stop_started,
                        stop_slack - old_stop_slack,
                        stop_raw - old_stop_raw,
                        stop_read - old_stop_read);
            }

            if (block_start_idx == start_limit) {
                break;
            }

            keep_going = false;
            while (block_start_idx > start_limit) {
                block_start_idx--;
                if (zns_read_bypass_same_plane(&samples[block_start_idx],
                                               &samples[i])) {
                    keep_going = true;
                    break;
                }
            }
        }
    }

    if (best_host_gain) {
        struct zns_mixed_diag_queue *queue =
            zns_mixed_diag_get_queue(&samples[best_target].ppa);
        struct zns_plane *pl = get_plane(zns, &samples[best_target].ppa);
        uint64_t old_len = queue ? queue->len : 0;
        uint64_t block_next = best_bypass_start;
        uint64_t block_pos = 0;
        uint64_t pos;

        if (!queue) {
            g_free(scheduled);
            return false;
        }

        zns_mixed_diag_queue_reserve(queue, best_block_count);

        for (pos = old_len; pos > best_insertion_pos; pos--) {
            struct zns_mixed_diag_entry victim = queue->entries[pos - 1];
            uint64_t old_start = victim.start;
            uint64_t old_finish = victim.finish;
            uint64_t old_slack = victim.slack;

            victim.start += best_block_delay;
            victim.finish += best_block_delay;
            victim.slack -= best_block_delay;
            queue->entries[pos + best_block_count - 1] = victim;

            if (row_diag_enabled) {
                ftl_log("ZNS_ROW_EXEC_HOP,reqid=%lu,target_subidx=%lu,"
                        "victim_flush_id=%lu,victim_write_subidx=%lu,"
                        "victim_old_start_ns=%lu,victim_old_finish_ns=%lu,"
                        "victim_new_start_ns=%lu,victim_new_finish_ns=%lu,"
                        "victim_old_slack_ns=%lu,victim_new_slack_ns=%lu,"
                        "block_delay_ns=%lu\n",
                        reqid, best_target, victim.flush_id,
                        victim.write_subidx, old_start, old_finish,
                        victim.start, victim.finish, old_slack, victim.slack,
                        best_block_delay);
            }
        }

        for (i = best_block_start; i <= best_target; i++) {
            struct zns_mixed_diag_entry entry;
            uint64_t start;
            uint64_t finish;

            if (!zns_read_bypass_block_contains(samples, scheduled,
                                                best_block_start,
                                                best_target, i)) {
                continue;
            }

            start = block_next;
            finish = start + samples[i].read_delay;
            block_next = finish;

            entry = (struct zns_mixed_diag_entry) { 0 };
            entry.cmd = ZNS_MIXED_DIAG_READ;
            entry.start = start;
            entry.finish = finish;
            entry.delay = samples[i].read_delay;
            entry.deadline = read_stime + best_new_maxlat;
            entry.slack = (entry.deadline > entry.finish) ?
                (entry.deadline - entry.finish) : 0;
            entry.reqid = reqid;
            entry.subidx = i;
            entry.lpn = samples[i].lpn;
            entry.ch = samples[i].ch;
            entry.lun = samples[i].lun;
            entry.pl = samples[i].pl;
            queue->entries[best_insertion_pos + block_pos] = entry;
            block_pos++;

            row_scheduled[i] = true;
            samples[i].exec_start = start;
            samples[i].exec_finish = finish;
            samples[i].exec_hops = best_victim_writes;
            samples[i].exec_benefit =
                (samples[i].est_finish > finish) ?
                (samples[i].est_finish - finish) : 0;
        }

        queue->len = old_len + best_block_count;
        pl->next_plane_avail_time = queue->entries[queue->len - 1].finish;

        if (queue->entries[queue->len - 1].cmd == ZNS_MIXED_DIAG_WRITE) {
            struct zns_mixed_diag_entry *tail =
                &queue->entries[queue->len - 1];
            struct zns_plane_last_op_diag *last_op =
                zns_get_plane_last_op_diag(&samples[best_target].ppa);

            if (last_op) {
                last_op->cmd = NAND_WRITE;
                last_op->start = tail->start;
                last_op->finish = tail->finish;
                last_op->delay = tail->delay;
                last_op->flush_id = tail->flush_id;
                last_op->write_subidx = tail->write_subidx;
                last_op->write_slack = tail->slack;
            }
        } else {
            zns_set_plane_last_read_diag(&samples[best_target].ppa,
                                         queue->entries[queue->len - 1].start,
                                         queue->entries[queue->len - 1].finish,
                                         queue->entries[queue->len - 1].delay);
        }

        committed_blocks = 1;
        total_bypassed_writes = best_victim_writes;

        ftl_log("ZNS_ROW_EXEC,reqid=%lu,target_subidx=%lu,"
                "block_start_subidx=%lu,block_count=%lu,bypassed_writes=%lu,"
                "block_delay_ns=%lu,bypass_start_ns=%lu,old_maxlat_ns=%lu,"
                "new_maxlat_ns=%lu,host_gain_ns=%lu,critset_covered=%lu,"
                "critset_gain_ns=%lu,raw_checked=%u\n",
                reqid, best_target, best_block_start, best_block_count,
                best_victim_writes, best_block_delay, best_bypass_start,
                est_maxlat, best_new_maxlat, best_host_gain,
                best_critset_covered, best_critset_gain, 1);
    }

    ftl_log("ZNS_ROW_EXEC_SUMMARY,reqid=%lu,subops=%lu,target_subops=%lu,"
            "near_critical_targets=%lu,candidate_blocks=%lu,"
            "valid_blocks=%lu,committed_blocks=%lu,bypassed_writes=%lu,"
            "total_host_gain_ns=%lu,stop_started_write=%lu,"
            "stop_insufficient_slack=%lu,stop_raw_conflict=%lu,"
            "stop_read_entry=%lu\n",
            reqid, sample_cnt, target_subops, near_critical_targets,
            candidate_blocks, valid_blocks, committed_blocks,
            total_bypassed_writes, best_host_gain, stop_started, stop_slack,
            stop_raw, stop_read);

    g_free(scheduled);
    return committed_blocks > 0;
}

static void zns_mixed_bypass_update_tail(struct zns_ssd *zns, struct ppa *ppa,
                                         struct zns_mixed_diag_queue *queue)
{
    struct zns_plane *pl = get_plane(zns, ppa);
    struct zns_plane_last_op_diag *last_op = zns_get_plane_last_op_diag(ppa);
    struct zns_mixed_diag_entry *tail;

    if (!queue || !queue->len) {
        return;
    }

    tail = &queue->entries[queue->len - 1];
    pl->next_plane_avail_time = tail->finish;

    if (!last_op) {
        return;
    }

    if (tail->cmd == ZNS_MIXED_DIAG_WRITE) {
        last_op->cmd = NAND_WRITE;
        last_op->start = tail->start;
        last_op->finish = tail->finish;
        last_op->delay = tail->delay;
        last_op->flush_id = tail->flush_id;
        last_op->write_subidx = tail->write_subidx;
        last_op->write_slack = tail->slack;
    } else {
        last_op->cmd = NAND_READ;
        last_op->start = tail->start;
        last_op->finish = tail->finish;
        last_op->delay = tail->delay;
        last_op->flush_id = 0;
        last_op->write_subidx = 0;
        last_op->write_slack = 0;
    }
}

static struct zns_mixed_bypass_probe zns_mixed_bypass_probe(
    uint64_t reqid, struct zns_read_slack_sample *samples,
    uint64_t sample_cnt, bool *scheduled, uint64_t read_stime,
    uint64_t est_maxlat, uint64_t block_start_idx, uint64_t target_idx)
{
    struct zns_mixed_bypass_probe probe = { 0 };
    struct zns_mixed_diag_queue *queue =
        zns_mixed_diag_get_queue(&samples[target_idx].ppa);
    uint64_t critset_size;
    uint64_t pos;
    uint64_t i;

    probe.block_start_idx = block_start_idx;
    probe.target_idx = target_idx;
    probe.new_maxlat = est_maxlat;

    for (i = block_start_idx; i <= target_idx; i++) {
        if (zns_read_bypass_block_contains(samples, scheduled,
                                           block_start_idx, target_idx, i)) {
            probe.block_count++;
            probe.block_delay += samples[i].read_delay;
        }
    }

    if (!probe.block_count) {
        return probe;
    }

    if (!queue) {
        probe.reject_no_queue = true;
        return probe;
    }

    zns_mixed_diag_prune_started(queue, read_stime);
    if (!queue->len) {
        probe.reject_empty_queue = true;
        return probe;
    }

    if (queue->entries[queue->len - 1].finish !=
        samples[block_start_idx].est_start) {
        struct zns_mixed_diag_entry *tail =
            &queue->entries[queue->len - 1];
        uint64_t block_start = samples[block_start_idx].est_start;
        bool tail_before = tail->finish < block_start;

        probe.tail_mismatch_tail_read =
            tail->cmd == ZNS_MIXED_DIAG_READ;
        probe.tail_mismatch_tail_write =
            tail->cmd == ZNS_MIXED_DIAG_WRITE;
        probe.tail_mismatch_tail_before = tail_before;
        probe.tail_mismatch_tail_after = tail->finish > block_start;
        probe.tail_mismatch_gap = zns_absdiff_u64(tail->finish, block_start);

        if (!tail_before ||
            !zns_mixed_bypass_allow_idle_gap_enabled()) {
            probe.reject_tail_mismatch = true;
            return probe;
        }

        probe.tail_idle_gap_allowed = true;
        probe.tail_idle_gap = block_start - tail->finish;
    }

    for (pos = queue->len; pos > 0; pos--) {
        struct zns_mixed_diag_entry *victim = &queue->entries[pos - 1];

        if (victim->cmd == ZNS_MIXED_DIAG_READ) {
            if (victim->reqid == reqid) {
                probe.reject_same_req_read = true;
                break;
            }
            if (victim->slack < probe.block_delay) {
                probe.reject_read_slack = true;
                break;
            }
            probe.victim_reads++;
        } else {
            if (victim->start < read_stime) {
                probe.reject_started_write = true;
                break;
            }
            if (victim->slack < probe.block_delay) {
                probe.reject_write_slack = true;
                break;
            }
            if (zns_mixed_row_has_raw_conflict(samples, scheduled,
                                               block_start_idx, target_idx,
                                               victim)) {
                probe.reject_raw = true;
                break;
            }
            probe.victim_writes++;
        }

        probe.victim_count++;
    }

    if (!probe.victim_count) {
        return probe;
    }

    probe.insertion_pos = queue->len - probe.victim_count;
    probe.bypass_start = queue->entries[probe.insertion_pos].start;
    probe.new_maxlat = zns_mixed_diag_model_maxlat_after_block(
        samples, scheduled, sample_cnt, read_stime, est_maxlat,
        block_start_idx, target_idx, probe.bypass_start, &critset_size,
        &probe.critset_covered, &probe.critset_gain);
    probe.host_gain = (est_maxlat > probe.new_maxlat) ?
        (est_maxlat - probe.new_maxlat) : 0;
    probe.valid = probe.host_gain > 0;

    return probe;
}

static void zns_mixed_bypass_commit_probe(
    uint64_t reqid, struct zns_ssd *zns,
    struct zns_read_slack_sample *samples, bool *scheduled,
    bool *mixed_scheduled, uint64_t read_stime,
    uint64_t req_deadline, struct zns_mixed_bypass_probe *probe,
    bool row_diag_enabled)
{
    struct zns_mixed_diag_queue *queue =
        zns_mixed_diag_get_queue(&samples[probe->target_idx].ppa);
    uint64_t old_len = queue ? queue->len : 0;
    uint64_t block_next = probe->bypass_start;
    uint64_t block_pos = 0;
    uint64_t pos;
    uint64_t i;

    if (!queue || !probe->valid) {
        return;
    }

    zns_mixed_diag_queue_reserve(queue, probe->block_count);

    for (pos = old_len; pos > probe->insertion_pos; pos--) {
        struct zns_mixed_diag_entry victim = queue->entries[pos - 1];
        uint64_t old_start = victim.start;
        uint64_t old_finish = victim.finish;
        uint64_t old_slack = victim.slack;

        victim.start += probe->block_delay;
        victim.finish += probe->block_delay;
        victim.slack -= probe->block_delay;
        queue->entries[pos + probe->block_count - 1] = victim;

        if (row_diag_enabled) {
            ftl_log("ZNS_MIXED_BYPASS_HOP,reqid=%lu,target_subidx=%lu,"
                    "block_start_subidx=%lu,victim_type=%s,"
                    "victim_reqid=%lu,victim_subidx=%lu,"
                    "victim_flush_id=%lu,victim_write_subidx=%lu,"
                    "victim_old_start_ns=%lu,victim_old_finish_ns=%lu,"
                    "victim_new_start_ns=%lu,victim_new_finish_ns=%lu,"
                    "victim_old_slack_ns=%lu,victim_new_slack_ns=%lu,"
                    "block_delay_ns=%lu\n",
                    reqid, probe->target_idx, probe->block_start_idx,
                    victim.cmd == ZNS_MIXED_DIAG_READ ? "READ" : "WRITE",
                    victim.reqid, victim.subidx, victim.flush_id,
                    victim.write_subidx, old_start, old_finish,
                    victim.start, victim.finish, old_slack, victim.slack,
                    probe->block_delay);
        }
    }

    for (i = probe->block_start_idx; i <= probe->target_idx; i++) {
        struct zns_mixed_diag_entry entry;
        uint64_t start;
        uint64_t finish;

        if (!zns_read_bypass_block_contains(samples, scheduled,
                                            probe->block_start_idx,
                                            probe->target_idx, i)) {
            continue;
        }

        start = block_next;
        finish = start + samples[i].read_delay;
        block_next = finish;

        entry = (struct zns_mixed_diag_entry) { 0 };
        entry.cmd = ZNS_MIXED_DIAG_READ;
        entry.start = start;
        entry.finish = finish;
        entry.delay = samples[i].read_delay;
        entry.deadline = req_deadline;
        entry.slack = (req_deadline > finish) ? (req_deadline - finish) : 0;
        entry.reqid = reqid;
        entry.subidx = i;
        entry.lpn = samples[i].lpn;
        entry.ch = samples[i].ch;
        entry.lun = samples[i].lun;
        entry.pl = samples[i].pl;
        queue->entries[probe->insertion_pos + block_pos] = entry;
        block_pos++;

        scheduled[i] = true;
        mixed_scheduled[i] = true;
        samples[i].exec_start = start;
        samples[i].exec_finish = finish;
        samples[i].exec_hops = probe->victim_count;
        samples[i].exec_benefit = (samples[i].est_finish > finish) ?
            (samples[i].est_finish - finish) : 0;
        samples[i].exec_candidate = true;

        if (zns_detail_log_enabled()) {
            femu_log("[PU] cmd=READ req=%lu old_avail=%lu start=%lu finish=%lu delay=%lu "
                     "lat=%lu wait=%lu idle=%lu ch=%u lun=%u pl=%u blk=%u pg=%u spg=%u\n",
                     read_stime, samples[i].est_start, start, finish,
                     samples[i].read_delay, finish - read_stime,
                     (start > read_stime) ? (start - read_stime) : 0,
                     (uint64_t)0, samples[i].ppa.g.ch, samples[i].ppa.g.fc,
                     samples[i].ppa.g.pl, samples[i].ppa.g.blk,
                     samples[i].ppa.g.pg, samples[i].ppa.g.spg);
        }
    }

    queue->len = old_len + probe->block_count;
    zns_mixed_bypass_update_tail(zns, &samples[probe->target_idx].ppa, queue);
}

static uint64_t zns_mixed_bypass_append_subop(
    uint64_t reqid, uint64_t subidx, struct zns_ssd *zns,
    struct zns_read_slack_sample *sample, uint64_t read_stime,
    uint64_t req_deadline)
{
    struct zns_mixed_diag_queue *queue =
        zns_mixed_diag_get_queue(&sample->ppa);
    struct zns_plane *pl = get_plane(zns, &sample->ppa);
    uint64_t old_avail = pl->next_plane_avail_time;
    uint64_t start = (old_avail < read_stime) ? read_stime : old_avail;
    uint64_t finish = start + sample->read_delay;
    struct zns_mixed_diag_entry entry;

    if (queue) {
        zns_mixed_diag_prune_started(queue, read_stime);
        entry = (struct zns_mixed_diag_entry) { 0 };
        entry.cmd = ZNS_MIXED_DIAG_READ;
        entry.start = start;
        entry.finish = finish;
        entry.delay = sample->read_delay;
        entry.deadline = req_deadline;
        entry.slack = (req_deadline > finish) ? (req_deadline - finish) : 0;
        entry.reqid = reqid;
        entry.subidx = subidx;
        entry.lpn = sample->lpn;
        entry.ch = sample->ch;
        entry.lun = sample->lun;
        entry.pl = sample->pl;
        zns_mixed_diag_queue_push(queue, &entry);
        zns_mixed_bypass_update_tail(zns, &sample->ppa, queue);
    } else {
        pl->next_plane_avail_time = finish;
        zns_set_plane_last_read_diag(&sample->ppa, start, finish,
                                     sample->read_delay);
    }

    sample->exec_start = start;
    sample->exec_finish = finish;
    sample->exec_hops = 0;
    sample->exec_benefit = 0;

    if (zns_detail_log_enabled()) {
        femu_log("[PU] cmd=READ req=%lu old_avail=%lu start=%lu finish=%lu delay=%lu "
                 "lat=%lu wait=%lu idle=%lu ch=%u lun=%u pl=%u blk=%u pg=%u spg=%u\n",
                 read_stime, old_avail, start, finish, sample->read_delay,
                 finish - read_stime,
                 (start > read_stime) ? (start - read_stime) : 0,
                 (read_stime > old_avail) ? (read_stime - old_avail) : 0,
                 sample->ppa.g.ch, sample->ppa.g.fc, sample->ppa.g.pl,
                 sample->ppa.g.blk, sample->ppa.g.pg, sample->ppa.g.spg);
    }

    return finish - read_stime;
}

static bool zns_mixed_bypass_execute(
    uint64_t reqid, struct zns_ssd *zns,
    struct zns_read_slack_sample *samples, uint64_t sample_cnt,
    uint64_t read_stime, uint64_t est_maxlat, bool *mixed_scheduled,
    bool row_diag_enabled)
{
    enum zns_read_bypass_dynamic_block_mode block_mode =
        zns_read_bypass_get_dynamic_block_mode();
    bool critset_policy =
        zns_read_bypass_dynamic_critset_policy_enabled();
    uint64_t neartie_reads =
        zns_read_bypass_get_dynamic_critset_neartie_reads();
    bool target_critical_only =
        zns_read_bypass_dynamic_target_critical_only_enabled();
    bool *scheduled;
    struct zns_mixed_bypass_probe best = { 0 };
    uint64_t target_subops = 0;
    uint64_t near_critical_targets = 0;
    uint64_t candidate_blocks = 0;
    uint64_t valid_blocks = 0;
    uint64_t reject_no_queue = 0;
    uint64_t reject_empty_queue = 0;
    uint64_t reject_tail_mismatch = 0;
    uint64_t tail_mismatch_tail_read = 0;
    uint64_t tail_mismatch_tail_write = 0;
    uint64_t tail_mismatch_tail_before = 0;
    uint64_t tail_mismatch_tail_after = 0;
    uint64_t tail_mismatch_gap_total = 0;
    uint64_t tail_mismatch_gap_max = 0;
    uint64_t tail_idle_gap_allowed = 0;
    uint64_t tail_idle_gap_total = 0;
    uint64_t tail_idle_gap_max = 0;
    uint64_t reject_same_req_read = 0;
    uint64_t reject_started_write = 0;
    uint64_t reject_read_slack = 0;
    uint64_t reject_write_slack = 0;
    uint64_t reject_raw = 0;
    uint64_t committed_blocks = 0;
    uint64_t committed_subops = 0;
    uint64_t i;

    if (!sample_cnt) {
        return false;
    }

    scheduled = g_malloc0(sizeof(*scheduled) * sample_cnt);

    for (i = 0; i < sample_cnt; i++) {
        uint64_t gap = (samples[i].est_sublat < est_maxlat) ?
            (est_maxlat - samples[i].est_sublat) : 0;
        uint64_t start_limit;
        uint64_t block_start_idx;
        bool keep_going = true;

        target_subops++;
        if ((target_critical_only && gap) ||
            (!target_critical_only && gap > samples[i].read_delay)) {
            continue;
        }
        near_critical_targets++;

        start_limit = zns_read_bypass_block_prefix_start(
            samples, sample_cnt, scheduled, i);
        block_start_idx =
            (block_mode == ZNS_READ_BYPASS_DYNAMIC_BLOCK_PREFIX) ?
            start_limit : i;

        while (keep_going) {
            struct zns_mixed_bypass_probe probe =
                zns_mixed_bypass_probe(reqid, samples, sample_cnt, scheduled,
                                       read_stime, est_maxlat,
                                       block_start_idx, i);
            uint64_t tie_window = neartie_reads * samples[i].read_delay;
            bool better = false;

            candidate_blocks++;
            reject_no_queue += probe.reject_no_queue ? 1 : 0;
            reject_empty_queue += probe.reject_empty_queue ? 1 : 0;
            reject_tail_mismatch += probe.reject_tail_mismatch ? 1 : 0;
            tail_mismatch_tail_read +=
                probe.tail_mismatch_tail_read ? 1 : 0;
            tail_mismatch_tail_write +=
                probe.tail_mismatch_tail_write ? 1 : 0;
            tail_mismatch_tail_before +=
                probe.tail_mismatch_tail_before ? 1 : 0;
            tail_mismatch_tail_after +=
                probe.tail_mismatch_tail_after ? 1 : 0;
            tail_mismatch_gap_total += probe.tail_mismatch_gap;
            tail_mismatch_gap_max =
                (probe.tail_mismatch_gap > tail_mismatch_gap_max) ?
                probe.tail_mismatch_gap : tail_mismatch_gap_max;
            tail_idle_gap_allowed +=
                probe.tail_idle_gap_allowed ? 1 : 0;
            tail_idle_gap_total += probe.tail_idle_gap;
            tail_idle_gap_max =
                (probe.tail_idle_gap > tail_idle_gap_max) ?
                probe.tail_idle_gap : tail_idle_gap_max;
            reject_same_req_read += probe.reject_same_req_read ? 1 : 0;
            reject_started_write += probe.reject_started_write ? 1 : 0;
            reject_read_slack += probe.reject_read_slack ? 1 : 0;
            reject_write_slack += probe.reject_write_slack ? 1 : 0;
            reject_raw += probe.reject_raw ? 1 : 0;

            if (probe.valid) {
                valid_blocks++;
                if (!best.valid) {
                    better = true;
                } else if (critset_policy &&
                           zns_absdiff_u64(probe.host_gain,
                                           best.host_gain) <= tie_window) {
                    better =
                        probe.critset_covered > best.critset_covered ||
                        (probe.critset_covered == best.critset_covered &&
                         probe.critset_gain > best.critset_gain) ||
                        (probe.critset_covered == best.critset_covered &&
                         probe.critset_gain == best.critset_gain &&
                         probe.host_gain > best.host_gain) ||
                        (probe.critset_covered == best.critset_covered &&
                         probe.critset_gain == best.critset_gain &&
                         probe.host_gain == best.host_gain &&
                         probe.block_count < best.block_count);
                } else {
                    better =
                        probe.host_gain > best.host_gain ||
                        (probe.host_gain == best.host_gain &&
                         probe.block_count < best.block_count) ||
                        (probe.host_gain == best.host_gain &&
                         probe.block_count == best.block_count &&
                         probe.victim_count < best.victim_count);
                }

                if (better) {
                    best = probe;
                }
            }

            if (block_mode != ZNS_READ_BYPASS_DYNAMIC_BLOCK_VARIANTS ||
                block_start_idx == start_limit) {
                break;
            }

            keep_going = false;
            while (block_start_idx > start_limit) {
                block_start_idx--;
                if (!scheduled[block_start_idx] &&
                    zns_read_bypass_same_plane(&samples[block_start_idx],
                                               &samples[i])) {
                    keep_going = true;
                    break;
                }
            }
        }
    }

    if (best.valid) {
        zns_mixed_bypass_commit_probe(reqid, zns, samples, scheduled,
                                      mixed_scheduled, read_stime,
                                      read_stime + best.new_maxlat, &best,
                                      row_diag_enabled);
        committed_blocks = 1;
        committed_subops = best.block_count;

        ftl_log("ZNS_MIXED_BYPASS_EXEC,reqid=%lu,target_subidx=%lu,"
                "block_start_subidx=%lu,block_count=%lu,victim_count=%lu,"
                "victim_reads=%lu,victim_writes=%lu,block_delay_ns=%lu,"
                "bypass_start_ns=%lu,old_maxlat_ns=%lu,new_maxlat_ns=%lu,"
                "host_gain_ns=%lu,critset_covered=%lu,critset_gain_ns=%lu\n",
                reqid, best.target_idx, best.block_start_idx,
                best.block_count, best.victim_count, best.victim_reads,
                best.victim_writes, best.block_delay, best.bypass_start,
                est_maxlat, best.new_maxlat, best.host_gain,
                best.critset_covered, best.critset_gain);
    }

    for (i = 0; i < sample_cnt; i++) {
        if (!scheduled[i]) {
            zns_mixed_bypass_append_subop(reqid, i, zns, &samples[i],
                                         read_stime,
                                         read_stime + est_maxlat);
            scheduled[i] = true;
            mixed_scheduled[i] = true;
        }
    }

    if (zns_mixed_bypass_should_log_summary(reqid, committed_blocks,
                                            valid_blocks)) {
        ftl_log("ZNS_MIXED_BYPASS_SUMMARY,reqid=%lu,subops=%lu,"
                "target_subops=%lu,near_critical_targets=%lu,"
                "candidate_blocks=%lu,valid_blocks=%lu,committed_blocks=%lu,"
                "committed_subops=%lu,victim_reads=%lu,victim_writes=%lu,"
                "total_host_gain_ns=%lu,reject_no_queue=%lu,"
                "reject_empty_queue=%lu,reject_tail_mismatch=%lu,"
                "tail_mismatch_tail_read=%lu,"
                "tail_mismatch_tail_write=%lu,"
                "tail_mismatch_tail_before=%lu,"
                "tail_mismatch_tail_after=%lu,"
                "tail_mismatch_gap_total_ns=%lu,"
                "tail_mismatch_gap_max_ns=%lu,"
                "tail_idle_gap_allowed=%lu,"
                "tail_idle_gap_total_ns=%lu,"
                "tail_idle_gap_max_ns=%lu,"
                "reject_same_req_read=%lu,reject_started_write=%lu,"
                "reject_read_slack=%lu,reject_write_slack=%lu,"
                "reject_raw=%lu,target_critical_only=%u\n",
                reqid, sample_cnt, target_subops, near_critical_targets,
                candidate_blocks, valid_blocks, committed_blocks,
                committed_subops, best.victim_reads, best.victim_writes,
                best.host_gain, reject_no_queue, reject_empty_queue,
                reject_tail_mismatch, tail_mismatch_tail_read,
                tail_mismatch_tail_write, tail_mismatch_tail_before,
                tail_mismatch_tail_after, tail_mismatch_gap_total,
                tail_mismatch_gap_max, tail_idle_gap_allowed,
                tail_idle_gap_total, tail_idle_gap_max, reject_same_req_read,
                reject_started_write, reject_read_slack, reject_write_slack,
                reject_raw,
                target_critical_only ? 1 : 0);
    }

    g_free(scheduled);
    return committed_blocks > 0;
}

static void zns_mixed_diag_read_over_write_dryrun(
    uint64_t reqid, struct zns_read_slack_sample *samples,
    uint64_t sample_cnt, uint64_t read_stime, uint64_t est_maxlat)
{
    bool *scheduled;
    uint64_t target_subops = 0;
    uint64_t near_critical_targets = 0;
    uint64_t candidate_blocks = 0;
    uint64_t opportunity_blocks = 0;
    uint64_t positive_host_gain_blocks = 0;
    uint64_t total_victim_writes = 0;
    uint64_t max_victim_writes = 0;
    uint64_t total_block_delay = 0;
    uint64_t total_target_benefit = 0;
    uint64_t total_host_gain = 0;
    uint64_t max_host_gain = 0;
    uint64_t i;

    if (!sample_cnt) {
        return;
    }

    scheduled = g_malloc0(sizeof(*scheduled) * sample_cnt);

    for (i = 0; i < sample_cnt; i++) {
        struct zns_mixed_diag_queue *queue;
        uint64_t gap;
        uint64_t start_limit;
        uint64_t block_start_idx;
        bool keep_going = true;

        target_subops++;
        gap = (samples[i].est_sublat < est_maxlat) ?
            (est_maxlat - samples[i].est_sublat) : 0;
        if (gap > samples[i].read_delay) {
            continue;
        }
        near_critical_targets++;

        queue = zns_mixed_diag_get_queue(&samples[i].ppa);
        if (!queue) {
            continue;
        }

        zns_mixed_diag_prune_started(queue, read_stime);
        if (!queue->len) {
            continue;
        }

        if (queue->entries[queue->len - 1].finish != samples[i].est_start) {
            continue;
        }

        start_limit = zns_read_bypass_block_prefix_start(
            samples, sample_cnt, scheduled, i);
        block_start_idx = i;

        while (keep_going) {
            uint64_t block_delay = zns_mixed_diag_block_delay(
                samples, scheduled, block_start_idx, i);
            uint64_t block_count = zns_mixed_diag_block_count(
                samples, scheduled, block_start_idx, i);
            uint64_t pos;
            uint64_t victim_count = 0;
            uint64_t insertion_pos;
            uint64_t bypass_start;
            uint64_t target_finish;
            uint64_t target_benefit;
            uint64_t new_maxlat;
            uint64_t host_gain;
            uint64_t critset_size;
            uint64_t critset_covered;
            uint64_t critset_gain;

            if (!block_delay) {
                goto next_variant;
            }

            candidate_blocks++;

            for (pos = queue->len; pos > 0; pos--) {
                struct zns_mixed_diag_entry *victim =
                    &queue->entries[pos - 1];

                if (victim->cmd != ZNS_MIXED_DIAG_WRITE) {
                    break;
                }
                if (victim->start < read_stime) {
                    break;
                }
                if (victim->slack < block_delay) {
                    break;
                }
                if (zns_mixed_row_has_raw_conflict(samples, scheduled,
                                                   block_start_idx, i,
                                                   victim)) {
                    break;
                }

                victim_count++;
            }

            if (!victim_count) {
                goto next_variant;
            }

            insertion_pos = queue->len - victim_count;
            bypass_start = queue->entries[insertion_pos].start;
            target_finish = zns_mixed_diag_block_target_finish(
                samples, scheduled, block_start_idx, i, bypass_start);
            target_benefit = (samples[i].est_finish > target_finish) ?
                (samples[i].est_finish - target_finish) : 0;
            new_maxlat = zns_mixed_diag_model_maxlat_after_block(
                samples, scheduled, sample_cnt, read_stime, est_maxlat,
                block_start_idx, i, bypass_start, &critset_size,
                &critset_covered, &critset_gain);
            host_gain = (est_maxlat > new_maxlat) ?
                (est_maxlat - new_maxlat) : 0;

            opportunity_blocks++;
            total_victim_writes += victim_count;
            max_victim_writes = (victim_count > max_victim_writes) ?
                victim_count : max_victim_writes;
            total_block_delay += block_delay;
            total_target_benefit += target_benefit;
            if (host_gain) {
                positive_host_gain_blocks++;
                total_host_gain += host_gain;
                max_host_gain = (host_gain > max_host_gain) ?
                    host_gain : max_host_gain;
            }

            ftl_log("ZNS_MIXED_ROW_DRYRUN,reqid=%lu,target_subidx=%lu,"
                    "block_start_subidx=%lu,block_count=%lu,"
                    "victim_writes=%lu,bypass_start_ns=%lu,"
                    "block_delay_ns=%lu,target_benefit_ns=%lu,"
                    "old_maxlat_ns=%lu,new_maxlat_ns=%lu,host_gain_ns=%lu,"
                    "critical_set_size=%lu,critical_set_covered=%lu,"
                    "critical_set_gain_ns=%lu,first_victim_flush_id=%lu,"
                    "first_victim_write_subidx=%lu\n",
                    reqid, i, block_start_idx, block_count,
                    victim_count, bypass_start, block_delay, target_benefit,
                    est_maxlat, new_maxlat, host_gain, critset_size,
                    critset_covered, critset_gain,
                    queue->entries[insertion_pos].flush_id,
                    queue->entries[insertion_pos].write_subidx);

next_variant:
            if (block_start_idx == start_limit) {
                break;
            }

            keep_going = false;
            while (block_start_idx > start_limit) {
                block_start_idx--;
                if (zns_read_bypass_same_plane(&samples[block_start_idx],
                                               &samples[i])) {
                    keep_going = true;
                    break;
                }
            }
        }
    }

    if (candidate_blocks) {
        ftl_log("ZNS_MIXED_ROW_DRYRUN_SUMMARY,reqid=%lu,subops=%lu,"
                "target_subops=%lu,near_critical_targets=%lu,"
                "candidate_blocks=%lu,opportunity_blocks=%lu,"
                "positive_host_gain_blocks=%lu,total_victim_writes=%lu,"
                "max_victim_writes=%lu,total_block_delay_ns=%lu,"
                "total_target_benefit_ns=%lu,total_host_gain_ns=%lu,"
                "max_host_gain_ns=%lu\n",
                reqid, sample_cnt, target_subops, near_critical_targets,
                candidate_blocks, opportunity_blocks,
                positive_host_gain_blocks, total_victim_writes,
                max_victim_writes, total_block_delay, total_target_benefit,
                total_host_gain, max_host_gain);
    }

    g_free(scheduled);
}

static struct zns_read_bypass_block_probe zns_read_bypass_probe_block(
    uint64_t reqid, struct zns_ssd *zns,
    struct zns_read_slack_sample *samples, uint64_t sample_cnt,
    bool *scheduled, uint64_t block_start_idx, uint64_t target_idx,
    uint64_t read_stime)
{
    struct zns_read_bypass_block_probe probe = { 0 };
    struct zns_read_bypass_queue *queue =
        zns_read_bypass_get_queue(&samples[target_idx].ppa);
    struct zns_plane *pl = get_plane(zns, &samples[target_idx].ppa);
    uint64_t old_avail = pl->next_plane_avail_time;
    uint64_t pos;
    uint64_t i;

    probe.block_start_idx = block_start_idx;
    probe.target_idx = target_idx;
    probe.baseline_start = (old_avail < read_stime) ? read_stime : old_avail;
    probe.start = probe.baseline_start;

    for (i = block_start_idx; i <= target_idx; i++) {
        if (zns_read_bypass_block_contains(samples, scheduled,
                                           block_start_idx, target_idx, i)) {
            probe.block_count++;
            probe.block_delay += samples[i].read_delay;
        }
    }

    if (!probe.block_count) {
        return probe;
    }

    probe.baseline_finish = probe.baseline_start + probe.block_delay;
    probe.finish = probe.baseline_finish;

    if (!queue) {
        probe.reject_no_queue = true;
        return probe;
    }

    zns_read_bypass_prune_started(queue, read_stime);
    probe.queue_len = queue->len;

    if (!queue->len) {
        probe.reject_empty_queue = true;
        return probe;
    }

    if (queue->entries[queue->len - 1].finish != probe.baseline_start) {
        probe.reject_tail_not_ready = true;
        return probe;
    }

    if (queue->entries[queue->len - 1].reqid == reqid) {
        probe.reject_same_req_tail = true;
        return probe;
    }

    probe.candidate = true;

    for (pos = queue->len; pos > 0; pos--) {
        struct zns_read_bypass_entry *victim = &queue->entries[pos - 1];

        if (victim->reqid == reqid) {
            probe.reject_same_req_blocker = true;
            break;
        }

        if (victim->slack < probe.block_delay) {
            probe.reject_slack_insufficient = true;
            break;
        }

        probe.victim_count++;
    }

    if (probe.victim_count) {
        probe.valid = true;
        probe.insertion_pos = queue->len - probe.victim_count;
        probe.start = queue->entries[probe.insertion_pos].start;
        probe.finish = probe.start + probe.block_delay;
    }

    return probe;
}

static uint64_t zns_read_bypass_model_maxlat_after_block_probe(
    struct zns_ssd *zns, struct zns_read_slack_sample *samples,
    uint64_t sample_cnt, bool *scheduled, uint64_t read_stime,
    struct zns_read_bypass_block_probe *probe, uint64_t *probe_finish)
{
    struct zns_read_shadow_plane *shadow_planes;
    bool *in_block;
    uint64_t shadow_cnt = 0;
    uint64_t block_next = probe->start;
    uint64_t maxlat = 0;
    uint64_t i;

    shadow_planes = g_malloc0(sizeof(*shadow_planes) * sample_cnt);
    in_block = g_malloc0(sizeof(*in_block) * sample_cnt);

    for (i = 0; i < sample_cnt; i++) {
        in_block[i] = zns_read_bypass_block_contains(samples, scheduled,
                                                     probe->block_start_idx,
                                                     probe->target_idx, i);
    }

    for (i = 0; i < sample_cnt; i++) {
        struct zns_read_shadow_plane *shadow;
        struct ppa *sppa = &samples[i].ppa;
        uint64_t finish;
        uint64_t sublat;

        if (scheduled[i]) {
            finish = samples[i].exec_finish;
            goto update_maxlat;
        }

        if (in_block[i]) {
            finish = block_next + samples[i].read_delay;
            block_next = finish;
            goto update_maxlat;
        }

        shadow = zns_find_read_shadow_plane(shadow_planes, shadow_cnt, sppa);
        if (!shadow) {
            shadow = &shadow_planes[shadow_cnt++];
            shadow->ch = sppa->g.ch;
            shadow->lun = sppa->g.fc;
            shadow->pl = sppa->g.pl;
            shadow->avail = get_plane(zns, sppa)->next_plane_avail_time;

            /*
             * block 插入后，目标 plane 的队尾时间与“把这个 block 按普通
             * 顺序接到队尾”相同，都是 baseline_finish。后续未纳入 block
             * 的同 plane sibling 需要从这个时间之后继续排队。
             */
            if (zns_read_bypass_same_plane(&samples[i],
                                           &samples[probe->target_idx]) &&
                shadow->avail < probe->baseline_finish) {
                shadow->avail = probe->baseline_finish;
            }
        }

        finish = ((shadow->avail < read_stime) ? read_stime : shadow->avail) +
            samples[i].read_delay;
        shadow->avail = finish;

update_maxlat:
        if (probe_finish) {
            probe_finish[i] = finish;
        }
        sublat = finish - read_stime;
        maxlat = (sublat > maxlat) ? sublat : maxlat;
    }

    g_free(in_block);
    g_free(shadow_planes);
    return maxlat;
}

static uint64_t zns_read_bypass_append_subop(
    uint64_t reqid, uint64_t subidx, struct zns_ssd *zns,
    struct zns_read_slack_sample *sample, uint64_t read_stime,
    uint64_t req_deadline)
{
    struct zns_read_bypass_queue *queue =
        zns_read_bypass_get_queue(&sample->ppa);
    struct zns_plane *pl = get_plane(zns, &sample->ppa);
    uint64_t old_avail = pl->next_plane_avail_time;
    uint64_t start = (old_avail < read_stime) ? read_stime : old_avail;
    uint64_t finish = start + sample->read_delay;
    struct zns_read_bypass_entry entry;

    if (!queue) {
        pl->next_plane_avail_time = finish;
        zns_set_plane_last_read_diag(&sample->ppa, start, finish,
                                     sample->read_delay);
    } else {
        zns_read_bypass_prune_started(queue, read_stime);
        entry = zns_read_bypass_make_entry(reqid, subidx, sample, read_stime,
                                           req_deadline, start, finish);
        zns_read_bypass_queue_push(queue, &entry);
        pl->next_plane_avail_time = queue->entries[queue->len - 1].finish;
        zns_set_plane_last_read_diag(&sample->ppa,
                                     queue->entries[queue->len - 1].start,
                                     queue->entries[queue->len - 1].finish,
                                     queue->entries[queue->len - 1].read_delay);
    }

    sample->exec_start = start;
    sample->exec_finish = finish;
    sample->exec_hops = 0;
    sample->exec_benefit = 0;

    if (zns_detail_log_enabled()) {
        femu_log("[PU] cmd=READ req=%lu old_avail=%lu start=%lu finish=%lu delay=%lu "
                 "lat=%lu wait=%lu idle=%lu ch=%u lun=%u pl=%u blk=%u pg=%u spg=%u\n",
                 read_stime, old_avail, start, finish, sample->read_delay,
                 finish - read_stime,
                 (start > read_stime) ? (start - read_stime) : 0,
                 (read_stime > old_avail) ? (read_stime - old_avail) : 0,
                 sample->ppa.g.ch, sample->ppa.g.fc, sample->ppa.g.pl,
                 sample->ppa.g.blk, sample->ppa.g.pg, sample->ppa.g.spg);
    }

    return finish - read_stime;
}

static void zns_read_bypass_commit_block_probe(
    uint64_t reqid, struct zns_ssd *zns,
    struct zns_read_slack_sample *samples, bool *scheduled,
    uint64_t read_stime, uint64_t req_deadline,
    struct zns_read_bypass_block_probe *probe)
{
    struct zns_read_bypass_queue *queue =
        zns_read_bypass_get_queue(&samples[probe->target_idx].ppa);
    struct zns_plane *pl = get_plane(zns, &samples[probe->target_idx].ppa);
    uint64_t pos;
    uint64_t i;
    uint64_t block_pos = 0;
    uint64_t block_next = probe->start;

    if (!queue || !probe->valid) {
        return;
    }

    zns_read_bypass_queue_reserve(queue, probe->block_count);

    for (pos = queue->len; pos > probe->insertion_pos; pos--) {
        struct zns_read_bypass_entry victim = queue->entries[pos - 1];
        uint64_t old_start = victim.start;
        uint64_t old_finish = victim.finish;
        uint64_t old_slack = victim.slack;

        victim.start += probe->block_delay;
        victim.finish += probe->block_delay;
        victim.slack -= probe->block_delay;
        victim.slack_consumed_by_pred_critical += probe->block_delay;
        queue->entries[pos + probe->block_count - 1] = victim;

        if (zns_read_bypass_summary_log_enabled()) {
            ftl_log("ZNS_READ_BYPASS_DYNAMIC_BLOCK_HOP,reqid=%lu,"
                    "target_subidx=%lu,hop=%lu,victim_reqid=%lu,"
                    "victim_subidx=%lu,victim_lpn=%lu,"
                    "victim_old_start_ns=%lu,"
                    "victim_old_finish_ns=%lu,victim_new_start_ns=%lu,"
                    "victim_new_finish_ns=%lu,victim_deadline_ns=%lu,"
                    "victim_old_slack_ns=%lu,victim_new_slack_ns=%lu,"
                    "slack_used_ns=%lu,block_count=%lu\n",
                    reqid, probe->target_idx,
                    queue->len - pos + 1, victim.reqid, victim.subidx,
                    victim.lpn, old_start, old_finish, victim.start,
                    victim.finish, victim.deadline, old_slack, victim.slack,
                    probe->block_delay, probe->block_count);
        }
    }

    for (i = probe->block_start_idx; i <= probe->target_idx; i++) {
        struct zns_read_bypass_entry entry;
        uint64_t start;
        uint64_t finish;
        uint64_t baseline_finish;

        if (!zns_read_bypass_block_contains(samples, scheduled,
                                            probe->block_start_idx,
                                            probe->target_idx, i)) {
            continue;
        }

        start = block_next;
        finish = start + samples[i].read_delay;
        block_next = finish;
        baseline_finish = probe->baseline_start + block_next - probe->start;

        entry = zns_read_bypass_make_entry(reqid, i, &samples[i], read_stime,
                                           req_deadline, start, finish);
        queue->entries[probe->insertion_pos + block_pos] = entry;

        samples[i].exec_start = start;
        samples[i].exec_finish = finish;
        samples[i].exec_hops = probe->victim_count;
        samples[i].exec_benefit = (baseline_finish > finish) ?
            (baseline_finish - finish) : 0;
        samples[i].exec_consumed_victim_slack =
            probe->victim_count * samples[i].read_delay;
        samples[i].exec_candidate = true;
        scheduled[i] = true;

        if (zns_read_bypass_summary_log_enabled()) {
            ftl_log("ZNS_READ_BYPASS_DYNAMIC_BLOCK_ENTRY,reqid=%lu,"
                    "subidx=%lu,lpn=%lu,target_subidx=%lu,"
                    "block_start_subidx=%lu,block_pos=%lu,"
                    "victim_count=%lu,baseline_finish_ns=%lu,"
                    "exec_start_ns=%lu,exec_finish_ns=%lu,benefit_ns=%lu,"
                    "pred_critical=%u,pred_critical_gap_ns=%lu\n",
                    reqid, i, samples[i].lpn, probe->target_idx,
                    probe->block_start_idx, block_pos,
                    probe->victim_count, baseline_finish, start, finish,
                    samples[i].exec_benefit,
                    samples[i].pred_critical ? 1 : 0,
                    samples[i].pred_critical_gap);
        }

        if (zns_detail_log_enabled()) {
            femu_log("[PU] cmd=READ req=%lu old_avail=%lu start=%lu finish=%lu delay=%lu "
                     "lat=%lu wait=%lu idle=%lu ch=%u lun=%u pl=%u blk=%u pg=%u spg=%u\n",
                     read_stime, probe->baseline_start, start, finish,
                     samples[i].read_delay, finish - read_stime,
                     (start > read_stime) ? (start - read_stime) : 0,
                     (uint64_t)0, samples[i].ppa.g.ch, samples[i].ppa.g.fc,
                     samples[i].ppa.g.pl, samples[i].ppa.g.blk,
                     samples[i].ppa.g.pg, samples[i].ppa.g.spg);
        }

        block_pos++;
    }

    queue->len += probe->block_count;
    pl->next_plane_avail_time = queue->entries[queue->len - 1].finish;
    zns_set_plane_last_read_diag(&samples[probe->target_idx].ppa,
                                 queue->entries[queue->len - 1].start,
                                 queue->entries[queue->len - 1].finish,
                                 queue->entries[queue->len - 1].read_delay);
}

static bool zns_read_bypass_find_best_dynamic_probe(
    uint64_t reqid, struct zns_ssd *zns,
    struct zns_read_slack_sample *samples, uint64_t sample_cnt,
    bool *scheduled, uint64_t *model_finish, uint64_t read_stime,
    struct zns_read_bypass_block_probe *best_probe, uint64_t *best_gain,
    uint64_t *best_cost, uint64_t *best_gap, uint64_t *best_block_count,
    uint64_t *probed_subops, struct zns_read_bypass_dynamic_funnel *funnel,
    bool mark_samples)
{
    enum zns_read_bypass_dynamic_block_mode block_mode =
        zns_read_bypass_get_dynamic_block_mode();
    bool critset_policy =
        zns_read_bypass_dynamic_critset_policy_enabled();
    uint64_t critset_neartie_reads =
        zns_read_bypass_get_dynamic_critset_neartie_reads();
    bool target_critical_only =
        zns_read_bypass_dynamic_target_critical_only_enabled();
    uint64_t old_maxlat;
    bool found = false;
    uint64_t best_critset_covered = 0;
    uint64_t best_critset_gain = 0;
    uint64_t i;

    *best_probe = (struct zns_read_bypass_block_probe) { 0 };
    *best_gain = 0;
    *best_cost = 0;
    *best_gap = 0;
    *best_block_count = 0;

    old_maxlat = zns_read_bypass_model_maxlat(model_finish, sample_cnt,
                                              read_stime);

    for (i = 0; i < sample_cnt; i++) {
        struct zns_read_bypass_block_probe probe;
        uint64_t gap;
        uint64_t new_maxlat;
        uint64_t gain;
        uint64_t cost;

        if (scheduled[i]) {
            continue;
        }

        if (funnel) {
            funnel->target_subops++;
        }

        gap = (model_finish[i] - read_stime < old_maxlat) ?
            (old_maxlat - (model_finish[i] - read_stime)) : 0;

        /*
         * 默认沿用旧逻辑：target 可以是 critical/near-critical。
         * 打开 critical-only 后，只允许真正 critical subop 作为 target；
         * 非关键 sibling 仍可通过 block/set 被一起移动。
         */
        if ((target_critical_only && gap) ||
            (!target_critical_only && gap > samples[i].read_delay)) {
            if (funnel) {
                funnel->reject_noncritical_gap++;
            }
            continue;
        }

        if (funnel) {
            funnel->near_critical_targets++;
        }

        {
            bool target_reject_no_queue = false;
            bool target_reject_empty_queue = false;
            bool target_reject_tail_not_ready = false;
            bool target_reject_same_req_tail = false;
            bool target_candidate = false;
            bool target_reject_same_req_blocker = false;
            bool target_reject_slack_insufficient = false;
            bool target_valid = false;
            bool target_positive_gain = false;
            uint64_t start_limit = zns_read_bypass_block_prefix_start(
                samples, sample_cnt, scheduled, i);
            uint64_t block_start_idx =
                (block_mode == ZNS_READ_BYPASS_DYNAMIC_BLOCK_PREFIX) ?
                start_limit : i;
            bool keep_going = true;

            while (keep_going) {
                probe = zns_read_bypass_probe_block(reqid, zns, samples,
                                                    sample_cnt, scheduled,
                                                    block_start_idx, i,
                                                    read_stime);
                target_reject_no_queue |= probe.reject_no_queue;
                target_reject_empty_queue |= probe.reject_empty_queue;
                target_reject_tail_not_ready |= probe.reject_tail_not_ready;
                target_reject_same_req_tail |= probe.reject_same_req_tail;
                target_candidate |= probe.candidate;
                target_reject_same_req_blocker |=
                    probe.reject_same_req_blocker;
                target_reject_slack_insufficient |=
                    probe.reject_slack_insufficient;
                (*probed_subops)++;

                if (probe.valid) {
                    uint64_t *probe_finish;
                    uint64_t old_critical_idx;
                    uint64_t new_critical_idx;
                    uint64_t target_sublat_before;
                    uint64_t old_critical_sublat;
                    uint64_t new_critical_sublat;
                    uint64_t critset_size;
                    uint64_t critset_covered;
                    uint64_t critset_gain;
                    uint64_t critset_remaining_max;
                    uint64_t victim_cost;
                    bool positive_gain;

                    target_valid = true;
                    probe_finish = g_malloc0(sizeof(*probe_finish) * sample_cnt);
                    new_maxlat = zns_read_bypass_model_maxlat_after_block_probe(
                        zns, samples, sample_cnt, scheduled, read_stime,
                        &probe, probe_finish);
                    old_critical_idx = zns_read_bypass_model_critical_idx(
                        model_finish, sample_cnt, read_stime);
                    new_critical_idx = zns_read_bypass_model_critical_idx(
                        probe_finish, sample_cnt, read_stime);
                    target_sublat_before = model_finish[i] - read_stime;
                    old_critical_sublat =
                        model_finish[old_critical_idx] - read_stime;
                    new_critical_sublat =
                        probe_finish[new_critical_idx] - read_stime;
                    zns_read_bypass_compute_critset_metrics(
                        samples, scheduled, sample_cnt, read_stime,
                        old_maxlat, model_finish, probe_finish, &probe,
                        &critset_size, &critset_covered, &critset_gain,
                        &critset_remaining_max);
                    victim_cost = probe.block_delay * probe.victim_count;
                    probe.critset_size = critset_size;
                    probe.critset_covered = critset_covered;
                    probe.critset_gain = critset_gain;
                    probe.critset_remaining_max = critset_remaining_max;
                    probe.victim_cost = victim_cost;
                    positive_gain = new_maxlat < old_maxlat;

                    if (zns_read_bypass_summary_log_enabled()) {
                        ftl_log("ZNS_READ_BYPASS_DYNAMIC_CRITSET,reqid=%lu,"
                                "target_subidx=%lu,block_start_subidx=%lu,"
                                "block_count=%lu,victim_count=%lu,"
                                "old_maxlat_ns=%lu,new_maxlat_ns=%lu,"
                                "host_gain_ns=%lu,critical_set_size=%lu,"
                                "critical_set_covered=%lu,"
                                "critical_set_gain_ns=%lu,"
                                "critical_set_remaining_max_ns=%lu,"
                                "victim_cost_ns=%lu,"
                                "old_critical_subidx=%lu,"
                                "new_critical_subidx=%lu,"
                                "positive_gain=%u,critset_policy=%u,"
                                "critset_neartie_reads=%lu,"
                                "target_critical_only=%u\n",
                                reqid, probe.target_idx,
                                probe.block_start_idx,
                                probe.block_count, probe.victim_count,
                                old_maxlat, new_maxlat,
                                positive_gain ? old_maxlat - new_maxlat : 0,
                                critset_size, critset_covered, critset_gain,
                                critset_remaining_max, victim_cost,
                                old_critical_idx,
                                new_critical_idx, positive_gain ? 1 : 0,
                                critset_policy ? 1 : 0, critset_neartie_reads,
                                target_critical_only ? 1 : 0);
                    }

                    if (positive_gain) {
                        uint64_t neartie_window =
                            critset_neartie_reads * samples[i].read_delay;
                        bool gain_near_best;
                        bool critset_can_break_tie;
                        bool better;

                        target_positive_gain = true;
                        gain = old_maxlat - new_maxlat;
                        cost = victim_cost;

                        gain_near_best = found &&
                            (*best_gain > gain ?
                             (*best_gain - gain <= neartie_window) :
                             (gain - *best_gain <= neartie_window));
                        critset_can_break_tie = critset_policy &&
                            (gain == *best_gain ||
                             (neartie_window && gain_near_best));

                        if (!found) {
                            better = true;
                        } else if (critset_can_break_tie) {
                            better =
                                critset_covered > best_critset_covered ||
                                (critset_covered ==
                                 best_critset_covered &&
                                 critset_gain > best_critset_gain) ||
                                (critset_covered ==
                                 best_critset_covered &&
                                 critset_gain == best_critset_gain &&
                                 gain > *best_gain) ||
                                (critset_covered ==
                                 best_critset_covered &&
                                 critset_gain == best_critset_gain &&
                                 gain == *best_gain &&
                                 probe.block_count < *best_block_count) ||
                                (critset_covered ==
                                 best_critset_covered &&
                                 critset_gain == best_critset_gain &&
                                 gain == *best_gain &&
                                 probe.block_count ==
                                 *best_block_count && cost < *best_cost) ||
                                (critset_covered ==
                                 best_critset_covered &&
                                 critset_gain == best_critset_gain &&
                                 gain == *best_gain &&
                                 probe.block_count ==
                                 *best_block_count &&
                                 cost == *best_cost && gap < *best_gap);
                        } else {
                            better =
                                gain > *best_gain ||
                                (gain == *best_gain &&
                                 probe.block_count < *best_block_count) ||
                                (gain == *best_gain &&
                                 probe.block_count == *best_block_count &&
                                 cost < *best_cost) ||
                                (gain == *best_gain &&
                                 probe.block_count == *best_block_count &&
                                 cost == *best_cost && gap < *best_gap);
                        }

                        if (better) {
                            found = true;
                            *best_probe = probe;
                            *best_gain = gain;
                            *best_gap = gap;
                            *best_cost = cost;
                            *best_block_count = probe.block_count;
                            best_critset_covered = critset_covered;
                            best_critset_gain = critset_gain;
                        }
                    } else {
                        if (zns_read_bypass_summary_log_enabled()) {
                            ftl_log("ZNS_READ_BYPASS_DYNAMIC_NO_GAIN,reqid=%lu,"
                                    "target_subidx=%lu,"
                                    "block_start_subidx=%lu,"
                                    "block_count=%lu,victim_count=%lu,"
                                    "old_maxlat_ns=%lu,new_maxlat_ns=%lu,"
                                    "target_sublat_before_ns=%lu,"
                                    "target_gap_to_old_max_ns=%lu,"
                                    "old_critical_subidx=%lu,"
                                    "old_critical_sublat_ns=%lu,"
                                    "new_critical_subidx=%lu,"
                                    "new_critical_sublat_ns=%lu,"
                                    "target_is_old_critical=%u,"
                                    "target_is_new_critical=%u\n",
                                    reqid, probe.target_idx,
                                    probe.block_start_idx, probe.block_count,
                                    probe.victim_count, old_maxlat,
                                    new_maxlat, target_sublat_before, gap,
                                    old_critical_idx, old_critical_sublat,
                                    new_critical_idx, new_critical_sublat,
                                    old_critical_idx == i ? 1 : 0,
                                    new_critical_idx == i ? 1 : 0);
                        }
                    }
                    g_free(probe_finish);
                }

                if (block_mode != ZNS_READ_BYPASS_DYNAMIC_BLOCK_VARIANTS ||
                    block_start_idx == start_limit) {
                    break;
                }

                keep_going = false;
                while (block_start_idx > start_limit) {
                    block_start_idx--;
                    if (!scheduled[block_start_idx] &&
                        zns_read_bypass_same_plane(&samples[block_start_idx],
                                                   &samples[i])) {
                        keep_going = true;
                        break;
                    }
                }
            }

            if (funnel) {
                funnel->reject_no_queue += target_reject_no_queue ? 1 : 0;
                funnel->reject_empty_queue += target_reject_empty_queue ? 1 : 0;
                funnel->reject_tail_not_ready +=
                    target_reject_tail_not_ready ? 1 : 0;
                funnel->reject_same_req_tail +=
                    target_reject_same_req_tail ? 1 : 0;
                funnel->queue_candidate_targets += target_candidate ? 1 : 0;
                funnel->reject_same_req_blocker +=
                    target_reject_same_req_blocker ? 1 : 0;
                funnel->reject_slack_insufficient +=
                    target_reject_slack_insufficient ? 1 : 0;
                funnel->valid_block_targets += target_valid ? 1 : 0;
                funnel->reject_no_host_gain +=
                    (target_valid && !target_positive_gain) ? 1 : 0;
                funnel->positive_gain_targets += target_positive_gain ? 1 : 0;
            }
            if (mark_samples) {
                samples[i].exec_candidate =
                    samples[i].exec_candidate || target_candidate;
                samples[i].exec_reject_slack_insufficient =
                    samples[i].exec_reject_slack_insufficient ||
                    target_reject_slack_insufficient;
            }
        }
    }

    return found;
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
    uint64_t max_hops = zns_read_bypass_get_max_hops();
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
        zns_set_plane_last_read_diag(&sample->ppa, baseline_start,
                                     baseline_finish, sample->read_delay);
        return baseline_finish - read_stime;
    }

    zns_read_bypass_prune_started(queue, read_stime);

    /*
     * 只有队尾 read 和当前 read 之间没有其他 NAND op，且它来自更早的
     * host read，incoming read 才算 candidate。中间可以有 idle gap；
     * gap 不是请求，不能单独阻止绕行。
     */
    if (queue->len &&
        zns_read_bypass_tail_has_no_intervening_op(
            &sample->ppa, &queue->entries[queue->len - 1],
            baseline_start) &&
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

            if (max_hops && hops >= max_hops) {
                break;
            }

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

            if (zns_detail_log_enabled()) {
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
        }

        entry = zns_read_bypass_make_entry(reqid, subidx, sample, read_stime,
                                           req_deadline, start, finish);
        queue->entries[insertion_pos] = entry;
        queue->len++;

        sample->exec_hops = hops;
        sample->exec_benefit =
            (baseline_finish > finish) ? (baseline_finish - finish) : 0;

        if (zns_detail_log_enabled()) {
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
        }
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
    zns_set_plane_last_read_diag(&sample->ppa,
                                 queue->entries[queue->len - 1].start,
                                 queue->entries[queue->len - 1].finish,
                                 queue->entries[queue->len - 1].read_delay);

    if (zns_detail_log_enabled()) {
        femu_log("[PU] cmd=READ req=%lu old_avail=%lu start=%lu finish=%lu delay=%lu "
                 "lat=%lu wait=%lu idle=%lu ch=%u lun=%u pl=%u blk=%u pg=%u spg=%u\n",
                 read_stime, old_avail, start, finish, sample->read_delay,
                 finish - read_stime,
                 (start > read_stime) ? (start - read_stime) : 0,
                 (read_stime > old_avail) ? (read_stime - old_avail) : 0,
                 sample->ppa.g.ch, sample->ppa.g.fc, sample->ppa.g.pl,
                 sample->ppa.g.blk, sample->ppa.g.pg, sample->ppa.g.spg);
    }

    return finish - read_stime;
}

/*
 * dynamic execute：以整条 host read 的 maxlat 为目标。
 *
 * 当前版本围绕关键/近关键 subop 构造一个同 plane block。默认 prefix 模式
 * 仍然使用“target 以及它前面尚未调度的同 plane sibling”这个完整前缀 block；
 * 若开启 variants 模式，则会同时尝试 [C] / [B C] / [A B C] 这类不同大小的
 * suffix/prefix 变体，再按 host_gain、block_count、cost、gap 选最优。
 *
 * prefix block 可以处理：
 *
 *   A B C  ->  B C A
 *
 * C 是关键，B slack 小导致 C 单独绕不过；但 [B C] 整体可以绕过 slack
 * 充足的 A，从而让 C 也提前。
 *
 * 为避免追着关键路径反复调整，第一版每条 host read 最多 commit 一次
 * 最优 block move。后续实验可以通过 FEMU_ZNS_READ_BYPASS_DYNAMIC_MAX_ROUNDS
 * 把这个上限提高到 2/3，做 bounded multi-round dynamic。
 */
static void zns_read_bypass_execute_dynamic(
    uint64_t reqid, struct zns_ssd *zns,
    struct zns_read_slack_sample *samples, uint64_t sample_cnt,
    uint64_t read_stime, uint64_t req_deadline)
{
    bool *scheduled;
    uint64_t *model_finish;
    uint64_t initial_maxlat;
    uint64_t old_maxlat;
    uint64_t final_maxlat;
    uint64_t rounds = 0;
    uint64_t probed_subops = 0;
    uint64_t committed_subops = 0;
    uint64_t total_host_gain = 0;
    uint64_t critical_shift_count = 0;
    uint64_t max_rounds = zns_read_bypass_get_dynamic_max_rounds();
    uint64_t limit_next_gain = 0;
    uint64_t limit_next_block_count = 0;
    uint64_t limit_next_victim_count = 0;
    uint64_t limit_next_cost = 0;
    uint64_t limit_next_target_idx = UINT64_MAX;
    uint64_t limit_next_gap = 0;
    bool stopped_by_max_rounds = false;
    struct zns_read_bypass_dynamic_funnel funnel = { 0 };
    bool target_critical_only =
        zns_read_bypass_dynamic_target_critical_only_enabled();
    uint64_t i;

    if (!sample_cnt) {
        return;
    }

    scheduled = g_malloc0(sizeof(*scheduled) * sample_cnt);
    model_finish = g_malloc0(sizeof(*model_finish) * sample_cnt);

    zns_read_bypass_refresh_dynamic_model(zns, samples, sample_cnt,
                                          scheduled, read_stime,
                                          model_finish);
    initial_maxlat = zns_read_bypass_model_maxlat(model_finish, sample_cnt,
                                                  read_stime);

    while (committed_subops < sample_cnt && rounds < max_rounds) {
        struct zns_read_bypass_block_probe best_probe = { 0 };
        uint64_t best_gain = 0;
        uint64_t best_gap = 0;
        uint64_t best_cost = 0;
        uint64_t best_block_count = 0;
        bool found;

        old_maxlat = zns_read_bypass_model_maxlat(model_finish, sample_cnt,
                                                  read_stime);
        found = zns_read_bypass_find_best_dynamic_probe(
            reqid, zns, samples, sample_cnt, scheduled, model_finish,
            read_stime, &best_probe, &best_gain, &best_cost, &best_gap,
            &best_block_count, &probed_subops, &funnel, true);

        if (!found) {
            break;
        }

        rounds++;
        zns_read_bypass_commit_block_probe(reqid, zns, samples, scheduled,
                                           read_stime, req_deadline,
                                           &best_probe);
        committed_subops += best_probe.block_count;
        total_host_gain += best_gain;

        if (zns_read_bypass_summary_log_enabled()) {
            ftl_log("ZNS_READ_BYPASS_DYNAMIC_BLOCK,reqid=%lu,round=%lu,"
                    "target_subidx=%lu,block_start_subidx=%lu,target_lpn=%lu,"
                    "block_count=%lu,"
                    "victim_count=%lu,block_delay_ns=%lu,old_maxlat_ns=%lu,"
                    "new_maxlat_ns=%lu,host_gain_ns=%lu,cost_ns=%lu,"
                    "pred_critical_gap_ns=%lu,critical_set_size=%lu,"
                    "critical_set_covered=%lu,critical_set_gain_ns=%lu,"
                    "critical_set_remaining_max_ns=%lu,"
                    "victim_cost_ns=%lu\n",
                    reqid, rounds, best_probe.target_idx,
                    best_probe.block_start_idx,
                    samples[best_probe.target_idx].lpn,
                    best_probe.block_count, best_probe.victim_count,
                    best_probe.block_delay, old_maxlat, old_maxlat - best_gain,
                    best_gain, best_cost,
                    samples[best_probe.target_idx].pred_critical_gap,
                    best_probe.critset_size, best_probe.critset_covered,
                    best_probe.critset_gain, best_probe.critset_remaining_max,
                    best_probe.victim_cost);
        }

        zns_read_bypass_refresh_dynamic_model(zns, samples, sample_cnt,
                                              scheduled, read_stime,
                                              model_finish);
        final_maxlat = zns_read_bypass_model_maxlat(model_finish, sample_cnt,
                                                    read_stime);
        for (i = 0; i < sample_cnt; i++) {
            if (model_finish[i] - read_stime == final_maxlat) {
                if (i != best_probe.target_idx) {
                    critical_shift_count++;
                }
                break;
            }
        }
    }

    if (rounds >= max_rounds && committed_subops < sample_cnt) {
        struct zns_read_bypass_block_probe next_probe = { 0 };
        uint64_t diag_probed_subops = 0;
        uint64_t next_block_count = 0;

        stopped_by_max_rounds = true;
        if (zns_read_bypass_find_best_dynamic_probe(
                reqid, zns, samples, sample_cnt, scheduled, model_finish,
                read_stime, &next_probe, &limit_next_gain,
                &limit_next_cost, &limit_next_gap, &next_block_count,
                &diag_probed_subops, NULL, false)) {
            limit_next_block_count = next_probe.block_count;
            limit_next_victim_count = next_probe.victim_count;
            limit_next_target_idx = next_probe.target_idx;
            if (zns_read_bypass_summary_log_enabled()) {
                ftl_log("ZNS_READ_BYPASS_DYNAMIC_LIMIT_NEXT,reqid=%lu,"
                        "max_rounds=%lu,rounds=%lu,target_subidx=%lu,"
                        "block_start_subidx=%lu,"
                        "block_count=%lu,victim_count=%lu,host_gain_ns=%lu,"
                        "cost_ns=%lu,pred_critical_gap_ns=%lu,"
                        "diag_probed_subops=%lu\n",
                        reqid, max_rounds, rounds, limit_next_target_idx,
                        next_probe.block_start_idx,
                        limit_next_block_count, limit_next_victim_count,
                        limit_next_gain, limit_next_cost, limit_next_gap,
                        diag_probed_subops);
            }
        }
    }

    /*
     * 还没有被 dynamic commit 的 subop 按当前队列顺序接到队尾。它们不再尝试
     * bypass，因为前面的循环已经证明没有 host-level gain。
     */
    for (i = 0; i < sample_cnt; i++) {
        if (!scheduled[i]) {
            zns_read_bypass_append_subop(reqid, i, zns, &samples[i],
                                         read_stime, req_deadline);
            scheduled[i] = true;
        }
    }

    final_maxlat = 0;
    for (i = 0; i < sample_cnt; i++) {
        uint64_t sublat = samples[i].exec_finish - read_stime;
        final_maxlat = (sublat > final_maxlat) ? sublat : final_maxlat;
    }

    if (zns_read_bypass_summary_log_enabled()) {
        ftl_log("ZNS_READ_BYPASS_DYNAMIC_SUMMARY,reqid=%lu,subops=%lu,"
                "rounds=%lu,probed_subops=%lu,committed_subops=%lu,"
                "initial_maxlat_ns=%lu,final_maxlat_ns=%lu,"
                "total_host_gain_ns=%lu,"
                "critical_shift_count=%lu,max_rounds=%lu,"
                "stopped_by_max_rounds=%u,"
                "limit_next_host_gain_ns=%lu,limit_next_target_subidx=%lu,"
                "limit_next_block_count=%lu,limit_next_victim_count=%lu,"
                "limit_next_cost_ns=%lu,limit_next_pred_critical_gap_ns=%lu,"
                "target_critical_only=%u\n",
                reqid, sample_cnt, rounds, probed_subops, committed_subops,
                initial_maxlat, final_maxlat, total_host_gain,
                critical_shift_count, max_rounds,
                stopped_by_max_rounds ? 1 : 0, limit_next_gain,
                limit_next_target_idx, limit_next_block_count,
                limit_next_victim_count, limit_next_cost, limit_next_gap,
                target_critical_only ? 1 : 0);

        ftl_log("ZNS_READ_BYPASS_DYNAMIC_FUNNEL,reqid=%lu,subops=%lu,"
                "max_rounds=%lu,rounds=%lu,target_subops=%lu,"
                "reject_noncritical_gap=%lu,near_critical_targets=%lu,"
                "reject_no_queue=%lu,reject_empty_queue=%lu,"
                "reject_tail_not_ready=%lu,reject_same_req_tail=%lu,"
                "queue_candidate_targets=%lu,reject_same_req_blocker=%lu,"
                "reject_slack_insufficient=%lu,valid_block_targets=%lu,"
                "reject_no_host_gain=%lu,positive_gain_targets=%lu,"
                "committed_blocks=%lu,committed_subops=%lu,"
                "total_host_gain_ns=%lu,target_critical_only=%u\n",
                reqid, sample_cnt, max_rounds, rounds, funnel.target_subops,
                funnel.reject_noncritical_gap, funnel.near_critical_targets,
                funnel.reject_no_queue, funnel.reject_empty_queue,
                funnel.reject_tail_not_ready, funnel.reject_same_req_tail,
                funnel.queue_candidate_targets, funnel.reject_same_req_blocker,
                funnel.reject_slack_insufficient, funnel.valid_block_targets,
                funnel.reject_no_host_gain, funnel.positive_gain_targets,
                rounds, committed_subops, total_host_gain,
                target_critical_only ? 1 : 0);
    }

    g_free(model_finish);
    g_free(scheduled);
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
static uint64_t zns_advance_status_sampled(struct zns_ssd *zns, struct ppa *ppa,
                                           struct nand_cmd *ncmd,
                                           struct zns_nand_timing_sample *sample)
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
    struct zns_plane_last_op_diag *last_op =
        zns_get_plane_last_op_diag(ppa);

    if (sample) {
        sample->req_stime = req_stime;
        sample->old_avail = old_avail;
        sample->start = nand_stime;
        sample->finish = finish_time;
        sample->delay = op_delay;
        sample->lat = lat;
        sample->wait = queue_wait;
        sample->idle = idle_gap;
    }

    if (last_op) {
        last_op->cmd = ncmd->cmd;
        last_op->start = nand_stime;
        last_op->finish = finish_time;
        last_op->delay = op_delay;
        if (ncmd->cmd == NAND_WRITE && zns_wc_diag_active) {
            last_op->flush_id = zns_wc_diag_flush_id;
            last_op->write_subidx = zns_wc_diag_write_subidx;
            last_op->write_slack = 0;
        } else {
            last_op->flush_id = 0;
            last_op->write_subidx = 0;
            last_op->write_slack = 0;
        }
    }

    /*
     * 每一行代表一次 NAND 子操作占用一个 plane 的时间段。
     */
    if (zns_detail_log_enabled()) {
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
    }
    return lat;
}

static uint64_t zns_advance_status(struct zns_ssd *zns, struct ppa *ppa,
                                   struct nand_cmd *ncmd)
{
    return zns_advance_status_sampled(zns, ppa, ncmd, NULL);
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

/* 根据当前写指针和本次 write zone 生成新 PPA 的基础坐标。 */
static struct ppa get_new_page(struct zns_ssd *zns, uint32_t zone_idx)
{
    struct write_pointer *wpp = &zns->wp;
    struct ppa ppa;
    ppa.ppa = 0;
    ppa.g.ch = wpp->ch;
    ppa.g.fc = wpp->lun;
    ppa.g.blk = zone_idx;
    ppa.g.V = 1; // 1 表示这不是 padding page
    if(!valid_ppa(zns,&ppa))
    {
        ftl_err("[Misao] invalid ppa: ch %u lun %u pl %u blk %u pg %u subpg  %u \n",ppa.g.ch,ppa.g.fc,ppa.g.pl,ppa.g.blk,ppa.g.pg,ppa.g.spg);
        ppa.ppa = UNMAPPED_PPA;
    }
    return ppa;
}

/* 查找当前 write zone 已绑定的 write cache 槽位；找不到返回 -1。 */
static int zns_get_wcidx(struct zns_ssd* zns, uint32_t zone_idx)
{
    int i;
    for(i = 0;i < zns->cache.num_wc;i++)
    {
        if(zns->cache.write_cache[i].sblk==zone_idx)
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
    uint64_t global_baseline_maxlat = 0;
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
    bool mixed_bypass_exec_enabled = zns_mixed_bypass_exec_enabled();
    bool exec_enabled =
        zns_read_bypass_exec_enabled() && !mixed_bypass_exec_enabled;
    enum zns_read_bypass_policy policy = zns_read_bypass_get_policy();
    bool row_diag_enabled = zns_row_diag_enabled();
    bool row_exec_enabled =
        zns_row_exec_enabled() && !exec_enabled && !mixed_bypass_exec_enabled;
    bool row_tracking_enabled =
        row_diag_enabled || row_exec_enabled || mixed_bypass_exec_enabled;
    struct zns_read_slack_sample *samples;
    bool *row_scheduled;
    uint64_t blocked_by_write_subops = 0;
    uint64_t read_over_write_opportunity_subops = 0;
    uint64_t total_wait_behind_write = 0;
    uint64_t total_opportunity_gain = 0;

    samples = g_malloc0(sizeof(*samples) * nr_lpn);
    row_scheduled = g_malloc0(sizeof(*row_scheduled) * nr_lpn);

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

            if (row_tracking_enabled) {
                struct zns_plane_last_op_diag *last_op =
                    zns_get_plane_last_op_diag(sppa);

                if (last_op && last_op->cmd == NAND_WRITE &&
                    last_op->finish > read_stime &&
                    samples[i].est_start == last_op->finish) {
                    uint64_t wait_behind_write =
                        last_op->finish - read_stime;
                    bool write_not_started =
                        last_op->start >= read_stime;
                    bool opportunity =
                        write_not_started &&
                        read_delay <= last_op->write_slack;

                    blocked_by_write_subops++;
                    total_wait_behind_write += wait_behind_write;
                    if (opportunity) {
                        read_over_write_opportunity_subops++;
                        total_opportunity_gain += wait_behind_write;
                    }

                    samples[i].blocked_by_write = true;
                    samples[i].blocking_write_flush_id = last_op->flush_id;
                    samples[i].blocking_write_subidx =
                        last_op->write_subidx;
                    samples[i].blocking_write_start = last_op->start;
                    samples[i].blocking_write_finish = last_op->finish;
                    samples[i].blocking_write_slack = last_op->write_slack;
                    samples[i].blocking_write_wait = wait_behind_write;
                    samples[i].blocking_write_not_started =
                        write_not_started;
                    samples[i].blocking_write_opportunity = opportunity;

                    if (row_diag_enabled) {
                        ftl_log("ZNS_READ_BLOCKED_BY_WRITE,reqid=%lu,"
                                "subidx=%lu,lpn=%lu,ch=%u,lun=%u,pl=%u,"
                                "read_stime_ns=%lu,read_delay_ns=%lu,"
                                "blocking_write_flush_id=%lu,"
                                "blocking_write_subidx=%lu,"
                                "write_start_ns=%lu,write_finish_ns=%lu,"
                                "write_slack_ns=%lu,"
                                "read_wait_behind_write_ns=%lu,"
                                "write_not_started=%u,opportunity=%u\n",
                                reqid, i, samples[i].lpn, samples[i].ch,
                                samples[i].lun, samples[i].pl, read_stime,
                                read_delay, last_op->flush_id,
                                last_op->write_subidx, last_op->start,
                                last_op->finish, last_op->write_slack,
                                wait_behind_write,
                                write_not_started ? 1 : 0,
                                opportunity ? 1 : 0);
                    }
                }
            }

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

        {
            uint64_t critical_subops = 0;
            uint64_t critical_blocked_by_write = 0;
            uint64_t critical_row_opportunities = 0;
            uint64_t noncritical_subops = 0;
            uint64_t noncritical_blocked_by_write = 0;
            uint64_t noncritical_row_opportunities = 0;
            uint64_t pred_critical_subops = 0;
            uint64_t pred_critical_blocked_by_write = 0;
            uint64_t pred_critical_row_opportunities = 0;

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

            if (samples[i].est_sublat == est_maxlat) {
                critical_subops++;
                if (samples[i].blocked_by_write) {
                    critical_blocked_by_write++;
                    if (samples[i].blocking_write_opportunity) {
                        critical_row_opportunities++;
                    }
                }
            } else {
                noncritical_subops++;
                if (samples[i].blocked_by_write) {
                    noncritical_blocked_by_write++;
                    if (samples[i].blocking_write_opportunity) {
                        noncritical_row_opportunities++;
                    }
                }
            }

            if (samples[i].pred_critical) {
                pred_critical_subops++;
                if (samples[i].blocked_by_write) {
                    pred_critical_blocked_by_write++;
                    if (samples[i].blocking_write_opportunity) {
                        pred_critical_row_opportunities++;
                    }
                }
            }

            if (row_diag_enabled && samples[i].blocked_by_write) {
                bool is_critical = samples[i].est_sublat == est_maxlat;

                ftl_log("ZNS_READ_BLOCKED_BY_WRITE_DETAIL,reqid=%lu,"
                        "subidx=%lu,lpn=%lu,ch=%u,lun=%u,pl=%u,"
                        "read_stime_ns=%lu,read_delay_ns=%lu,"
                        "est_start_ns=%lu,est_finish_ns=%lu,"
                        "est_sublat_ns=%lu,est_maxlat_ns=%lu,"
                        "est_slack_ns=%lu,is_critical=%u,"
                        "pred_critical=%u,pred_critical_gap_ns=%lu,"
                        "blocking_write_flush_id=%lu,"
                        "blocking_write_subidx=%lu,"
                        "write_start_ns=%lu,write_finish_ns=%lu,"
                        "write_slack_ns=%lu,"
                        "read_wait_behind_write_ns=%lu,"
                        "write_not_started=%u,opportunity=%u\n",
                        reqid, i, samples[i].lpn, samples[i].ch,
                        samples[i].lun, samples[i].pl, read_stime,
                        samples[i].read_delay, samples[i].est_start,
                        samples[i].est_finish, samples[i].est_sublat,
                        est_maxlat, samples[i].est_slack,
                        is_critical ? 1 : 0,
                        samples[i].pred_critical ? 1 : 0,
                        samples[i].pred_critical_gap,
                        samples[i].blocking_write_flush_id,
                        samples[i].blocking_write_subidx,
                        samples[i].blocking_write_start,
                        samples[i].blocking_write_finish,
                        samples[i].blocking_write_slack,
                        samples[i].blocking_write_wait,
                        samples[i].blocking_write_not_started ? 1 : 0,
                        samples[i].blocking_write_opportunity ? 1 : 0);
            }
        }

        global_baseline_maxlat =
            zns_read_global_baseline_model(zns, samples, sample_cnt,
                                           read_stime);
        zns_read_paired_model_run(zns, reqid, lba, nlb, samples, sample_cnt,
                                  read_stime);

        g_free(shadow_planes);

        if (row_diag_enabled && blocked_by_write_subops) {
            ftl_log("ZNS_READ_OVER_WRITE_SUMMARY,reqid=%lu,subops=%lu,"
                    "blocked_by_write_subops=%lu,"
                    "opportunity_subops=%lu,total_wait_behind_write_ns=%lu,"
                    "total_opportunity_gain_ns=%lu,"
                    "critical_subops=%lu,critical_blocked_by_write=%lu,"
                    "critical_opportunity_subops=%lu,"
                    "noncritical_subops=%lu,"
                    "noncritical_blocked_by_write=%lu,"
                    "noncritical_opportunity_subops=%lu,"
                    "pred_critical_subops=%lu,"
                    "pred_critical_blocked_by_write=%lu,"
                    "pred_critical_opportunity_subops=%lu\n",
                    reqid, sample_cnt, blocked_by_write_subops,
                    read_over_write_opportunity_subops,
                    total_wait_behind_write, total_opportunity_gain,
                    critical_subops, critical_blocked_by_write,
                    critical_row_opportunities, noncritical_subops,
                    noncritical_blocked_by_write,
                    noncritical_row_opportunities, pred_critical_subops,
                    pred_critical_blocked_by_write,
                    pred_critical_row_opportunities);
        }
    }
    }

    if (sample_cnt) {
        /*
         * 只做机会统计，不改真实调度：按论文方式从 plane 队尾向前扫描，
         * 看当前 read 子请求最多能跨过多少个 waiting read。
         */
        zns_read_bypass_dryrun(reqid, samples, sample_cnt, read_stime);
        if (row_diag_enabled) {
            zns_mixed_diag_read_over_write_dryrun(reqid, samples, sample_cnt,
                                                  read_stime, est_maxlat);
        }
    }

    /*
     * 第三步：根据运行模式选择真实调度路径。
     *
     * baseline 模式继续走原来的 zns_advance_status()；
     * execute 模式则真正使用 read-read bypass 队列调度。
     */
    if (sample_cnt && mixed_bypass_exec_enabled) {
        zns_mixed_bypass_execute(reqid, zns, samples, sample_cnt,
                                 read_stime, est_maxlat, row_scheduled,
                                 row_diag_enabled);
    } else if (sample_cnt && exec_enabled &&
        policy == ZNS_READ_BYPASS_POLICY_DYNAMIC) {
        zns_read_bypass_execute_dynamic(reqid, zns, samples, sample_cnt,
                                        read_stime, read_stime + est_maxlat);
    } else if (sample_cnt && row_exec_enabled) {
        zns_mixed_row_execute_suffix(reqid, zns, samples, sample_cnt,
                                     read_stime, est_maxlat, row_scheduled,
                                     row_diag_enabled);
    }

    for (uint64_t i = 0; i < sample_cnt; i++) {
        ppa = samples[i].ppa;
        if (row_scheduled[i]) {
            sublat = samples[i].exec_finish - read_stime;
        } else if (exec_enabled && policy == ZNS_READ_BYPASS_POLICY_DYNAMIC) {
            sublat = samples[i].exec_finish - read_stime;
        } else if (exec_enabled) {
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
        if (zns_detail_log_enabled()) {
            femu_log("[R] lpn:\t%lu\t<--ch:\t%u\tlun:\t%u\tpl:\t%u\tblk:\t%u\tpg:\t%u\tsubpg:\t%u\tlat\t%lu\n",samples[i].lpn,ppa.g.ch,ppa.g.fc,ppa.g.pl,ppa.g.blk,ppa.g.pg,ppa.g.spg,sublat);
        }

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
        uint64_t sum_wait_ns = 0;
        uint64_t max_wait_ns = 0;
        uint64_t critical_wait_ns = 0;
        uint64_t critical_start_ns = 0;
        uint64_t critical_finish_ns = 0;
        uint64_t critical_subidx = 0;
        uint16_t critical_ch = 0;
        uint16_t critical_lun = 0;
        uint16_t critical_pl = 0;
        uint64_t critical_exec_hops = 0;
        uint64_t critical_exec_benefit = 0;
        uint64_t critical_est_sublat = 0;
        uint64_t critical_est_start = 0;
        uint64_t critical_est_finish = 0;
        uint64_t critical_blocked_by_noncritical_slack = 0;
        bool critical_exec_candidate = false;
        bool critical_reject_slack_insufficient = false;
        bool critical_reject_after_noncritical = false;
        bool critical_seen = false;
        uint64_t est_critical_wait_ns = 0;
        uint64_t est_critical_start_ns = 0;
        uint64_t est_critical_finish_ns = 0;
        uint64_t est_critical_subidx = 0;
        uint16_t est_critical_ch = 0;
        uint16_t est_critical_lun = 0;
        uint16_t est_critical_pl = 0;
        bool est_critical_seen = false;
        uint64_t i, j;

        for (i = 0; i < sample_cnt; i++) { //遍历每个子请求
            uint64_t slack = maxlat - samples[i].sublat;
            uint64_t obs_finish = read_stime + samples[i].sublat;
            uint64_t obs_start = (samples[i].sublat >= samples[i].read_delay) ?
                (obs_finish - samples[i].read_delay) : read_stime;
            uint64_t wait_ns = (obs_start > read_stime) ?
                (obs_start - read_stime) : 0;
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

            sum_wait_ns += wait_ns;
            max_wait_ns = (wait_ns > max_wait_ns) ? wait_ns : max_wait_ns;
            if (obs_critical && (!critical_seen ||
                wait_ns > critical_wait_ns)) {
                critical_seen = true;
                critical_wait_ns = wait_ns;
                critical_start_ns = obs_start;
                critical_finish_ns = obs_finish;
                critical_subidx = i;
                critical_ch = samples[i].ch;
                critical_lun = samples[i].lun;
                critical_pl = samples[i].pl;
                critical_exec_hops = samples[i].exec_hops;
                critical_exec_benefit = samples[i].exec_benefit;
                critical_exec_candidate = samples[i].exec_candidate;
                critical_reject_slack_insufficient =
                    samples[i].exec_reject_slack_insufficient;
                critical_reject_after_noncritical =
                    samples[i].exec_reject_after_pred_noncritical;
                critical_blocked_by_noncritical_slack =
                    samples[i].exec_blocked_by_pred_noncritical_slack;
                critical_est_sublat = samples[i].est_sublat;
                critical_est_start = samples[i].est_start;
                critical_est_finish = samples[i].est_finish;
            }
            if (true_baseline_critical && (!est_critical_seen ||
                samples[i].est_start > est_critical_start_ns)) {
                est_critical_seen = true;
                est_critical_wait_ns =
                    (samples[i].est_start > read_stime) ?
                    (samples[i].est_start - read_stime) : 0;
                est_critical_start_ns = samples[i].est_start;
                est_critical_finish_ns = samples[i].est_finish;
                est_critical_subidx = i;
                est_critical_ch = samples[i].ch;
                est_critical_lun = samples[i].lun;
                est_critical_pl = samples[i].pl;
            }

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
            if (zns_detail_log_enabled()) {
                ftl_log("ZNS_READ_SUBSLACK,reqid=%lu,subidx=%lu,lpn=%lu,"
                        "ch=%u,lun=%u,pl=%u,sublat_ns=%lu,slack_ns=%lu\n",
                        reqid, i, samples[i].lpn, samples[i].ch,
                        samples[i].lun, samples[i].pl, samples[i].sublat,
                        slack);
            }

            /*
             * 子操作级在线估计日志：
             *   est_*：estimated，真实调度前用 shadow timeline 算出的估计值；
             *   obs_*：observed，zns_advance_status() 真实执行后的观测值；
             *   *_err：估计值和观测值的绝对误差。
             */
            if (zns_detail_log_enabled()) {
                ftl_log("ZNS_READ_SUBESTSLACK,reqid=%lu,subidx=%lu,lpn=%lu,"
                        "ch=%u,lun=%u,pl=%u,est_start_ns=%lu,est_finish_ns=%lu,"
                        "est_sublat_ns=%lu,est_slack_ns=%lu,obs_sublat_ns=%lu,"
                        "obs_slack_ns=%lu,sublat_err_ns=%lu,slack_err_ns=%lu\n",
                        reqid, i, samples[i].lpn, samples[i].ch,
                        samples[i].lun, samples[i].pl, samples[i].est_start,
                        samples[i].est_finish, samples[i].est_sublat,
                        est_slack, samples[i].sublat, slack, sublat_err,
                        slack_err);
            }

        }

        exec_host_improvement = (est_maxlat > maxlat) ?
            (est_maxlat - maxlat) : 0;

        ftl_log("ZNS_READ_TIMELINE,reqid=%lu,slba=%lu,nlb=%u,"
                "read_stime_ns=%lu,subops=%lu,unique_planes=%lu,"
                "maxlat_ns=%lu,avg_wait_ns=%lu,max_wait_ns=%lu,"
                "critical_wait_ns=%lu,critical_start_ns=%lu,"
                "critical_finish_ns=%lu,critical_subidx=%lu,"
                "critical_ch=%u,critical_lun=%u,critical_pl=%u,"
                "critical_exec_hops=%lu,critical_exec_benefit_ns=%lu,"
                "critical_exec_candidate=%u,"
                "critical_reject_slack_insufficient=%u,"
                "critical_reject_after_noncritical=%u,"
                "critical_blocked_by_noncritical_slack_ns=%lu,"
                "critical_est_sublat_ns=%lu,critical_est_start_ns=%lu,"
                "critical_est_finish_ns=%lu,"
                "est_maxlat_ns=%lu,est_critical_wait_ns=%lu,"
                "est_critical_start_ns=%lu,est_critical_finish_ns=%lu,"
                "est_critical_subidx=%lu,est_critical_ch=%u,"
                "est_critical_lun=%u,est_critical_pl=%u,"
                "exec_enabled=%u,candidate_subops=%lu,bypass_subops=%lu,"
                "total_hops=%lu,max_hops=%lu,host_improved=%u,"
                "host_improvement_ns=%lu,pred_critical_executed_subops=%lu,"
                "pred_noncritical_executed_subops=%lu\n",
                reqid, lba, nlb, read_stime, sample_cnt, unique_planes,
                maxlat, sum_wait_ns / sample_cnt, max_wait_ns,
                critical_wait_ns, critical_start_ns, critical_finish_ns,
                critical_subidx, critical_ch, critical_lun, critical_pl,
                critical_exec_hops, critical_exec_benefit,
                critical_exec_candidate ? 1 : 0,
                critical_reject_slack_insufficient ? 1 : 0,
                critical_reject_after_noncritical ? 1 : 0,
                critical_blocked_by_noncritical_slack,
                critical_est_sublat, critical_est_start,
                critical_est_finish,
                est_maxlat, est_critical_wait_ns, est_critical_start_ns,
                est_critical_finish_ns, est_critical_subidx, est_critical_ch,
                est_critical_lun, est_critical_pl, exec_enabled ? 1 : 0,
                exec_candidate_subops, exec_bypass_subops, exec_total_hops,
                exec_max_hops, exec_host_improvement ? 1 : 0,
                exec_host_improvement, pred_critical_executed_subops,
                pred_noncritical_executed_subops);

        if (exec_enabled && exec_candidate_subops &&
            zns_read_bypass_summary_log_enabled()) {
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

        if (exec_enabled && global_baseline_maxlat &&
            zns_read_bypass_summary_log_enabled()) {
            ftl_log("ZNS_READ_GLOBAL_BASELINE,reqid=%lu,slba=%lu,nlb=%u,"
                    "read_stime_ns=%lu,subops=%lu,"
                    "global_baseline_maxlat_ns=%lu,exec_maxlat_ns=%lu,"
                    "local_baseline_maxlat_ns=%lu,total_hops=%lu,"
                    "bypass_subops=%lu\n",
                    reqid, lba, nlb, read_stime, sample_cnt,
                    global_baseline_maxlat, maxlat, est_maxlat,
                    exec_total_hops, exec_bypass_subops);
        }

        if (!exec_enabled && !mixed_bypass_exec_enabled) {
            /*
             * baseline 模式下，真实 read 已经走完原始调度，再把当前 read
             * 记入队列，继续支持后续请求的 dry-run opportunity analysis。
             */
            for (i = 0; i < sample_cnt; i++) {
                zns_read_bypass_remember_baseline_read(reqid, i, &samples[i],
                                                       read_stime, maxlat);
                if (row_tracking_enabled && !row_scheduled[i]) {
                    zns_mixed_diag_remember_read(reqid, i, &samples[i],
                                                 read_stime, maxlat);
                }
            }
        } else if (exec_enabled) {
            /*
             * execute 模式下，当前请求刚排入队列时只拿到了临时 deadline。
             * 现在 maxlat 已经确定，必须在 zns_read() 返回前把当前请求所有
             * entry 的 deadline 收紧成真正会承诺给 host 的完成时间。
             */
            zns_read_bypass_finalize_deadline(reqid, samples, sample_cnt,
                                              read_stime, maxlat);
        } else if (mixed_bypass_exec_enabled) {
            zns_mixed_diag_finalize_read_deadline(reqid, samples, sample_cnt,
                                                  read_stime, maxlat);
        }

        /*
         * 请求级汇总日志：unique_planes 是 FEMU/ZNS 里的等价指标，用来表示
         * 一条读取请求实际覆盖了多少个内部闪存平面资源。
         */
        ftl_log("ZNS_READ_SLACK,reqid=%lu,slba=%lu,nlb=%u,read_stime_ns=%lu,"
                "subops=%lu,unique_planes=%lu,minlat_ns=%lu,maxlat_ns=%lu,"
                "avg_sublat_ns=%lu,total_slack_ns=%lu,avg_slack_ns=%lu,"
                "max_slack_ns=%lu\n",
                reqid, lba, nlb, read_stime, sample_cnt, unique_planes,
                minlat, maxlat, sumlat / sample_cnt, total_slack,
                total_slack / sample_cnt, max_slack);

        /*
         * 请求级在线估计日志。
         *
         * 这行把一整条 host read 的 baseline 估计结果和当前 execute 调度
         * 结果放在一起。进入 bypass execute 后，*_err 字段不再只表示
         * estimator 误差，也会包含当前 read 因绕行而获得的调度收益。
         */
        if (zns_detail_log_enabled()) {
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
                    total_sublat_err / sample_cnt,
                    total_slack_err / sample_cnt);
        }
    }

    g_free(row_scheduled);
    g_free(samples);

    return maxlat;
}

/* 把一个 write cache 槽位里的 LPN 刷到 NAND；这里才真正产生 NAND_WRITE。 */
static uint64_t zns_wc_flush(struct zns_ssd* zns, int wcidx, int type,
                             uint64_t stime, uint32_t zone_idx)
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
    bool row_diag_enabled = zns_row_diag_enabled();
    bool row_tracking_enabled =
        row_diag_enabled || zns_row_exec_enabled() ||
        zns_mixed_bypass_exec_enabled();
    uint64_t flush_id = row_tracking_enabled ? ++zns_wc_flush_next_id : 0;
    uint64_t write_subidx = 0;
    uint64_t flush_max_finish = 0;
    uint64_t total_write_slack = 0;
    uint64_t max_write_slack = 0;
    uint64_t slack_write_subops = 0;
    uint64_t critical_write_subops = 0;
    uint64_t diag_cap = row_tracking_enabled ?
        zns->cache.write_cache[wcidx].used : 0;
    struct zns_wc_write_diag *write_diag = row_tracking_enabled ?
        g_malloc0(sizeof(*write_diag) * diag_cap) : NULL;

    if (row_diag_enabled) {
        ftl_log("ZNS_WC_FLUSH_BEGIN,flush_id=%lu,wcidx=%d,zone_idx=%u,"
                "stime_ns=%lu,used_lpn=%lu,cap_lpn=%lu,type=%d\n",
                flush_id, wcidx, zone_idx, stime,
                zns->cache.write_cache[wcidx].used,
                zns->cache.write_cache[wcidx].cap, type);
    }

    /* i 指向当前还未刷入阵列的第一个 LPN。 */
    i = 0;
    while(i < zns->cache.write_cache[wcidx].used)
    {
        /*
         * 在当前 (channel, lun) 位置上，依次向各个 plane 分配页。
         * 同一轮会把尽可能多的 LPN 条带化写到不同 plane 上。
         */
        for(p = 0;p<zns->num_plane;p++){
            /* 为当前 plane 生成一个新的物理写入位置，block 由本次 write zone 决定。 */
            ppa = get_new_page(zns, zone_idx);

            /*
             * get_new_page() 只填 channel/LUN/block。
             * plane 由当前 p 循环决定，因此这层循环是在跨 plane 条带化写入。
             */
            ppa.g.pl = p;
            uint64_t write_lpns[ZNS_ROW_MAX_WRITE_LPNS];
            uint32_t nr_write_lpns = 0;

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
                    if (nr_write_lpns < G_N_ELEMENTS(write_lpns)) {
                        write_lpns[nr_write_lpns++] = lpn;
                    }

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
            if(ppa.g.V && nr_write_lpns)
            {
                struct nand_cmd swr;
                struct zns_nand_timing_sample timing;
                swr.type = type;
                swr.cmd = NAND_WRITE;

                /*
                 * stime 是触发这次 flush 的主机请求时间。
                 * 本次 flush 内的多个 NAND_WRITE 都用这个时间作到达时间，
                 * 是否排队由各自目标 plane 的 next_plane_avail_time 决定。
                 */
                swr.stime = stime;
                /* 按当前 plane 的可用时间推进一次 NAND program 时延。 */
                if (row_tracking_enabled) {
                    zns_wc_diag_active = true;
                    zns_wc_diag_flush_id = flush_id;
                    zns_wc_diag_write_subidx = write_subidx;
                    sublat = zns_advance_status_sampled(zns, &ppa, &swr,
                                                         &timing);
                    zns_wc_diag_active = false;
                    if (write_subidx < diag_cap) {
                        write_diag[write_subidx].flush_id = flush_id;
                        write_diag[write_subidx].subidx = write_subidx;
                        write_diag[write_subidx].ppa = ppa;
                        write_diag[write_subidx].timing = timing;
                        write_diag[write_subidx].nr_lpns = nr_write_lpns;
                        for (uint32_t li = 0; li < nr_write_lpns; li++) {
                            write_diag[write_subidx].lpns[li] =
                                write_lpns[li];
                        }
                    }
                    flush_max_finish = (timing.finish > flush_max_finish) ?
                        timing.finish : flush_max_finish;
                } else {
                    sublat = zns_advance_status(zns, &ppa, &swr);
                }
                maxlat = (sublat > maxlat) ? sublat : maxlat;
                write_subidx++;
            }
        }
        /*
         * 当前 (channel, lun) 这一轮的各个 plane 都写完后，
         * 再把全局写指针推进到下一个 channel；channel 用完后推进 lun。
         */
        zns_advance_write_pointer(zns);
    }

    if (row_tracking_enabled) {
        for (uint64_t k = 0; k < write_subidx && k < diag_cap; k++) {
            struct zns_wc_write_diag *wd = &write_diag[k];
            struct zns_plane_last_op_diag *last_op =
                zns_get_plane_last_op_diag(&wd->ppa);

            wd->slack = (flush_max_finish > wd->timing.finish) ?
                (flush_max_finish - wd->timing.finish) : 0;
            total_write_slack += wd->slack;
            max_write_slack = (wd->slack > max_write_slack) ?
                wd->slack : max_write_slack;
            if (wd->slack) {
                slack_write_subops++;
            } else {
                critical_write_subops++;
            }

            if (last_op && last_op->cmd == NAND_WRITE &&
                last_op->flush_id == flush_id &&
                last_op->write_subidx == wd->subidx) {
                last_op->write_slack = wd->slack;
            }

            zns_mixed_diag_remember_write(&wd->ppa, wd->flush_id, wd->subidx,
                                          &wd->timing, flush_max_finish,
                                          wd->slack, wd->lpns, wd->nr_lpns);

            if (row_diag_enabled) {
                ftl_log("ZNS_WC_WRITE_SUBOP,flush_id=%lu,subidx=%lu,"
                        "ch=%u,lun=%u,pl=%u,blk=%u,pg=%u,spg=%u,"
                        "req_stime_ns=%lu,old_avail_ns=%lu,start_ns=%lu,"
                        "finish_ns=%lu,delay_ns=%lu,lat_ns=%lu,wait_ns=%lu,"
                        "idle_ns=%lu,flush_max_finish_ns=%lu,"
                        "write_slack_ns=%lu,nr_lpns=%u\n",
                        wd->flush_id, wd->subidx, wd->ppa.g.ch,
                        wd->ppa.g.fc, wd->ppa.g.pl, wd->ppa.g.blk,
                        wd->ppa.g.pg, wd->ppa.g.spg, wd->timing.req_stime,
                        wd->timing.old_avail, wd->timing.start,
                        wd->timing.finish, wd->timing.delay, wd->timing.lat,
                        wd->timing.wait, wd->timing.idle, flush_max_finish,
                        wd->slack, wd->nr_lpns);
            }
        }

        if (row_diag_enabled && write_subidx) {
            ftl_log("ZNS_WC_FLUSH_SUMMARY,flush_id=%lu,wcidx=%d,zone_idx=%u,"
                    "write_subops=%lu,flush_max_finish_ns=%lu,"
                    "critical_write_subops=%lu,slack_write_subops=%lu,"
                    "total_write_slack_ns=%lu,avg_write_slack_ns=%lu,"
                    "max_write_slack_ns=%lu,maxlat_ns=%lu\n",
                    flush_id, wcidx, zone_idx, write_subidx, flush_max_finish,
                    critical_write_subops, slack_write_subops,
                    total_write_slack, total_write_slack / write_subidx,
                    max_write_slack, maxlat);
        }
    }

    g_free(write_diag);
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
    uint32_t zone_idx = req->zns_zone_idx;

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
    int wcidx = zns_get_wcidx(zns, zone_idx);
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
        if(t_used) {
            maxlat = zns_wc_flush(zns, wcidx, USER_IO, req->stime,
                                  zns->cache.write_cache[wcidx].sblk);
        }
        /* 将选中的缓存槽位绑定到当前请求所在的 zone。 */
        zns->cache.write_cache[wcidx].sblk = zone_idx;
    }

    for (lpn = start_lpn; lpn <= end_lpn; lpn++) {
        //如果当前 write cache 满了，先 flush
        if(zns->cache.write_cache[wcidx].used==zns->cache.write_cache[wcidx].cap)
        {
            //打印 flush 日志，写缓存的使用量
            femu_log("[W] flush wc %d (%u/%u)\n",wcidx,(int)zns->cache.write_cache[wcidx].used,(int)zns->cache.write_cache[wcidx].cap);
            sublat = zns_wc_flush(zns, wcidx, USER_IO, req->stime,
                                  zone_idx);
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
