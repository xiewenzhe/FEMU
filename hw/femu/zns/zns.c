#include "./zns.h"

#define MIN_DISCARD_GRANULARITY     (4 * KiB)
#define NVME_DEFAULT_ZONE_SIZE      (128 * MiB)
#define NVME_DEFAULT_MAX_AZ_SIZE    (128 * KiB)

//zone索引快速定位核心工具
static inline uint32_t zns_zone_idx(NvmeNamespace *ns, uint64_t slba)
{
    FemuCtrl *n = ns->ctrl;

    return (n->zone_size_log2 > 0 ? slba >> n->zone_size_log2 : slba / n->zone_size);
}

static inline NvmeZone *zns_get_zone_by_slba(NvmeNamespace *ns, uint64_t slba)
{
    FemuCtrl *n = ns->ctrl;
    uint32_t zone_idx = zns_zone_idx(ns, slba);

    assert(zone_idx < n->num_zones);
    return &n->zone_array[zone_idx];
}

//zone几何参数初始化与合法性校验函数
static int zns_init_zone_geometry(NvmeNamespace *ns, Error **errp)
{
    FemuCtrl *n = ns->ctrl;
    uint64_t zone_size, zone_cap;
    uint32_t lbasz = 1 << zns_ns_lbads(ns);

    if (n->zone_size_bs) {
        zone_size = n->zone_size_bs;
    } else {
        zone_size = NVME_DEFAULT_ZONE_SIZE;
    }

    if (n->zone_cap_bs) {
        zone_cap = n->zone_cap_bs;
    } else {
        zone_cap = zone_size;
    }

    if (zone_cap > zone_size) {
        femu_err("zone capacity %luB > zone size %luB", zone_cap, zone_size);
        return -1;
    }
    if (zone_size < lbasz) {
        femu_err("zone size %luB too small, must >= %uB", zone_size, lbasz);
        return -1;
    }
    if (zone_cap < lbasz) {
        femu_err("zone capacity %luB too small, must >= %uB", zone_cap, lbasz);
        return -1;
    }

    n->zone_size = zone_size / lbasz;
    n->zone_capacity = zone_cap / lbasz;
    n->num_zones = ns->size / lbasz / n->zone_size;

    if (n->max_open_zones > n->num_zones) {
        femu_err("max_open_zones value %u exceeds the number of zones %u",
                 n->max_open_zones, n->num_zones);
        return -1;
    }
    if (n->max_active_zones > n->num_zones) {
        femu_err("max_active_zones value %u exceeds the number of zones %u",
                 n->max_active_zones, n->num_zones);
        return -1;
    }

    if (n->zd_extension_size) {
        if (n->zd_extension_size & 0x3f) {
            femu_err("zone descriptor extension size must be multiples of 64B");
            return -1;
        }
        if ((n->zd_extension_size >> 6) > 0xff) {
            femu_err("zone descriptor extension size is too large");
            return -1;
        }
    }

    return 0;
}

//zone状态初始化核心函数
static void zns_init_zoned_state(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    uint64_t start = 0, zone_size = n->zone_size;
    uint64_t capacity = n->num_zones * zone_size;
    NvmeZone *zone;
    int i;

    n->zone_array = g_new0(NvmeZone, n->num_zones);
    if (n->zd_extension_size) {
        n->zd_extensions = g_malloc0(n->zd_extension_size * n->num_zones);
    }

    QTAILQ_INIT(&n->exp_open_zones);
    QTAILQ_INIT(&n->imp_open_zones);
    QTAILQ_INIT(&n->closed_zones);
    QTAILQ_INIT(&n->full_zones);

    zone = n->zone_array;
    for (i = 0; i < n->num_zones; i++, zone++) {
        if (start + zone_size > capacity) {
            zone_size = capacity - start;
        }
        zone->d.zt = NVME_ZONE_TYPE_SEQ_WRITE;
        zns_set_zone_state(zone, NVME_ZONE_STATE_EMPTY);
        zone->d.za = 0;
        zone->d.zcap = n->zone_capacity;
        zone->d.zslba = start;
        zone->d.wp = start;
        zone->w_ptr = start;
        start += zone_size;
    }

    n->zone_size_log2 = 0;
    if (is_power_of_2(n->zone_size)) {
        n->zone_size_log2 = 63 - clz64(n->zone_size);
    }
}

//zone命名空间标识初始化函数
static void zns_init_zone_identify(FemuCtrl *n, NvmeNamespace *ns, int lba_index)
{
    NvmeIdNsZoned *id_ns_z;

    zns_init_zoned_state(ns);

    id_ns_z = g_malloc0(sizeof(NvmeIdNsZoned));

    /* MAR/MOR are zeroes-based, 0xffffffff means no limit */
    id_ns_z->mar = cpu_to_le32(n->max_active_zones - 1);
    id_ns_z->mor = cpu_to_le32(n->max_open_zones - 1);
    id_ns_z->zoc = 0;
    id_ns_z->ozcs = n->cross_zone_read ? 0x01 : 0x00;

    id_ns_z->lbafe[lba_index].zsze = cpu_to_le64(n->zone_size);
    id_ns_z->lbafe[lba_index].zdes = n->zd_extension_size >> 6; /* Units of 64B */

    n->csi = NVME_CSI_ZONED;
    ns->id_ns.nsze = cpu_to_le64(n->num_zones * n->zone_size);
    ns->id_ns.ncap = ns->id_ns.nsze;
    ns->id_ns.nuse = ns->id_ns.ncap;

    ns->id_ns.noiob = 1;
    /* NvmeIdNs */
    /*
     * The device uses the BDRV_BLOCK_ZERO flag to determine the "deallocated"
     * status of logical blocks. Since the spec defines that logical blocks
     * SHALL be deallocated when then zone is in the Empty or Offline states,
     * we can only support DULBE if the zone size is a multiple of the
     * calculated NPDG.
     */
    if (n->zone_size % (ns->id_ns.npdg + 1)) {
        femu_err("the zone size (%"PRIu64" blocks) is not a multiple of the"
                 "calculated deallocation granularity (%"PRIu16" blocks); DULBE"
                 "support disabled", n->zone_size, ns->id_ns.npdg + 1);
        ns->id_ns.nsfeat &= ~0x4;
    }

    n->id_ns_zoned = id_ns_z;
}

//zone元数据重置函数
static void zns_clear_zone(NvmeNamespace *ns, NvmeZone *zone)
{
    FemuCtrl *n = ns->ctrl;
    uint8_t state;

    zone->w_ptr = zone->d.wp;
    state = zns_get_zone_state(zone);
    if (zone->d.wp != zone->d.zslba || (zone->d.za & NVME_ZA_ZD_EXT_VALID)) {
        if (state != NVME_ZONE_STATE_CLOSED) {
            zns_set_zone_state(zone, NVME_ZONE_STATE_CLOSED);
        }
        zns_aor_inc_active(ns);
        QTAILQ_INSERT_HEAD(&n->closed_zones, zone, entry);
    } else {
        zns_set_zone_state(zone, NVME_ZONE_STATE_EMPTY);
    }
}

// 命名空间停机时的核心清理函数
static void zns_zoned_ns_shutdown(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    NvmeZone *zone, *next;

    QTAILQ_FOREACH_SAFE(zone, &n->closed_zones, entry, next) {
        QTAILQ_REMOVE(&n->closed_zones, zone, entry);
        zns_aor_dec_active(ns);
        zns_clear_zone(ns, zone);
    }
    QTAILQ_FOREACH_SAFE(zone, &n->imp_open_zones, entry, next) {
        QTAILQ_REMOVE(&n->imp_open_zones, zone, entry);
        zns_aor_dec_open(ns);
        zns_aor_dec_active(ns);
        zns_clear_zone(ns, zone);
    }
    QTAILQ_FOREACH_SAFE(zone, &n->exp_open_zones, entry, next) {
        QTAILQ_REMOVE(&n->exp_open_zones, zone, entry);
        zns_aor_dec_open(ns);
        zns_aor_dec_active(ns);
        zns_clear_zone(ns, zone);
    }

    assert(n->nr_open_zones == 0);
}

//停机收尾入口函数
void zns_ns_shutdown(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    if (n->zoned) {
        zns_zoned_ns_shutdown(ns);
    }
}

//资源清理函数
void zns_ns_cleanup(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    if (n->zoned) {
        g_free(n->id_ns_zoned);
        g_free(n->zone_array);
        g_free(n->zd_extensions);
    }
}

//zone状态更新核心函数
static void zns_assign_zone_state(NvmeNamespace *ns, NvmeZone *zone, NvmeZoneState state)
{
    FemuCtrl *n = ns->ctrl;

    if (QTAILQ_IN_USE(zone, entry)) {
        switch (zns_get_zone_state(zone)) {
        case NVME_ZONE_STATE_EXPLICITLY_OPEN:
            QTAILQ_REMOVE(&n->exp_open_zones, zone, entry);
            break;
        case NVME_ZONE_STATE_IMPLICITLY_OPEN:
            QTAILQ_REMOVE(&n->imp_open_zones, zone, entry);
            break;
        case NVME_ZONE_STATE_CLOSED:
            QTAILQ_REMOVE(&n->closed_zones, zone, entry);
            break;
        case NVME_ZONE_STATE_FULL:
            QTAILQ_REMOVE(&n->full_zones, zone, entry);
        default:
            ;
        }
    }

    zns_set_zone_state(zone, state);

    switch (state) {
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        QTAILQ_INSERT_TAIL(&n->exp_open_zones, zone, entry);
        break;
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        QTAILQ_INSERT_TAIL(&n->imp_open_zones, zone, entry);
        break;
    case NVME_ZONE_STATE_CLOSED:
        QTAILQ_INSERT_TAIL(&n->closed_zones, zone, entry);
        break;
    case NVME_ZONE_STATE_FULL:
        QTAILQ_INSERT_TAIL(&n->full_zones, zone, entry);
    case NVME_ZONE_STATE_READ_ONLY:
        break;
    default:
        zone->d.za = 0;
    }
}

/*
 * Check if we can open a zone without exceeding open/active limits.
 * AOR stands for "Active and Open Resources" (see TP 4053 section 2.5).
 */

//资源校验核心函数
static int zns_aor_check(NvmeNamespace *ns, uint32_t act, uint32_t opn)
{
    FemuCtrl *n = ns->ctrl;
    //当前激活数 + 拟新增数 > 最大激活数（超限）
    if (n->max_active_zones != 0 &&
        //返回错误码：激活分区过多 + DNR（Do Not Retry，不可重试）
        n->nr_active_zones + act > n->max_active_zones) {
        return NVME_ZONE_TOO_MANY_ACTIVE | NVME_DNR;
    }
    //当前打开数 + 拟新增数 > 最大打开数（超限）
    if (n->max_open_zones != 0 &&
        n->nr_open_zones + opn > n->max_open_zones) {
        // 返回错误码：打开分区过多 + DNR（不可重试）
        return NVME_ZONE_TOO_MANY_OPEN | NVME_DNR;
    }

    return NVME_SUCCESS;
}

//zone写操作状态校验函数
static uint16_t zns_check_zone_state_for_write(NvmeZone *zone)
{
    uint16_t status;

    switch (zns_get_zone_state(zone)) {
    case NVME_ZONE_STATE_EMPTY:
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
    case NVME_ZONE_STATE_CLOSED:
        status = NVME_SUCCESS;
        break;
    case NVME_ZONE_STATE_FULL:
        status = NVME_ZONE_FULL;
        break;
    case NVME_ZONE_STATE_OFFLINE:
        status = NVME_ZONE_OFFLINE;
        break;
    case NVME_ZONE_STATE_READ_ONLY:
        status = NVME_ZONE_READ_ONLY;
        break;
    default:
        assert(false);
    }

    return status;
}

//zone写操作合法性校验函数
static uint16_t zns_check_zone_write(FemuCtrl *n, NvmeNamespace *ns,
                                     NvmeZone *zone, uint64_t slba,
                                     uint32_t nlb, bool append)
{
    uint16_t status;

    if (unlikely((slba + nlb) > zns_zone_wr_boundary(zone))) {
        status = NVME_ZONE_BOUNDARY_ERROR;
    } else {
        status = zns_check_zone_state_for_write(zone);
    }

    if (status != NVME_SUCCESS) {
    } else {
        assert(zns_wp_is_valid(zone));
        if (append) {
            if (unlikely(slba != zone->d.zslba)) {
                status = NVME_INVALID_FIELD;
            }
            if (zns_l2b(ns, nlb) > (n->page_size << n->zasl)) {
                status = NVME_INVALID_FIELD;
            }
        } else if (unlikely(slba != zone->w_ptr)) {
            status = NVME_ZONE_INVALID_WRITE;
        }
    }

    return status;
}

//zone读状态专项校验函数
static uint16_t zns_check_zone_state_for_read(NvmeZone *zone)
{
    uint16_t status;

    switch (zns_get_zone_state(zone)) {
    case NVME_ZONE_STATE_EMPTY:
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
    case NVME_ZONE_STATE_FULL:
    case NVME_ZONE_STATE_CLOSED:
    case NVME_ZONE_STATE_READ_ONLY:
        status = NVME_SUCCESS;
        break;
    case NVME_ZONE_STATE_OFFLINE:
        status = NVME_ZONE_OFFLINE;
        break;
    default:
        assert(false);
    }

    return status;
}

//zone读操作合法性校验函数
static uint16_t zns_check_zone_read(NvmeNamespace *ns, uint64_t slba, uint32_t nlb)
{
    FemuCtrl *n = ns->ctrl;
    NvmeZone *zone = zns_get_zone_by_slba(ns, slba);
    uint64_t bndry = zns_zone_rd_boundary(ns, zone);
    uint64_t end = slba + nlb;
    uint16_t status;

    status = zns_check_zone_state_for_read(zone);
    if (status != NVME_SUCCESS) {
        ;
    } else if (unlikely(end > bndry)) {
        if (!n->cross_zone_read) {
            status = NVME_ZONE_BOUNDARY_ERROR;
        } else {
            /*
             * Read across zone boundary - check that all subsequent
             * zones that are being read have an appropriate state.
             */
            do {
                zone++;
                status = zns_check_zone_state_for_read(zone);
                if (status != NVME_SUCCESS) {
                    break;
                }
            } while (end > zns_zone_rd_boundary(ns, zone));
        }
    }

    return status;
}

//zone自动状态转换函数
static void zns_auto_transition_zone(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    NvmeZone *zone;

    if (n->max_open_zones &&
        n->nr_open_zones == n->max_open_zones) {
        zone = QTAILQ_FIRST(&n->imp_open_zones);
        if (zone) {
             /* Automatically close this implicitly open zone */
            QTAILQ_REMOVE(&n->imp_open_zones, zone, entry);
            zns_aor_dec_open(ns);
            zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_CLOSED);
        }
    }
}

//zone自动打开前置校验函数
static uint16_t zns_auto_open_zone(NvmeNamespace *ns, NvmeZone *zone)
{
    uint16_t status = NVME_SUCCESS;
    uint8_t zs = zns_get_zone_state(zone);

    //分区当前是空闲（EMPTY）状态 → 首次自动打开，需占用激活+打开资源
    if (zs == NVME_ZONE_STATE_EMPTY) {
        zns_auto_transition_zone(ns);
        status = zns_aor_check(ns, 1, 1);
    //分区当前是关闭（CLOSED）状态 → 仅需占用打开资源
    } else if (zs == NVME_ZONE_STATE_CLOSED) {
        zns_auto_transition_zone(ns);
        status = zns_aor_check(ns, 0, 1);
    }

    return status;
}

//写入操作收尾函数
static void zns_finalize_zoned_write(NvmeNamespace *ns, NvmeRequest *req, bool failed)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)&req->cmd;
    NvmeZone *zone;
    NvmeZonedResult *res = (NvmeZonedResult *)&req->cqe;
    uint64_t slba;
    uint32_t nlb;

    slba = le64_to_cpu(rw->slba);
    nlb = le16_to_cpu(rw->nlb) + 1;
    zone = zns_get_zone_by_slba(ns, slba);

    //同步更新分区协议态WP（d.wp是标准化存储字段，与运行时w_ptr对应）
    zone->d.wp += nlb;

    if (failed) {
        res->slba = 0;
    }

    //写入后WP是否达到分区写入边界
    if (zone->d.wp == zns_zone_wr_boundary(zone)) {
        switch (zns_get_zone_state(zone)) {
        case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        case NVME_ZONE_STATE_EXPLICITLY_OPEN:
            zns_aor_dec_open(ns);
            /* fall through */
        case NVME_ZONE_STATE_CLOSED:
            zns_aor_dec_active(ns);
            /* fall through */
        case NVME_ZONE_STATE_EMPTY:
            zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_FULL);
            /* fall through */
        case NVME_ZONE_STATE_FULL:
            break;
        default:
            assert(false);
        }
    }
}

// Add some function
// --------------------------------

static inline uint64_t zone_slba(FemuCtrl *n, uint32_t zone_idx)
{
    return (zone_idx) * n->zone_size;
}

//zone写入指针（WP）推进函数
static uint64_t zns_advance_zone_wp(NvmeNamespace *ns, NvmeZone *zone, uint32_t nlb)
{
    uint64_t result = zone->w_ptr;
    uint8_t zs;

    //按写入块数（nlb）推进分区写入指针
    zone->w_ptr += nlb;

    //推进后WP是否未达到分区写入边界
    if (zone->w_ptr < zns_zone_wr_boundary(zone)) {
        zs = zns_get_zone_state(zone);
        switch (zs) {
        case NVME_ZONE_STATE_EMPTY:
            zns_aor_inc_active(ns);
            /* fall through */
        case NVME_ZONE_STATE_CLOSED:
            zns_aor_inc_open(ns);
            zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_IMPLICITLY_OPEN);
        }
    }

    return result;
}

struct zns_zone_reset_ctx {
    NvmeRequest *req;
    NvmeZone    *zone;
};

//zone重置异步回调函数
static void zns_aio_zone_reset_cb(NvmeRequest *req, NvmeZone *zone)
{
    NvmeNamespace *ns = req->ns;

    /* FIXME, We always assume reset SUCCESS */
    switch (zns_get_zone_state(zone)) {
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        /* fall through */
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        zns_aor_dec_open(ns);
        /* fall through */
    case NVME_ZONE_STATE_CLOSED:
        zns_aor_dec_active(ns);
        /* fall through */
    case NVME_ZONE_STATE_FULL:
        zone->w_ptr = zone->d.zslba;
        zone->d.wp = zone->w_ptr;
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_EMPTY);
    default:
        break;
    }

#if 0
    FemuCtrl *n = ns->ctrl;
    int ch, lun;
    struct zns_ssd *zns = n->zns;
    uint64_t num_ch = zns->num_ch;
    uint64_t num_lun = zns->num_lun;

    struct ppa ppa;
    for (ch = 0; ch < num_ch; ch++) {
        for (lun = 0; lun < num_lun; lun++) {
            ppa.g.ch = ch;
            ppa.g.fc = lun;
            ppa.g.blk = zns_zone_idx(ns, zone->d.zslba);
            //FIXME: no erase
        }
    }
#endif
}

typedef uint16_t (*op_handler_t)(NvmeNamespace *, NvmeZone *, NvmeZoneState,
                                 NvmeRequest *);

enum NvmeZoneProcessingMask {
    NVME_PROC_CURRENT_ZONE    = 0,
    NVME_PROC_OPENED_ZONES    = 1 << 0,
    NVME_PROC_CLOSED_ZONES    = 1 << 1,
    NVME_PROC_READ_ONLY_ZONES = 1 << 2,
    NVME_PROC_FULL_ZONES      = 1 << 3,
};

//zone打开操作函数
static uint16_t zns_open_zone(NvmeNamespace *ns, NvmeZone *zone,
                              NvmeZoneState state, NvmeRequest *req)
{
    uint16_t status;

    switch (state) {
    case NVME_ZONE_STATE_EMPTY:
        status = zns_aor_check(ns, 1, 0);
        if (status != NVME_SUCCESS) {
            return status;
        }
        zns_aor_inc_active(ns);
        /* fall through */
    case NVME_ZONE_STATE_CLOSED:
        status = zns_aor_check(ns, 0, 1);
        if (status != NVME_SUCCESS) {
            if (state == NVME_ZONE_STATE_EMPTY) {
                zns_aor_dec_active(ns);
            }
            return status;
        }
        zns_aor_inc_open(ns);
        /* fall through */
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_EXPLICITLY_OPEN);
        /* fall through */
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        return NVME_SUCCESS;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }
}

//zone关闭函数
static uint16_t zns_close_zone(NvmeNamespace *ns, NvmeZone *zone,
                               NvmeZoneState state, NvmeRequest *req)
{
    switch (state) {
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        /* fall through */
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        zns_aor_dec_open(ns);
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_CLOSED);
        /* fall through */
    case NVME_ZONE_STATE_CLOSED:
        return NVME_SUCCESS;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }
}

//zone完成操作函数
static uint16_t zns_finish_zone(NvmeNamespace *ns, NvmeZone *zone,
                                NvmeZoneState state, NvmeRequest *req)
{
    switch (state) {
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        /* fall through */
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        zns_aor_dec_open(ns);
        /* fall through */
    case NVME_ZONE_STATE_CLOSED:
        zns_aor_dec_active(ns);
        /* fall through */
    case NVME_ZONE_STATE_EMPTY:
        zone->w_ptr = zns_zone_wr_boundary(zone);
        zone->d.wp = zone->w_ptr;
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_FULL);
        /* fall through */
    case NVME_ZONE_STATE_FULL:
        return NVME_SUCCESS;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }
}

//zone重置函数
static uint16_t zns_reset_zone(NvmeNamespace *ns, NvmeZone *zone,
                               NvmeZoneState state, NvmeRequest *req)
{
    switch (state) {
    case NVME_ZONE_STATE_EMPTY:
        return NVME_SUCCESS;
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
    case NVME_ZONE_STATE_CLOSED:
    case NVME_ZONE_STATE_FULL:
        break;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }

    //调用异步重置回调，完成zone重置
    zns_aio_zone_reset_cb(req, zone);

    return NVME_SUCCESS;
}

//zone离线操作专属函数
static uint16_t zns_offline_zone(NvmeNamespace *ns, NvmeZone *zone,
                                 NvmeZoneState state, NvmeRequest *req)
{
    //根据分区当前状态分支处理（仅支持只读→离线、离线→成功）
    switch (state) {
    case NVME_ZONE_STATE_READ_ONLY:
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_OFFLINE);
        /* fall through */
    case NVME_ZONE_STATE_OFFLINE:
        return NVME_SUCCESS;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }
}

//zone扩展区（ZD_EXT）设置函数
static uint16_t zns_set_zd_ext(NvmeNamespace *ns, NvmeZone *zone)
{
    uint16_t status;
    uint8_t state = zns_get_zone_state(zone);

    //仅允许对“空闲状态”的分区设置扩展区（ZNS协议约束）
    if (state == NVME_ZONE_STATE_EMPTY) {
        //校验AOR（Active Open Requirement）：激活zone数量是否超限
        //入参1=要新增激活zone数，0=保留参数
        status = zns_aor_check(ns, 1, 0);
        if (status != NVME_SUCCESS) {
            return status;
        }
        //增加激活zone计数（AOR计数+1）
        zns_aor_inc_active(ns);
        //标记zone扩展区为有效（ZD_EXT_VALID位置1）
        zone->d.za |= NVME_ZA_ZD_EXT_VALID;
        //将zone状态从“空闲”转为“关闭”（设置扩展区后的合规状态）
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_CLOSED);
        return NVME_SUCCESS;
    }

    return NVME_ZONE_INVAL_TRANSITION;
}

//批量zone操作单zone过滤 + 执行函数
static uint16_t zns_bulk_proc_zone(NvmeNamespace *ns, NvmeZone *zone,
                                   enum NvmeZoneProcessingMask proc_mask, //proc_mack:zone处理掩码
                                   op_handler_t op_hndlr, NvmeRequest *req)
{
    uint16_t status = NVME_SUCCESS;
    //获取当前分区的实际状态（如隐式打开、关闭、满等）
    NvmeZoneState zs = zns_get_zone_state(zone);
    bool proc_zone;//标记：该分区是否需要执行操作

    //根据分区状态，匹配处理掩码（proc_mask）
    switch (zs) {
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        proc_zone = proc_mask & NVME_PROC_OPENED_ZONES;
        break;
    case NVME_ZONE_STATE_CLOSED:
        proc_zone = proc_mask & NVME_PROC_CLOSED_ZONES;
        break;
    case NVME_ZONE_STATE_READ_ONLY:
        proc_zone = proc_mask & NVME_PROC_READ_ONLY_ZONES;
        break;
    case NVME_ZONE_STATE_FULL:
        proc_zone = proc_mask & NVME_PROC_FULL_ZONES;
        break;
    default:
        proc_zone = false;
    }

    if (proc_zone) {
        status = op_hndlr(ns, zone, zs, req);
    }

    return status;
}

//zone操作通用执行函数
static uint16_t zns_do_zone_op(NvmeNamespace *ns, NvmeZone *zone,
                               enum NvmeZoneProcessingMask proc_mask,
                               op_handler_t op_hndlr, NvmeRequest *req)
{
    FemuCtrl *n = ns->ctrl;
    NvmeZone *next;
    uint16_t status = NVME_SUCCESS;
    int i;

    //单分区操作（proc_mask=0）：直接执行操作回调
    if (!proc_mask) {
        status = op_hndlr(ns, zone, zns_get_zone_state(zone), req);
    //批量分区操作（proc_mask != 0）：按掩码遍历对应状态的zone
    } else {
        //处理所有“关闭状态”的分区
        if (proc_mask & NVME_PROC_CLOSED_ZONES) {
            //QTAILQ_FOREACH_SAFE：安全遍历链表（next缓存下一个节点，避免当前节点删除后遍历异常）
            QTAILQ_FOREACH_SAFE(zone, &n->closed_zones, entry, next) {
                //批量处理单个分区：调用zns_bulk_proc_zone执行操作回调
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr, req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }
        }
        //处理所有“打开状态”的分区
        if (proc_mask & NVME_PROC_OPENED_ZONES) {
            //遍历“隐式打开”分区链表
            QTAILQ_FOREACH_SAFE(zone, &n->imp_open_zones, entry, next) {
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr,
                                             req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }
            // 遍历“显式打开”分区链表
            QTAILQ_FOREACH_SAFE(zone, &n->exp_open_zones, entry, next) {
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr,
                                             req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }
        }
        // 处理所有“满状态”的分区
        if (proc_mask & NVME_PROC_FULL_ZONES) {
            QTAILQ_FOREACH_SAFE(zone, &n->full_zones, entry, next) {
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr, req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }
        }
        //处理所有“只读状态”的分区（无专属链表，遍历所有分区数组）
        if (proc_mask & NVME_PROC_READ_ONLY_ZONES) {
            for (i = 0; i < n->num_zones; i++, zone++) {
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr,
                                             req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }
        }
    }

out:
    return status;
}

//zone管理命令LBA/索引解析函数
static uint16_t zns_get_mgmt_zone_slba_idx(FemuCtrl *n, NvmeCmd *c,
                                           uint64_t *slba, uint32_t *zone_idx)
{
    NvmeNamespace *ns = &n->namespaces[0]; // 取第一个命名空间
    uint32_t dw10 = le32_to_cpu(c->cdw10);
    uint32_t dw11 = le32_to_cpu(c->cdw11);

    //校验1：控制器是否启用ZNS模式
    if (!n->zoned) {
        return NVME_INVALID_OPCODE | NVME_DNR;
    }

    //拼接64位起始LBA：dw11（高32位）<<32 + dw10（低32位）
    *slba = ((uint64_t)dw11) << 32 | dw10;
    //校验2：LBA是否超出命名空间容量（unlikely优化，标记极少发生）
    if (unlikely(*slba >= ns->id_ns.nsze)) {
        *slba = 0;
        return NVME_LBA_RANGE | NVME_DNR;
    }
    //根据LBA计算对应的分区索引（核心：LBA→分区的映射）
    *zone_idx = zns_zone_idx(ns, *slba);
    //断言：分区索引必须小于总分区数（调试用，防止逻辑错误）
    assert(*zone_idx < n->num_zones);

    return NVME_SUCCESS;
}

//LBA 地址边界校验内联函数
static inline uint16_t zns_check_bounds(NvmeNamespace *ns, uint64_t slba,
                                        uint32_t nlb)
{
    uint64_t nsze = le64_to_cpu(ns->id_ns.nsze);

    if (unlikely(UINT64_MAX - slba < nlb || slba + nlb > nsze)) {
        return NVME_LBA_RANGE | NVME_DNR;
    }

    return NVME_SUCCESS;
}

//DULBE 错误检测的占位函数
static uint16_t zns_check_dulbe(NvmeNamespace *ns, uint64_t slba, uint32_t nlb)
{
    return NVME_SUCCESS;
}

//数据指针映射函数
static uint16_t zns_map_dptr(FemuCtrl *n, size_t len, NvmeRequest *req)
{
    uint64_t prp1, prp2;// 存储转换后的PRP1/PRP2物理地址

    switch (req->cmd.psdt) {// 仅支持PRP格式（ZNS模拟器聚焦PRP，未支持SGL等）
    case NVME_PSDT_PRP://提取PRP1/PRP2地址（NVMe命令中是小端序，转换为主机字节序）
        prp1 = le64_to_cpu(req->cmd.dptr.prp1);
        prp2 = le64_to_cpu(req->cmd.dptr.prp2);

        return nvme_map_prp(&req->qsg, &req->iov, prp1, prp2, len, n);//完成PRP地址到控制器IO向量的映射
    default:
        return NVME_INVALID_FIELD;
    }
}

/*Misao: backend read/write without latency emulation*/
//核心读写/追加写处理函数
static uint16_t zns_nvme_rw(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                           NvmeRequest *req,bool append)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)&req->cmd; // 转换为NVMe读写命令结构体
    uint64_t slba = le64_to_cpu(rw->slba);// 起始LBA（小端序转主机序）
    uint32_t nlb = (uint32_t)le16_to_cpu(rw->nlb) + 1;
    uint64_t data_size = zns_l2b(ns, nlb);// 转换块数为字节数（LBA→Bytes）
    uint64_t data_offset;// 数据在后端存储的偏移量
    uint16_t status;// 返回状态码

    NvmeZone *zone;// 目标ZNS分区
    NvmeZonedResult *res = (NvmeZonedResult *)&req->cqe;
    assert(n->zoned);
    // Fix zone append not working as expected
    // 修复追加写异常：标记请求是否为写操作（WRITE/ZONE_APPEND均为写）
    req->is_write = ((rw->opcode == NVME_CMD_WRITE) || (rw->opcode == NVME_CMD_ZONE_APPEND)) ? 1 : 0;

    //数据大小不超过控制器最大传输尺寸（MDTS）
    status = nvme_check_mdts(n, data_size);
    if (status) {
        goto err;
    }

    status = zns_check_bounds(ns, slba, nlb);
    if (status) {
        goto err;
    }

    if(req->is_write)
    {
        //找到起始LBA对应的ZNS分区
        zone = zns_get_zone_by_slba(ns, slba);
        //校验分区写入规则（如是否顺序写、分区是否满/只读等）
        status = zns_check_zone_write(n, ns, zone, slba, nlb, append);
        if (status) {
            femu_err("Misao check zone write failed with status (%u)\n",status);
            goto err;
        }
        if(append)
        {
             status = zns_auto_open_zone(ns, zone);
             if(status)
             {
                goto err;
             }
             slba = zone->w_ptr;
        }
        res->slba = zns_advance_zone_wp(ns, zone, nlb);
    }
    else//读操作处理
    {
        status = zns_check_zone_read(ns, slba, nlb);
        if (status) {
            goto err;
        }

        /* Misao
           Deallocated or Unwritten Logical Block Error (DULBE) is an option on
           NVMe drives that allows a storage array to deallocate blocks that are
           part of a volume. Deallocating blocks on a drive can greatly reduce
           the time it takes to initialize volumes. In addition, hosts can
           deallocate logical blocks in the volume using the NVMe Dataset
           Management command.
        */
        if (NVME_ERR_REC_DULBE(n->features.err_rec)) { status =
            zns_check_dulbe(ns, slba, nlb); if (status) { goto err; } } }

        data_offset = zns_l2b(ns, slba);
        status = zns_map_dptr(n, data_size, req);
    if (status) {
        goto err;
    }

    req->slba = slba;
    req->status = NVME_SUCCESS;
    req->nlb = nlb;

    backend_rw(n->mbe, &req->qsg, &data_offset, req->is_write);

    if(req->is_write)
    {
        zns_finalize_zoned_write(ns, req, false);
    }

    n->zns->active_zone = zns_zone_idx(ns,slba);
    return NVME_SUCCESS;
err:
    return status | NVME_DNR;
}

//zone管理操作处理函数
static uint16_t zns_zone_mgmt_send(FemuCtrl *n, NvmeRequest *req)
{
    // 1. 提取NVMe命令核心参数
    NvmeCmd *cmd = (NvmeCmd *)&req->cmd;// 从请求中提取NVMe命令结构体
    NvmeNamespace *ns = req->ns;// 目标NVMe命名空间
    // PRP地址：DMA传输的主机内存地址
    uint64_t prp1 = le64_to_cpu(cmd->dptr.prp1);
    uint64_t prp2 = le64_to_cpu(cmd->dptr.prp2);
     // 临时变量声明
    NvmeZone *zone;             // 目标ZNS zone结构体
    uintptr_t *resets;          // zone重置操作的计数指针（用于标记重置状态）
    uint8_t *zd_ext;            // zone扩展区数据指针
    uint32_t dw13 = le32_to_cpu(cmd->cdw13);    // cdw13：zone管理操作参数（操作类型/批量标记）
    uint64_t slba = 0;          // zone起始LBA（逻辑块地址）
    uint32_t zone_idx = 0;      // 目标zone索引
    uint16_t status;            // 函数返回状态（NVME_SUCCESS/错误码）
    uint8_t action;             // zone管理操作类型（打开/关闭/重置等）
    bool all;                   // 是否对所有符合条件的zone执行操作（批量操作标记）
    // 分区处理掩码：标记要处理的分区范围（当前分区/所有关闭分区/所有打开分区等）
    enum NvmeZoneProcessingMask proc_mask = NVME_PROC_CURRENT_ZONE;

    action = dw13 & 0xff;       // 提取dw13低8位：操作类型（如打开/关闭）
    all = dw13 & 0x100;         // 提取dw13第9位：是否批量操作（1=批量，0=单分区）

    req->status = NVME_SUCCESS;// 初始化请求状态为成功

    //非批量操作：解析并校验目标分区的起始LBA和索引
    if (!all) {
        status = zns_get_mgmt_zone_slba_idx(n, cmd, &slba, &zone_idx);
        if (status) {
            return status;
        }
    }

    //校验zone起始LBA的合法性：必须匹配zone数组中该索引zone的起始LBA
    zone = &n->zone_array[zone_idx];
    if (slba != zone->d.zslba) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    //根据操作类型（action）执行对应的zone管理动作
    switch (action) {
    case NVME_ZONE_ACTION_OPEN:// 操作类型：打开zone
        if (all) {
            // 批量操作：处理所有“关闭状态”的zone
            proc_mask = NVME_PROC_CLOSED_ZONES;
        }
        // 调用通用zone操作函数，执行“打开zone”动作
        status = zns_do_zone_op(ns, zone, proc_mask, zns_open_zone, req);
        break;
    case NVME_ZONE_ACTION_CLOSE: // 操作类型：关闭zone
        if (all) {
            // 批量操作：处理所有“打开状态”的zone
            proc_mask = NVME_PROC_OPENED_ZONES;
        }
        // 执行“关闭zone”动作
        status = zns_do_zone_op(ns, zone, proc_mask, zns_close_zone, req);
        break;
    case NVME_ZONE_ACTION_FINISH: // 操作类型：完成zone（标记为只读/满）
        if (all) {
            // 批量操作：处理所有“打开/关闭状态”的zone
            proc_mask = NVME_PROC_OPENED_ZONES | NVME_PROC_CLOSED_ZONES;
        }
        // 执行“完成zone”动作
        status = zns_do_zone_op(ns, zone, proc_mask, zns_finish_zone, req);
        break;
    case NVME_ZONE_ACTION_RESET: // 操作类型：重置zone（清空数据，恢复初始状态）
        resets = (uintptr_t *)&req->opaque; // 关联请求的透明数据（用于计数）

        if (all) {
            // 批量操作：处理所有“打开/关闭/满状态”的zone
            proc_mask = NVME_PROC_OPENED_ZONES | NVME_PROC_CLOSED_ZONES |
                NVME_PROC_FULL_ZONES;
        }
        *resets = 1; // 标记重置操作开始
        // 执行“重置zone”动作
        status = zns_do_zone_op(ns, zone, proc_mask, zns_reset_zone, req);
        (*resets)--; // 标记重置操作结束
        return NVME_SUCCESS; // 重置操作直接返回成功（特殊处理）

    case NVME_ZONE_ACTION_OFFLINE: // 操作类型：标记zone为离线（不可用）
        if (all) {
            // 批量操作：处理所有“只读状态”的zone
            proc_mask = NVME_PROC_READ_ONLY_ZONES;
        }
        // 执行“标记zone离线”动作
        status = zns_do_zone_op(ns, zone, proc_mask, zns_offline_zone, req);
        break;
    case NVME_ZONE_ACTION_SET_ZD_EXT: // 操作类型：设置分区扩展区数据
        // 批量操作/无扩展区配置 → 非法，返回错误
        if (all || !n->zd_extension_size) {
            return NVME_INVALID_FIELD | NVME_DNR;
        }
        // 获取该zone的扩展区数据指针
        zd_ext = zns_get_zd_extension(ns, zone_idx);
        // DMA传输：从主机PRP内存读取扩展区数据，写入zone扩展区
        status = dma_write_prp(n, (uint8_t *)zd_ext, n->zd_extension_size, prp1,
                               prp2);
        if (status) {
            return status; // DMA传输失败，返回错误
        }
        // 应用扩展区数据到zone
        status = zns_set_zd_ext(ns, zone);
        if (status == NVME_SUCCESS) {
            return status; // 设置成功，返回
        }
        break;
    default:
        status = NVME_INVALID_FIELD;
    }

    //错误处理：若操作失败，追加“不重试”标记
    if (status) {
        status |= NVME_DNR;
    }

    return status;
}

//zone状态的筛选匹配函数
static bool zns_zone_matches_filter(uint32_t zafs, NvmeZone *zl)
{
    //获取当前分区的实际状态
    NvmeZoneState zs = zns_get_zone_state(zl);

    //根据筛选条件（zafs）匹配分区状态
    switch (zafs) {
    case NVME_ZONE_REPORT_ALL:
        return true;
    case NVME_ZONE_REPORT_EMPTY:
        return zs == NVME_ZONE_STATE_EMPTY;
    case NVME_ZONE_REPORT_IMPLICITLY_OPEN:
        return zs == NVME_ZONE_STATE_IMPLICITLY_OPEN;
    case NVME_ZONE_REPORT_EXPLICITLY_OPEN:
        return zs == NVME_ZONE_STATE_EXPLICITLY_OPEN;
    case NVME_ZONE_REPORT_CLOSED:
        return zs == NVME_ZONE_STATE_CLOSED;
    case NVME_ZONE_REPORT_FULL:
        return zs == NVME_ZONE_STATE_FULL;
    case NVME_ZONE_REPORT_READ_ONLY:
        return zs == NVME_ZONE_STATE_READ_ONLY;
    case NVME_ZONE_REPORT_OFFLINE:
        return zs == NVME_ZONE_STATE_OFFLINE;
    default:
        return false;
    }
}

//zone状态报告核心处理函数
static uint16_t zns_zone_mgmt_recv(FemuCtrl *n, NvmeRequest *req)
{
    NvmeCmd *cmd = (NvmeCmd *)&req->cmd; //提取NVMe命令结构体
    NvmeNamespace *ns = req->ns;// 目标命名空间
    uint64_t prp1 = le64_to_cpu(cmd->dptr.prp1);//DMA缓冲区首地址
    uint64_t prp2 = le64_to_cpu(cmd->dptr.prp2);//DMA缓冲区次地址
    /* cdw12 is zero-based number of dwords to return. Convert to bytes */
    uint32_t data_size = (le32_to_cpu(cmd->cdw12) + 1) << 2;
    uint32_t dw13 = le32_to_cpu(cmd->cdw13);// cdw13：zone报告类型/筛选条件参数

    uint32_t zone_idx;//zone 索引
    uint32_t zra;//zone报告类型
    uint32_t zrasf;//zone报告筛选条件
    uint32_t partial;//是否允许返回部分报告
    uint64_t max_zones;//最大分区报告条目数
    uint64_t nr_zones = 0;//实际匹配筛选条件的zone总数
    uint16_t status;//返回状态
    uint64_t slba;//zone起始LBA
    uint64_t capacity = zns_ns_nlbas(ns);//命名空间总容量
    NvmeZoneDescr *z;//存储zone状态描述
    NvmeZone *zone;//ZNS分区结构体
    NvmeZoneReportHeader *header;//分区报告头
    void *buf, *buf_p;//临时缓冲区和缓冲区指针
    size_t zone_entry_sz;//单个zone的总大小

    req->status = NVME_SUCCESS;

    //解析命令中的zone起始LBA/索引，校验合法性
    status = zns_get_mgmt_zone_slba_idx(n, cmd, &slba, &zone_idx);
    if (status) {
        return status;
    }

    //校验zone报告类型(ZRA )：仅支持标准/扩展报告
    zra = dw13 & 0xff;// 提取dw13的低8位：报告类型
    if (zra != NVME_ZONE_REPORT && zra != NVME_ZONE_REPORT_EXTENDED) {
        // 不支持的报告类型：返回“无效字段+不重试”错误
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    // 扩展报告需控制器启用扩展区，否则返回错误
    if (zra == NVME_ZONE_REPORT_EXTENDED && !n->zd_extension_size) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    
    //校验分区报告筛选条件（ZRASF）：范围合法性
    zrasf = (dw13 >> 8) & 0xff;// 提取dw13的8~15位：筛选条件
    if (zrasf > NVME_ZONE_REPORT_OFFLINE) {// 超出协议定义的合法范围
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    //校验报告数据大小：至少能容纳报告头
    if (data_size < sizeof(NvmeZoneReportHeader)) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    //校验数据大小不超过控制器最大传输尺寸（MDTS）
    status = nvme_check_mdts(n, data_size);
    if (status) {
        return status;
    }

    //提取“部分报告”标记：dw13的16位：1=允许部分报告，0=不允许
    partial = (dw13 >> 16) & 0x01;

    //计算单个zone报告条目的大小
    zone_entry_sz = sizeof(NvmeZoneDescr);
    if (zra == NVME_ZONE_REPORT_EXTENDED) {
        zone_entry_sz += n->zd_extension_size;
    }

    // 计算缓冲区可容纳的最大zone条目数：(总大小-报告头)/单条目大小
    max_zones = (data_size - sizeof(NvmeZoneReportHeader)) / zone_entry_sz;
    buf = g_malloc0(data_size);

    //第一步遍历：统计符合筛选条件的zone总数
    zone = &n->zone_array[zone_idx];// 从起始zone开始遍历
    for (; slba < capacity; slba += n->zone_size) {
        // 若允许部分报告，且已达最大条目数，终止遍历
        if (partial && nr_zones >= max_zones) {
            break;
        }
        // zone匹配筛选条件 → 计数+1，分区指针后移
        if (zns_zone_matches_filter(zrasf, zone++)) {
            nr_zones++;
        }
    }
    header = (NvmeZoneReportHeader *)buf;
    header->nr_zones = cpu_to_le64(nr_zones);

    //填充每个zone的详细状态到缓冲区
    buf_p = buf + sizeof(NvmeZoneReportHeader);
    // 遍历zone数组，直到遍历完所有zone或缓冲区满
    for (; zone_idx < n->num_zones && max_zones > 0; zone_idx++) {
        zone = &n->zone_array[zone_idx];
        // 仅处理符合筛选条件的zone
        if (zns_zone_matches_filter(zrasf, zone)) {
            z = (NvmeZoneDescr *)buf_p;
            buf_p += sizeof(NvmeZoneDescr);

            z->zt = zone->d.zt;// zone类型（如顺序写分区）
            z->zs = zone->d.zs;// zone状态（如空闲/打开/满）
            z->zcap = cpu_to_le64(zone->d.zcap);//填充zone容量
            z->zslba = cpu_to_le64(zone->d.zslba);//填充zone起始LBA
            z->za = zone->d.za;// zone属性（如是否支持扩展）

            // 填充zone写入指针（WP）
            if (zns_wp_is_valid(zone)) {
                z->wp = cpu_to_le64(zone->d.wp);
            } else {
                z->wp = cpu_to_le64(~0ULL);// 无效WP：置64位全1
            }
            // 扩展报告：若zone扩展属性有效，拷贝扩展数据
            if (zra == NVME_ZONE_REPORT_EXTENDED) {
                if (zone->d.za & NVME_ZA_ZD_EXT_VALID) {
                    // 拷贝扩展区数据到缓冲区
                    memcpy(buf_p, zns_get_zd_extension(ns, zone_idx),
                           n->zd_extension_size);
                }
                buf_p += n->zd_extension_size;// 指针后移（跳过扩展数据）
            }

            max_zones--;// 剩余可填充条目数-1
        }
    }

    //将分区报告数据写入主机指定的PRP内存
    status = dma_read_prp(n, (uint8_t *)buf, data_size, prp1, prp2);

    g_free(buf);

    return status;
}

//判断指定命名空间（Namespace）所属控制器的 CSI（Command Set Identifier）是否支持 NVM（非易失性存储器）相关功能
static inline bool nvme_csi_has_nvm_support(NvmeNamespace *ns)
{
    switch (ns->ctrl->csi) {
    //普通NVM命令集
    case NVME_CSI_NVM:
    //分区式NVM命令集
    case NVME_CSI_ZONED:
        return true;
    }
    //其他CSI返回false
    return false;
}

//管理命令占位处理函数（暂未实现）
static uint16_t zns_admin_cmd(FemuCtrl *n, NvmeCmd *cmd)
{
    switch (cmd->opcode) {
    default:
        return NVME_INVALID_OPCODE | NVME_DNR;
    }
}

//IO 命令分发核心函数
static uint16_t zns_io_cmd(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                           NvmeRequest *req)
{
    //根据NVMe命令的操作码 opcode 分发处理逻辑
    switch (cmd->opcode) {
    //读/写命令：转发到zns_nvme_rw，最后一个参数false表示非追加写    
    case NVME_CMD_READ:
    case NVME_CMD_WRITE:
        return zns_nvme_rw(n, ns, cmd, req,false);
    //zone追加命令：转发到zns_nvme_rw，true表示追加写
    case NVME_CMD_ZONE_APPEND:
        return zns_nvme_rw(n, ns, cmd, req,true);
    //zone管理发送命令：转发到zone管理发送处理函数
    case NVME_CMD_ZONE_MGMT_SEND:
        return zns_zone_mgmt_send(n, req);
    //zone管理接收命令：转发到zone管理接收处理函数
    case NVME_CMD_ZONE_MGMT_RECV:
        return zns_zone_mgmt_recv(n, req);
    }
    //未匹配到支持的命令：返回“无效操作码 + 不重试（DNR）”错误
    return NVME_INVALID_OPCODE | NVME_DNR;
}

//为模拟器设置设备的型号名等字符串
static void zns_set_ctrl_str(FemuCtrl *n)
{
    //ZNS设备的FSID（命名空间标识符）
    static int fsid_zns = 0;
    //定义ZNS控制器的型号名
    const char *zns_mn = "FEMU ZMS-SSD Controller [by Misao]";
    //定义ZNS控制器的序列号
    const char *zns_sn = "vZNSSD";

    //调用NVMe协议接口，将型号名、序列号、FSID绑定到控制器
    nvme_set_ctrl_name(n, zns_mn, zns_sn, &fsid_zns);
}

//PCI 配置参数的初始化函数
static void zns_set_ctrl(FemuCtrl *n)
{
    //uint8_t:固定占用8个二进制位的无符号整数
    uint8_t *pci_conf = n->parent_obj.config;//取出NVMe控制器的PCI配置空间指针

    //设置ZNS控制器的字符串类配置
    zns_set_ctrl_str(n);
    //设置PCI厂商ID为Intel
    pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_INTEL);
    //设置PCI设备ID为0x5845
    pci_config_set_device_id(pci_conf, 0x5845);
}

// Add zns init ch, zns init flash and zns init block
// ----------------------------
//块初始化
static void zns_init_blk(struct zns_blk *blk,int num_blk,int blkidx,int flash_type)
{
    //记录当前块的闪存类型
    blk->nand_type = flash_type;
    //初始化块的下一次可用时间
    blk->next_blk_avail_time = 0;
    //初始化块的写入指针
    blk->page_wp = 0;
}

//plane初始化
static void zns_init_plane(struct zns_plane *plane,int num_blk,int flash_type)
{
    plane->blk = g_malloc0(sizeof(struct zns_blk) * num_blk);
    for (int i = 0; i < num_blk; i++) {
        zns_init_blk(&plane->blk[i],num_blk,i,flash_type);
    }
    plane->next_plane_avail_time = 0;
}

//lun初始化
static void zns_init_fc(struct zns_fc *fc,uint8_t num_plane,uint8_t num_blk,int flash_type)
{
    //为当前LUN分配所有Plane的内存
    fc->plane = g_malloc0(sizeof(struct zns_plane) * num_plane);
    for(int i = 0;i < num_plane;i++)
    {
        zns_init_plane(&fc->plane[i],num_blk,flash_type);
    }
    fc->next_fc_avail_time = 0;
}

//channel初始化
static void zns_init_ch(struct zns_ch *ch, uint8_t num_lun,uint8_t num_plane, uint8_t num_blk,int flash_type)
{
    //zns_fc:ZNS Flash Controller闪存控制器
    ch->fc = g_malloc0(sizeof(struct zns_fc) * num_lun);//一个ch对应多个LUN，先分配所有LUN的内存空间，再逐个初始化。
    for (int i = 0; i < num_lun; i++) {
        zns_init_fc(&ch->fc[i],num_plane,num_blk,flash_type);
    }
    //初始化通道的下一次可用时间
    ch->next_ch_avail_time = 0;
}

//核心初始化函数，从「硬件参数解析」到「存储结构初始化」
static void zns_init_params(FemuCtrl *n)
{
    struct zns_ssd *id_zns;
    int i;

    //分配 ZNS SSD 结构体并填充基础硬件参数
    id_zns = g_malloc0(sizeof(struct zns_ssd));//分配ZNS SSD结构体内存，g_malloc0是GLib 库的内存分配函数，区别于malloc的是会将分配的内存清零，避免野值
    id_zns->num_ch = n->zns_params.zns_num_ch;//通道数，从控制器配置读取
    id_zns->num_lun = n->zns_params.zns_num_lun;//每通道LUN数（对应chip/die）
    id_zns->num_plane = n->zns_params.zns_num_plane;//每LUN平面数
    id_zns->num_blk = n->zns_params.zns_num_blk;//每个平面块数
    id_zns->num_page = n->ns_size/ZNS_PAGE_SIZE/(id_zns->num_ch*id_zns->num_lun*id_zns->num_blk);//每块的页数：总容量/页大小=总页数-->每块页数=总页数/块数
    //LBADS = LBA Data Size，命名空间的 LBA 位数，如 12 表示 LBA 大小为 2^12=4096 字节。
    id_zns->lbasz = 1 << zns_ns_lbads(&n->namespaces[0]); //1 << lbads：等价于 2^lbads，计算出单个 LBA（逻辑块地址）的字节大小（通常 4K/8K）；
    id_zns->flash_type = n->zns_params.zns_flash_type;//闪存类型：SLC/TLC/QLC（单/三/四电平单元）

    id_zns->ch = g_malloc0(sizeof(struct zns_ch) * id_zns->num_ch);//分配通道数组内存
    for (i =0; i < id_zns->num_ch; i++) {
        // 初始化每个通道：传入LUN数、平面数、块数、闪存类型
        zns_init_ch(&id_zns->ch[i], id_zns->num_lun,id_zns->num_plane,id_zns->num_blk,id_zns->flash_type);
    }

    //写入指针初始化
    id_zns->wp.ch = 0;
    id_zns->wp.lun = 0;

    //Misao: init mapping table
    id_zns->l2p_sz = n->ns_size/LOGICAL_PAGE_SIZE;//计算映射表大小：总逻辑页数 = 命名空间大小 / 逻辑页大小
    id_zns->maptbl = g_malloc0(sizeof(struct ppa) * id_zns->l2p_sz);//分配L2P表内存，PPA = 物理页地址，L2P = 逻辑页 to 物理页映射
    //初始化所有映射项为“未映射”
    for (i = 0; i < id_zns->l2p_sz; i++) {
        id_zns->maptbl[i].ppa = UNMAPPED_PPA;
    }

    //Misao: init sram
    //计算编程单元大小：单页大小 × 闪存类型 × 2（2个平面并行）
    id_zns->program_unit = ZNS_PAGE_SIZE*id_zns->flash_type*2; //PAGE_SIZE*flash_type*2 planes
    //计算条带单元大小：编程单元 × 通道数 × LUN数（跨通道/LUN并行）
    id_zns->stripe_unit = id_zns->program_unit*id_zns->num_ch*id_zns->num_lun;
    //初始化写入缓存参数
    id_zns->cache.num_wc = ZNS_DEFAULT_NUM_WRITE_CACHE;//缓存数量
    id_zns->cache.write_cache = g_malloc0(sizeof(struct zns_write_cache) * id_zns->cache.num_wc);//分配写缓存大小
    for(i =0; i < id_zns->cache.num_wc; i++)
    {
        id_zns->cache.write_cache[i].sblk = i;//缓存块编号
        id_zns->cache.write_cache[i].used = 0;//0-->未使用
        id_zns->cache.write_cache[i].cap = (id_zns->stripe_unit/LOGICAL_PAGE_SIZE);//缓存容量（逻辑页数）
        id_zns->cache.write_cache[i].lpns = g_malloc0(sizeof(uint64_t) * id_zns->cache.write_cache[i].cap);//分配缓存的LPN（逻辑页号）数组
    }

    //日志打印
    femu_log("===========================================\n");
    femu_log("|        ZMS HW Configuration()           |\n");      
    femu_log("===========================================\n");
    femu_log("|\tnchnl\t: %lu\t|\tchips per chnl\t: %lu\t|\tplanes per chip\t: %lu\t|\tblks per plane\t: %lu\t|\tpages per blk\t: %lu\t|\n",id_zns->num_ch,id_zns->num_lun,id_zns->num_plane,id_zns->num_blk,id_zns->num_page);
    //femu_log("|\tl2p sz\t: %lu\t|\tl2p cache sz\t: %u\t|\n",id_zns->l2p_sz,id_zns->cache.num_l2p_ent);
    femu_log("|\tprogram unit\t: %lu KiB\t|\tstripe unit\t: %lu KiB\t|\t# of write caches\t: %u\t|\t size of write caches (4KiB)\t: %lu\t|\n",id_zns->program_unit/(KiB),id_zns->stripe_unit/(KiB),id_zns->cache.num_wc,(id_zns->stripe_unit/LOGICAL_PAGE_SIZE));
    femu_log("===========================================\n"); 

    //Misao: use average read latency
    //读延迟
    id_zns->timing.pg_rd_lat[SLC] = SLC_READ_LATENCY_NS;
    id_zns->timing.pg_rd_lat[TLC] = TLC_READ_LATENCY_NS;
    id_zns->timing.pg_rd_lat[QLC] = QLC_READ_LATENCY_NS;

    //Misao: do not support partial programing
    //写延迟
    id_zns->timing.pg_wr_lat[SLC] = SLC_PROGRAM_LATENCY_NS;
    id_zns->timing.pg_wr_lat[TLC] = TLC_PROGRAM_LATENCY_NS;
    id_zns->timing.pg_wr_lat[QLC] = QLC_PROGRAM_LATENCY_NS;

    //Misao: copy from nand.h
    //块擦除延迟
    id_zns->timing.blk_er_lat[SLC] = SLC_BLOCK_ERASE_LATENCY_NS;
    id_zns->timing.blk_er_lat[TLC] = TLC_BLOCK_ERASE_LATENCY_NS;
    id_zns->timing.blk_er_lat[QLC] = QLC_BLOCK_ERASE_LATENCY_NS;

    //关联数据平面启动状态指针（用于判断设备是否就绪）
    id_zns->dataplane_started_ptr = &n->dataplane_started;

    //将初始化完成的ZNS SSD上下文绑定到控制器
    n->zns = id_zns;

    //Misao: init ftl
    //初始化FTL（闪存转换层）：完成L2P映射、垃圾回收、磨损均衡等核心逻辑
    zftl_init(n);
}

//配置分区核心容量和属性参数
static int zns_init_zone_cap(FemuCtrl *n)
{
    //强制要求 n->zns 不为空，即 ZNS 功能已启用
    assert(n->zns);
    struct zns_ssd* zns  = n->zns;//定义局部指针 zns ，指向FemuCtrl,简化参数访问
    n->zoned = true;// 是 ZNS 分区式设备
    n->zasl_bs = NVME_DEFAULT_MAX_AZ_SIZE;//默认最大append size
    n->zone_size_bs = zns->num_ch*zns->num_lun*zns->num_plane*zns->num_page*ZNS_PAGE_SIZE;//计算单个分区的总大小
    n->zone_cap_bs = 0;//初始化分区可用容量（默认 0）
    n->cross_zone_read = false;//禁用跨分区读取
    n->max_active_zones = 0;//设备支持的最大活跃分区数
    n->max_open_zones = 0;//设备支持的最大打开分区数
    n->zd_extension_size = 0;//扩展数据取大小

    return 0;
}

/*ZNS控制器初始化*/
static int zns_start_ctrl(FemuCtrl *n)
{
    /* Coperd: let's fail early before anything crazy happens */
    assert(n->page_size == 4096);

    //ZASL：Zone Append Size Limit，区域追加大小限制
    if (!n->zasl_bs) {
        n->zasl = n->mdts;
    } else {
        if (n->zasl_bs < n->page_size) {
            femu_err("ZASL too small (%dB), must >= 1 page (4K)\n", n->zasl_bs); //用户配置的 ZASL 字节数不能小于设备页大小（4096 字节）
            return -1;
        }
        //clz32 是 “Count Leading Zeros 32-bit” 的缩写，意为统计 32 位无符号整数的前导零个数
        n->zasl = 31 - clz32(n->zasl_bs / n->page_size);
    }

    return 0;
}
/*初始化ZNS
实现：配置控制器和命名空间，使其具备ZNS SSD特性
*/
static void zns_init(FemuCtrl *n, Error **errp)
{
   // 获取第一个命名空间对象。
    NvmeNamespace *ns = &n->namespaces[0];

    // 内部设置 ZNS 控制器状态。
    zns_set_ctrl(n);
    
    // 初始化 ZNS 运行所需的参数（如 Zone 数量/大小）。
    zns_init_params(n);

    // 初始化控制器报告的 ZNS 能力 (ZOCAP)。
    zns_init_zone_cap(n);

    // 关键：计算并初始化所有 Zone 的几何结构和元数据（Zone 状态、写指针）。
    if (zns_init_zone_geometry(ns, errp) != 0) {
        // 几何结构初始化失败，返回。
        return;
    }

    // 配置 NVMe Identify 数据，使主机识别为 ZNS 设备。
    zns_init_zone_identify(n, ns, 0);
}

static void zns_exit(FemuCtrl *n)
{
    /*
     * Release any extra resource (zones) allocated for ZNS mode
     */
}

//FEMU 启用 ZNS 模式的入口点
int nvme_register_znssd(FemuCtrl *n)
{
    n->ext_ops = (FemuExtCtrlOps) {
        .state            = NULL,
        .init             = zns_init,
        .exit             = zns_exit,
        .rw_check_req     = NULL,
        .start_ctrl       = zns_start_ctrl,
        .admin_cmd        = zns_admin_cmd,
        .io_cmd           = zns_io_cmd,
        .get_log          = NULL,
    };

    return 0;
}
