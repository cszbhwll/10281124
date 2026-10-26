/**
 * nav.c —— 走位控制与车道路线规划
 *
 * 控制思路（每 20ms 一次）：
 *   1. 看现在离目标还有多远（位置误差、角度误差）
 *   2. 按"越近越慢"算出速度大小：v = min(最大速度, 比例×距离, √(2×减速度×距离))
 *      最后一项保证来得及刹住车
 *   3. 把场地坐标下的速度方向换算到车体坐标，交给底盘
 */
#include "nav.h"
#include "chassis.h"
#include "field.h"
#include "robot_config.h"
#include "sys.h"
#include "obstacle.h"
#include <string.h>

#define MIN_SPEED_MMPS   22.0f       /* 低于这个速度驱动板可能转不动 */
#define MIN_W_RADPS      DEG2RAD(6.0f)
#define CORNER_SPEED     120.0f      /* 途经点处的拐弯速度 */
#define LOOKAHEAD_MM     120.0f      /* 沿线走：瞄准“线上前方这么远”的点，偏离了会被拉回线上 */

/* ---------------- 后台走位执行器 ----------------
 * 走位被拆成若干"段"（seg）：每段是一个目标位姿，要么"途经不停"，要么"到点停稳"。
 * nav_update() 由 sys_poll 每 20ms 调用一次，按顺序执行这些段。
 * 所以开车的同时，主流程可以让机械臂做别的动作（例如边平移边从载物槽取料），最后用 nav_wait() 等车到位。
 */
#define MAX_SEG 16
typedef struct { pose_t p; point_t a; bool pass; float vmax, v_exit, pass_tol, pos_tol, ang_tol; } seg_t;   /* a = 这一段的起点 */

static seg_t        s_seg[MAX_SEG];
static int          s_nseg, s_iseg;
static bool         s_active;
static nav_result_t s_result = NAV_OK;
static uint32_t     s_seg_t0;
static int          s_settled;
/* 避障：当前是不是在走“规划出来的路线”，目标是哪里（路线被新发现的障碍物挡住时要重新规划） */
static bool         s_route_on;
static pose_t       s_route_target;
static float        s_route_clear = OBS_CLEAR_MM;
static float        s_route_sig = 1.0f;
static int          s_replans;
static bool         s_replan_wait;  /* 路线被挡：先刹停，停稳后再按停下的位置重新规划（边跑边换路线会被惯性带偏） */
static uint32_t     s_replan_t0;
static bool segment_safe_m(pose_t a, pose_t b, float margin);
static void start_route(pose_t target);

static void bg_start(void)
{
    s_iseg = 0;
    s_settled = 0;
    s_seg_t0 = sys_now();
    s_result = NAV_OK;
    s_active = s_nseg > 0;
}

static void add_seg(pose_t p, bool pass, float vmax, float v_exit)
{
    if (s_nseg >= MAX_SEG) return;
    seg_t *g = &s_seg[s_nseg++];
    if (s_nseg >= 2) { g->a.x = s_seg[s_nseg - 2].p.x; g->a.y = s_seg[s_nseg - 2].p.y; }
    else { pose_t c = chassis_pose(); g->a.x = c.x; g->a.y = c.y; }
    g->p = p; g->pass = pass; g->vmax = vmax > 0 ? vmax : NAV_VMAX_MMPS; g->v_exit = v_exit;
    g->pass_tol = (v_exit < 80.0f) ? 15.0f : NAV_PASS_TOL_MM;
    g->pos_tol = NAV_POS_TOL_MM; g->ang_tol = DEG2RAD(NAV_ANG_TOL_DEG);   /* 急转弯的途经点要贴近了再转，避免甩出去 */
}

bool nav_busy(void) { return s_active; }

void nav_cancel(void) { s_active = false; s_nseg = 0; s_route_on = false; s_replan_wait = false; chassis_set_velocity(0, 0, 0); }

/* 剩下的路线（从当前位置开始）是否还安全：新发现的障碍物可能挡在路上 */
static bool route_still_safe(pose_t cur)
{
    field_obs_set_clearance(s_route_clear);
    field_obs_set_sigma_scale(s_route_sig);
    bool ok = true;
    pose_t a = cur;
    for (int i = s_iseg; i < s_nseg && ok; i++) { ok = segment_safe_m(a, s_seg[i].p, 0.0f); a = s_seg[i].p; }
    field_obs_set_clearance(OBS_CLEAR_MM);
    field_obs_set_sigma_scale(1.0f);
    return ok;
}

/* 每个控制周期算一次速度（原理见文件开头） */
void nav_update(void)
{
    if (!s_active) return;
    if (sys_aborted()) { s_result = NAV_ABORTED; nav_cancel(); return; }
    seg_t *g = &s_seg[s_iseg];
    if ((uint32_t)(sys_now() - s_seg_t0) > NAV_TIMEOUT_MS) {
        pose_t p = chassis_pose();
        LOG("nav: 超时 目标(%d,%d) 当前(%d,%d)", (int)g->p.x, (int)g->p.y, (int)p.x, (int)p.y);
        s_result = NAV_TIMEOUT;
        nav_cancel();
        return;
    }
    pose_t p = chassis_pose();

    /* 避障：障碍物地图有变化 → 检查剩下的路线，被挡住就先刹停，停稳后重新规划（会绕到别的车道） */
    if (s_replan_wait) {
        float mvx, mvy, mw;
        chassis_set_velocity(0, 0, 0);
        chassis_measured_velocity(&mvx, &mvy, &mw);
        if ((hypotf(mvx, mvy) > 15.0f || fabsf(mw) > 0.05f) && (uint32_t)(sys_now() - s_replan_t0) < 1200) return;
        s_replan_wait = false;
        start_route(s_route_target);
        if (!s_active) return;
        g = &s_seg[s_iseg];
        p = chassis_pose();
    } else if (s_route_on && obs_take_changed() && !route_still_safe(p)) {
        if (++s_replans > 20) { LOG("避障：重新规划次数太多，停车"); s_result = NAV_NO_PATH; nav_cancel(); return; }
        LOG("避障：前方路线被障碍物挡住，刹车后重新规划");
        s_replan_wait = true;
        s_replan_t0 = sys_now();
        s_seg_t0 = sys_now();
        chassis_set_velocity(0, 0, 0);
        return;
    }

    float ex = g->p.x - p.x, ey = g->p.y - p.y;
    float dist = hypotf(ex, ey);
    float eth = wrap_pi(g->p.th - p.th);

    bool done = false;
    if (g->pass) {
        done = dist < g->pass_tol;                          /* 途经点：到附近就转下一段，不停车 */
    } else if (dist < g->pos_tol && fabsf(eth) < g->ang_tol) {
        chassis_set_velocity(0, 0, 0);
        done = ++s_settled >= 3;                            /* 连续 3 个周期都在容差内才算停稳 */
        if (!done) return;
    } else {
        s_settled = 0;
    }
    if (done) {
        if (++s_iseg >= s_nseg) { s_active = false; s_nseg = 0; s_route_on = false; if (!g->pass) chassis_set_velocity(0, 0, 0); return; }
        s_settled = 0;
        s_seg_t0 = sys_now();
        g = &s_seg[s_iseg];
        ex = g->p.x - p.x; ey = g->p.y - p.y; dist = hypotf(ex, ey); eth = wrap_pi(g->p.th - p.th);
    }

    /* 线速度大小：越近越慢，并保证来得及刹车 */
    float v = g->vmax;
    float dist_b = dist;
    if (g->pass && g->v_exit < 80.0f) {
        /* 急转弯的途经点：电机和控制周期有约 0.1s 的滞后，按“0.1s 之后的位置”算刹车，
           否则会冲过拐点几十毫米（贴着加工区的外圈车道上就压线了） */
        float mvx, mvy, mw;
        chassis_measured_velocity(&mvx, &mvy, &mw);
        dist_b = dist - 0.1f * hypotf(mvx, mvy);
        if (dist_b < 0) dist_b = 0;
    }
    float v_brake = sqrtf(g->v_exit * g->v_exit + 2.0f * NAV_ACC_MMPS2 * 0.8f * dist_b);
    if (v > v_brake) v = v_brake;
    if (!g->pass && v > NAV_KP_POS * dist) v = NAV_KP_POS * dist;
    if (dist > g->pos_tol * 0.5f && v < MIN_SPEED_MMPS) v = MIN_SPEED_MMPS;
    if (!g->pass && dist < g->pos_tol * 0.5f) v = 0;

    /* 角速度 */
    float wmax = DEG2RAD(NAV_WMAX_DPS);
    float w = NAV_KP_ANG * eth;
    float w_brake = sqrtf(2.0f * DEG2RAD(NAV_WACC_DPS2) * 0.8f * fabsf(eth));
    if (fabsf(w) > w_brake) w = copysignf(w_brake, eth);
    w = clampf(w, -wmax, wmax);
    if (fabsf(eth) > g->ang_tol * 0.5f && fabsf(w) < MIN_W_RADPS) w = copysignf(MIN_W_RADPS, eth);
    if (!g->pass && fabsf(eth) < g->ang_tol * 0.5f) w = 0;

    /* 速度方向：不是直接对着终点，而是对着“这段直线上、比车当前位置往前 LOOKAHEAD_MM 的点”。
       这样过弯甩出去一点、或者被推偏了，车会斜着回到规划的直线上，而不是从偏了的位置直接斜插向终点
       （直插时整段都偏在一边，贴着禁行区的车道上就会压线）。离终点很近时再直接对准终点。 */
    float dx = ex, dy = ey;
    {
        float sx = g->p.x - g->a.x, sy = g->p.y - g->a.y, sl = hypotf(sx, sy);
        if (sl > 50.0f && dist > LOOKAHEAD_MM) {
            float ux = sx / sl, uy = sy / sl;
            float s_aim = (p.x - g->a.x) * ux + (p.y - g->a.y) * uy + LOOKAHEAD_MM;
            if (s_aim < sl) { dx = g->a.x + ux * s_aim - p.x; dy = g->a.y + uy * s_aim - p.y; }
        }
    }
    float dl = hypotf(dx, dy);

    /* 场地方向 → 车体方向 */
    float vx_f = dl > 1e-3f ? v * dx / dl : 0.0f;
    float vy_f = dl > 1e-3f ? v * dy / dl : 0.0f;
    /* 边走边转时：速度指令要过约 0.08s 才真正作用到轮子上，那时车已经又转了一点，
       按“那时的朝向”换算到车体坐标，否则整段转弯都往一边偏（实测偏出 90mm） */
    float mvx, mvy, mw;
    chassis_measured_velocity(&mvx, &mvy, &mw);
    float thc = p.th + 0.5f * (mw + w) * 0.08f;
    float c = cosf(thc), s = sinf(thc);
    chassis_set_velocity(c * vx_f + s * vy_f, -s * vx_f + c * vy_f, w);
}

nav_result_t nav_wait(void)
{
    while (s_active) sys_poll();
    return s_result;
}

void nav_start_goto(pose_t target, float vmax)
{
    s_route_on = false;
    s_replan_wait = false;
    s_nseg = 0;
    add_seg(target, false, vmax, 0);
    bg_start();
}

nav_result_t nav_goto(pose_t target, float vmax)
{
    nav_start_goto(target, vmax);
    return nav_wait();
}

nav_result_t nav_goto_precise(pose_t target, float vmax, float pos_tol_mm, float ang_tol_deg)
{
    nav_start_goto(target, vmax);
    s_seg[0].pos_tol = pos_tol_mm;
    s_seg[0].ang_tol = DEG2RAD(ang_tol_deg);
    return nav_wait();
}

nav_result_t nav_move_rel(float dx, float dy, float dth, float vmax)
{
    pose_t p = chassis_pose();
    point_t f = robot_to_field(p, dx, dy);
    pose_t t = { f.x, f.y, wrap_pi(p.th + dth) };
    return nav_goto(t, vmax);
}

nav_result_t nav_turn_to(float th)
{
    pose_t p = chassis_pose();
    p.th = wrap_pi(th);
    return nav_goto(p, NAV_VMAX_MMPS);
}

/* ---------------- 路线规划 ---------------- */

/*
 * 检查从 a 直线走到 b 的过程中车身是否一直在车道内。
 * 每 25mm 取一个位置；如果这一段要转弯，车在这段路上任何地方都可能处于"转了一半"的角度，
 * 所以每个位置都把起止之间的角度每隔 15° 检查一遍（偏保守，但保证不压线）。
 */
static bool segment_safe_m(pose_t a, pose_t b, float margin)
{
    float L = dist2d(a.x, a.y, b.x, b.y);
    int n = (int)(L / 25.0f) + 1;
    float dth = wrap_pi(b.th - a.th);
    int nh = (int)(fabsf(dth) / DEG2RAD(15.0f)) + 1;
    if (fabsf(dth) < DEG2RAD(2.0f)) nh = 0;
    for (int i = 0; i <= n; i++) {
        float t = (float)i / (float)n;
        for (int h = 0; h <= nh; h++) {
            pose_t q = { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                         wrap_pi(a.th + (nh ? dth * (float)h / (float)nh : 0.0f)) };
            if (!field_pose_on_lane_m(q, margin)) return false;
        }
    }
    return true;
}
static bool segment_safe(pose_t a, pose_t b) { return segment_safe_m(a, b, NAV_LANE_MARGIN_MM); }

/* 在 3×3 车道网格上找最短路（Dijkstra，节点只有 9 个，直接暴力） */
static int shortest_path(node_id_t from, node_id_t to, node_id_t out[NODE_COUNT])
{
    float distv[NODE_COUNT];
    int   prev[NODE_COUNT];
    bool  done[NODE_COUNT];
    for (int i = 0; i < NODE_COUNT; i++) { distv[i] = 1e9f; prev[i] = -1; done[i] = false; }
    distv[from] = 0;
    for (int it = 0; it < NODE_COUNT; it++) {
        int u = -1;
        for (int i = 0; i < NODE_COUNT; i++) if (!done[i] && (u < 0 || distv[i] < distv[u])) u = i;
        if (u < 0 || distv[u] >= 1e9f) break;
        done[u] = true;
        for (int v = 0; v < NODE_COUNT; v++) {
            if (!field_edge_exists((node_id_t)u, (node_id_t)v) || field_edge_blocked((node_id_t)u, (node_id_t)v)) continue;
            point_t a = field_node((node_id_t)u), b = field_node((node_id_t)v);
            float d = distv[u] + dist2d(a.x, a.y, b.x, b.y);
            if (d < distv[v]) { distv[v] = d; prev[v] = u; }
        }
    }
    if (distv[to] >= 1e9f) return 0;
    node_id_t rev[NODE_COUNT];
    int n = 0;
    for (int v = to; v >= 0 && n < NODE_COUNT; v = prev[v]) rev[n++] = (node_id_t)v;
    for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
    return n;
}

/*
 * 有障碍物时选起止节点：不能简单取“最近的节点”——最近的节点可能正好在障碍物后面。
 * 改成：从当前位置能安全直线开过去的节点里选起点，从能安全开到目标的节点里选终点，
 * 让“开到起点 + 网格最短路 + 终点开到目标”的总长最短。
 */
static int graph_route(pose_t from, pose_t target, node_id_t out[NODE_COUNT])
{
    point_t fp = { from.x, from.y }, tp = { target.x, target.y };
    if (field_obs_count() == 0) return shortest_path(field_nearest_node(fp), field_nearest_node(tp), out);
    float best = 1e9f; int bestk = 0;
    node_id_t tmp[NODE_COUNT];
    for (int s = 0; s < NODE_COUNT; s++) {
        point_t sn = field_node((node_id_t)s);
        pose_t sp = { sn.x, sn.y, from.th };
        if (!segment_safe(from, sp)) continue;
        for (int e = 0; e < NODE_COUNT; e++) {
            point_t en = field_node((node_id_t)e);
            pose_t ep = { en.x, en.y, target.th };
            if (!segment_safe(ep, target)) continue;
            int k = shortest_path((node_id_t)s, (node_id_t)e, tmp);
            if (k == 0) continue;
            float L = dist2d(from.x, from.y, sn.x, sn.y) + dist2d(en.x, en.y, target.x, target.y);
            for (int i = 1; i < k; i++) { point_t a = field_node(tmp[i - 1]), b = field_node(tmp[i]); L += dist2d(a.x, a.y, b.x, b.y); }
            if (L < best) { best = L; bestk = k; memcpy(out, tmp, sizeof tmp); }
        }
    }
    if (bestk == 0) return shortest_path(field_nearest_node(fp), field_nearest_node(tp), out);
    return bestk;
}

#define MAX_WP (NODE_COUNT + 3)

typedef struct { pose_t p; bool rotate_here; } wp_t;

/* 在点 (x,y) 原地从 th_a 转到 th_b，车身扫过的范围是否安全（每 10° 检查一次） */
static bool rotation_safe(float x, float y, float th_a, float th_b)
{
    float d = wrap_pi(th_b - th_a);
    int n = (int)(fabsf(d) / DEG2RAD(10.0f)) + 1;
    for (int i = 0; i <= n; i++) {
        pose_t q = { x, y, wrap_pi(th_a + d * (float)i / (float)n) };
        if (!field_pose_on_lane_m(q, NAV_LANE_MARGIN_MM)) return false;
    }
    return true;
}

/*
 * 路线规划：
 *  1. 车道网格上找最短路：起点 → 节点… → 终点
 *  2. 找一个"能安全原地转身"的点（车身转一圈扫过的范围不压线），在那里停下转到最终朝向；
 *     其余路段都保持朝向不变（麦轮可以横着走、斜着走，不需要边走边转）
 *  3. 能直接连过去的点就省掉
 */
static int plan_route(pose_t from, pose_t target, wp_t wp[MAX_WP])
{
    pose_t raw[MAX_WP];
    int n = 0;
    raw[n++] = from;
    pose_t from_same = from;  from_same.th = target.th;
    bool direct_ok = rotation_safe(from.x, from.y, from.th, target.th) ? segment_safe(from_same, target)
                                                                        : segment_safe(from, target);
    if (!direct_ok) {
        node_id_t path[NODE_COUNT];
        int k = graph_route(from, target, path);
        if (k == 0) return 0;
        for (int i = 0; i < k; i++) { point_t q = field_node(path[i]); pose_t w = { q.x, q.y, 0 }; raw[n++] = w; }
    }
    raw[n++] = target;

    /* 找转身点：第一个能安全原地转身的点；不需要转身时 k_rot = 0 */
    int k_rot = -1;
    bool need_rot = fabsf(wrap_pi(target.th - from.th)) > DEG2RAD(3.0f);
    if (!need_rot) k_rot = 0;
    for (int i = 0; i < n && k_rot < 0; i++)
        if (rotation_safe(raw[i].x, raw[i].y, from.th, target.th)) k_rot = i;
    /* 路线上的点（起点、节点、终点）都不能转身（例如障碍物在十字路口旁边）：
       沿路线每 50mm 找一个能转身的地方，插进去当转身点 */
    for (int i = 0; i < n - 1 && k_rot < 0 && n < MAX_WP; i++) {
        float L = dist2d(raw[i].x, raw[i].y, raw[i + 1].x, raw[i + 1].y);
        for (float d = 50.0f; d < L - 25.0f && k_rot < 0; d += 50.0f) {
            float t = d / L, x = raw[i].x + (raw[i + 1].x - raw[i].x) * t, y = raw[i].y + (raw[i + 1].y - raw[i].y) * t;
            if (!rotation_safe(x, y, from.th, target.th)) continue;
            for (int j = n; j > i + 1; j--) raw[j] = raw[j - 1];
            raw[i + 1].x = x; raw[i + 1].y = y;
            n++;
            k_rot = i + 1;
        }
    }
    if (k_rot < 0) { LOG("nav: 路线上没有能安全转身的地方，只能在终点转"); k_rot = n - 1; }
    for (int i = 0; i < n; i++) raw[i].th = (i < k_rot) ? from.th : target.th;

    /* 如果从转身点到下一个点"边走边转"整段都安全，就不用停车转身（省 1~2 秒） */
    bool rot_moving = false;
    if (need_rot && k_rot < n - 1) {
        pose_t a = raw[k_rot]; a.th = from.th;
        /* 这一段要够长，车开完这段时已经转完；否则剩下的转动会带到下一段（那段没按转动检查过） */
        float L = dist2d(a.x, a.y, raw[k_rot + 1].x, raw[k_rot + 1].y);
        float t_rot = fabsf(wrap_pi(target.th - from.th)) / DEG2RAD(NAV_WMAX_DPS) + 0.4f;
        if (L > NAV_VMAX_MMPS * t_rot && segment_safe(a, raw[k_rot + 1])) { rot_moving = true; raw[k_rot].th = from.th; }
    }

    /* 贪心省点：在转身点前后两段里，分别尽量直接连到最远的安全点（不能跨过转身点） */
    int m = 0;
    wp[m].p = from; wp[m].rotate_here = (need_rot && !rot_moving && k_rot == 0); m++;
    int i = 0;
    while (i < n - 1) {
        int lim = (i < k_rot) ? k_rot : n - 1;
        int j = lim;
        while (j > i + 1 && !segment_safe(raw[i], raw[j])) j--;
        wp[m].p = raw[j];
        wp[m].rotate_here = (need_rot && !rot_moving && j == k_rot && k_rot > 0);
        if (wp[m].rotate_here) wp[m].p.th = from.th;     /* 到这个点时还是原来的朝向，停下后再转 */
        m++;
        i = j;
    }
    return m;
}

#define APPROACH_MM  90.0f     /* 最后一段：在目标前方这么远处先摆正朝向，再直线开进去 */

static void start_route(pose_t target)
{
    pose_t cur = chassis_pose();
    s_nseg = 0;
    /* 情况1：已经在目标附近且朝向一致（如沿加工区从一个圆环挪到下一个）→ 直接平移过去 */
    if (dist2d(cur.x, cur.y, target.x, target.y) < 400.0f &&
        fabsf(wrap_pi(cur.th - target.th)) < DEG2RAD(5.0f) && segment_safe_m(cur, target, 0.0f)) {
        add_seg(target, false, NAV_VMAX_MMPS, 0);
        bg_start();
        s_route_on = true; s_route_target = target; s_route_clear = OBS_CLEAR_MM; s_route_sig = 1.0f;
        return;
    }
    /*
     * 情况2：目标贴着禁行区（加工区、转盘、二维码板前），先规划到"预备点"：
     * 目标后方 APPROACH_MM、朝向已经摆正；最后一段只直线前进，不转弯，避免车角扫进禁行区。
     */
    pose_t pre = target;
    pre.x -= cosf(target.th) * APPROACH_MM;
    pre.y -= sinf(target.th) * APPROACH_MM;
    bool use_pre = !field_pose_on_lane_m(target, NAV_LANE_MARGIN_MM) && segment_safe_m(pre, target, 0.0f);
    pose_t goal = use_pre ? pre : target;

    wp_t wp[MAX_WP];
    s_route_clear = OBS_CLEAR_MM; s_route_sig = 1.0f;
    int n = plan_route(cur, goal, wp);
    if (n == 0 && field_obs_count() > 0) {
        /* 找不到路，逐步放宽再试：
         *  1. 余量减到 5mm（车可能已经贴近障碍物了——看到得晚，先离开这里）；
         *  2. 远处看到的障碍物位置不准，规划时按“可能在的整个范围”都避开，可能把路全堵了：
         *     改成只避开一半范围、再到只避开估计位置。开过去的路上会离得更近、看得更准，地图随时修正。 */
        static const float CLR[3] = { 5.0f, 5.0f, 5.0f }, SIG[3] = { 1.0f, 0.5f, 0.0f };
        for (int k = 0; k < 3 && n == 0; k++) {
            s_route_clear = CLR[k]; s_route_sig = SIG[k];
            field_obs_set_clearance(CLR[k]);
            field_obs_set_sigma_scale(SIG[k]);
            n = plan_route(cur, goal, wp);
            if (n) LOG("避障：放宽到 余量%dmm、位置误差按%d%%算 后找到路线", (int)CLR[k], (int)(SIG[k] * 100));
        }
        field_obs_set_clearance(OBS_CLEAR_MM);
        field_obs_set_sigma_scale(1.0f);
    }
#ifdef NAV_DEBUG
    for (int i = 0; i < n; i++) LOG("nav: 路线点%d (%d,%d,%d°)%s%s", i, (int)wp[i].p.x, (int)wp[i].p.y, (int)RAD2DEG(wp[i].p.th),
                                    wp[i].rotate_here ? " 在此转身" : "", use_pre && i == n - 1 ? " 预备点" : "");
#endif
    if (n == 0) {
        if (field_obs_count() > 0) {           /* 被障碍物挡死了：不能硬闯，停在原地 */
            LOG("避障：去 (%d,%d) 的路都被障碍物挡住了，停车", (int)target.x, (int)target.y);
            s_active = false; s_route_on = false;
            chassis_set_velocity(0, 0, 0);
            s_result = NAV_NO_PATH;
            return;
        }
        LOG("nav: 没有可走的路线，尝试直接过去");
        add_seg(target, false, NAV_VMAX_MMPS, 0);
        bg_start();
        return;
    }
    if (wp[0].rotate_here) { pose_t r = cur; r.th = goal.th; add_seg(r, false, NAV_VMAX_MMPS, 0); }
    for (int i = 1; i < n; i++) {
        bool last = (i == n - 1);
        if (last || wp[i].rotate_here) {
            add_seg(wp[i].p, false, NAV_VMAX_MMPS, 0);
            if (wp[i].rotate_here) { pose_t r = wp[i].p; r.th = goal.th; add_seg(r, false, NAV_VMAX_MMPS, 0); }
        } else {
            /* 拐弯处减速：转角 20°~45° 以 CORNER_SPEED 经过；超过 45° 几乎停下再转，防止过弯甩出车道 */
            float a1 = atan2f(wp[i].p.y - wp[i - 1].p.y, wp[i].p.x - wp[i - 1].p.x);
            float a2 = atan2f(wp[i + 1].p.y - wp[i].p.y, wp[i + 1].p.x - wp[i].p.x);
            float turn = fabsf(wrap_pi(a2 - a1));
            float k = 1.0f - turn / DEG2RAD(20.0f);
            if (k < 0) k = 0;
            float v_exit = CORNER_SPEED + (NAV_VMAX_MMPS - CORNER_SPEED) * k;
            if (turn > DEG2RAD(45.0f)) v_exit = 50.0f;      /* 急转弯：几乎停下再转 */
            add_seg(wp[i].p, true, NAV_VMAX_MMPS, v_exit);
        }
    }
    if (use_pre) add_seg(target, false, 200.0f, 0);
    bg_start();
    s_route_on = true;
    s_route_target = target;
}

void nav_start_route(pose_t target)
{
    s_replans = 0;
    s_replan_wait = false;
    start_route(target);
}

nav_result_t nav_route_to(pose_t target)
{
    nav_start_route(target);
    return nav_wait();
}

uint32_t nav_estimate_route_ms(pose_t target) { return nav_estimate_path_ms(chassis_pose(), target); }

uint32_t nav_estimate_path_ms(pose_t from, pose_t target)
{
    wp_t wp[MAX_WP];
    int n = plan_route(from, target, wp);
    if (n == 0 && field_obs_count() > 0) {          /* 和 start_route 一样：被挡住时按放宽后的路线估计 */
        field_obs_set_clearance(5.0f);
        field_obs_set_sigma_scale(0.0f);
        n = plan_route(from, target, wp);
        field_obs_set_clearance(OBS_CLEAR_MM);
        field_obs_set_sigma_scale(1.0f);
    }
    if (n == 0) return 30000;
    float t = 0;
    for (int i = 1; i < n; i++) {
        float L = dist2d(wp[i - 1].p.x, wp[i - 1].p.y, wp[i].p.x, wp[i].p.y);
        t += L / (NAV_VMAX_MMPS * 0.85f) + 0.3f;           /* 系数按模拟实测标定：返程估计 ≈ 实际 +1s */
        if (wp[i].rotate_here || wp[i - 1].rotate_here) t += 1.2f;
    }
    return (uint32_t)(t * 1000.0f) + 1000;
}
