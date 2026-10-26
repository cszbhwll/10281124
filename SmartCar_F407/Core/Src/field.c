/**
 * field.c —— 初赛场地坐标与车道判断
 * 【现场】数字都来自规则图1/图3的标注，到场地后实测再改。
 */
#include "field.h"
#include "robot_config.h"
#include <string.h>

/* ---------- 场地上的固定区域（矩形：x0,y0,x1,y1） ---------- */
typedef struct { float x0, y0, x1, y1; } rect_t;

static const rect_t YELLOW[4] = {           /* 四块淡黄色禁行区 450×450，十字车道宽 400 */
    {  550.0f,  550.0f, 1000.0f, 1000.0f },
    { 1400.0f,  550.0f, 1850.0f, 1000.0f },
    {  550.0f, 1400.0f, 1000.0f, 1850.0f },
    { 1400.0f, 1400.0f, 1850.0f, 1850.0f },
};
static const rect_t ROUGH_ZONE = {  910.0f,    0.0f, 1490.0f,  150.0f };  /* 粗加工区（下边中间） */
static const rect_t STASH_ZONE = {    0.0f,  910.0f,  150.0f, 1490.0f };  /* 暂存区（左边中间） */

/* 原料转盘：圆心在上边线外侧，圆盘有一部分伸进场地 【现场】 */
#define TT_CENTER_X     1200.0f
#define TT_CENTER_Y     2470.0f    /* 2400 + 150 - 80（规则图标注 75~85 伸入） */
#define TT_RADIUS       150.0f
#define TT_MAT_RADIUS   95.0f      /* 物料中心离转盘圆心的半径 【现场】 */

/* 二维码板：竖直装在右边线内侧，中心 Y 约 1200 【现场：位置随机】 */
#define QR_BOARD_X      2400.0f
#define QR_BOARD_Y      1200.0f
#define QR_READ_DIST    250.0f     /* 摄像头离二维码板的距离 */

/* 圆环中心：编号 1..3 【现场：按实际编号方向改】 */
static const point_t ROUGH_RING[3] = { { 1350.0f, 75.0f }, { 1200.0f, 75.0f }, { 1050.0f, 75.0f } };
static const point_t STASH_RING[3] = { { 75.0f, 1350.0f }, { 75.0f, 1200.0f }, { 75.0f, 1050.0f } };

/* 车停在加工区前时，车头离区域边线留的空隙 */
#define ZONE_GAP_MM     25.0f      /* 留够朝向偏 3° 时车角的扫动（148×sin3°≈8mm）和定位误差 */

/* ---------- 车道网格 ---------- */
#define LANE_EDGE  330.0f          /* 外圈车道中心线离场地边 330mm（车宽一半 148 + 让开加工区 150 + 余量） */
static const point_t NODES[NODE_COUNT] = {
    { LANE_EDGE, FIELD_SIZE_MM - LANE_EDGE }, { 1200.0f, FIELD_SIZE_MM - LANE_EDGE }, { FIELD_SIZE_MM - LANE_EDGE, FIELD_SIZE_MM - LANE_EDGE },
    { LANE_EDGE, 1200.0f },                   { 1200.0f, 1200.0f },                   { FIELD_SIZE_MM - LANE_EDGE, 1200.0f },
    { LANE_EDGE, LANE_EDGE },                 { 1200.0f, LANE_EDGE },                 { FIELD_SIZE_MM - LANE_EDGE, LANE_EDGE },
};
static bool s_adj[NODE_COUNT][NODE_COUNT];
static bool s_blocked[NODE_COUNT][NODE_COUNT];

static void link(node_id_t a, node_id_t b) { s_adj[a][b] = s_adj[b][a] = true; }

void field_init(void)
{
    memset(s_adj, 0, sizeof s_adj);
    memset(s_blocked, 0, sizeof s_blocked);
    field_obs_clear();
    /* 横向三条：上外圈、中间十字横道、下外圈 */
    link(NODE_TL, NODE_TC); link(NODE_TC, NODE_TR);
    link(NODE_ML, NODE_C);  link(NODE_C,  NODE_MR);
    link(NODE_BL, NODE_BC); link(NODE_BC, NODE_BR);
    /* 纵向三条：左外圈、中间十字竖道、右外圈 */
    link(NODE_TL, NODE_ML); link(NODE_ML, NODE_BL);
    link(NODE_TC, NODE_C);  link(NODE_C,  NODE_BC);
    link(NODE_TR, NODE_MR); link(NODE_MR, NODE_BR);
}

point_t field_node(node_id_t n) { return NODES[n]; }
bool field_edge_exists(node_id_t a, node_id_t b) { return s_adj[a][b]; }
void field_block_edge(node_id_t a, node_id_t b, bool blocked) { s_blocked[a][b] = s_blocked[b][a] = blocked; }
bool field_edge_blocked(node_id_t a, node_id_t b) { return s_blocked[a][b]; }

node_id_t field_nearest_node(point_t p)
{
    node_id_t best = NODE_C;
    float bd = 1e9f;
    for (int i = 0; i < NODE_COUNT; i++) {
        float d = dist2d(p.x, p.y, NODES[i].x, NODES[i].y);
        if (d < bd) { bd = d; best = (node_id_t)i; }
    }
    return best;
}

/* ---------- 位姿计算 ---------- */
pose_t field_start_pose(int zone)
{
    /* 出发朝向：车头朝场地内侧（-X 方向）。若实际摆放不同，改这里 */
    pose_t p;
    p.x = FIELD_SIZE_MM - 150.0f;
    p.y = (zone == 2) ? 150.0f : (FIELD_SIZE_MM - 150.0f);
    p.th = PI_F;
    return p;
}

pose_t field_qr_pose(void)
{
    pose_t p = { QR_BOARD_X - QR_READ_DIST - ROBOT_HALF_LEN_MM, QR_BOARD_Y, 0.0f };   /* 车头朝 +X 对着二维码板 */
    return p;
}

point_t field_turntable_pick_point(void)
{
    point_t p = { TT_CENTER_X, TT_CENTER_Y - TT_MAT_RADIUS };
    return p;
}

/*
 * 已知"要让机械臂够到的目标点 tgt"和"车头朝向 th"，
 * 求车停在哪里，能让目标点落在车体坐标 (Xt, Yt) 处。
 * 取 Yt = 回转轴的 Y，使目标正好在回转轴的正前方 → 机械臂基本不用转，误差最小。
 */
static pose_t pose_for_target(point_t tgt, float th, float Xt, float Yt)
{
    pose_t p;
    float c = cosf(th), s = sinf(th);
    p.x = tgt.x - (c * Xt - s * Yt);
    p.y = tgt.y - (s * Xt + c * Yt);
    p.th = th;
    return p;
}

pose_t field_turntable_pose(void)
{
    /* 车头朝 +Y；车头离转盘伸入场地的部分留 ZONE_GAP_MM */
    point_t t = field_turntable_pick_point();
    float front_limit = TT_CENTER_Y - TT_RADIUS - ZONE_GAP_MM;       /* 车头最多到这里 */
    float cy = front_limit - ROBOT_HALF_LEN_MM;                       /* 车中心 Y */
    float Xt = t.y - cy;
    return pose_for_target(t, PI_F / 2.0f, Xt, ARM_BASE_Y_MM);
}

point_t field_ring_point(place_zone_t z, int ring_no)
{
    if (ring_no < 1) ring_no = 1;
    if (ring_no > 3) ring_no = 3;
    return (z == ZONE_ROUGH) ? ROUGH_RING[ring_no - 1] : STASH_RING[ring_no - 1];
}

pose_t field_place_pose(place_zone_t z, int ring_no)
{
    point_t t = field_ring_point(z, ring_no);
    if (z == ZONE_ROUGH) {
        /* 粗加工区在下边：车头朝 -Y，车头停在区域上边线(y=150)以外 */
        float cy = ROUGH_ZONE.y1 + ZONE_GAP_MM + ROBOT_HALF_LEN_MM;
        return pose_for_target(t, -PI_F / 2.0f, cy - t.y, ARM_BASE_Y_MM);
    } else {
        /* 暂存区在左边：车头朝 -X，车头停在区域右边线(x=150)以外 */
        float cx = STASH_ZONE.x1 + ZONE_GAP_MM + ROBOT_HALF_LEN_MM;
        return pose_for_target(t, PI_F, cx - t.x, ARM_BASE_Y_MM);
    }
}

/* ---------- 车道判断 ---------- */
static bool in_rect(const rect_t *r, float x, float y)
{
    return x > r->x0 && x < r->x1 && y > r->y0 && y < r->y1;
}

static bool point_ok(float x, float y)
{
    if (x < 0.0f || y < 0.0f || x > FIELD_SIZE_MM || y > FIELD_SIZE_MM) return false;
    for (int i = 0; i < 4; i++) if (in_rect(&YELLOW[i], x, y)) return false;
    if (in_rect(&ROUGH_ZONE, x, y) || in_rect(&STASH_ZONE, x, y)) return false;
    if (dist2d(x, y, TT_CENTER_X, TT_CENTER_Y) < TT_RADIUS) return false;
    return true;
}

/* ---------- 已发现的障碍物（由 obstacle.c 根据测距传感器加进来） ---------- */
#define FIELD_MAX_OBS 12
static point_t s_obs[FIELD_MAX_OBS];
static float   s_obs_w[FIELD_MAX_OBS];         /* 位置估计的累计权重（近处看到的更准，权重大） */
static float   s_obs_s[FIELD_MAX_OBS];         /* 位置不确定范围：测距传感器视场 25°，远处看到时横向可能偏很多 */
static int     s_nobs;
static float   s_obs_clear = OBS_CLEAR_MM;      /* 规划时离障碍物留的余量（被逼到障碍物旁边时临时减小） */
static float   s_sig_k = 1.0f;                  /* 不确定范围的倍数 */

static float seg_dist(point_t c, point_t a, point_t b)
{
    float vx = b.x - a.x, vy = b.y - a.y, L2 = vx * vx + vy * vy;
    float t = L2 > 0 ? ((c.x - a.x) * vx + (c.y - a.y) * vy) / L2 : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return dist2d(c.x, c.y, a.x + vx * t, a.y + vy * t);
}

/* 障碍物离某段车道中心线太近（车身贴着中心线走就会碰到）→ 这段车道不通 */
static void update_blocked_edges(void)
{
    memset(s_blocked, 0, sizeof s_blocked);
    for (int k = 0; k < s_nobs; k++)
        for (int a = 0; a < NODE_COUNT; a++)
            for (int b = a + 1; b < NODE_COUNT; b++)
                if (s_adj[a][b] && seg_dist(s_obs[k], NODES[a], NODES[b]) < ROBOT_HALF_WID_MM + OBS_RADIUS_MM + OBS_CLEAR_MM + s_sig_k * s_obs_s[k])
                    s_blocked[a][b] = s_blocked[b][a] = true;
}

void field_obs_clear(void) { s_nobs = 0; memset(s_blocked, 0, sizeof s_blocked); s_obs_clear = OBS_CLEAR_MM; s_sig_k = 1.0f; }
int field_obs_count(void) { return s_nobs; }
point_t field_obs_get(int i) { return s_obs[i]; }
float field_obs_sigma(int k) { return (k >= 0 && k < s_nobs) ? s_obs_s[k] : 0.0f; }
void field_obs_set_clearance(float mm) { s_obs_clear = mm; }
void field_obs_set_sigma_scale(float f) { if (f != s_sig_k) { s_sig_k = f; update_blocked_edges(); } }

int field_obs_find(point_t c, float within_mm)
{
    int best = -1;
    float bd = within_mm;
    for (int k = 0; k < s_nobs; k++) {
        float d = dist2d(c.x, c.y, s_obs[k].x, s_obs[k].y);
        if (d < bd) { bd = d; best = k; }
    }
    return best;
}

void field_obs_update(int k, point_t c, float weight, float sigma)
{
    if (k < 0 || k >= s_nobs) return;
    if (sigma < s_obs_s[k]) s_obs_s[k] = sigma;
    float w = s_obs_w[k] + weight;
    s_obs[k].x = (s_obs[k].x * s_obs_w[k] + c.x * weight) / w;
    s_obs[k].y = (s_obs[k].y * s_obs_w[k] + c.y * weight) / w;
    s_obs_w[k] = w;
    /* 远处看到时位置不准，可能被当成了两个；越看越准之后两个靠到一起了，就合并成一个 */
    for (int j = 0; j < s_nobs; j++) {
        if (j == k || dist2d(s_obs[j].x, s_obs[j].y, s_obs[k].x, s_obs[k].y) >= OBS_MERGE_MM) continue;
        float W = s_obs_w[j] + s_obs_w[k];
        s_obs[k].x = (s_obs[k].x * s_obs_w[k] + s_obs[j].x * s_obs_w[j]) / W;
        s_obs[k].y = (s_obs[k].y * s_obs_w[k] + s_obs[j].y * s_obs_w[j]) / W;
        s_obs_w[k] = W;
        if (s_obs_s[j] < s_obs_s[k]) s_obs_s[k] = s_obs_s[j];
        s_obs[j] = s_obs[s_nobs - 1]; s_obs_w[j] = s_obs_w[s_nobs - 1]; s_obs_s[j] = s_obs_s[s_nobs - 1];
        s_nobs--;
        break;
    }
    update_blocked_edges();
}

int field_obs_add(point_t c, float weight, float sigma, bool *is_new)
{
    int k = field_obs_find(c, OBS_MERGE_MM);
    if (is_new) *is_new = (k < 0);
    if (k >= 0) { field_obs_update(k, c, weight, sigma); return field_obs_find(c, 1e9f); }   /* 同一个：按权重平均，越看越准 */
    if (s_nobs >= FIELD_MAX_OBS) return -1;
    k = s_nobs++;
    s_obs[k] = c; s_obs_w[k] = weight; s_obs_s[k] = sigma;
    update_blocked_edges();
    return k;
}

/* 测到的东西可能是障碍物吗？障碍物只会在车道上：
   场地外（二维码板、围观的人）、加工区里（已经放好的物料）、转盘附近 都不是 */
bool field_obs_candidate(point_t c)
{
    const float m = 40.0f;
    if (c.x < 20.0f || c.y < 20.0f || c.x > FIELD_SIZE_MM - 20.0f || c.y > FIELD_SIZE_MM - 20.0f) return false;
    if (c.x > ROUGH_ZONE.x0 - m && c.x < ROUGH_ZONE.x1 + m && c.y < ROUGH_ZONE.y1 + m) return false;
    if (c.y > STASH_ZONE.y0 - m && c.y < STASH_ZONE.y1 + m && c.x < STASH_ZONE.x1 + m) return false;
    if (dist2d(c.x, c.y, TT_CENTER_X, TT_CENTER_Y) < TT_RADIUS + 60.0f) return false;
    return true;
}

/* ---------- 已知固定物体的距离（判断测距读数是不是“墙/转盘/加工区里的物料”） ---------- */
static float ray_rect(point_t o, float ux, float uy, const rect_t *r)
{
    float t0 = 0.0f, t1 = 1e9f;
    float lo[2] = { r->x0, r->y0 }, hi[2] = { r->x1, r->y1 }, oo[2] = { o.x, o.y }, uu[2] = { ux, uy };
    for (int k = 0; k < 2; k++) {
        if (fabsf(uu[k]) < 1e-6f) { if (oo[k] < lo[k] || oo[k] > hi[k]) return 1e9f; continue; }
        float a = (lo[k] - oo[k]) / uu[k], b = (hi[k] - oo[k]) / uu[k];
        if (a > b) { float t = a; a = b; b = t; }
        if (a > t0) t0 = a;
        if (b < t1) t1 = b;
        if (t0 > t1) return 1e9f;
    }
    return t0;
}

float field_static_range(point_t o, float dir, float half_fov)
{
    float best = 1e9f;
    for (int r = -3; r <= 3; r++) {                     /* 视场里取 7 条射线，取最近 */
        float a = dir + half_fov * (float)r / 3.0f, ux = cosf(a), uy = sinf(a), t;
        if (ux > 1e-6f) best = fminf(best, (FIELD_SIZE_MM - o.x) / ux);
        if (ux < -1e-6f) best = fminf(best, -o.x / ux);
        if (uy > 1e-6f) best = fminf(best, (FIELD_SIZE_MM - o.y) / uy);
        if (uy < -1e-6f) best = fminf(best, -o.y / uy);
        best = fminf(best, ray_rect(o, ux, uy, &ROUGH_ZONE));
        best = fminf(best, ray_rect(o, ux, uy, &STASH_ZONE));
        float vx = TT_CENTER_X - o.x, vy = TT_CENTER_Y - o.y, tca = vx * ux + vy * uy;
        float d2 = vx * vx + vy * vy - tca * tca;
        if (tca > 0 && d2 < TT_RADIUS * TT_RADIUS) { t = tca - sqrtf(TT_RADIUS * TT_RADIUS - d2); best = fminf(best, t < 0 ? 0 : t); }
    }
    return best;
}

/* 车身（放大 m 后）离已知障碍物是否太近 */
static bool clear_of_obstacles(pose_t p, float m)
{
    for (int k = 0; k < s_nobs; k++) {
        point_t o = field_to_robot(p, s_obs[k].x, s_obs[k].y);
        float dx = fabsf(o.x) - (ROBOT_HALF_LEN_MM + m), dy = fabsf(o.y) - (ROBOT_HALF_WID_MM + m);
        dx = dx > 0 ? dx : 0; dy = dy > 0 ? dy : 0;
        if (hypotf(dx, dy) < OBS_RADIUS_MM + s_obs_clear + s_sig_k * s_obs_s[k]) return false;
    }
    return true;
}

bool field_pose_on_lane_m(pose_t p, float m)
{
    if (s_nobs && !clear_of_obstacles(p, m)) return false;
    /* 在（放大后的）车身矩形外框上取采样点逐个检查 */
    float hl = ROBOT_HALF_LEN_MM + m, hw = ROBOT_HALF_WID_MM + m;
    for (int i = 0; i <= 6; i++) {
        for (int j = 0; j <= 6; j++) {
            if (i != 0 && i != 6 && j != 0 && j != 6) continue;   /* 只查外框 */
            float px = -hl + (float)i * (hl / 3.0f);
            float py = -hw + (float)j * (hw / 3.0f);
            point_t f = robot_to_field(p, px, py);
            if (!point_ok(f.x, f.y)) return false;
        }
    }
    return true;
}

bool field_pose_on_lane(pose_t p) { return field_pose_on_lane_m(p, 0.0f); }
