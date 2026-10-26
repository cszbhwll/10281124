/**
 * obstacle.c —— 避障：测距读数 → 障碍物地图（原理见 obstacle.h）
 *
 * 位置怎么算：传感器在车体坐标 (x, y)、朝向 a；读到距离 d。
 *   被测到的是障碍物表面，障碍物中心还要再往里 OBS_RADIUS_MM，
 *   所以中心 ≈ 传感器位置 + (d + 25) × 朝向，再用车的当前位姿换算到场地坐标。
 * 误差：VL53L0X 视场角 25°，在 500mm 处横向可能偏 ±110mm；越近越准。
 *   所以同一个障碍物多次看到时按 1/d² 加权平均，近处的读数说了算。
 */
#include "obstacle.h"
#include "tof.h"
#include "field.h"
#include "chassis.h"
#include "sys.h"
#include "robot_config.h"
#include <string.h>

typedef struct { point_t c; int n; } pend_t;     /* 某个传感器“待确认”的点 */
static pend_t s_pend[TOF_COUNT];
static bool   s_changed;
static bool   s_enabled;
static bool   s_mapping;          /* 比赛开始（obs_reset）之后才往地图里加：上电时车的位置还没设好 */

void obs_init(void)
{
    s_enabled = tof_init() > 0;
    memset(s_pend, 0, sizeof s_pend);
}

bool obs_enabled(void) { return s_enabled; }

void obs_reset(void)
{
    field_obs_clear();
    memset(s_pend, 0, sizeof s_pend);
    s_changed = false;
    s_mapping = true;
}

bool obs_take_changed(void) { bool c = s_changed; s_changed = false; return c; }

void obs_update(void)
{
    if (!s_enabled) return;
    tof_poll();
    pose_t p = chassis_pose();
    for (int i = 0; i < tof_count(); i++) {
        int d = tof_get_mm(i, 120);
        if (d <= 0) { s_pend[i].n = 0; continue; }
        const tof_mount_t *m = tof_mount(i);
        float a = DEG2RAD(m->yaw_deg), r = (float)d + OBS_RADIUS_MM;
        /* 先看这个读数能不能用“已知的固定物体”解释：传感器视场有 25°，贴着围挡或加工区走时，
           视场边缘会斜着扫到它们，读数比正前方短，直接按正前方算会以为车道上有东西。
           读数不比固定物体明显近 → 看到的就是它们，不是障碍物 */
        point_t so = robot_to_field(p, m->x, m->y);
        float known = field_static_range(so, p.th + a, DEG2RAD(TOF_FOV_DEG) * 0.5f);
        if ((float)d > known - (30.0f + 0.05f * known)) { s_pend[i].n = 0; continue; }
        point_t c = robot_to_field(p, m->x + cosf(a) * r, m->y + sinf(a) * r);
        if (!field_obs_candidate(c)) { s_pend[i].n = 0; continue; }
        if (!s_mapping) continue;
        float w = 1.0f / ((float)d * (float)d + 1e4f);
        /* 位置可能偏多少：障碍物可能在视场里任何地方，横向最多偏 d×tan(12.5°)，再加测距噪声 */
        float sig = (float)d * tanf(DEG2RAD(TOF_FOV_DEG) * 0.5f) + 10.0f;
        /* 视场 25°：距离 d 处横向可能偏 ±0.22d，所以“是不是同一个”的判断范围随距离放大 */
        float same = OBS_MERGE_MM + 0.25f * (float)d;
        int k0 = field_obs_find(c, same);
        if (k0 >= 0) {                                      /* 已知的障碍物：只更新位置，挪动明显时通知重新检查路线 */
            int n0 = field_obs_count();
            point_t b = field_obs_get(k0);
            float s0 = field_obs_sigma(k0);
            field_obs_update(k0, c, w, sig);
            if (field_obs_count() != n0) { s_changed = true; continue; }
            point_t a2 = field_obs_get(k0);
            if (dist2d(b.x, b.y, a2.x, a2.y) > 15.0f || field_obs_sigma(k0) < s0 - 15.0f) s_changed = true;
            s_pend[i].n = 0;
            continue;
        }
        if (s_pend[i].n > 0 && dist2d(c.x, c.y, s_pend[i].c.x, s_pend[i].c.y) < same) {
            s_pend[i].n++;
            s_pend[i].c.x = (s_pend[i].c.x + c.x) * 0.5f;
            s_pend[i].c.y = (s_pend[i].c.y + c.y) * 0.5f;
        } else {
            s_pend[i].c = c; s_pend[i].n = 1;
        }
        if (s_pend[i].n >= OBS_CONFIRM) {
            bool is_new = false;
            int k = field_obs_add(s_pend[i].c, w, sig, &is_new);
            if (k >= 0 && is_new)
                LOG("避障：发现障碍物 #%d (%d, %d)，传感器 %d 测得 %d mm", k + 1, (int)s_pend[i].c.x, (int)s_pend[i].c.y, i, d);
            s_pend[i].n = 0;
            s_changed = true;
        }
    }
}

void obs_debug(void)
{
    if (!s_enabled) { LOG("测距传感器未启用（没找到多路开关或传感器）"); return; }
    static const char *FACE[] = { "车头左", "车头右", "车尾左", "车尾右", "左侧前", "左侧后", "右侧前", "右侧后" };
    for (int i = 0; i < tof_count(); i++)
        LOG("tof %d %-6s %s  %4d mm", i, i < 8 ? FACE[i] : "", tof_ok(i) ? "正常" : "掉线", tof_get_mm(i, 500));
    for (int k = 0; k < field_obs_count(); k++) LOG("障碍物 #%d (%d, %d)", k + 1, (int)field_obs_get(k).x, (int)field_obs_get(k).y);
}
