/**
 * mission.c —— 初赛整场流程（按规则第 4 节"现场初赛"逐条实现）
 *
 *  1. 一键启动，离开启停区，到二维码板前读任务码并显示
 *  2. 第一批：到原料转盘，按颜色顺序逐个抓取，每抓一个先放到车上载物槽，再抓下一个
 *  3. 运到粗加工区，按顺序放到指定圆环
 *  4. 再按顺序把粗加工区的三个物料取回车上
 *  5. 运到暂存区，按顺序放到指定圆环（圆环号与粗加工区相同）
 *  6. 回原料区取第二批，同样经过粗加工区放置、取回
 *  7. 第二批在暂存区叠放到第一批"同颜色"物料上（码垛）
 *  8. 回到出发的启停区，屏幕显示抓取/放置正确次数
 *
 *  规则要点："物料没有放到车上不能向下一个区域运行"——本程序每次抓取后都先放进载物槽，
 *  只有三个物料都在槽里（或已放下）才开车去下一个区域。
 */
#include "mission.h"
#include "task_code.h"
#include "field.h"
#include "nav.h"
#include "chassis.h"
#include "arm.h"
#include "vision.h"
#include "obstacle.h"
#include "display.h"
#include "robot_config.h"
#include "sys.h"
#include <string.h>
#include <stdio.h>

static task_t           T;
static mission_result_t *R;
static int              s_zone;
static bool             s_have[3];          /* 载物槽 k 里有没有物料 */
static bool             s_on_rough[3];      /* 本批第 k 个物料是否已放在粗加工区 */
static bool             s_base_ok[4];       /* 暂存区第 n 号环上第一批物料是否放好（第二批码垛要用） */

#define STAGE_EST_MS  14000u    /* 估算：一个阶段（3 次取放）大约要多久，用于时间管理 */

static void stage(const char *name)
{
    LOG("==== %s 开始（已用 %lu.%lus）====", name, (unsigned long)(sys_match_elapsed() / 1000),
        (unsigned long)(sys_match_elapsed() % 1000 / 100));
}

static void update_counts(void)
{
    display_set_counts(R->grabs, R->places);
}

/* 剩余时间够不够再做一个耗时 need_ms 的动作，并且做完还能回家？ */
static bool time_ok(uint32_t need_ms)
{
    uint32_t el = sys_match_elapsed();
    uint32_t back = nav_estimate_route_ms(field_start_pose(s_zone));
    bool ok = el + need_ms + back + MATCH_RETURN_RESERVE_MS < MATCH_TIME_LIMIT_MS;
    if (!ok && !R->aborted_by_time) {
        LOG("mission: 时间不够了（已用 %lus，返程约 %lus），放弃剩余任务直接回家",
            (unsigned long)(el / 1000), (unsigned long)(back / 1000));
        R->aborted_by_time = true;
    }
    return ok;
}

/* 先开到 at 再做 work_ms 的事，做完从 at 回家，时间还够吗？
   （有障碍物时去下一个地方可能要绕路，只按“当前位置回家”估计会低估） */
static bool time_ok_at(pose_t at, uint32_t work_ms)
{
    uint32_t el = sys_match_elapsed();
    uint32_t go = nav_estimate_route_ms(at);
    uint32_t back = nav_estimate_path_ms(at, field_start_pose(s_zone));
    /* 这里“过去”的路程已经单独估计了，余量比 time_ok 少留 2 秒 */
    bool ok = el + go + work_ms + back + MATCH_RETURN_RESERVE_MS - 2000 < MATCH_TIME_LIMIT_MS;
    if (!ok && !R->aborted_by_time) {
        LOG("mission: 时间不够了（已用 %lus，过去约 %lus，再回家约 %lus），放弃剩余任务直接回家",
            (unsigned long)(el / 1000), (unsigned long)(go / 1000), (unsigned long)(back / 1000));
        R->aborted_by_time = true;
    }
    return ok;
}

/* 场地上的点 → 车体坐标（按里程计当前位姿换算，给机械臂用） */
static vec3_t to_arm(point_t field_pt, float z)
{
    point_t r = field_to_robot(chassis_pose(), field_pt.x, field_pt.y);
    vec3_t v = { r.x, r.y, z };
    return v;
}

/* "理想点"：假如车正好停在 ideal_pose，目标点在车体坐标里的位置。相机给的 dx/dy 就是相对这个点的偏差 */
static vec3_t ideal_point(pose_t ideal_pose, point_t field_pt, float z)
{
    point_t r = field_to_robot(ideal_pose, field_pt.x, field_pt.y);
    vec3_t v = { r.x, r.y, z };
    return v;
}

/*
 * 省时间的关键：开车和机械臂同时动。
 *   nav_start_route() 让车在后台开，接着机械臂可以做别的，最后 nav_wait() 等车到位。
 * 但要守规则：跨区域行驶时物料必须都在车上载物槽里，不能夹在手爪里。
 * 所以"边开边取料"只在同一个加工区内、从一个圆环挪到下一个圆环时使用。
 */

/* 开车去某个区域，同时把机械臂收回行车姿态 */
static void drive_to_zone(pose_t target)
{
    nav_start_route(target);
    arm_home();
    nav_wait();
}

/* ---------------- 1. 读二维码 ---------------- */
static bool read_qr(void)
{
    char buf[24];
    const char *err = NULL;
    vision_set_mode(VMODE_QR);
    nav_start_route(field_qr_pose());
    /* 规则：出发后方可自行展开 —— 等车身离开启停区再展开机械臂 */
    pose_t sp = field_start_pose(s_zone);
    while (nav_busy()) {
        pose_t p = chassis_pose();
        if (dist2d(p.x, p.y, sp.x, sp.y) > 350.0f) break;
        sys_poll();
    }
    arm_home();
    nav_wait();

    /* 二维码板位置随机：先原地看，看不到就沿板左右平移搜索 */
    static const float SEARCH_DY[] = { 0, 120, -120, 240, -240, 0 };
    for (unsigned k = 0; k < sizeof SEARCH_DY / sizeof SEARCH_DY[0]; k++) {
        if (k > 0) {
            pose_t p = field_qr_pose();
            p.y += SEARCH_DY[k];
            nav_goto(p, 250);
        }
        uint32_t t0 = sys_now();
        while ((uint32_t)(sys_now() - t0) < QR_TIMEOUT_MS / 3) {
            sys_poll();
            if (vision_get_qr(buf, sizeof buf, 500) && task_parse(buf, &T, &err)) {
                LOG("mission: 任务码 %s", T.text);
                return true;
            }
            if (sys_aborted()) return false;
        }
        if (err) LOG("mission: 收到的任务码无效：%s", err);
    }
    return false;
}

/* ---------------- 2. 转盘取料 ---------------- */
static void turntable_batch(int b)
{
    if (!time_ok(STAGE_EST_MS)) return;
    display_set_status(b == 0 ? "B1 RAW" : "B2 RAW");
    stage(b == 0 ? "第1批 转盘取料" : "第2批 转盘取料");
    vision_set_mode(VMODE_TURNTABLE);
    drive_to_zone(field_turntable_pose());

    for (int i = 0; i < 3; i++) {
        s_have[i] = false;
        int want = T.color[b][i];
        if (!time_ok(6000)) return;
        LOG("mission: 第%d批第%d个，等%s色转到取料点", b + 1, i + 1, color_name(want));

        for (int attempt = 0; attempt < 2 && !s_have[i]; attempt++) {
            /* 等目标颜色转过来并停稳 */
            vision_tt_t tt;
            bool seen = false;
            uint32_t t0 = sys_now();
            while ((uint32_t)(sys_now() - t0) < TURNTABLE_WAIT_MAX_MS) {
                sys_poll();
                if (sys_aborted()) return;
                if (vision_get_tt(&tt, 300) && tt.color == want && tt.stable) { seen = true; break; }
            }
            if (!seen) { LOG("mission: 等不到%s色", color_name(want)); break; }

            /* 取料点 = 理想取料点 + 相机测得的偏差（不依赖里程计误差） */
            vec3_t p = ideal_point(field_turntable_pose(), field_turntable_pick_point(), TURNTABLE_TOP_Z_MM);
            p.x += tt.dx;
            p.y += tt.dy;
            int r = arm_pick(p);
            if (r == 0) {
                R->grabs++;
                update_counts();
                s_have[i] = (arm_place_to_slot(i) == 0);   /* 先放到车上，再抓下一个 */
                if (!s_have[i]) LOG("mission: 放进载物槽%d失败", i);
            } else {
                LOG("mission: 抓取失败(%d)，%s", r, attempt == 0 ? "再试一次" : "跳过");
            }
        }
    }
    vision_set_mode(VMODE_IDLE);
}

/*
 * 车到位后，返回圆环中心的车体坐标（给机械臂用）。
 * 相机看到圆环时：用"理想点+偏差"，并顺便用这个已知位置的圆环把里程计校正回来，
 * 这样走得越远误差也不会越积越大。相机看不到时：只能按里程计算。
 */
static vec3_t ring_target(place_zone_t z, int ring, float surface_z)
{
    pose_t pp = field_place_pose(z, ring);
    point_t rf = field_ring_point(z, ring);
    nav_wait();
    vision_ring_t vr;
    uint32_t t0 = sys_now();
    while ((uint32_t)(sys_now() - t0) < 150) sys_poll();      /* 等相机拿到停稳后的新画面 */
    if (vision_get_ring(&vr, 120) && vr.found && fabsf(vr.dx) < 75.0f && fabsf(vr.dy) < 75.0f) {
        vec3_t p = ideal_point(pp, rf, surface_z);
        p.x += vr.dx;
        p.y += vr.dy;
        /*
         * 里程计校正：圆环在车体坐标 (p.x,p.y)、在场地坐标 rf → 反推车的真实位置。
         * 相机还给了朝向误差 dth 时，连朝向一起校正（否则沿用陀螺仪/里程计的朝向）。
         */
        pose_t cur = chassis_pose();
        float th = vr.has_dth ? wrap_pi(pp.th + DEG2RAD(vr.dth_deg)) : cur.th;
        float c = cosf(th), s = sinf(th);
        pose_t fixed = { rf.x - (c * p.x - s * p.y), rf.y - (s * p.x + c * p.y), th };
        chassis_set_pose(fixed);
        return p;
    }
    return to_arm(rf, surface_z);
}

/*
 * 把车上的物料按顺序放到某个区域的圆环上（粗加工区放置、暂存区放置/码垛共用）
 * 第一个物料：先开到区域、停稳，再从载物槽取（跨区时物料必须在槽里）；
 * 之后的物料：车在区域内平移到下一个圆环的同时，机械臂从载物槽取料。
 */
static void place_all(int b, place_zone_t z, const int rings[3], bool stacking)
{
    bool first = true;
    for (int i = 0; i < 3; i++) {
        if (!s_have[i]) continue;
        int ring = rings[i];
        if (stacking && !s_base_ok[ring]) {
            LOG("mission: %d号环没有第一批同色物料，第二批%s色不能码垛，留在车上", ring, color_name(T.color[1][i]));
            continue;
        }
        if (!time_ok_at(field_place_pose(z, ring), 3500)) return;
        if (first) { drive_to_zone(field_place_pose(z, ring)); first = false; }
        else       nav_start_route(field_place_pose(z, ring));
        int r = arm_pick_from_slot(i);                    /* 与平移同时进行 */
        s_have[i] = false;
        if (r != 0) { nav_wait(); LOG("mission: 从载物槽%d取料失败", i); continue; }
        float surface = stacking ? MAT_HEIGHT_MM : FLOOR_Z_MM;   /* 码垛：叠在第一批顶上 */
        vec3_t p = ring_target(z, ring, surface);
        if (arm_place(p) == 0) {
            R->places++;
            if (z == ZONE_ROUGH) s_on_rough[i] = true;
            else if (b == 0) s_base_ok[ring] = true;
            else R->stacks++;
            update_counts();
        }
    }
}

/* ---------------- 3. 粗加工区放置 ---------------- */
static void rough_place(int b)
{
    if (!time_ok(STAGE_EST_MS)) return;
    display_set_status(b == 0 ? "B1 ROUGH" : "B2 ROUGH");
    stage(b == 0 ? "第1批 粗加工放置" : "第2批 粗加工放置");
    vision_set_mode(VMODE_RING);
    memset(s_on_rough, 0, sizeof s_on_rough);
    place_all(b, ZONE_ROUGH, T.rough_ring[b], false);
}

/* ---------------- 4. 粗加工区取回 ---------------- */
static void rough_retrieve(int b)
{
    stage(b == 0 ? "第1批 粗加工取回" : "第2批 粗加工取回");
    int pending = -1;                                     /* 夹在手里、还没放进载物槽的物料 */
    for (int i = 0; i < 3; i++) {
        if (!s_on_rough[i]) continue;
        int ring = T.rough_ring[b][i];
        if (!time_ok_at(field_place_pose(ZONE_ROUGH, ring), 3500)) break;
        nav_start_route(field_place_pose(ZONE_ROUGH, ring));
        if (pending >= 0) {                               /* 边平移边把上一个放进载物槽 */
            s_have[pending] = (arm_place_to_slot(pending) == 0);
            pending = -1;
        }
        vec3_t p = ring_target(ZONE_ROUGH, ring, FLOOR_Z_MM);
        if (arm_pick(p) == 0) {
            R->grabs++;
            update_counts();
            pending = i;
        }
        s_on_rough[i] = false;
    }
    if (pending >= 0) s_have[pending] = (arm_place_to_slot(pending) == 0);   /* 离开前必须放回车上 */
    vision_set_mode(VMODE_IDLE);
}

/* ---------------- 5. 暂存区放置 / 码垛 ---------------- */
static void stash_place(int b)
{
    if (!time_ok(b == 0 ? STAGE_EST_MS : 7000)) return;      /* 码垛：哪怕只来得及叠 1 个（15 分）也值得去 */
    display_set_status(b == 0 ? "B1 STASH" : "B2 STACK");
    stage(b == 0 ? "第1批 暂存放置" : "第2批 码垛");
    vision_set_mode(VMODE_RING);
    place_all(b, ZONE_STASH, T.stash_ring[b], b == 1);
    vision_set_mode(VMODE_IDLE);
}

/* ---------------- 主流程 ---------------- */
void mission_run(const mission_cfg_t *cfg, mission_result_t *res)
{
    R = res;
    memset(res, 0, sizeof *res);
    memset(s_base_ok, 0, sizeof s_base_ok);
    memset(s_have, 0, sizeof s_have);
    s_zone = (cfg->start_zone == 2) ? 2 : 1;

    sys_set_abort(false);
    obs_reset();                 /* 障碍物位置每场都不同：清空上一场的地图 */
    sys_match_start();
    chassis_set_pose(field_start_pose(s_zone));
    display_set_status("RUN");
    update_counts();
    LOG("mission: 出发！启停区 %d", s_zone);

    /* 1. 任务码 */
    if (cfg->skip_qr) {
        const char *err = NULL;
        if (!task_parse(cfg->fixed_task, &T, &err)) { LOG("mission: 固定任务码无效：%s", err); return; }
        drive_to_zone(field_qr_pose());
        res->qr_ok = true;
    } else {
        res->qr_ok = read_qr();
    }

    if (res->qr_ok) {
        display_set_task(T.text);
        snprintf(res->task, sizeof res->task, "%s", T.text);
        for (int b = 0; b < 2 && !sys_aborted(); b++) {
            turntable_batch(b);
            rough_place(b);
            rough_retrieve(b);
            stash_place(b);
            if (R->aborted_by_time) break;
        }
    } else {
        LOG("mission: 没读到任务码，直接返程");
    }

    /* 8. 回启停区 */
    if (!sys_aborted()) {
        display_set_status("RETURN");
        stage("返程");
        nav_start_route(field_start_pose(s_zone));
        arm_home();
        nav_result_t r = nav_wait();
        /* 车宽 296mm，启停区只有 300mm：左右各只剩 2mm，普通的到点容差（6mm、1.5°）不够，
           最后慢速再对一次（位置 1mm、角度 0.3°，0.3° 让车身两端偏 0.6mm） */
        if (r == NAV_OK) r = nav_goto_precise(field_start_pose(s_zone), 80.0f, 1.0f, 0.3f);
        chassis_set_velocity(0, 0, 0);
        res->returned = (r == NAV_OK);
    }
    res->time_ms = sys_match_elapsed();
    display_set_status(res->returned ? "DONE" : "STOP");
    update_counts();
    LOG("mission: 结束 用时 %lu.%lus 抓取%d 放置%d 码垛%d %s",
        (unsigned long)(res->time_ms / 1000), (unsigned long)(res->time_ms % 1000 / 100),
        res->grabs, res->places, res->stacks, res->returned ? "已回启停区" : "未回启停区");
}
