/**
 * sim_main.c —— 在电脑上跑整场比赛
 *
 * 用法：  双击"编译并运行仿真.bat"（Windows） 先重新编译最新代码，再按提示输入任务码和启停区，跑完自动打开浏览器回放
 *         （直接双击 sim.exe 也会自动转去运行这个 .bat，保证用的是最新代码）
 *         ./sim 156+123+516+231 2        指定任务码和启停区（第3个参数：打滑百分比；第4个：1=装陀螺仪；
 *                                        第5个：障碍物摆法编号，0 = 不放障碍物，不写 = 随机）
 *         ./sim --batch                  批量：多种任务码×两个启停区×打滑/噪声，打印汇总
 *
 * 单场运行结束后会生成 replay_<任务码>_zone<区>.html，用浏览器打开就能看动画回放。
 */
#include "sim.h"
#include "sys.h"
#include "field.h"
#include "chassis.h"
#include "motor_drv.h"
#include "arm.h"
#include "vision.h"
#include "display.h"
#include "mission.h"
#include "task_code.h"
#include "robot_config.h"
#include "obstacle.h"
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#endif

static char g_summary[512];
static char g_result[640];   /* 结构化结果（JSON），三维回放页用来画成绩卡片 */

static int run_one(const char *task, int zone, unsigned seed, float slip, float noise, bool verbose, bool vision_ring, bool imu,
                   int n_obs, unsigned obs_seed, int *score_out)
{
    sim_cfg_t c = { seed, zone, "", slip, noise, verbose, vision_ring, imu, n_obs, obs_seed };
    snprintf(c.task, sizeof c.task, "%s", task);
    sim_init(&c);

    sys_init();
    field_init();
    chassis_init();
    vision_init();
    display_init();
    motor_drv_init();
    arm_init();
    obs_init();
    sys_wait_ms(500);

    mission_cfg_t mc = { zone, false, "" };
    mission_result_t r;
    mission_run(&mc, &r);

    sim_report_t rep;
    sim_report(&rep, r.grabs, r.places);

    /* 近似计分（评分规则摘要：读码1、正确抓取2/件、平面放置15/10/7/5/3/1、码垛按底层分、返程2、统计各2） */
    int score = 0;
    score += (r.qr_ok && rep.qr_before_end) ? 1 : 0;
    score += 2 * rep.correct_grabs;
    score += rep.rough_ring_score + rep.stash_ring_score + rep.stack_score;
    score += rep.in_start_zone ? 2 : 0;
    if (!rep.ended_ms) {          /* 正常跑完才有"抓取/放置数量统计正确"的分 */
        score += (r.grabs == rep.correct_grabs) ? 2 : 0;
        score += 2;   /* 放置统计：模拟器里程序计数与实际一致，按满分估 */
    }
    if (score_out) *score_out = score;

    printf("%-16s 区%d 种子%-3u 打滑%.0f%%%s | 用时 %5.1fs %s | 抓%2d 放%2d 码垛%d | 压线%3d | 粗加工%3d 暂存%3d 码垛分%3d | 最大偏差%5.1fmm | 回区%s | 约%3d分\n",
           task, zone, seed, slip * 100.0f, imu ? "+陀螺" : "     ", r.time_ms / 1000.0f, r.time_ms <= MATCH_TIME_LIMIT_MS ? "  " : "超时",
           r.grabs, r.places, r.stacks, rep.lane_violations,
           rep.rough_ring_score, rep.stash_ring_score, rep.stack_score, rep.max_place_err_mm,
           rep.in_start_zone ? "是" : "否", score);
    if (rep.n_obs) {
        printf("    障碍物 %d 个（摆法编号 %u）：", rep.n_obs, obs_seed ? obs_seed : seed);
        for (int k = 0; k < sim_obstacle_count(); k++) { float x, y; sim_obstacle_get(k, &x, &y); printf("(%.0f,%.0f) ", x, y); }
        printf("| 车身碰到 %d 个 | 程序发现 %d 个，误报 %d 个\n", rep.obs_hits, rep.obs_found, rep.obs_false);
    }
    if (rep.lane_violations) printf("    第 %.1f 秒首次压线，位置 (%.0f, %.0f)；按规则本轮在此结束，得分只算这之前的\n", rep.ended_ms / 1000.0f, rep.worst_violation_x, rep.worst_violation_y);
    char ended[96] = "", obs_txt[160] = "";
    if (rep.ended_ms) snprintf(ended, sizeof ended, "【第 %.1f 秒越出车道，按规则本轮结束，之后的动作不计分】 ", rep.ended_ms / 1000.0f);
    if (rep.n_obs) snprintf(obs_txt, sizeof obs_txt, " · 障碍物 %d 个（摆法编号 %u），车身碰到 %d 个，程序发现 %d 个、误报 %d 个",
                            rep.n_obs, obs_seed ? obs_seed : seed, rep.obs_hits, rep.obs_found, rep.obs_false);
    snprintf(g_summary, sizeof g_summary, "%s结果：用时 %.1f s · 抓取 %d 次 · 放置 %d 次 · 码垛 %d 组 · 压线 %d 次 · 粗加工得分 %d · 暂存得分 %d · 码垛得分 %d · 最大放置偏差 %.1f mm · 回到启停区：%s · 估算总分约 %d（仿真估算，不代表实车得分）%s",
             ended, r.time_ms / 1000.0f, r.grabs, r.places, r.stacks, rep.lane_violations, rep.rough_ring_score, rep.stash_ring_score, rep.stack_score,
             rep.max_place_err_mm, rep.in_start_zone ? "是" : "否", score, obs_txt);
    snprintf(g_result, sizeof g_result,
             "{\"time_ms\":%u,\"grabs\":%d,\"places\":%d,\"stacks\":%d,\"correct_grabs\":%d,\"violations\":%d,\"ended_ms\":%u,"
             "\"viol_x\":%.0f,\"viol_y\":%.0f,\"qr\":%d,\"rough\":%d,\"stash\":%d,\"stack\":%d,\"max_err\":%.1f,\"in_zone\":%d,\"score\":%d,\"limit_ms\":%d,"
             "\"obs\":%d,\"obs_hit\":%d,\"obs_found\":%d,\"obs_false\":%d,\"obs_seed\":%u}",
             (unsigned)r.time_ms, r.grabs, r.places, r.stacks, rep.correct_grabs, rep.lane_violations, (unsigned)rep.ended_ms,
             rep.worst_violation_x, rep.worst_violation_y, (r.qr_ok && rep.qr_before_end) ? 1 : 0,
             rep.rough_ring_score, rep.stash_ring_score, rep.stack_score, rep.max_place_err_mm, rep.in_start_zone ? 1 : 0, score, MATCH_TIME_LIMIT_MS,
             rep.n_obs, rep.obs_hits, rep.obs_found, rep.obs_false, obs_seed ? obs_seed : seed);
    return (r.time_ms <= MATCH_TIME_LIMIT_MS && rep.lane_violations == 0 && rep.in_start_zone && rep.obs_hits == 0) ? 0 : 1;
}

static int sim_main_body(int argc, char **argv);

int main(int argc, char **argv)
{
#ifdef _WIN32
    UINT old_out = GetConsoleOutputCP(), old_in = GetConsoleCP();
    SetConsoleOutputCP(CP_UTF8);     /* 让 Windows 控制台正常显示中文 */
    SetConsoleCP(CP_UTF8);
#endif
    int rc = sim_main_body(argc, argv);
#ifdef _WIN32
    SetConsoleOutputCP(old_out);     /* 还原，免得影响调用它的 .bat 脚本 */
    SetConsoleCP(old_in);
#endif
    return rc;
}

/* 避障统计：--obs N  跑 N 场随机障碍物（理想标定），只打印汇总。比较不同的传感器布置用 */
static int obs_stats(int N)
{
    static const char *TASKS[] = { "156+123+516+231", "134+123+431+231", "256+312+625+213", "421+321+142+132", "365+231+653+312" };
    int n = 0, total = 0, fails = 0, hit_runs = 0, hits = 0, viol = 0, tout = 0, nobs = 0, found = 0, fals = 0, sc;
    for (int k = 0; k < N; k++) {
        const char *task = TASKS[k % 5];
        int zone = 1 + (k / 5) % 2;
        int rc = run_one(task, zone, 1 + (unsigned)k, 0.0f, 0.0f, false, true, false, -1, 5000 + (unsigned)k, &sc);
        sim_report_t rep; sim_report(&rep, 0, 0);
        n++; total += sc; fails += rc;
        hits += rep.obs_hits; hit_runs += rep.obs_hits > 0; viol += rep.lane_violations > 0;
        tout += sim_time_ms() > MATCH_TIME_LIMIT_MS + 1000;
        nobs += rep.n_obs; found += rep.obs_found; fals += rep.obs_false;
    }
    printf("\n传感器 %d 个 | %d 场：未达标 %d 场，撞障碍物 %d 场（共 %d 次），压线 %d 场，超时 %d 场，平均约 %d 分 | 障碍物共 %d 个，发现 %d 个（%.0f%%），误报 %d 个\n",
           TOF_COUNT, n, fails, hit_runs, hits, viol, tout, total / n, nobs, found, nobs ? 100.0 * found / nobs : 0.0, fals);
    return 0;
}

static int sim_main_body(int argc, char **argv)
{
    int rc;
    if (argc > 2 && strcmp(argv[1], "--obs") == 0) return obs_stats(atoi(argv[2]));
    if (argc > 1 && strcmp(argv[1], "--batch") == 0) {
        static const char *TASKS[] = { "156+123+516+231", "134+123+431+231", "256+312+625+213", "421+321+142+132", "365+231+653+312" };
        int fails = 0, n = 0, total = 0, sc;
        for (unsigned t = 0; t < sizeof TASKS / sizeof TASKS[0]; t++)
            for (int z = 1; z <= 2; z++)
                for (unsigned seed = 1; seed <= 2; seed++) {
                    fails += run_one(TASKS[t], z, seed + t * 7, 0.0f, 0.0f, false, true, false, 0, 0, &sc); n++; total += sc;
                }
        printf("共 %d 场（理想标定，没有障碍物），未达标（超时/压线/未回区）%d 场，平均约 %d 分\n", n, fails, total / n);
        printf("---- 理想标定 + 随机障碍物（每场 1~3 个，摆法各不相同） ----\n");
        int fo = 0, no = 0, to = 0;
        for (unsigned t = 0; t < sizeof TASKS / sizeof TASKS[0]; t++)
            for (int z = 1; z <= 2; z++)
                for (unsigned k = 0; k < 2; k++) {
                    fo += run_one(TASKS[t], z, 1 + t * 7 + k, 0.0f, 0.0f, false, true, false, -1, 100 + t * 10 + (unsigned)z * 3 + k, &sc); no++; to += sc;
                }
        printf("障碍物组 %d 场，未达标（超时/压线/未回区/撞障碍物）%d 场，平均约 %d 分\n", no, fo, to / no);
        int f2 = 0, n2 = 0, t2 = 0;
        printf("---- 轮子打滑 2%% + 随机噪声，没有陀螺仪 ----\n");
        for (int z = 1; z <= 2; z++) { f2 += run_one(TASKS[0], z, 11, 0.02f, 0.02f, false, true, false, 0, 0, &sc); }
        printf("---- 轮子打滑 2%% / 5%% + 随机噪声，装了陀螺仪 ----\n");
        for (int z = 1; z <= 2; z++) for (unsigned seed = 11; seed <= 13; seed++) { f2 += run_one(TASKS[0], z, seed, 0.02f, 0.02f, false, true, true, -1, 0, &sc); n2++; t2 += sc; }
        for (int z = 1; z <= 2; z++) for (unsigned seed = 14; seed <= 15; seed++) { f2 += run_one(TASKS[2], z, seed, 0.05f, 0.03f, false, true, true, -1, 0, &sc); n2++; t2 += sc; }
        printf("---- 打滑 2%% + 陀螺仪，但相机看不到圆环（只靠里程计放置） ----\n");
        for (int z = 1; z <= 2; z++) { f2 += run_one(TASKS[1], z, 21, 0.02f, 0.02f, false, false, true, -1, 0, &sc); }
        printf("陀螺仪组 %d 场，平均约 %d 分；以上打滑测试中未达标 %d 场\n", n2, n2 ? t2 / n2 : 0, f2);
        return 0;
    }
#ifdef _WIN32
    /* 直接双击 sim.exe 时：它是上次编译出来的，不含之后改的代码。
       所以改为自动启动"编译并运行仿真.bat"：先重新编译，再运行。 */
    if (argc <= 1) {
        wchar_t dir[MAX_PATH];
        DWORD n = GetModuleFileNameW(NULL, dir, MAX_PATH);
        while (n > 0 && dir[n - 1] != L'\\') n--;
        dir[n] = 0;
        printf("正在启动“编译并运行仿真.bat”：先把你最新改的代码重新编译，再运行仿真……\n");
        ShellExecuteW(NULL, L"open", L"编译并运行仿真.bat", NULL, dir, SW_SHOWNORMAL);
        return 0;
    }
#endif
    /* --from-bat：由"编译并运行仿真.bat"刚编译完后启动，交互输入任务码 */
    if (argc > 1 && strcmp(argv[1], "--from-bat") == 0) { argc = 1; }

    char task[64] = "156+123+516+231";
    int zone = 1;
    float slip = 0.0f;
    bool imu = false, interactive = (argc <= 1);
    if (argc > 1) snprintf(task, sizeof task, "%s", argv[1]);
    if (argc > 2) zone = atoi(argv[2]);
    if (argc > 3) slip = (float)atof(argv[3]) / 100.0f;      /* 第3个参数：打滑百分比 */
    if (argc > 4) imu = atoi(argv[4]) != 0;                   /* 第4个参数：1=装陀螺仪 */
    /* 第5个参数：障碍物。不写 = 随机摆（每次不同）；0 = 不放；其他数字 = 复现那个编号的摆法 */
    int n_obs = -1;
    unsigned obs_seed = (unsigned)time(NULL) % 9000u + 1000u;
    if (argc > 5) { obs_seed = (unsigned)strtoul(argv[5], NULL, 10); if (obs_seed == 0) n_obs = 0; }
    unsigned seed = 1;                                         /* 第6个参数：随机种子（转盘方向、物料顺序） */
    if (argc > 6) seed = (unsigned)strtoul(argv[6], NULL, 10);

    if (interactive) {
        printf("==== 智能搬运小车 电脑仿真 ====\n");
        printf("这个程序把单片机上的比赛程序原样放在电脑上运行，模拟场地、电机、舵机、转盘和摄像头。\n\n");
        for (;;) {
            char line[64];
            printf("请输入任务码（直接回车 = %s）：", task);
            fflush(stdout);
            if (!fgets(line, sizeof line, stdin)) break;
            line[strcspn(line, "\r\n")] = 0;
            if (line[0]) snprintf(task, sizeof task, "%s", line);
            task_t tt; const char *err;
            if (task_parse(task, &tt, &err)) break;
            printf("  任务码不对：%s。格式例如 156+123+516+231\n", err);
            snprintf(task, sizeof task, "156+123+516+231");
        }
        {
            char line[16];
            printf("请输入出发启停区 1 或 2（直接回车 = 1）：");
            fflush(stdout);
            if (fgets(line, sizeof line, stdin) && (line[0] == '2')) zone = 2;
        }
        {
            char line[32];
            printf("障碍物：直接回车 = 随机摆 1~3 个（每次不同）；输入 0 = 不放；输入编号 = 复现上次那一场的摆法：");
            fflush(stdout);
            if (fgets(line, sizeof line, stdin) && line[0] >= '0' && line[0] <= '9') {
                obs_seed = (unsigned)strtoul(line, NULL, 10);
                if (obs_seed == 0) n_obs = 0;
            }
        }
        printf("\n开始仿真（约 1~2 秒）……\n\n");
    }
    if (zone != 2) zone = 1;

    int sc;
    rc = run_one(task, zone, seed, slip, slip > 0 ? 0.02f : 0.0f, true, true, imu, n_obs, obs_seed, &sc);

    char path[128], jpath[128];
    snprintf(path, sizeof path, "replay_%s_zone%d.html", task, zone);
    snprintf(jpath, sizeof jpath, "replay_%s_zone%d.js", task, zone);
    bool ok2d = sim_write_replay(path, g_summary, g_result) == 0;
    bool ok3d = sim_write_replay_js(jpath, g_summary, g_result) == 0 &&
                sim_write_replay_js("replay_latest.js", g_summary, g_result) == 0;
    if (ok2d) printf("\n已生成二维回放：%s（用浏览器打开即可看动画）\n", path);
    if (ok3d) printf("已生成三维回放数据：replay_latest.js（用“三维仿真回放.html”查看；另存一份 %s）\n", jpath);
    if (!ok2d || !ok3d) printf("\n回放文件写入失败（文件夹是否只读？）\n");
#ifdef _WIN32
    if (interactive) {
        /* 有三维回放页就打开三维的，没有就打开二维的 */
        if (ok3d && GetFileAttributesW(L"三维仿真回放.html") != INVALID_FILE_ATTRIBUTES)
            ShellExecuteW(NULL, L"open", L"三维仿真回放.html", NULL, NULL, SW_SHOWNORMAL);
        else if (ok2d)
            ShellExecuteA(NULL, "open", path, NULL, NULL, SW_SHOWNORMAL);
    }
#endif
    if (interactive) {
        printf("\n按回车键退出……");
        fflush(stdout);
        getchar();
    }
    return rc;
}
