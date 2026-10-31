/**
 * console.c —— 串口调试命令
 *
 * 为什么要命令行？上车调试时不用每改一个数就重新下载程序：
 * 例如输入 "mot 1 20" 就能单独让 M1 通道转 2 秒，看是哪个轮子、往哪转。
 */
#include "console.h"
#include "port.h"
#include "sys.h"
#include "motor_drv.h"
#include "chassis.h"
#include "nav.h"
#include "arm.h"
#include "bus_servo.h"
#include "vision.h"
#include "mission.h"
#include "task_code.h"
#include "field.h"
#include "obstacle.h"
#include "robot_config.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static char s_buf[64];
static int  s_len;
static char s_cmd[64];
static bool s_pending;

void console_poll(void)
{
    uint8_t c;
    while (port_uart_read_byte(PORT_UART_LOG, &c)) {
        if (c == '\r' || c == '\n') {
            if (s_len == 0) continue;
            s_buf[s_len] = 0;
            s_len = 0;
            if (strcmp(s_buf, "stop") == 0 || strcmp(s_buf, "s") == 0) {   /* 急停：立即生效 */
                sys_set_abort(true);
                LOG("急停！输入任意命令解除");
                continue;
            }
            if (!s_pending) { strcpy(s_cmd, s_buf); s_pending = true; }
        } else if (s_len < (int)sizeof s_buf - 1) {
            s_buf[s_len++] = (char)c;
        }
    }
}

static int argi(char **a, int n, int i, int def) { return i < n ? atoi(a[i]) : def; }
static float argf(char **a, int n, int i, float def) { return i < n ? (float)atof(a[i]) : def; }

static void help(void)
{
    LOG("---------- 调试命令 ----------");
    LOG("i2c                 扫描 I2C 总线");
    LOG("bat                 读电池电压");
    LOG("mot <1-4> <速度>    单独转某个通道 2 秒（速度 -45~45，脉冲/10ms）");
    LOG("wheel               依次让 左前/右前/左后/右后 向前转，检查接线与方向");
    LOG("enc                 显示四个轮子累计脉冲");
    LOG("move <前mm> <左mm> <转角度>  相对移动（转角在 ±180° 内）");
    LOG("spin <圈数>         原地逆时针转 N 圈（标定 ROT_SCALE 用）");
    LOG("imu [0|off|on]      陀螺仪读数；0 累计角度清零；off/on 临时关闭/打开");
    LOG("goto <x> <y> <角度>  走到场地坐标；route 同参数但沿车道走");
    LOG("pose <x> <y> <角度>  设定当前位置；odom 显示当前位置");
    LOG("sscan               扫描舵机总线上的 ID 1~20，打印位置/电压/温度（舵机不动）");
    LOG("snudge <ID> <步数>  ST3215 从当前位置慢慢转一小步（±200 以内，4096 步 = 一圈）");
    LOG("snudge g <步数>     夹爪 SC09 慢慢转一小步（±60 以内，1024 步 = 300°）");
    LOG("sfree <ID>          该 ST3215 卸力（可以用手掰）；sfree g 夹爪卸力");
    LOG("scal <ID>           ST3215 中位校准：把它现在的位置定义为 2048（掉电保存，舵机不动）");
    LOG("sping               检查 4 个舵机（只检查，不动）；sread 读舵机原始位置");
    LOG("storque <0/1>       舵机卸力/上力（卸力后可用手掰，配合 sread 示教）");
    LOG("sraw <关节0-3> <值> 单独转一个舵机（0回转 1肩 2肘 3夹爪）");
    LOG("sid <st|sc> <旧ID> <新ID>  改舵机 ID（总线上只接一个）；夹爪 SC09：sid sc 1 4");
    LOG("arm <x> <y> <z>     机械臂夹持中心移动到车体坐标；ik 同参数只计算不动");
    LOG("grip <o|c>          夹爪开/合；fold 折叠；home 行车姿态");
    LOG("slot <0-2> <p|t>    放进(p)/取出(t)第 k 个载物槽");
    LOG("qr / tt             显示相机最近一次二维码/转盘数据");
    LOG("tof                 8 个测距传感器的读数和已发现的障碍物；tof clear 清空障碍物");
    LOG("task <任务码> [区]  跳过读码，用给定任务码跑全程（调试用）");
    LOG("run [区]            跑全程（与按 KEY0 相同）");
    LOG("stop 或 s           急停");
}

static void cmd_wheel(void)
{
    static const char *NAME[4] = { "左前", "右前", "左后", "右后" };
    chassis_enable(false);
    for (int w = 0; w < 4; w++) {
        /* 用底盘的"单轮向前"效果：只给这个轮子向前的速度 */
        int8_t sp[4] = { 0, 0, 0, 0 };
        static const int8_t CH[4] = WHEEL_CH_INIT;
        static const int8_t SG[4] = WHEEL_SIGN_INIT;
        sp[CH[w]] = (int8_t)(15 * SG[w]);
        LOG("现在应该是【%s】轮向前转（通道 M%d）", NAME[w], CH[w] + 1);
        motor_drv_set_speed(sp);
        sys_wait_ms(1500);
        int8_t z[4] = { 0, 0, 0, 0 };
        motor_drv_set_speed(z);
        sys_wait_ms(500);
    }
    chassis_enable(true);
}

static void execute(char *line)
{
    char *a[8];
    int n = 0;
    for (char *t = strtok(line, " "); t && n < 8; t = strtok(NULL, " ")) a[n++] = t;
    if (n == 0) return;
    sys_set_abort(false);
    const char *c = a[0];

    if (!strcmp(c, "help") || !strcmp(c, "?")) help();
    else if (!strcmp(c, "i2c")) {
        for (int ad = 1; ad < 127; ad++) if (port_i2c_probe((uint8_t)ad) == 0) LOG("I2C 设备 0x%02X", ad);
        LOG("扫描结束（驱动板应为 0x34）");
    }
    else if (!strcmp(c, "bat")) { uint16_t mv = 0; if (motor_drv_read_battery_mv(&mv) == 0) LOG("电池 %u mV", mv); else LOG("读取失败"); }
    else if (!strcmp(c, "mot")) {
        int ch = argi(a, n, 1, 1) - 1, sp = argi(a, n, 2, 15);
        if (ch < 0 || ch > 3) { LOG("通道 1~4"); return; }
        chassis_enable(false);
        int8_t s[4] = { 0, 0, 0, 0 };
        s[ch] = (int8_t)sp;
        motor_drv_set_speed(s);
        sys_wait_ms(2000);
        s[ch] = 0;
        motor_drv_set_speed(s);
        chassis_enable(true);
    }
    else if (!strcmp(c, "wheel")) cmd_wheel();
    else if (!strcmp(c, "enc")) {
        LOG("左前 %ld  右前 %ld  左后 %ld  右后 %ld", (long)chassis_wheel_count(0), (long)chassis_wheel_count(1),
            (long)chassis_wheel_count(2), (long)chassis_wheel_count(3));
    }
    else if (!strcmp(c, "move")) {
        nav_result_t r = nav_move_rel(argf(a, n, 1, 0), argf(a, n, 2, 0), DEG2RAD(argf(a, n, 3, 0)), 250);
        LOG("move 结果 %d", r);
    }
    else if (!strcmp(c, "spin")) {
        /* 按里程计累计转过的角度，转满 N 圈后停：实际转多了/少了就说明 ROT_SCALE 要改 */
        float target = DEG2RAD(360.0f) * argf(a, n, 1, 2), acc = 0;
        float last = chassis_pose().th;
        uint32_t t0 = sys_now();
        chassis_set_velocity(0, 0, DEG2RAD(90.0f));
        while (acc < target - DEG2RAD(20.0f) && !sys_aborted() && (uint32_t)(sys_now() - t0) < 60000) {
            sys_poll();
            float th = chassis_pose().th;
            acc += wrap_pi(th - last);
            last = th;
        }
        nav_turn_to(chassis_pose().th + (target - acc));    /* 最后 20° 用走位控制精确停住 */
        float yaw;
        if (imu_get_yaw(&yaw))
            LOG("陀螺仪认为转了 %d°，请看实际转了多少：IMU_SCALE = 旧值 × 实际 / %d", (int)RAD2DEG(target), (int)RAD2DEG(target));
        else
            LOG("轮子认为转了 %d°，请看实际转了多少：ROT_SCALE = 旧值 × 实际 / %d", (int)RAD2DEG(target), (int)RAD2DEG(target));
    }
    else if (!strcmp(c, "imu")) {
        int act = 0;
        if (n > 1) act = !strcmp(a[1], "0") ? 1 : !strcmp(a[1], "off") ? 2 : !strcmp(a[1], "on") ? 3 : 0;
        imu_debug(act);
    }
    else if (!strcmp(c, "goto") || !strcmp(c, "route")) {
        pose_t t = { argf(a, n, 1, 0), argf(a, n, 2, 0), DEG2RAD(argf(a, n, 3, 0)) };
        nav_result_t r = !strcmp(c, "goto") ? nav_goto(t, 0) : nav_route_to(t);
        LOG("结果 %d", r);
    }
    else if (!strcmp(c, "pose")) { pose_t p = { argf(a, n, 1, 0), argf(a, n, 2, 0), DEG2RAD(argf(a, n, 3, 0)) }; chassis_set_pose(p); }
    else if (!strcmp(c, "odom")) { pose_t p = chassis_pose(); LOG("x=%d y=%d 角度=%d", (int)p.x, (int)p.y, (int)RAD2DEG(p.th)); }
    else if (!strcmp(c, "sping")) {
        /* 只检查应答，不上力、不动（以前这里会顺便收折叠姿态，没标定时动作很大） */
        static const char *NAME[4] = { "回转 ST ID", "肩 ST ID", "肘 ST ID", "夹爪 SC ID" };
        static const uint8_t ID[4] = { SERVO_ID_YAW, SERVO_ID_SHOULDER, SERVO_ID_ELBOW, SERVO_ID_GRIP };
        int ok = 0;
        for (int j = 0; j < 4; j++) {
            servo_t s = { j < 3 ? PORT_UART_SERVO_ST : PORT_UART_GRIP, j < 3 ? SERVO_STS : SERVO_SCS, ID[j] };
            bool r = servo_ping(&s) == 0;
            ok += r;
            LOG("%s%u  %s", NAME[j], ID[j], r ? "有应答" : "无应答");
        }
        LOG("%d / 4 个舵机有应答", ok);
    }
    else if (!strcmp(c, "sscan")) {
        int found = 0;
        for (int bus = 0; bus < 2; bus++) {
            if (bus == 1 && GRIP_ON_ST_BUS) break;        /* 夹爪也在 ST 总线上时，USART6 上什么都没接 */
            for (uint8_t id = 1; id <= 20; id++) {
                /* 电压/温度寄存器两个系列一样；位置的字节顺序不同：配置里的夹爪 ID 按 SC09 读，其余按 ST3215 读 */
                bool grip = (bus == 0) ? (GRIP_ON_ST_BUS && id == SERVO_ID_GRIP) : true;
                servo_t s = { bus ? PORT_UART_SERVO_SC : PORT_UART_SERVO_ST, grip ? SERVO_SCS : SERVO_STS, id };
                if (servo_ping(&s) != 0) continue;
                int pos = -1; uint8_t v = 0, t = 0;
                servo_read_pos(&s, &pos);
                servo_read_byte(&s, SREG_VOLTAGE, &v);
                servo_read_byte(&s, SREG_TEMP, &t);
                LOG("%s 总线  ID %2u  %s  位置 %4d  电压 %d.%dV  温度 %u°C", bus ? "SC" : "ST", id,
                    grip ? "SC09夹爪" : "ST3215  ", pos, v / 10, v % 10, t);
                if (v > 75 && grip) LOG("  !! 夹爪电压 %d.%dV 超过 7.5V：降压模块没接对，马上断电！", v / 10, v % 10);
                found++;
            }
        }
        LOG("共找到 %d 个舵机。注意：几个舵机 ID 相同时只会显示成 1 个，而且一起动——先一个一个改 ID", found);
        LOG("（SC09 出厂 ID 1 时会被当成 ST3215 读，位置数不准，改成 ID %d 后就正常了）", SERVO_ID_GRIP);
    }
    else if (!strcmp(c, "snudge") && n >= 3 && a[1][0] == 'g') {
        /* 夹爪 SC09：1024 步 = 300°，一次最多 ±60 步 ≈ ±18° */
        servo_t s = { PORT_UART_GRIP, SERVO_SCS, SERVO_ID_GRIP };
        int d = atoi(a[2]), pos;
        if (d > 60) d = 60;
        if (d < -60) d = -60;
        if (servo_read_pos(&s, &pos) != 0) { LOG("夹爪 SC09 ID %u 无应答", s.id); return; }
        servo_torque(&s, true);
        servo_write_pos(&s, pos + d, 100, 0);
        sys_wait_ms((uint32_t)(abs(d) * 1000 / 100 + 300));
        int now = -1;
        servo_read_pos(&s, &now);
        LOG("夹爪 SC09 ID %u：%d → 目标 %d，现在 %d（%+d 步 ≈ %+d°）", s.id, pos, pos + d, now, d, d * 300 / 1024);
    }
    else if (!strcmp(c, "snudge") && n >= 3) {
        servo_t s = { PORT_UART_SERVO_ST, SERVO_STS, (uint8_t)atoi(a[1]) };
        int d = atoi(a[2]), pos;
        if (d > 200) d = 200;
        if (d < -200) d = -200;
        if (servo_read_pos(&s, &pos) != 0) { LOG("ST ID %u 无应答", s.id); return; }
        servo_torque(&s, true);
        servo_write_pos(&s, pos + d, 400, 10);           /* 400 步/秒 ≈ 35°/s，加速度很小 */
        sys_wait_ms((uint32_t)(abs(d) * 1000 / 400 + 400));
        int now = -1;
        servo_read_pos(&s, &now);
        LOG("ST ID %u：%d → 目标 %d，现在 %d（%+d 步 ≈ %+d°）", s.id, pos, pos + d, now, d, d * 360 / 4096);
    }
    else if (!strcmp(c, "scal") && n >= 2) {
        servo_t s = { PORT_UART_SERVO_ST, SERVO_STS, (uint8_t)atoi(a[1]) };
        int before = -1, after = -1;
        if (servo_read_pos(&s, &before) != 0) { LOG("ST ID %u 无应答", s.id); return; }
        int r = servo_define_middle(&s);
        sys_wait_ms(20);
        servo_read_pos(&s, &after);
        LOG("ST ID %u 中位校准%s：校准前读数 %d，校准后读数 %d（应接近 2048）", s.id, r == 0 ? "完成" : "可能失败", before, after);
    }
    else if (!strcmp(c, "sfree") && n >= 2 && a[1][0] == 'g') {
        servo_t s = { PORT_UART_GRIP, SERVO_SCS, SERVO_ID_GRIP };
        LOG("夹爪 SC09 ID %u %s", s.id, servo_torque(&s, false) == 0 ? "已卸力" : "无应答");
    }
    else if (!strcmp(c, "sfree") && n >= 2) {
        servo_t s = { PORT_UART_SERVO_ST, SERVO_STS, (uint8_t)atoi(a[1]) };
        LOG("ST ID %u %s", s.id, servo_torque(&s, false) == 0 ? "已卸力" : "无应答");
    }
    else if (!strcmp(c, "sread")) {
        int raw[4];
        arm_read_raw(raw);
        LOG("回转 %d  肩 %d  肘 %d  夹爪 %d", raw[0], raw[1], raw[2], raw[3]);
    }
    else if (!strcmp(c, "storque")) arm_torque(argi(a, n, 1, 1) != 0);
    else if (!strcmp(c, "sraw")) arm_write_raw(argi(a, n, 1, 0), argi(a, n, 2, 2048));
    else if (!strcmp(c, "sid") && n >= 4) {
        bool st = !strcmp(a[1], "st");
        servo_t s = { st ? PORT_UART_SERVO_ST : PORT_UART_GRIP, st ? SERVO_STS : SERVO_SCS, (uint8_t)atoi(a[2]) };
        if (!st && GRIP_ON_ST_BUS) {
            /* 夹爪和 ST3215 在同一条总线上：改 SC09 的 ID 时 ST3215 必须拔掉，
               否则 ID 1 的底座舵机会一起收到改 ID 命令（两个系列的寄存器不同，会把底座搞乱） */
            for (uint8_t id = 1; id <= 3; id++) {
                servo_t t = { PORT_UART_SERVO_ST, SERVO_STS, id };
                if (id != s.id && servo_ping(&t) == 0) {
                    LOG("总线上还有 ID %u 的舵机在应答：先把 3 个 ST3215 的线拔掉，只留夹爪，再改 ID", id);
                    return;
                }
            }
        }
        if (servo_ping(&s) != 0) { LOG("ID %u 没有应答，检查接线/供电", s.id); return; }
        LOG("%s", servo_set_id(&s, (uint8_t)atoi(a[3])) == 0 ? "改 ID 成功" : "改 ID 失败");
    }
    else if (!strcmp(c, "arm") || !strcmp(c, "ik")) {
        vec3_t p = { argf(a, n, 1, 150), argf(a, n, 2, -45), argf(a, n, 3, 150) };
        float q[3];
        if (!arm_ik(p, q)) { LOG("够不到"); return; }
        LOG("关节角 回转%d 肩%d 肘%d（度）", (int)RAD2DEG(q[0]), (int)RAD2DEG(q[1]), (int)RAD2DEG(q[2]));
        if (!strcmp(c, "arm")) arm_move_safe(p);
    }
    else if (!strcmp(c, "grip")) { if (n > 1 && a[1][0] == 'c') { bool h; arm_grip_close(&h); LOG("%s", h ? "夹到东西" : "空夹"); } else arm_grip_open(); }
    else if (!strcmp(c, "fold")) arm_fold();
    else if (!strcmp(c, "home")) arm_home();
    else if (!strcmp(c, "slot")) {
        int k = argi(a, n, 1, 0);
        if (n > 2 && a[2][0] == 't') arm_pick_from_slot(k); else arm_place_to_slot(k);
    }
    else if (!strcmp(c, "qr")) {
        char b[24];
        if (vision_get_qr(b, sizeof b, 5000)) LOG("二维码：%s", b);
        else LOG("5 秒内没有二维码数据");
    }
    else if (!strcmp(c, "tt")) {
        vision_tt_t t;
        if (vision_get_tt(&t, 2000)) LOG("转盘：颜色%d(%s) 稳定%d 偏差 %d,%d mm", t.color, color_name(t.color), t.stable, (int)t.dx, (int)t.dy);
        else LOG("没有转盘数据（先发送 $MODE,2 或运行流程）");
    }
    else if (!strcmp(c, "tof")) {
        if (n > 1 && !strcmp(a[1], "clear")) { obs_reset(); LOG("已清空障碍物地图"); }
        else obs_debug();
    }
    else if (!strcmp(c, "task") && n >= 2) {
        mission_cfg_t cfg = { argi(a, n, 2, 1), true, "" };
        snprintf(cfg.fixed_task, sizeof cfg.fixed_task, "%s", a[1]);
        mission_result_t r;
        mission_run(&cfg, &r);
    }
    else if (!strcmp(c, "run")) {
        mission_cfg_t cfg = { argi(a, n, 1, 1), false, "" };
        mission_result_t r;
        mission_run(&cfg, &r);
    }
    else LOG("未知命令 %s，输入 help 查看", c);
}

bool console_execute_pending(void)
{
    if (!s_pending) return false;
    char line[64];
    strcpy(line, s_cmd);
    s_pending = false;
    LOG("> %s", line);
    execute(line);
    return true;
}
