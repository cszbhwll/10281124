/**
 * app.c —— 上电后的总流程
 *
 *   上电 → 初始化各模块 → 待机（屏幕显示 READY 和启停区号）
 *     按 KEY1：切换启停区 1/2
 *     按 KEY0：一键启动，跑完整场比赛
 *     电脑串口输入命令：单项调试（输入 help 查看）
 */
#include "app.h"
#include "port.h"
#include "sys.h"
#include "field.h"
#include "chassis.h"
#include "motor_drv.h"
#include "arm.h"
#include "vision.h"
#include "display.h"
#include "console.h"
#include "mission.h"
#include "obstacle.h"
#include "robot_config.h"
#include <stdio.h>

static void show_ready(int zone)
{
    char s[20];
    snprintf(s, sizeof s, "READY Z%d", zone);
    display_set_status(s);
}

static void beep(int ms)
{
    port_beep(true);
    sys_wait_ms((uint32_t)ms);
    port_beep(false);
}

void app_main(void)
{
    port_init();
    sys_init();
    LOG("==== 智能搬运小车 启动 ====");
    field_init();
    chassis_init();
    vision_init();
    display_init();

    if (motor_drv_init() != 0) LOG("警告：电机驱动板未就绪，底盘不会动");
    if (imu_init() == 0) LOG("陀螺仪已启用：朝向用陀螺仪测量");
#if SERVO_TEST_MODE
    LOG("==== 舵机测试模式：机械臂上电不动 ====");
    LOG("先输入 sscan 看总线上有哪些舵机；snudge <ID> <步数> 小幅度转（最多 ±200 步 ≈ ±17°）；sfree <ID> 卸力");
    LOG("三个舵机出厂 ID 都是 1：改 ID 时总线上只能接一个（sid st 1 2 / sid st 1 3）");
    LOG("夹爪 SC09：只接夹爪时 sid sc 1 4；之后 snudge g <步数> 小幅度转，sfree g 卸力");
#else
    if (arm_init() != 0)       LOG("警告：有舵机未应答，机械臂可能不完整");
#endif
    obs_init();                /* 避障测距传感器；没接时只打印提示，比赛照常进行（不避障） */

    int zone = 1;
    show_ready(zone);
    LOG("待机：KEY1 切换启停区，KEY0 启动；串口输入 help 查看调试命令");
    beep(80);

    uint32_t last_blink = 0;
    bool led = false;
    for (;;) {
        sys_poll();

        if (sys_key_pressed(PORT_KEY_ZONE)) {
            zone = (zone == 1) ? 2 : 1;
            show_ready(zone);
            LOG("出发启停区 = %d", zone);
            beep(40);
        }

        if (sys_key_pressed(PORT_KEY_START)) {
            beep(150);
            mission_cfg_t cfg = { zone, false, "" };
            mission_result_t res;
            imu_track_bias(false);          /* 出发后车在动，不能再修正零漂 */
            mission_run(&cfg, &res);
            imu_track_bias(true);
            beep(400);
        }

        console_execute_pending();

        /* 绿灯慢闪表示程序在正常运行 */
        uint32_t now = sys_now();
        if ((uint32_t)(now - last_blink) > 500) {
            last_blink = now;
            led = !led;
            port_led(1, led);
        }
    }
}
