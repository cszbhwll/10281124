/**
 * tof.h —— VL53L0X 激光测距（940nm 红外）× TOF_COUNT，经 TCA9548A 多路开关接 I2C
 *
 * 用法：tof_init() 上电时调用一次；之后 sys_poll 会不停调用 tof_poll() 读新数据；
 *       要用的时候 tof_get_mm(i, 最长多久以前) 取第 i 个传感器的距离。
 * 模拟器里这几个函数由 sim/world.c 提供（按模拟场地里的障碍物算读数），本文件的硬件驱动不参与编译。
 */
#ifndef TOF_H
#define TOF_H

#include <stdbool.h>
#include <stdint.h>

typedef struct { float x, y, yaw_deg; } tof_mount_t;

int  tof_init(void);                         /* 返回找到的传感器个数 */
void tof_poll(void);                         /* 读已经测完的新数据（不等待） */
int  tof_get_mm(int i, uint32_t max_age_ms); /* 0 = 没看到东西 / 超出量程 / 数据太旧 / 传感器坏了 */
bool tof_ok(int i);
int  tof_count(void);
const tof_mount_t *tof_mount(int i);         /* 第 i 个传感器的安装位置（robot_config.h 的 TOF_LAYOUT_INIT） */

#endif
