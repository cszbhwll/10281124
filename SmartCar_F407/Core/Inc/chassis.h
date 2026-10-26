/**
 * chassis.h —— 麦克纳姆轮底盘：运动学 + 里程计
 *
 * 麦轮（X 型安装，从上往下看滚子呈"X"）四个轮子的速度关系：
 *     左前 FL = vx - vy - k·ω
 *     右前 FR = vx + vy + k·ω
 *     左后 RL = vx + vy - k·ω
 *     右后 RR = vx - vy + k·ω        其中 k = a + b（半轮距 + 半轴距）
 *   vx：向前速度，vy：向左速度，ω：逆时针角速度
 * 反过来，由四个轮子实际转了多少，就能算出车走了多少（里程计）。
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>
#include "geom.h"

enum { WHEEL_FL = 0, WHEEL_FR, WHEEL_RL, WHEEL_RR };

void   chassis_init(void);
void   chassis_update(float dt_s);                        /* 由 sys_poll 每 20ms 调用 */

void   chassis_set_velocity(float vx, float vy, float w); /* 目标速度（车体坐标，mm/s，rad/s），内部限加速度 */
void   chassis_stop_now(void);                            /* 立即停 */
void   chassis_enable(bool en);                           /* false：不再发送速度（单轮调试时用） */

pose_t chassis_pose(void);                                /* 当前位姿（场地坐标） */
void   chassis_set_pose(pose_t p);                        /* 重新设定位姿（出发时、视觉校正时） */
void   chassis_measured_velocity(float *vx, float *vy, float *w);
int32_t chassis_wheel_count(int wheel);                   /* 某个轮子累计脉冲（已按方向修正） */

/* 可选：陀螺仪（imu_mpu6050.c）。imu_get_yaw 返回 true 表示 yaw 有效（弧度，逆时针为正）。 */
bool   imu_get_yaw(float *yaw);
int    imu_init(void);
void   imu_update(void);
void   imu_track_bias(bool on);                           /* 待机时 true：车不动时自动修正零漂 */
void   imu_debug(int action);                             /* 0 显示 1 累计清零 2 关闭 3 打开 */

#endif
