/**
 * nav.h —— 走位：去某个坐标、沿车道路线走、相对移动
 */
#ifndef NAV_H
#define NAV_H

#include <stdbool.h>
#include <stdint.h>
#include "geom.h"

typedef enum {
    NAV_OK = 0,
    NAV_TIMEOUT,
    NAV_ABORTED,
    NAV_NO_PATH,
} nav_result_t;

/*
 * 两种用法：
 *   nav_route_to(...)       —— 一直等到车到位才返回（简单）
 *   nav_start_route(...) 然后做别的事（如机械臂动作），最后 nav_wait() —— 开车和机械臂同时进行，省时间
 */
void nav_update(void);                         /* 由 sys_poll 每 20ms 调用 */
bool nav_busy(void);
nav_result_t nav_wait(void);
void nav_cancel(void);
void nav_start_goto(pose_t target, float vmax_mmps);
void nav_start_route(pose_t target);

/* 走到场地坐标 (x,y,th)，到点停稳后返回 */
nav_result_t nav_goto(pose_t target, float vmax_mmps);
nav_result_t nav_goto_precise(pose_t target, float vmax_mmps, float pos_tol_mm, float ang_tol_deg);   /* 容差更小的到点（回启停区用） */

/* 沿车道网格规划路线去目标（自动避开禁行区和被标记挡住的路段） */
nav_result_t nav_route_to(pose_t target);

/* 相对当前位置在车体坐标下移动 (dx 向前, dy 向左, dth 逆时针弧度) */
nav_result_t nav_move_rel(float dx, float dy, float dth, float vmax_mmps);

/* 原地转到绝对朝向 */
nav_result_t nav_turn_to(float th);

/* 估算从当前位置沿车道回到 target 需要的时间（ms），用于比赛末尾决定何时返程 */
uint32_t nav_estimate_route_ms(pose_t target);
uint32_t nav_estimate_path_ms(pose_t from, pose_t target);   /* 从 from 走到 target 大约要多久（不必是当前位置） */

#endif
