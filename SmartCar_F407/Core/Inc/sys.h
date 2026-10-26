/**
 * sys.h —— 主循环调度与日志
 *
 * 本程序采用"顺序写法"：比赛流程像菜谱一样一步步写下去，例如
 *     nav_goto(取料点); arm_pick(...); arm_place_to_slot(...);
 * 每个需要等待的函数内部都会反复调用 sys_poll()，
 * 由 sys_poll() 负责每 20ms 更新一次底盘控制、接收摄像头数据等"后台工作"。
 * 所以：任何循环等待都必须调用 sys_poll() 或 sys_wait_ms()，不能用死循环空等。
 */
#ifndef SYS_H
#define SYS_H

#include <stdint.h>
#include <stdbool.h>

void     sys_init(void);
void     sys_poll(void);                 /* 后台工作：底盘控制、视觉接收、按键扫描、控制台 */
void     sys_wait_ms(uint32_t ms);       /* 等待的同时保持后台工作 */
uint32_t sys_now(void);                  /* 毫秒时间 */

/* 按键（已消抖），按下的那一刻返回 true 一次 */
bool     sys_key_pressed(int key);

/* 比赛计时：开始后经过的毫秒数 */
void     sys_match_start(void);
uint32_t sys_match_elapsed(void);
bool     sys_match_running(void);

/* 急停：任何时候设置后，底盘立刻停，等待函数尽快返回 */
void     sys_set_abort(bool abort);
bool     sys_aborted(void);

/* 日志：通过 USART1 打印到电脑串口助手（115200） */
void     log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#define LOG(...) log_printf(__VA_ARGS__)

#endif
