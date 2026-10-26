/**
 * sys.c —— 主循环调度、按键消抖、比赛计时、日志
 */
#include "sys.h"
#include "port.h"
#include "robot_config.h"
#include "chassis.h"
#include "nav.h"
#include "vision.h"
#include "console.h"
#include "display.h"
#include "obstacle.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static uint32_t s_last_ctrl;
static uint32_t s_match_t0;
static bool     s_match_on;
static volatile bool s_abort;
static bool     s_in_poll;

/* 按键消抖：连续 3 次（每次间隔 10ms）读到相同状态才算数 */
static uint8_t  s_key_cnt[PORT_KEY_COUNT];
static bool     s_key_state[PORT_KEY_COUNT];
static bool     s_key_event[PORT_KEY_COUNT];
static uint32_t s_last_key;

void sys_init(void)
{
    s_last_ctrl = port_millis();
    s_last_key  = s_last_ctrl;
}

uint32_t sys_now(void) { return port_millis(); }

static void scan_keys(void)
{
    for (int k = 0; k < PORT_KEY_COUNT; k++) {
        bool raw = port_key_down((port_key_t)k);
        if (raw != s_key_state[k]) {
            if (++s_key_cnt[k] >= 3) {
                s_key_state[k] = raw;
                s_key_cnt[k] = 0;
                if (raw) s_key_event[k] = true;    /* 记录"按下"事件 */
            }
        } else {
            s_key_cnt[k] = 0;
        }
    }
}

void sys_poll(void)
{
    if (s_in_poll) { port_idle(); return; }   /* 防止重入：后台工作里如果又调用了等待函数，不会无限嵌套 */
    s_in_poll = true;

    uint32_t now = port_millis();

    /* 1) 底盘控制：固定 20ms 周期 */
    if ((uint32_t)(now - s_last_ctrl) >= CTRL_PERIOD_MS) {
        uint32_t dt = now - s_last_ctrl;
        if (dt > 100) dt = CTRL_PERIOD_MS;        /* 长时间阻塞后不要用异常大的 dt */
        s_last_ctrl = now;
        obs_update();                             /* 避障：读测距传感器，更新障碍物地图 */
        nav_update();                             /* 走位控制器：算出这个周期的目标速度 */
        chassis_update((float)dt / 1000.0f);      /* 底盘：读编码器、下发轮速 */
    }

    /* 2) 按键：10ms 扫描一次 */
    if ((uint32_t)(now - s_last_key) >= 10) {
        s_last_key = now;
        scan_keys();
    }

    /* 3) 摄像头数据、串口命令、屏幕刷新 */
    vision_poll();
    console_poll();
    display_poll();

    s_in_poll = false;
    port_idle();
}

void sys_wait_ms(uint32_t ms)
{
    uint32_t t0 = port_millis();
    while ((uint32_t)(port_millis() - t0) < ms && !s_abort) sys_poll();
}

bool sys_key_pressed(int key)
{
    if (key < 0 || key >= PORT_KEY_COUNT) return false;
    bool e = s_key_event[key];
    s_key_event[key] = false;
    return e;
}

void sys_match_start(void) { s_match_t0 = port_millis(); s_match_on = true; }
uint32_t sys_match_elapsed(void) { return s_match_on ? (uint32_t)(port_millis() - s_match_t0) : 0; }
bool sys_match_running(void) { return s_match_on; }

void sys_set_abort(bool abort)
{
    s_abort = abort;
    if (abort) { nav_cancel(); chassis_stop_now(); }
}
bool sys_aborted(void) { return s_abort; }

void log_printf(const char *fmt, ...)
{
    char buf[200];
    uint32_t t = port_millis();
    int n = snprintf(buf, sizeof buf, "[%3lu.%03lu] ", (unsigned long)(t / 1000), (unsigned long)(t % 1000));
    va_list ap;
    va_start(ap, fmt);
    n += vsnprintf(buf + n, sizeof buf - (size_t)n - 3, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof buf - 3) n = (int)sizeof buf - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';
    port_uart_write(PORT_UART_LOG, (const uint8_t *)buf, (uint16_t)n);
}
