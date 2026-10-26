/**
 * display.c —— 显示内容管理（与具体屏幕无关）
 */
#include "display.h"
#include "sys.h"
#include <stdio.h>
#include <string.h>

static void log_init(void) {}
static void log_clear(void) {}
/* 没接屏幕时：4 行攒成 1 行打印，减少串口占用时间 */
static char s_lines[4][20];
static void log_text(int line, const char *s)
{
    if (line < 0 || line > 3) return;
    snprintf(s_lines[line], sizeof s_lines[line], "%s", s);
    if (line == 3) LOG("[屏幕] %s | %s | %s | %s", s_lines[0], s_lines[1], s_lines[2], s_lines[3]);
}
static const display_backend_t LOG_BACKEND = { log_init, log_clear, log_text };

static const display_backend_t *s_be = &LOG_BACKEND;
static char s_task[20] = "---+---+";
static char s_task2[12] = "---+---";
static int  s_grab, s_place;
static char s_status[20] = "BOOT";
static bool s_dirty = true;
static uint32_t s_last;

void display_init(void) { s_be->init(); s_be->clear(); s_dirty = true; }

void display_set_backend(const display_backend_t *b)
{
    s_be = b ? b : &LOG_BACKEND;
    s_be->init();
    s_be->clear();
    s_dirty = true;
}

void display_set_task(const char *code)
{
    /* 拆成两行："156+123+" 和 "516+231"，每行 8 个字符以内，方便用大字号 */
    if (strlen(code) >= 15) {
        memcpy(s_task, code, 8);  s_task[8] = 0;
        memcpy(s_task2, code + 8, 7); s_task2[7] = 0;
    } else {
        snprintf(s_task, sizeof s_task, "%s", code);
        s_task2[0] = 0;
    }
    s_dirty = true;
}

void display_set_counts(int g, int p) { if (g != s_grab || p != s_place) { s_grab = g; s_place = p; s_dirty = true; } }

void display_set_status(const char *s)
{
    if (strncmp(s, s_status, sizeof s_status - 1) != 0) {
        snprintf(s_status, sizeof s_status, "%s", s);
        s_dirty = true;
    }
}

void display_poll(void)
{
    uint32_t now = sys_now();
    if (!s_dirty || (uint32_t)(now - s_last) < 200) return;   /* 最快 5 次/秒，避免占用太多时间 */
    s_last = now;
    s_dirty = false;
    char l3[20];
    snprintf(l3, sizeof l3, "G%d P%d", s_grab, s_place);
    s_be->text(0, s_task);
    s_be->text(1, s_task2);
    s_be->text(2, l3);
    s_be->text(3, s_status);
}
