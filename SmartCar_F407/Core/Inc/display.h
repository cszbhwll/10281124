/**
 * display.h —— 任务码显示屏
 *
 * 规则：屏幕在车体上部醒目处、发光、不被遮挡；字高 ≥12mm；显示全部任务码和完成情况。
 * 显示内容（4 行，等宽大字）：
 *   第1行  156+123+
 *   第2行  516+231
 *   第3行  G12 P12        （抓取正确次数、放置正确次数）
 *   第4行  状态（READY / RUN / DONE 等）
 *
 * 屏幕型号确认后，只需写一个"后端"（3 个函数），用 display_set_backend() 挂上即可，
 * 上层代码不用改。没有挂屏幕时，内容会打印到串口日志。
 */
#ifndef DISPLAY_H
#define DISPLAY_H

typedef struct {
    void (*init)(void);
    void (*clear)(void);
    void (*text)(int line, const char *s);   /* 在第 line 行（0~3）显示一串 ASCII 字符 */
} display_backend_t;

void display_init(void);
void display_set_backend(const display_backend_t *b);
void display_set_task(const char *code);     /* "156+123+516+231" */
void display_set_counts(int grabs, int places);
void display_set_status(const char *s);
void display_poll(void);                     /* 由 sys_poll 调用，有变化才刷新 */

#endif
