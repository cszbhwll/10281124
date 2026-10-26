/**
 * console.h —— 串口调试命令（电脑串口助手 115200，发送时勾选"发送新行"）
 * 输入 help 查看全部命令。调试顺序见 README 第 4 节。
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>

void console_poll(void);              /* 由 sys_poll 调用：只负责收字符；"stop" 立即生效 */
bool console_execute_pending(void);   /* 由主循环调用：执行收到的一条命令；执行了返回 true */

#endif
