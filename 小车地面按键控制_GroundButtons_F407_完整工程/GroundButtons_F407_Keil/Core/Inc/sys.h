#ifndef DEBUG_SYS_H
#define DEBUG_SYS_H
#include <stdint.h>
void sys_wait_ms(uint32_t ms);
void log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#define LOG(...) log_printf(__VA_ARGS__)
#endif
