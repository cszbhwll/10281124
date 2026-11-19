#include "sys.h"
#include "port.h"
#include <stdarg.h>
#include <stdio.h>

/* Only used for the two short setup delays in motor_drv_init(), while stopped. */
void sys_wait_ms(uint32_t ms)
{
    uint32_t start = port_millis();
    while ((uint32_t)(port_millis() - start) < ms) port_idle();
}

void log_printf(const char *fmt, ...)
{
    char buf[240];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 3) n = (int)sizeof(buf) - 3;
    buf[n++] = '\r'; buf[n++] = '\n';
    port_uart_write(PORT_UART_LOG, (const uint8_t *)buf, (uint16_t)n);
}
