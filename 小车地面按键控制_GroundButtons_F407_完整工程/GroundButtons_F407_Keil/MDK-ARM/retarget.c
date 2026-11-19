#include <stdio.h>
#include <rt_sys.h>
#include "port.h"
#if defined(__ARMCC_VERSION)
__asm(".global __use_no_semihosting\n");
__asm(".global __ARM_use_no_argv\n");
#endif
FILE __stdout;
FILE __stdin;
int fputc(int ch, FILE *stream) {
    (void)stream; unsigned char c=(unsigned char)ch;
    port_uart_write(PORT_UART_LOG,&c,1); return ch;
}
int fgetc(FILE *stream) { (void)stream; return EOF; }
int ferror(FILE *stream) { (void)stream; return 0; }
void _ttywrch(int ch) { (void)fputc(ch,&__stdout); }
int _sys_istty(int handle) { (void)handle; return 1; }
void _sys_exit(int code) { (void)code; for (;;) {} }
char *_sys_command_string(char *cmd, int len) { (void)len; return cmd; }
