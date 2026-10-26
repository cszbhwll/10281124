/**
 * port.h —— 硬件抽象层（所有和具体芯片/引脚打交道的函数都在这里声明）
 *
 * 为什么要有这一层？
 *   上层代码（底盘、机械臂、比赛流程）只调用这里的函数，不直接碰 HAL 库。
 *   这样同一套上层代码既能在 STM32 上跑（port_stm32.c 实现），
 *   也能在电脑上用模拟器跑（sim/port_sim.c 实现），先在电脑上把流程验证对。
 */
#ifndef PORT_H
#define PORT_H

#include <stdint.h>
#include <stdbool.h>

/* 串口通道编号 */
typedef enum {
    PORT_UART_LOG = 0,     /* USART1 PA9/PA10，经板载 CH340 接电脑：打印日志、输入调试命令 */
    PORT_UART_SERVO_ST,    /* USART2 PA2/PA3：ST3215 大舵机总线（经舵机驱动板，12V 支路） */
    PORT_UART_SERVO_SC,    /* USART6 PC6/PC7：SC09 夹爪舵机总线（经另一块舵机驱动板，6V 支路） */
    PORT_UART_VISION,      /* USART3 PB10/PB11：摄像头/视觉模块 */
    PORT_UART_COUNT
} port_uart_t;

/* 按键编号（探索者板载按键） */
typedef enum {
    PORT_KEY_START = 0,    /* KEY0 PE4：一键启动 */
    PORT_KEY_ZONE,         /* KEY1 PE3：切换出发启停区 1/2 */
    PORT_KEY_AUX,          /* KEY2 PE2：备用 */
    PORT_KEY_WKUP,         /* WK_UP PA0：备用（按下为高电平） */
    PORT_KEY_COUNT
} port_key_t;

void     port_init(void);

/* 时间 */
uint32_t port_millis(void);            /* 上电后经过的毫秒数 */
void     port_idle(void);              /* 主循环空闲时调用：STM32 上什么都不做；模拟器里推进 1 ms 仿真时间 */

/* I2C（7 位地址）；返回 0 表示成功 */
int      port_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len);
int      port_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);
int      port_i2c_probe(uint8_t addr7);

/* 串口：发送为阻塞式；接收由中断放进环形缓冲区，这里取出 */
void     port_uart_write(port_uart_t ch, const uint8_t *data, uint16_t len);
int      port_uart_read_byte(port_uart_t ch, uint8_t *out);   /* 取到 1 个字节返回 1，没有返回 0 */
void     port_uart_flush_rx(port_uart_t ch);

/* 按键、指示灯、蜂鸣器 */
bool     port_key_down(port_key_t key);    /* 当前是否按下（未消抖） */
void     port_led(int idx, bool on);       /* idx 0 = 红灯 DS0(PF9)，1 = 绿灯 DS1(PF10) */
void     port_beep(bool on);               /* PF8 */

#endif
