/**
 * bus_servo.h —— 飞特(Feetech)协议总线舵机：ST3215（STS 系列）与 SC09（SCS 系列）
 *
 * 数据包格式：0xFF 0xFF ID 长度 指令 参数... 校验
 *   长度 = 参数个数 + 2；校验 = ~(ID + 长度 + 指令 + 所有参数) 的低 8 位
 * 两个系列的区别：
 *   STS（ST3215）：位置 0~4095 对应 360°，两字节数据"低字节在前"
 *   SCS（SC09）  ：位置 0~1023 对应 300°，两字节数据"高字节在前"
 * 波特率默认 1Mbps。舵机驱动板（Bus Servo Adapter A）负责把串口转成单线总线。
 */
#ifndef BUS_SERVO_H
#define BUS_SERVO_H

#include <stdint.h>
#include <stdbool.h>
#include "port.h"

typedef enum { SERVO_STS = 0, SERVO_SCS = 1 } servo_proto_t;

typedef struct {
    port_uart_t   uart;
    servo_proto_t proto;
    uint8_t       id;
} servo_t;

/* 常用寄存器地址（两个系列相同的部分） */
#define SREG_ID            5
#define SREG_TORQUE        40
#define SREG_ACC           41      /* 仅 STS */
#define SREG_GOAL_POS      42
#define SREG_GOAL_TIME     44
#define SREG_GOAL_SPEED    46
#define SREG_LOCK          55      /* STS: 0 解锁 EEPROM；SCS 为 48 */
#define SREG_PRESENT_POS   56
#define SREG_PRESENT_LOAD  60
#define SREG_VOLTAGE       62
#define SREG_TEMP          63
#define SREG_MOVING        66

int  servo_ping(const servo_t *s);
int  servo_torque(const servo_t *s, bool on);
int  servo_write_pos(const servo_t *s, int pos, int speed, int acc);  /* speed：计数/秒，0 表示最快 */
int  servo_read_pos(const servo_t *s, int *pos);
int  servo_read_byte(const servo_t *s, uint8_t reg, uint8_t *val);
int  servo_set_id(const servo_t *s, uint8_t new_id);                  /* 总线上只接这一个舵机时用 */
int  servo_define_middle(const servo_t *s);                           /* 仅 STS：把舵机当前位置定义为 2048（中位校准，掉电保存） */

/* 同时给同一总线上多个 STS 舵机下发位置（同步写，几个关节一起动） */
int  servo_sync_write_pos(port_uart_t uart, const uint8_t *ids, const int *pos,
                          const int *speed, int acc, int n);

#endif
