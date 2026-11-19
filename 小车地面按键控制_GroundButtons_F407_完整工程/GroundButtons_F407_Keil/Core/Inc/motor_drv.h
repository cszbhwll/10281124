/**
 * motor_drv.h —— 幻尔四路编码电机驱动板（I2C 地址 0x34）
 *
 * 寄存器（十进制地址，来自厂家示例）：
 *   0   电池电压         读 2 字节，小端，单位 mV（【确认】按实物读数验证）
 *   20  电机类型         写 1 字节
 *   21  编码器极性       写 1 字节（0/1）
 *   31  固定 PWM         写 4 字节 int8，-100~100，开环
 *   51  固定速度         写 4 字节 int8，单位"编码器脉冲/10ms"，闭环（驱动板内部自己做 PID）
 *   60  累计脉冲         读 16 字节 = 4 个 int32 小端
 */
#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#include <stdint.h>
#include <stdbool.h>

#define MDRV_REG_BATTERY    0
#define MDRV_REG_TYPE       20
#define MDRV_REG_POLARITY   21
#define MDRV_REG_PWM        31
#define MDRV_REG_SPEED      51
#define MDRV_REG_ENC_TOTAL  60

int  motor_drv_init(void);                       /* 0=成功 */
bool motor_drv_ok(void);
int  motor_drv_set_speed(const int8_t sp[4]);    /* 通道 M1..M4 的目标速度（脉冲/10ms） */
int  motor_drv_set_pwm(const int8_t pwm[4]);     /* 开环 PWM，调试用 */
int  motor_drv_read_counts(int32_t cnt[4]);      /* 累计脉冲 */
int  motor_drv_read_battery_mv(uint16_t *mv);
uint32_t motor_drv_error_count(void);

#endif
