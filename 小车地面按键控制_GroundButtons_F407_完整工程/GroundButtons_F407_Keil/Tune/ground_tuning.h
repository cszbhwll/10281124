/* 落地调试日常参数：只改本文件，保存 -> Rebuild -> Download -> 复位。 */
#ifndef GROUND_TUNING_H
#define GROUND_TUNING_H

/* 已在用户硬件上完成四轮闭环测试。保持目前电机板设置。 */
#define MOTOR_DRV_I2C_ADDR       0x34
#define MOTOR_DRV_TYPE           3
#define MOTOR_DRV_ENC_POLARITY   0
#define LOG_BAUD                 115200
#define I2C_CLOCK_HZ             100000
#define BOARD_HSE_HZ             8000000U

/* 顺序FL、FR、RL、RR；照片下方是车头，车辆左侧在照片右侧。 */
#define WHEEL_CH_INIT            {2, 3, 0, 1}     /* M3 M4 M1 M2 */
#define WHEEL_SIGN_INIT          {-1, -1, -1, -1}
/* 可按实测分别微调，整数速度量化会限制低速微调效果；不能代替航向闭环。 */
#define WHEEL_TRIM_PCT_INIT      {100, 100, 100, 100}

/* 速度单位：编码器脉冲/10ms；不是PWM、RPM或mm/s。 */
#define MOTOR_SPEED_LIMIT        6
#define GROUND_DEFAULT_SPEED     3
#define GROUND_DEFAULT_TIME_MS   400U
#define GROUND_MAX_TIME_MS       1500U
#define GROUND_ENABLE_WINDOW_MS  15000U  /* enable后15秒内可启动一次动作 */
#define GROUND_REFRESH_MS        20U
#define GROUND_SLEW_STEP_MS      80U     /* 每80ms最多改变一个速度单位 */
#define GROUND_ENCODER_MS        50U
#define GROUND_STALL_GRACE_MS    250U
#define GROUND_STALL_TIMEOUT_MS  400U    /* 非零输出且相应通道持续无计数变化则锁定 */
#define GROUND_WATCH_MS          500U

/* KEY0前进、KEY1/WK_UP原地左转/右转；动作短且限速。 */
#define GROUND_BUTTON_SPEED      2
#define GROUND_BUTTON_TIME_MS    350U
#define GROUND_KEY_DEBOUNCE_MS   30U

#if MOTOR_SPEED_LIMIT < 1 || MOTOR_SPEED_LIMIT > 20
#error Ground_speed_limit_must_be_1_to_20
#endif
#if GROUND_DEFAULT_SPEED < 1 || GROUND_DEFAULT_SPEED > MOTOR_SPEED_LIMIT
#error Invalid_ground_default_speed
#endif
#if GROUND_DEFAULT_TIME_MS < 100 || GROUND_DEFAULT_TIME_MS > GROUND_MAX_TIME_MS || GROUND_MAX_TIME_MS > 3000
#error Invalid_ground_duration
#endif
#if GROUND_REFRESH_MS < 5 || GROUND_SLEW_STEP_MS < GROUND_REFRESH_MS || GROUND_ENCODER_MS < GROUND_REFRESH_MS
#error Invalid_ground_control_period
#endif
#if GROUND_STALL_TIMEOUT_MS <= GROUND_ENCODER_MS || GROUND_STALL_GRACE_MS < GROUND_ENCODER_MS
#error Invalid_ground_stall_period
#endif
#if GROUND_ENABLE_WINDOW_MS < 1000 || GROUND_ENABLE_WINDOW_MS > 60000 || BOARD_HSE_HZ != 8000000U
#error Invalid_ground_enable_window_or_clock
#endif
#if GROUND_BUTTON_SPEED < 1 || GROUND_BUTTON_SPEED > MOTOR_SPEED_LIMIT || GROUND_BUTTON_TIME_MS < 100 || GROUND_BUTTON_TIME_MS > GROUND_MAX_TIME_MS
#error Invalid_button_speed_or_duration
#endif
#if GROUND_KEY_DEBOUNCE_MS < 10 || GROUND_KEY_DEBOUNCE_MS > 200
#error Invalid_key_debounce
#endif
#endif
