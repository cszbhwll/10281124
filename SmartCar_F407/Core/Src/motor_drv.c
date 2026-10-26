/**
 * motor_drv.c —— 幻尔四路编码电机驱动板 I2C 通信
 */
#include "motor_drv.h"
#include "port.h"
#include "robot_config.h"
#include "sys.h"

static bool     s_ok;
static uint32_t s_err;

static int wr(uint8_t reg, const uint8_t *d, uint16_t n)
{
    int r = port_i2c_write(MOTOR_DRV_I2C_ADDR, reg, d, n);
    if (r) s_err++;
    return r;
}

int motor_drv_init(void)
{
    s_ok = false;
    if (port_i2c_probe(MOTOR_DRV_I2C_ADDR) != 0) {
        LOG("motor_drv: 0x%02X 无应答，检查接线/供电/上拉", MOTOR_DRV_I2C_ADDR);
        return -1;
    }
    uint8_t type = MOTOR_DRV_TYPE, pol = MOTOR_DRV_ENC_POLARITY;
    if (wr(MDRV_REG_TYPE, &type, 1)) return -2;
    sys_wait_ms(5);
    if (wr(MDRV_REG_POLARITY, &pol, 1)) return -3;
    sys_wait_ms(5);
    int8_t zero[4] = { 0, 0, 0, 0 };
    if (motor_drv_set_speed(zero)) return -4;
    s_ok = true;
    uint16_t mv = 0;
    if (motor_drv_read_battery_mv(&mv) == 0) LOG("motor_drv: 就绪，电池 %u mV", mv);
    return 0;
}

bool motor_drv_ok(void) { return s_ok; }
uint32_t motor_drv_error_count(void) { return s_err; }

int motor_drv_set_speed(const int8_t sp[4])
{
    uint8_t d[4];
    for (int i = 0; i < 4; i++) {
        int v = sp[i];
        if (v >  MOTOR_SPEED_LIMIT) v =  MOTOR_SPEED_LIMIT;
        if (v < -MOTOR_SPEED_LIMIT) v = -MOTOR_SPEED_LIMIT;
        d[i] = (uint8_t)(int8_t)v;
    }
    return wr(MDRV_REG_SPEED, d, 4);
}

int motor_drv_set_pwm(const int8_t pwm[4])
{
    uint8_t d[4];
    for (int i = 0; i < 4; i++) {
        int v = pwm[i];
        if (v >  100) v =  100;
        if (v < -100) v = -100;
        d[i] = (uint8_t)(int8_t)v;
    }
    return wr(MDRV_REG_PWM, d, 4);
}

int motor_drv_read_counts(int32_t cnt[4])
{
    uint8_t b[16];
    if (port_i2c_read(MOTOR_DRV_I2C_ADDR, MDRV_REG_ENC_TOTAL, b, 16)) { s_err++; return -1; }
    for (int i = 0; i < 4; i++) {
        uint32_t u = (uint32_t)b[4 * i] | ((uint32_t)b[4 * i + 1] << 8) |
                     ((uint32_t)b[4 * i + 2] << 16) | ((uint32_t)b[4 * i + 3] << 24);
        cnt[i] = (int32_t)u;
    }
    return 0;
}

int motor_drv_read_battery_mv(uint16_t *mv)
{
    uint8_t b[2];
    if (port_i2c_read(MOTOR_DRV_I2C_ADDR, MDRV_REG_BATTERY, b, 2)) { s_err++; return -1; }
    *mv = (uint16_t)(b[0] | (b[1] << 8));
    return 0;
}
