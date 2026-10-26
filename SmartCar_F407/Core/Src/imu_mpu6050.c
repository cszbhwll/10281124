/**
 * imu_mpu6050.c —— MPU6050 陀螺仪测车头朝向（GY-521 模块）
 *
 * 为什么需要？麦轮会打滑，尤其横移和原地转身时。只靠轮子算朝向，转几次身就会偏几度，
 * 车走 1 米就横向偏出几十毫米。陀螺仪直接测"转了多少度"，不受打滑影响。
 *
 * 接线（GY-521 模块）：VCC→开发板 3.3V，GND→GND，SCL→PB8，SDA→PB9。
 *   XDA、XCL、AD0、INT 都不接（模块上 AD0 已下拉，地址就是 0x68）。
 *   与电机驱动板（0x34）、板载 24C02（0x50）同一条 I2C 总线，地址不冲突。
 * 安装：模块平放、芯片朝上，用 M3 螺丝固定牢靠，尽量远离电机。芯片朝下安装的话把
 *   robot_config.h 里的 IMU_YAW_SIGN 改成 -1。
 * 启用：robot_config.h 里 USE_MPU6050 = 1（默认已开）。没接模块时程序会提示"无应答"，
 *   自动退回只用轮子算朝向，不会卡死。
 *
 * 零漂：陀螺仪静止时读数也不是 0，这个偏差叫零漂，会随温度慢慢变。
 *   上电时静止 1 秒测一次；之后在待机（没出发）期间，只要车不动就一直慢慢跟踪修正，
 *   所以上电后放在启停区多等一会儿反而更准。
 * 比例：每颗芯片的灵敏度有 ±3% 左右的误差，转 180° 可能差 5°，必须用 spin 命令标定
 *   IMU_SCALE（见 README 第 5 节"陀螺仪标定"）。
 */
#include "robot_config.h"

#if USE_MPU6050
#include "chassis.h"
#include "port.h"
#include "sys.h"
#include <math.h>

#define MPU_ADDR         0x68
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_GYRO_ZOUT_H  0x47
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
#define GYRO_LSB_PER_DPS 65.5f      /* 量程 ±500°/s */
#define FAIL_LIMIT       5          /* 连续读失败 5 次（约 0.1 秒）就认为陀螺仪掉线 */

static bool     s_ok;
static bool     s_off;              /* 调试时用 imu off 手动关掉 */
static bool     s_track;            /* 待机时自动跟踪零漂 */
static float    s_bias;             /* 零漂（°/s，原始读数） */
static float    s_yaw;              /* 积分得到的角度（弧度，-π~π） */
static float    s_total_deg;        /* 累计转过的角度（度，不回绕，标定用） */
static uint32_t s_last;
static int      s_fail, s_fail_total;

static int read_gz_raw(float *dps)
{
    uint8_t b[2];
    if (port_i2c_read(MPU_ADDR, REG_GYRO_ZOUT_H, b, 2)) return -1;
    int16_t raw = (int16_t)((b[0] << 8) | b[1]);
    *dps = (float)raw / GYRO_LSB_PER_DPS;
    return 0;
}

int imu_init(void)
{
    uint8_t v = 0;
    s_ok = false;
    if (port_i2c_probe(MPU_ADDR) != 0) { LOG("imu: MPU6050 无应答（没接或接线错），只用轮子算朝向"); return -1; }
    port_i2c_read(MPU_ADDR, REG_WHO_AM_I, &v, 1);
    /* 正品读 0x68；国产兼容芯片可能读 0x70/0x72/0x98 等，只要角速度正常就能用 */
    LOG("imu: WHO_AM_I = 0x%02X%s", v, v == 0x68 ? "" : "（不是 0x68，可能是兼容芯片，用 imu 命令检查读数）");
    v = 0x80; port_i2c_write(MPU_ADDR, REG_PWR_MGMT_1, &v, 1);   /* 复位 */
    sys_wait_ms(100);
    v = 0x01; port_i2c_write(MPU_ADDR, REG_PWR_MGMT_1, &v, 1);   /* 唤醒，用陀螺仪 X 轴时钟（比内部振荡器稳） */
    sys_wait_ms(50);
    v = 0x04; port_i2c_write(MPU_ADDR, REG_CONFIG, &v, 1);       /* 低通滤波 ~21Hz：程序每 20ms 读一次，滤掉更快的振动 */
    v = 0x04; port_i2c_write(MPU_ADDR, REG_SMPLRT_DIV, &v, 1);   /* 采样 200Hz */
    v = 0x08; port_i2c_write(MPU_ADDR, REG_GYRO_CONFIG, &v, 1);  /* ±500°/s */
    sys_wait_ms(100);

    /* 静止 1 秒测零漂 */
    float sum = 0;
    int n = 0;
    for (int i = 0; i < 200; i++) {
        float d;
        if (read_gz_raw(&d) == 0) { sum += d; n++; }
        sys_wait_ms(5);
    }
    if (n < 150) { LOG("imu: 读数失败，只用轮子算朝向"); return -2; }
    s_bias = sum / (float)n;
    if (fabsf(s_bias) > 5.0f) LOG("imu: 零漂 %d°/s 偏大，上电时车可能在动，或芯片有问题", (int)s_bias);
    s_yaw = 0;
    s_total_deg = 0;
    s_fail = 0;
    s_last = sys_now();
    s_ok = true;
    s_track = true;
    LOG("imu: 就绪，零漂 %c%d.%02d °/s", s_bias < 0 ? '-' : '+', (int)fabsf(s_bias), (int)(fabsf(s_bias) * 100) % 100);
    return 0;
}

/* 由底盘每个控制周期（20ms）调用：读角速度并积分 */
void imu_update(void)
{
    if (!s_ok) return;
    float raw;
    if (read_gz_raw(&raw) != 0) {
        /* 读失败：这段时间不积分，也不更新 s_last，下次成功时把整段时间补上 */
        s_fail_total++;
        if (++s_fail >= FAIL_LIMIT) {
            s_ok = false;
            LOG("imu: 连续读不到数据，判定陀螺仪掉线，改用轮子算朝向（检查 PB8/PB9 杜邦线是否松动）");
        }
        return;
    }
    s_fail = 0;
    uint32_t now = sys_now();
    float dt = (float)(now - s_last) / 1000.0f;
    s_last = now;
    if (dt > 0.2f) dt = 0.2f;                           /* 防止长时间没调用后一次积太多 */

    float d = raw - s_bias;
    /* 待机时车不动：让零漂慢慢跟上温度变化（时间常数约 10 秒） */
    if (s_track && fabsf(d) < 0.5f) s_bias += 0.002f * d;
    if (fabsf(d) < 0.3f) d = 0;                         /* 很小的抖动当成静止，减少漂移 */
    d *= IMU_SCALE * (float)IMU_YAW_SIGN;
    s_total_deg += d * dt;
    s_yaw = wrap_pi(s_yaw + DEG2RAD(d) * dt);
}

bool imu_get_yaw(float *yaw)
{
    if (!s_ok || s_off) return false;
    *yaw = s_yaw;
    return true;
}

void imu_track_bias(bool on) { s_track = on; }

void imu_debug(int action)
{
    if (action == 1) { s_total_deg = 0; LOG("imu: 累计角度清零"); }
    if (action == 2) { s_off = true;  LOG("imu: 已手动关闭，朝向改用轮子计算（标定 ROT_SCALE 用）"); }
    if (action == 3) {
        s_off = false;
        chassis_set_pose(chassis_pose());              /* 重新对齐陀螺仪和当前朝向 */
        LOG("imu: 已重新打开");
    }
    if (!s_ok) { LOG("imu: 没有工作（未接、掉线或 USE_MPU6050=0）；读失败累计 %d 次", s_fail_total); return; }
    float r = 0;
    read_gz_raw(&r);
    LOG("imu: 角度 %d°  累计 %d.%d°  当前角速度 %d°/s  零漂 %d.%02d°/s  读失败累计 %d 次%s",
        (int)RAD2DEG(s_yaw), (int)s_total_deg, (int)fabsf(s_total_deg * 10) % 10,
        (int)((r - s_bias) * IMU_SCALE * IMU_YAW_SIGN),
        (int)s_bias, (int)(fabsf(s_bias) * 100) % 100, s_fail_total, s_off ? "  [已手动关闭]" : "");
}

#else
/* 没启用陀螺仪：提供空实现，imu_get_yaw() 返回 false，底盘只用轮子算朝向 */
#include "chassis.h"
#include "sys.h"
int  imu_init(void) { return -1; }
void imu_update(void) {}
void imu_track_bias(bool on) { (void)on; }
void imu_debug(int action) { (void)action; LOG("imu: 程序里没有启用陀螺仪（USE_MPU6050=0）"); }
#ifndef SIM_HAS_IMU          /* 电脑模拟器自己模拟陀螺仪（world.c），那时不在这里定义 */
bool imu_get_yaw(float *yaw) { (void)yaw; return false; }
#endif
#endif
