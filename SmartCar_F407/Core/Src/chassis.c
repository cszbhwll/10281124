/**
 * chassis.c —— 麦轮运动学、速度下发、编码器里程计
 */
#include "chassis.h"
#include "motor_drv.h"
#include "robot_config.h"
#include "sys.h"
#include <string.h>

static const int8_t WHEEL_CH[4]   = WHEEL_CH_INIT;
static const int8_t WHEEL_SIGN[4] = WHEEL_SIGN_INIT;

#define MM_PER_COUNT   ((PI_F * WHEEL_DIAMETER_MM) / ENC_COUNTS_PER_WHEEL_REV)
#define K_ROT          (CHASSIS_HALF_TRACK_MM + CHASSIS_HALF_BASE_MM)

static pose_t  s_pose;
static float   s_cmd_vx, s_cmd_vy, s_cmd_w;      /* 上层要求的目标速度 */
static float   s_out_vx, s_out_vy, s_out_w;      /* 限加速度后实际下发的速度 */
static float   s_meas_vx, s_meas_vy, s_meas_w;   /* 编码器测得的速度 */
static float   s_frac[4];                        /* 速度量化的小数部分累积（见 send_wheels） */
static int32_t s_last_cnt[4];
static int32_t s_wheel_total[4];
static bool    s_have_cnt;
static bool    s_enabled = true;
static float   s_imu_offset;
static bool    s_imu_ref;

/* 陀螺仪函数在 imu_mpu6050.c 里：USE_MPU6050=0 时那里提供"不工作"的空实现 */

void chassis_init(void)
{
    memset(&s_pose, 0, sizeof s_pose);
    s_cmd_vx = s_cmd_vy = s_cmd_w = 0;
    s_out_vx = s_out_vy = s_out_w = 0;
    s_have_cnt = false;
    memset(s_frac, 0, sizeof s_frac);
}

void chassis_enable(bool en) { s_enabled = en; if (!en) chassis_stop_now(); }

void chassis_set_velocity(float vx, float vy, float w)
{
    s_cmd_vx = vx; s_cmd_vy = vy; s_cmd_w = w;
}

void chassis_stop_now(void)
{
    s_cmd_vx = s_cmd_vy = s_cmd_w = 0;
    s_out_vx = s_out_vy = s_out_w = 0;
    int8_t z[4] = { 0, 0, 0, 0 };
    if (motor_drv_ok()) motor_drv_set_speed(z);
}

pose_t chassis_pose(void) { return s_pose; }

void chassis_set_pose(pose_t p)
{
    s_pose = p;
    float yaw;
    if (imu_get_yaw(&yaw)) { s_imu_offset = wrap_pi(p.th - yaw); s_imu_ref = true; }
}

void chassis_measured_velocity(float *vx, float *vy, float *w)
{
    *vx = s_meas_vx; *vy = s_meas_vy; *w = s_meas_w;
}

int32_t chassis_wheel_count(int wheel) { return s_wheel_total[wheel & 3]; }

/* 把四个轮子的线速度(mm/s)换成驱动板单位(脉冲/10ms)发出去 */
static void send_wheels(const float wheel_mmps[4])
{
    int8_t ch_speed[4] = { 0, 0, 0, 0 };
    for (int i = 0; i < 4; i++) {
        float u = wheel_mmps[i] / MM_PER_COUNT * 0.01f;   /* 脉冲/10ms，可能带小数 */
        /*
         * 驱动板只收整数，1 个单位约等于 12.6mm/s，低速时误差很大。
         * 办法：把每次舍掉的小数部分累积起来，攒够 1 就多发 1——平均下来速度就准了。
         */
        u += s_frac[i];
        float r = roundf(u);
        s_frac[i] = clampf(u - r, -1.0f, 1.0f);
        int v = (int)r;
        if (v >  MOTOR_SPEED_LIMIT) v =  MOTOR_SPEED_LIMIT;
        if (v < -MOTOR_SPEED_LIMIT) v = -MOTOR_SPEED_LIMIT;
        ch_speed[WHEEL_CH[i]] = (int8_t)(v * WHEEL_SIGN[i]);
    }
    motor_drv_set_speed(ch_speed);
}

/* 限制速度变化率：避免猛起猛停导致打滑、物料在槽里晃 */
static float approach(float cur, float target, float max_step)
{
    if (target > cur + max_step) return cur + max_step;
    if (target < cur - max_step) return cur - max_step;
    return target;
}

static void update_odometry(float dt)
{
    int32_t cnt[4];
    if (!motor_drv_ok() || motor_drv_read_counts(cnt) != 0) return;
    if (!s_have_cnt) { memcpy(s_last_cnt, cnt, sizeof cnt); s_have_cnt = true; return; }

    float d[4];      /* 每个轮子本周期前进了多少 mm */
    for (int i = 0; i < 4; i++) {
        int ch = WHEEL_CH[i];
        int32_t dc = (cnt[ch] - s_last_cnt[ch]) * WHEEL_SIGN[i];
        s_wheel_total[i] += dc;
        d[i] = (float)dc * MM_PER_COUNT;
    }
    memcpy(s_last_cnt, cnt, sizeof cnt);

    /* 正运动学：由四轮位移求车体位移（车体坐标） */
    float dx  = ( d[WHEEL_FL] + d[WHEEL_FR] + d[WHEEL_RL] + d[WHEEL_RR]) * 0.25f * FWD_SCALE;
    float dy  = (-d[WHEEL_FL] + d[WHEEL_FR] + d[WHEEL_RL] - d[WHEEL_RR]) * 0.25f * STRAFE_SCALE;
    float dth = (-d[WHEEL_FL] + d[WHEEL_FR] - d[WHEEL_RL] + d[WHEEL_RR]) * 0.25f / K_ROT * ROT_SCALE;

    /* 有陀螺仪就用陀螺仪的角度（更准），否则用轮子算出来的 */
    float yaw;
    float th_new = s_pose.th + dth;
    if (s_imu_ref && imu_get_yaw(&yaw)) th_new = wrap_pi(yaw + s_imu_offset);
    float th_mid = s_pose.th + wrap_pi(th_new - s_pose.th) * 0.5f;

    /* 转到场地坐标累加（用中间时刻的朝向，转弯时更准） */
    s_pose.x += cosf(th_mid) * dx - sinf(th_mid) * dy;
    s_pose.y += sinf(th_mid) * dx + cosf(th_mid) * dy;
    s_pose.th = wrap_pi(th_new);

    if (dt > 0.0f) { s_meas_vx = dx / dt; s_meas_vy = dy / dt; s_meas_w = dth / dt; }
}

void chassis_update(float dt)
{
    imu_update();
    update_odometry(dt);
    if (!s_enabled || !motor_drv_ok()) return;

    /* 加速度限制（线速度按合成向量限，角速度单独限） */
    float dvx = s_cmd_vx - s_out_vx, dvy = s_cmd_vy - s_out_vy;
    float dv = hypotf(dvx, dvy), maxdv = NAV_ACC_MMPS2 * dt;
    if (dv > maxdv) { s_out_vx += dvx * maxdv / dv; s_out_vy += dvy * maxdv / dv; }
    else            { s_out_vx = s_cmd_vx;        s_out_vy = s_cmd_vy; }
    s_out_w = approach(s_out_w, s_cmd_w, DEG2RAD(NAV_WACC_DPS2) * dt);

    /* 逆运动学：车体速度 → 四个轮子线速度 */
    float k = K_ROT / ROT_SCALE;
    float vx = s_out_vx / FWD_SCALE, vy = s_out_vy / STRAFE_SCALE;   /* 打滑多的方向，轮子要多转一点 */
    float w[4];
    w[WHEEL_FL] = vx - vy - k * s_out_w;
    w[WHEEL_FR] = vx + vy + k * s_out_w;
    w[WHEEL_RL] = vx + vy - k * s_out_w;
    w[WHEEL_RR] = vx - vy + k * s_out_w;

    /* 有轮子超过最大速度时，四个轮子按比例一起缩小，保证方向不变 */
    float vmax_wheel = (float)MOTOR_SPEED_LIMIT * MM_PER_COUNT * 100.0f;
    float m = 0;
    for (int i = 0; i < 4; i++) if (fabsf(w[i]) > m) m = fabsf(w[i]);
    if (m > vmax_wheel) for (int i = 0; i < 4; i++) w[i] *= vmax_wheel / m;

    send_wheels(w);
}
