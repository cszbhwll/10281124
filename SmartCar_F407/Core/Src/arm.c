/**
 * arm.c —— 机械臂运动学与取放动作
 *
 * 关节定义：
 *   q[0] 回转角：小臂指向车头为 0，逆时针（向车左侧）为正
 *   q[1] 肩角：  大臂水平向前为 0，向上抬为正
 *   q[2] 肘角：  小臂与大臂共线为 0，向上折为正，向下折为负
 */
#include "arm.h"
#include "bus_servo.h"
#include "robot_config.h"
#include "geom.h"
#include "sys.h"

enum { J_YAW = 0, J_SHOULDER, J_ELBOW, J_GRIP };

static const servo_t SV[4] = {
    { PORT_UART_SERVO_ST, SERVO_STS, SERVO_ID_YAW },
    { PORT_UART_SERVO_ST, SERVO_STS, SERVO_ID_SHOULDER },
    { PORT_UART_SERVO_ST, SERVO_STS, SERVO_ID_ELBOW },
    { PORT_UART_GRIP,     SERVO_SCS, SERVO_ID_GRIP },
};
static const int ZERO[3] = { YAW_ZERO_RAW, SHOULDER_ZERO_RAW, ELBOW_ZERO_RAW };
static const int DIR[3]  = { YAW_DIR, SHOULDER_DIR, ELBOW_DIR };
static const float QMIN[3] = { DEG2RAD(YAW_MIN_DEG), DEG2RAD(SHOULDER_MIN_DEG), DEG2RAD(ELBOW_MIN_DEG) };
static const float QMAX[3] = { DEG2RAD(YAW_MAX_DEG), DEG2RAD(SHOULDER_MAX_DEG), DEG2RAD(ELBOW_MAX_DEG) };

static bool s_ok;
static int  s_raw[3];          /* 最后一次下发的三个关节位置 */

/* ---------------- 运动学 ---------------- */

static int q_to_raw(int j, float q) { return ZERO[j] + (int)lroundf((float)DIR[j] * q * ST_COUNTS_PER_RAD); }
static float raw_to_q(int j, int raw) { return (float)(raw - ZERO[j]) / ((float)DIR[j] * ST_COUNTS_PER_RAD); }

/*
 * 俯视看：夹持中心 = 回转轴 + r·(小臂伸出方向) + ARM_TOOL_SIDE_MM·(伸出方向左转 90°)
 *   r 是在大臂/小臂摆动平面里量的水平伸出距离（含肩轴相对回转轴的偏移 ARM_SHOULDER_R0_MM）
 */
void arm_fk(const float q[3], vec3_t *p)
{
    float r = ARM_SHOULDER_R0_MM + ARM_L1_MM * cosf(q[1]) + ARM_L2_MM * cosf(q[1] + q[2]);
    p->z = ARM_SHOULDER_Z_MM + ARM_L1_MM * sinf(q[1]) + ARM_L2_MM * sinf(q[1] + q[2]);
    p->x = ARM_BASE_X_MM + r * cosf(q[0]) - ARM_TOOL_SIDE_MM * sinf(q[0]);
    p->y = ARM_BASE_Y_MM + r * sinf(q[0]) + ARM_TOOL_SIDE_MM * cosf(q[0]);
}

bool arm_ik(vec3_t p, float q[3])
{
    float dx = p.x - ARM_BASE_X_MM, dy = p.y - ARM_BASE_Y_MM;
    float c = hypotf(dx, dy);                               /* 回转轴到目标的水平距离 */
    if (c <= fabsf(ARM_TOOL_SIDE_MM)) return false;
    float reach = sqrtf(c * c - ARM_TOOL_SIDE_MM * ARM_TOOL_SIDE_MM);   /* 摆动平面内的水平伸出距离 */
    q[0] = atan2f(dy, dx) - atan2f(ARM_TOOL_SIDE_MM, reach); /* 回转：夹爪侧偏 16mm，所以要比"正对目标"少转一点 */
    if (q[0] < QMIN[0]) q[0] += 2.0f * PI_F;                /* 换算到回转的工作范围内（例如 -177° → 183°） */
    if (q[0] > QMAX[0]) q[0] -= 2.0f * PI_F;
    float r = reach - ARM_SHOULDER_R0_MM;                   /* 肩轴到目标的水平距离 */
    float h = p.z - ARM_SHOULDER_Z_MM;                      /* 肩轴到目标的竖直距离 */
    float d2 = r * r + h * h;
    /* 余弦定理求肘角 */
    float c3 = (d2 - ARM_L1_MM * ARM_L1_MM - ARM_L2_MM * ARM_L2_MM) / (2.0f * ARM_L1_MM * ARM_L2_MM);
    if (c3 > 1.0f || c3 < -1.0f) return false;              /* 太远或太近，够不到 */
    q[2] = -acosf(c3);                                      /* 取"肘部朝上"的解（和动画一致） */
    q[1] = atan2f(h, r) - atan2f(ARM_L2_MM * sinf(q[2]), ARM_L1_MM + ARM_L2_MM * cosf(q[2]));
    for (int j = 0; j < 3; j++) {
        if (q[j] < QMIN[j] || q[j] > QMAX[j]) return false;  /* 超出软限位 */
        int raw = q_to_raw(j, q[j]);
        if (raw < 0 || raw > 4095) return false;             /* 超出舵机行程：ZERO 标定不合理 */
    }
    return true;
}

vec3_t arm_current_point(void)
{
    float q[3];
    for (int j = 0; j < 3; j++) q[j] = raw_to_q(j, s_raw[j]);
    vec3_t p;
    arm_fk(q, &p);
    return p;
}

/* ---------------- 底层动作 ---------------- */

/* 三个关节同时出发、同时到达：按最大的转动量算时间，再给每个关节分配速度 */
static int move_raw(const int target[3])
{
    int maxd = 0;
    for (int j = 0; j < 3; j++) {
        int d = target[j] - s_raw[j];
        if (d < 0) d = -d;
        if (d > maxd) maxd = d;
    }
    if (maxd < 3) return 0;
    float T = (float)maxd / (float)ARM_SPEED_RAW_PER_S;     /* 秒 */
    uint8_t ids[3] = { SERVO_ID_YAW, SERVO_ID_SHOULDER, SERVO_ID_ELBOW };
    int spd[3];
    for (int j = 0; j < 3; j++) {
        int d = target[j] - s_raw[j];
        if (d < 0) d = -d;
        spd[j] = (int)((float)d / T) + 20;
    }
    servo_sync_write_pos(PORT_UART_SERVO_ST, ids, target, spd, ARM_ACC, 3);
    for (int j = 0; j < 3; j++) s_raw[j] = target[j];
    /* 先等理论时间（含加速段），再查询舵机"是否还在动"，三个都停了才算完成 */
    float ramp = (float)ARM_SPEED_RAW_PER_S / ((float)ARM_ACC * 100.0f);
    sys_wait_ms((uint32_t)((T + ramp) * 1000.0f));
    uint32_t t0 = sys_now();
    while ((uint32_t)(sys_now() - t0) < 400) {
        bool moving = false;
        for (int j = 0; j < 3; j++) {
            uint8_t m = 0;
            if (servo_read_byte(&SV[j], SREG_MOVING, &m) == 0 && m) moving = true;
        }
        if (!moving) break;
        sys_wait_ms(10);
    }
    return 0;
}

int arm_move_to(vec3_t p)
{
    float q[3];
    if (!arm_ik(p, q)) {
        LOG("arm: 够不到 (%d,%d,%d)", (int)p.x, (int)p.y, (int)p.z);
        return -2;
    }
    int raw[3];
    for (int j = 0; j < 3; j++) raw[j] = q_to_raw(j, q[j]);
    return move_raw(raw);
}

/* 目标正上方的"过渡点"：不低于安全高度 */
static vec3_t above_of(vec3_t p)
{
    vec3_t a = { p.x, p.y, p.z + ARM_APPROACH_DZ_MM };
    if (a.z < ARM_SAFE_Z_MM) a.z = ARM_SAFE_Z_MM;
    return a;
}

int arm_move_safe(vec3_t p)
{
    vec3_t cur = arm_current_point();
    if (cur.z < ARM_SAFE_Z_MM - 5.0f) {                        /* 1. 当前位置偏低：先原地抬高 */
        vec3_t up = { cur.x, cur.y, ARM_SAFE_Z_MM };
        if (arm_move_to(up) != 0 && arm_home() != 0) return -2;
    }
    if (arm_move_to(above_of(p)) != 0) return -2;             /* 2. 在高处转到目标上方 */
    return arm_move_to(p);                                    /* 3. 下降到目标 */
}

int arm_grip_open(void)
{
    servo_write_pos(&SV[J_GRIP], GRIP_RAW_OPEN, 0, 0);
    sys_wait_ms(GRIP_TIME_MS);
    return 0;
}

int arm_grip_close(bool *has_object)
{
    servo_write_pos(&SV[J_GRIP], GRIP_RAW_CLOSE, 0, 0);
    sys_wait_ms(GRIP_TIME_MS);
    int pos;
    bool has = true;
    if (servo_read_pos(&SV[J_GRIP], &pos) == 0) {
        int d = pos - GRIP_RAW_EMPTY_CLOSED;
        if (d < 0) d = -d;
        has = d > GRIP_EMPTY_THRESHOLD;                       /* 没被物料挡住、合到了底 → 空夹 */
    }
    if (has_object) *has_object = has;
    return 0;
}

/* 物料底面中心 → 夹持中心 */
static vec3_t grip_point(vec3_t base) { vec3_t g = { base.x, base.y, base.z + MAT_GRIP_FROM_BASE_MM }; return g; }

/*
 * 取：张爪（不等，边移动边张开）→ 到上方 → 下降 → 合爪 → 直接抬回安全高度
 * 放：到上方 → 下降 → 张爪 → 抬回安全高度
 * 每次取或放只用 3 个移动动作；结束时机械臂总在安全高度，下一步可以直接转过去。
 */
int arm_pick(vec3_t base)
{
    vec3_t g = grip_point(base);
    float q[3];
    if (!arm_ik(g, q) || !arm_ik(above_of(g), q)) { LOG("arm: 取料点够不到 (%d,%d,%d)", (int)g.x, (int)g.y, (int)g.z); return -2; }
    servo_write_pos(&SV[J_GRIP], GRIP_RAW_OPEN, 0, 0);
    if (arm_move_safe(g) != 0) return -2;
    bool has = false;
    arm_grip_close(&has);
    arm_move_to(above_of(g));
    if (!has) { LOG("arm: 没夹到"); servo_write_pos(&SV[J_GRIP], GRIP_RAW_OPEN, 0, 0); return -1; }
    return 0;
}

int arm_place(vec3_t base)
{
    vec3_t g = grip_point(base);
    g.z += 1.0f;                                              /* 留 1mm，让物料自己落稳而不是被压下去 */
    float q[3];
    if (!arm_ik(g, q) || !arm_ik(above_of(g), q)) { LOG("arm: 放置点够不到 (%d,%d,%d)", (int)g.x, (int)g.y, (int)g.z); return -2; }
    if (arm_move_safe(g) != 0) return -2;
    arm_grip_open();
    arm_move_to(above_of(g));
    return 0;
}

vec3_t arm_slot_base(int k)
{
    static const float SX[3] = { SLOT0_X_MM, SLOT1_X_MM, SLOT2_X_MM };
    static const float SY[3] = { SLOT0_Y_MM, SLOT1_Y_MM, SLOT2_Y_MM };
    vec3_t p = { SX[k % 3], SY[k % 3], CAR_TRAY_FLOOR_Z_MM };
    return p;
}

int arm_pick_from_slot(int k) { return arm_pick(arm_slot_base(k)); }
int arm_place_to_slot(int k)  { return arm_place(arm_slot_base(k)); }

/* ---------------- 姿态与初始化 ---------------- */

int arm_fold(void)
{
    int raw[3] = { FOLD_YAW_RAW, FOLD_SHOULDER_RAW, FOLD_ELBOW_RAW };
    return move_raw(raw);
}

int arm_home(void)
{
    /* 行车姿态：夹持中心收在回转轴正上方偏前，高于屏幕和槽内物料 */
    vec3_t p = { ARM_BASE_X_MM + 90.0f, ARM_BASE_Y_MM, ARM_SAFE_Z_MM };
    return arm_move_to(p);
}

int arm_init(void)
{
    s_ok = true;
    for (int j = 0; j < 4; j++) {
        if (servo_ping(&SV[j]) != 0) {
            LOG("arm: 舵机 %d (总线%s, ID %u) 无应答", j, j < 3 ? "ST" : "SC", SV[j].id);
            s_ok = false;
        }
    }
    /* 用舵机当前实际位置作为起点，避免上电第一下猛甩 */
    for (int j = 0; j < 3; j++) {
        int pos;
        s_raw[j] = (servo_read_pos(&SV[j], &pos) == 0) ? pos : ZERO[j];
    }
    arm_torque(true);
    arm_fold();
    servo_write_pos(&SV[J_GRIP], GRIP_RAW_OPEN, 0, 0);
    return s_ok ? 0 : -1;
}

bool arm_ok(void) { return s_ok; }

void arm_torque(bool on)
{
    for (int j = 0; j < 4; j++) servo_torque(&SV[j], on);
}

int arm_read_raw(int raw[4])
{
    int bad = 0;
    for (int j = 0; j < 4; j++) if (servo_read_pos(&SV[j], &raw[j]) != 0) { raw[j] = -1; bad++; }
    return bad ? -1 : 0;
}

int arm_write_raw(int joint, int raw)
{
    if (joint < 0 || joint > 3) return -1;
    if (joint == J_GRIP) return servo_write_pos(&SV[J_GRIP], raw, 0, 0);
    int t[3] = { s_raw[0], s_raw[1], s_raw[2] };
    t[joint] = raw;
    return move_raw(t);
}
