/**
 * arm.h —— 机械臂：回转 + 肩 + 肘（3×ST3215）+ 夹爪（SC09）
 *
 * 机械臂"逆运动学"（IK）：告诉它夹持中心要到车体坐标的 (x,y,z)，
 * 它自己算出三个关节各转多少度。原理见 README 的讲解部分。
 */
#ifndef ARM_H
#define ARM_H

#include <stdbool.h>

typedef struct { float x, y, z; } vec3_t;        /* 车体坐标，mm */

int    arm_init(void);                 /* 检查舵机、上力矩、收到折叠姿态。0=成功 */
bool   arm_ok(void);
int    arm_fold(void);                 /* 出发折叠姿态（必须满足 300×300×400 包络） */
int    arm_home(void);                 /* 行车姿态：抬高收在车上方 */

bool   arm_ik(vec3_t p, float q[3]);   /* 位置 → 关节角（弧度）；够不到返回 false */
void   arm_fk(const float q[3], vec3_t *p);
vec3_t arm_current_point(void);

int    arm_move_to(vec3_t p);          /* 直接移动（不绕行） */
int    arm_move_safe(vec3_t p);        /* 先抬到安全高度、转过去、再下降 */

int    arm_grip_open(void);
int    arm_grip_close(bool *has_object);

/* 取/放一个物料：p 为物料"底面中心"的位置（z=物料所在表面高度） */
int    arm_pick(vec3_t base_point);    /* 0=夹到，-1=没夹到，-2=够不到 */
int    arm_place(vec3_t base_point);   /* 0=放下，-2=够不到 */

/* 车上三个载物槽 k=0,1,2 */
vec3_t arm_slot_base(int k);
int    arm_pick_from_slot(int k);
int    arm_place_to_slot(int k);

/* 调试用 */
void   arm_torque(bool on);
int    arm_read_raw(int raw[4]);       /* 读四个舵机当前原始位置 */
int    arm_write_raw(int joint, int raw);

#endif
