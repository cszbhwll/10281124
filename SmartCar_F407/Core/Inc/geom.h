/**
 * geom.h —— 小工具：点、位姿、角度换算
 */
#ifndef GEOM_H
#define GEOM_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define PI_F       3.14159265f
#define DEG2RAD(d) ((d) * (PI_F / 180.0f))
#define RAD2DEG(r) ((r) * (180.0f / PI_F))

typedef struct { float x, y; } point_t;
typedef struct { float x, y, th; } pose_t;    /* th：朝向角，弧度 */

static inline float wrap_pi(float a)          /* 把角度规整到 (-π, π] */
{
    while (a > PI_F)  a -= 2.0f * PI_F;
    while (a <= -PI_F) a += 2.0f * PI_F;
    return a;
}
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float dist2d(float ax, float ay, float bx, float by) { return hypotf(ax - bx, ay - by); }

/* 车体坐标(px,py) → 场地坐标 */
static inline point_t robot_to_field(pose_t r, float px, float py)
{
    point_t o;
    float c = cosf(r.th), s = sinf(r.th);
    o.x = r.x + c * px - s * py;
    o.y = r.y + s * px + c * py;
    return o;
}
/* 场地坐标(fx,fy) → 车体坐标 */
static inline point_t field_to_robot(pose_t r, float fx, float fy)
{
    point_t o;
    float c = cosf(r.th), s = sinf(r.th);
    float dx = fx - r.x, dy = fy - r.y;
    o.x =  c * dx + s * dy;
    o.y = -s * dx + c * dy;
    return o;
}

#endif
