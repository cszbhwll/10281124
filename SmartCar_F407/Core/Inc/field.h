/**
 * field.h —— 场地地图（初赛，规则图1）
 *
 * 场地坐标：原点在左下角，X 向右，Y 向上，单位 mm，全场 2400×2400。
 *   启停区1：右上角 300×300；启停区2：右下角 300×300
 *   原料转盘：上边中间（中心 X 约 1200，规则写 1100~1300）
 *   二维码板：右边中间（Y≈1200），竖直安装、面向场内
 *   粗加工区：下边中间，580×150，三个圆环间距 150
 *   暂存区：  左边中间，150×580，三个圆环间距 150
 *   四块淡黄色禁行区：450×450，中间十字车道宽 400
 *
 * 【现场】所有坐标都"以比赛现场为准"，到场后先量，再改 field.c 里的数字。
 */
#ifndef FIELD_H
#define FIELD_H

#include <stdbool.h>
#include "geom.h"

#define FIELD_SIZE_MM 2400.0f

/* 车道网格节点（3×3）：车只沿这些节点之间的连线走，保证始终在灰色车道上 */
typedef enum {
    NODE_TL, NODE_TC, NODE_TR,
    NODE_ML, NODE_C,  NODE_MR,
    NODE_BL, NODE_BC, NODE_BR,
    NODE_COUNT
} node_id_t;

typedef enum { ZONE_ROUGH = 0, ZONE_STASH = 1 } place_zone_t;

void   field_init(void);
point_t field_node(node_id_t n);
bool   field_edge_exists(node_id_t a, node_id_t b);
void   field_block_edge(node_id_t a, node_id_t b, bool blocked);   /* 发现障碍物挡路时标记 */
bool   field_edge_blocked(node_id_t a, node_id_t b);
node_id_t field_nearest_node(point_t p);

pose_t field_start_pose(int zone);            /* 启停区中心、出发朝向 */
pose_t field_qr_pose(void);                   /* 读二维码时车停的位置 */
point_t field_turntable_pick_point(void);     /* 转盘上"停下来的那个物料"的中心 */
pose_t field_turntable_pose(void);            /* 取料时车停的位置 */
point_t field_ring_point(place_zone_t z, int ring_no);   /* ring_no = 1..3 */
pose_t field_place_pose(place_zone_t z, int ring_no);    /* 对准该圆环放料时车停的位置 */

/* 已发现的障碍物（避障用）：路线规划和"车身在不在车道内"的检查都会把它们算进去 */
void    field_obs_clear(void);                      /* 每场开始时清空 */
int     field_obs_add(point_t c, float weight, float sigma, bool *is_new);   /* 加入/合并一个障碍物，返回编号
                                                    sigma = 这次测量的位置可能偏多少 mm（规划时额外让开这么多） */
int     field_obs_find(point_t c, float within_mm); /* 附近有没有已知障碍物（最近的那个），没有返回 -1 */
void    field_obs_update(int k, point_t c, float weight, float sigma);   /* 用新的一次测量修正第 k 个的位置 */
float   field_obs_sigma(int k);                     /* 第 k 个障碍物位置的不确定范围 mm */
void    field_obs_set_sigma_scale(float f);         /* 规划时不确定范围按多少倍算（默认 1；实在没路时临时减小） */
int     field_obs_count(void);
point_t field_obs_get(int i);
bool    field_obs_candidate(point_t c);             /* 这个位置可能是障碍物吗（在车道上，不是加工区/转盘/场外） */
void    field_obs_set_clearance(float mm);          /* 规划时离障碍物留多少余量（默认 OBS_CLEAR_MM） */
float   field_static_range(point_t o, float dir_rad, float half_fov_rad);
        /* 从 o 点朝 dir 方向、在 ±half_fov 视场里，最近的“已知固定物体”有多远：
           场地四周围挡/二维码板、原料转盘、两个加工区（里面会有物料）。看不到返回 1e9 */

/* 检查车在某个位姿时，车身是否完全在灰色车道里（不压禁行区、不出界、不进加工区），并且没碰到已知障碍物 */
bool   field_pose_on_lane(pose_t p);
bool   field_pose_on_lane_m(pose_t p, float margin_mm);   /* 车身四周再放大 margin 后检查 */

#endif
