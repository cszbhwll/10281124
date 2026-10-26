/* sim.h —— 电脑端模拟器内部接口（只在模拟时编译，不会下载到单片机） */
#ifndef SIM_H
#define SIM_H

#include <stdint.h>
#include <stdbool.h>
#include "geom.h"

typedef struct {
    unsigned seed;
    int      start_zone;
    char     task[20];
    float    slip;            /* 轮子打滑比例（0 = 理想；0.02 = 实际只走了 98%） */
    float    noise_mm;        /* 每步随机位置噪声 */
    bool     verbose;
    bool     vision_ring;     /* 相机能否看到圆环（关掉可看只靠里程计的效果） */
    bool     imu;             /* 是否装了陀螺仪 */
    int      n_obs;           /* 障碍物个数：-1 = 随机 1~3 个（默认，和比赛一样抽签随机），0 = 不放 */
    unsigned obs_seed;        /* 障碍物摆法编号（同一个编号 = 同一种摆法，方便复现）；0 = 用 seed */
} sim_cfg_t;

typedef struct {
    int   lane_violations;    /* 车身压到禁行区/加工区/出界的采样次数 */
    float worst_violation_x, worst_violation_y;
    int   rough_ring_score;   /* 粗加工区放置得分（按放下时偏差算） */
    int   stash_ring_score;   /* 暂存区第一批平放得分 */
    int   stack_score;        /* 第二批码垛得分（按底层分值） */
    int   correct_grabs;
    float max_place_err_mm;
    bool  in_start_zone;
    bool  qr_before_end;      /* 读码是在本轮结束之前完成的 */
    uint32_t ended_ms;        /* 第一次越出车道的时刻（0 = 没越线）：按规则本轮在此结束 */
    int   n_obs;              /* 场上障碍物个数 */
    int   obs_hits;           /* 被车身碰到的障碍物个数 */
    uint32_t first_hit_ms;    /* 第一次碰到障碍物的时刻（0 = 没碰） */
    int   obs_found;          /* 程序发现的障碍物里，和真实位置对得上的个数 */
    int   obs_false;          /* 程序以为有、实际没有的（误报） */
} sim_report_t;

void  sim_init(const sim_cfg_t *cfg);
void  sim_step_1ms(void);
uint32_t sim_time_ms(void);
void  sim_report(sim_report_t *r, int display_grabs, int display_places);
pose_t sim_true_pose(void);
int   sim_obstacle_count(void);
void  sim_obstacle_get(int i, float *x, float *y);
int   sim_write_replay(const char *path, const char *summary, const char *result_json);     /* 二维回放页（独立 HTML） */
int   sim_write_replay_js(const char *path, const char *summary, const char *result_json);  /* 三维回放用的数据文件 */

/* port_sim.c 调用 */
void  sim_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *d, uint16_t n);
int   sim_i2c_read(uint8_t addr, uint8_t reg, uint8_t *d, uint16_t n);
void  sim_uart_tx(int ch, const uint8_t *d, uint16_t n);
int   sim_uart_rx(int ch, uint8_t *out);

#endif
