/**
 * mission.h —— 整场比赛流程（初赛）
 */
#ifndef MISSION_H
#define MISSION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int  start_zone;          /* 1 或 2：抽签确定的出发启停区 */
    bool skip_qr;             /* 调试用：不读二维码，直接用 fixed_task */
    char fixed_task[20];
} mission_cfg_t;

typedef struct {
    bool     qr_ok;
    char     task[20];
    int      grabs;           /* 任务抓取成功次数（不含车上载物槽的中转） */
    int      places;          /* 任务放置成功次数 */
    int      stacks;          /* 第二批在暂存区码垛次数 */
    bool     returned;        /* 是否回到启停区 */
    bool     aborted_by_time; /* 是否因为时间不够提前返程 */
    uint32_t time_ms;
} mission_result_t;

void mission_run(const mission_cfg_t *cfg, mission_result_t *res);

#endif
