/**
 * obstacle.h —— 避障：把测距传感器的读数变成“场地上哪里有障碍物”
 *
 * 每 20ms 由 sys_poll 调用 obs_update()：
 *   1. 读各个 VL53L0X 的距离；
 *   2. 按传感器的安装位置和车现在的位置，算出被测到的东西在场地上的坐标；
 *   3. 只保留“可能是障碍物”的点（在车道上；加工区里的物料、转盘、二维码板、场外的东西都排除）；
 *   4. 同一位置连续看到 OBS_CONFIRM 次才确认，加入 field.c 的障碍物地图。
 * 地图变了之后，nav.c 会检查当前路线是否被挡，被挡就重新规划（绕到别的车道）。
 */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>

void obs_init(void);          /* 上电：初始化测距传感器 */
void obs_reset(void);         /* 每场开始：清空障碍物地图 */
void obs_update(void);        /* 每 20ms 调用 */
bool obs_take_changed(void);  /* 地图从上次调用以来有没有变化（读一次就清零） */
bool obs_enabled(void);       /* 传感器是否可用 */
void obs_debug(void);         /* 串口命令 tof：打印各传感器读数和已发现的障碍物 */

#endif
