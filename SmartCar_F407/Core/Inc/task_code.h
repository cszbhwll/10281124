/**
 * task_code.h —— 任务码解析   例：156+123+516+231
 *
 *   第1组 156：第一批颜色及顺序（1红 2黄 3蓝 4绿 5黑 6浅蓝）
 *   第2组 123：第一批在粗加工区【和暂存区】放的圆环号
 *   第3组 516：第二批颜色及顺序
 *   第4组 231：第二批在粗加工区放的圆环号
 *   第二批在暂存区：叠放在第一批"同颜色"物料上面（不是用第4组！）
 */
#ifndef TASK_CODE_H
#define TASK_CODE_H

#include <stdbool.h>

typedef struct {
    int color[2][3];        /* [批次][第几个] = 颜色 1~6 */
    int rough_ring[2][3];   /* [批次][第几个] = 粗加工区圆环号 1~3 */
    int stash_ring[2][3];   /* [批次][第几个] = 暂存区圆环号 1~3（第二批由颜色推出） */
    char text[20];
} task_t;

/* 解析成功返回 true；格式不对、颜色重复等返回 false，并在 err 里写原因 */
bool task_parse(const char *s, task_t *t, const char **err);
const char *color_name(int c);

#endif
