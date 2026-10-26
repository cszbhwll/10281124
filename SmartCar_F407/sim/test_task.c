/* test_task.c —— 任务码解析的单元测试 */
#include "task_code.h"
#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("失败: %s (第%d行)\n", #c, __LINE__); fails++; } } while (0)

int main(void)
{
    task_t t;
    const char *e;
    /* 规则原文例子：156+123+516+231 */
    CHECK(task_parse("156+123+516+231", &t, &e));
    CHECK(t.color[0][0] == 1 && t.color[0][1] == 5 && t.color[0][2] == 6);
    CHECK(t.rough_ring[0][0] == 1 && t.rough_ring[0][1] == 2 && t.rough_ring[0][2] == 3);
    CHECK(t.color[1][0] == 5 && t.color[1][1] == 1 && t.color[1][2] == 6);
    CHECK(t.rough_ring[1][0] == 2 && t.rough_ring[1][1] == 3 && t.rough_ring[1][2] == 1);
    /* 第二批暂存：黑→叠到第一批黑(2号)，红→1号，浅蓝→3号 —— 与规则解释一致 */
    CHECK(t.stash_ring[1][0] == 2 && t.stash_ring[1][1] == 1 && t.stash_ring[1][2] == 3);
    CHECK(t.stash_ring[0][0] == 1 && t.stash_ring[0][1] == 2 && t.stash_ring[0][2] == 3);
    CHECK(strcmp(t.text, "156+123+516+231") == 0);
    /* 容忍空格 */
    CHECK(task_parse(" 156 + 123 + 516 + 231 ", &t, &e));
    /* 错误情况 */
    CHECK(!task_parse("156+123+516", &t, &e));
    CHECK(!task_parse("155+123+516+231", &t, &e));     /* 颜色重复 */
    CHECK(!task_parse("156+122+516+231", &t, &e));     /* 位置不是排列 */
    CHECK(!task_parse("156+123+234+231", &t, &e));     /* 两批颜色不同 */
    CHECK(!task_parse("176+123+716+231", &t, &e));     /* 颜色超范围 */
    CHECK(!task_parse("156+123+516+2311", &t, &e));
    CHECK(!task_parse("156-123+516+231", &t, &e));
    printf(fails ? "任务码测试：%d 项失败\n" : "任务码测试：全部通过%.0d\n", fails);
    return fails != 0;
}
