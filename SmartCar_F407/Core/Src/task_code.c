/**
 * task_code.c —— 任务码解析与校验
 */
#include "task_code.h"
#include <string.h>
#include <ctype.h>

const char *color_name(int c)
{
    static const char *N[] = { "?", "红", "黄", "蓝", "绿", "黑", "浅蓝" };
    return (c >= 1 && c <= 6) ? N[c] : N[0];
}

static bool is_perm123(const int v[3])
{
    int seen = 0;
    for (int i = 0; i < 3; i++) {
        if (v[i] < 1 || v[i] > 3) return false;
        seen |= 1 << v[i];
    }
    return seen == 0x0E;
}

bool task_parse(const char *s, task_t *t, const char **err)
{
    const char *e = NULL;
    int g[4][3];
    int gi = 0, di = 0;
    memset(t, 0, sizeof *t);

    /* 只保留数字和 '+'，容忍相机多发的空格 */
    for (const char *p = s; *p && gi < 4; p++) {
        if (isdigit((unsigned char)*p)) {
            if (di >= 3) { e = "某组超过3位"; goto fail; }
            g[gi][di++] = *p - '0';
        } else if (*p == '+') {
            if (di != 3) { e = "某组不足3位"; goto fail; }
            gi++; di = 0;
        } else if (*p != ' ') {
            e = "含非法字符"; goto fail;
        }
    }
    if (gi != 3 || di != 3) { e = "不是4组"; goto fail; }

    for (int b = 0; b < 2; b++) {
        int *col = g[b * 2], *pos = g[b * 2 + 1];
        for (int i = 0; i < 3; i++) {
            if (col[i] < 1 || col[i] > 6) { e = "颜色编号超出1~6"; goto fail; }
            for (int j = 0; j < i; j++) if (col[j] == col[i]) { e = "同一批颜色重复"; goto fail; }
        }
        if (!is_perm123(pos)) { e = "位置不是1、2、3的排列"; goto fail; }
        for (int i = 0; i < 3; i++) { t->color[b][i] = col[i]; t->rough_ring[b][i] = pos[i]; }
    }
    /* 两批必须是同样的三种颜色 */
    for (int i = 0; i < 3; i++) {
        bool found = false;
        for (int j = 0; j < 3; j++) if (t->color[1][i] == t->color[0][j]) { found = true; t->stash_ring[1][i] = t->rough_ring[0][j]; }
        if (!found) { e = "两批颜色不一致"; goto fail; }
        t->stash_ring[0][i] = t->rough_ring[0][i];
    }
    for (int i = 0, k = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) t->text[k++] = (char)('0' + g[i][j]);
        if (i < 3) t->text[k++] = '+';
        t->text[k] = 0;
    }
    if (err) *err = NULL;
    return true;
fail:
    if (err) *err = e;
    return false;
}
