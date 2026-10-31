/**
 * world.c —— 模拟器：模拟小车底盘、驱动板、舵机、夹爪、转盘、物料和摄像头
 *
 * 用途：在电脑上把"整场比赛"的程序逻辑跑一遍，检查顺序、计数、路线是否压线、时间够不够。
 * 它假设 robot_config.h 里的参数都是"标定正确"的；真实小车的误差要上车后才能知道。
 */
#include "sim.h"
#include "robot_config.h"
#include "field.h"
#include "arm.h"
#include "port.h"
#include "task_code.h"
#include "tof.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

#define MM_PER_COUNT  ((PI_F * WHEEL_DIAMETER_MM) / ENC_COUNTS_PER_WHEEL_REV)
#define K_ROT         ((CHASSIS_HALF_TRACK_MM + CHASSIS_HALF_BASE_MM) / ROT_SCALE)

static sim_cfg_t C;
static uint32_t  s_t;
static pose_t    P;                       /* 小车真实位姿 */
static float     ch_cmd[4], ch_spd[4];
static double    ch_cnt[4];
static task_t    T;

/* ---------- 舵机 ---------- */
typedef struct { float pos; int goal; int speed; float vel; } sv_t;   /* vel：当前速度（模拟加减速） */
static sv_t st[4];   /* ST 总线：ID 1..3 */
static sv_t sc[2];   /* SC 总线：ID 1 */

/* ---------- 物料 ---------- */
typedef enum { M_NONE, M_TT, M_FIELD, M_SLOT, M_GRIP, M_FALLEN } mloc_t;
typedef struct { int color, batch; mloc_t loc; float x, y, z; float ang; } mat_t;
static mat_t M[6];
static int   s_gripped = -1;

/* ---------- 转盘 ---------- */
#define TT_CX 1200.0f
#define TT_CY 2470.0f
#define TT_R  95.0f
static float tt_ang;                 /* 转盘转过的角度 */
static float tt_dir;                 /* +1 / -1 */
static float tt_stop_left;           /* 剩余停顿时间 s */
static bool  tt_batch2_loaded;
static float tt_refill_timer = -1;

/* ---------- 裁判用的"真实"圆环位置（按规则图，独立于被测程序的 field.c） ----------
 * 模拟器判分和模拟相机都用这里的坐标。这样即使 field.c 里写错了圆环位置或编号，
 * 模拟器也能发现（如果判分也用 field.c 的函数，程序错了裁判跟着错，就查不出来）。 */
static const point_t TRUE_RING[2][3] = {
    { { 1350.0f, 75.0f }, { 1200.0f, 75.0f }, { 1050.0f, 75.0f } },     /* 粗加工区 1、2、3 号 */
    { { 75.0f, 1350.0f }, { 75.0f, 1200.0f }, { 75.0f, 1050.0f } },     /* 暂存区 1、2、3 号 */
};
static point_t true_ring(int zone, int no) { if (no < 1) no = 1; if (no > 3) no = 3; return TRUE_RING[zone][no - 1]; }

/* ---------- 裁判用的“真实”场地（按规则图1，独立于被测程序的 field.c） ----------
 * 判压线用这里的尺寸和车身真实外形，不用 field.c：否则程序里把禁行区改小、把车身尺寸改小，
 * 裁判也会跟着放宽，压线就查不出来了。 */
#define REF_HL 117.0f                     /* 车身真实外形（CAD，含麦轮）：长 234、宽 296 */
#define REF_HW 148.0f
typedef struct { float x0, y0, x1, y1; } rrect_t;
static const rrect_t REF_YELLOW[4] = { { 550, 550, 1000, 1000 }, { 1400, 550, 1850, 1000 }, { 550, 1400, 1000, 1850 }, { 1400, 1400, 1850, 1850 } };
static const rrect_t REF_ROUGH = { 910, 0, 1490, 150 }, REF_STASH = { 0, 910, 150, 1490 };
static bool in_rr(const rrect_t *r, float x, float y) { return x > r->x0 && x < r->x1 && y > r->y0 && y < r->y1; }
static bool ref_point_ok(float x, float y)
{
    if (x < 0 || y < 0 || x > 2400.0f || y > 2400.0f) return false;
    for (int i = 0; i < 4; i++) if (in_rr(&REF_YELLOW[i], x, y)) return false;
    if (in_rr(&REF_ROUGH, x, y) || in_rr(&REF_STASH, x, y)) return false;
    if (dist2d(x, y, 1200.0f, 2470.0f) < 150.0f) return false;              /* 原料转盘 */
    return true;
}
static bool ref_pose_on_lane(pose_t p)
{
    for (int i = 0; i <= 6; i++) for (int j = 0; j <= 6; j++) {
        if (i != 0 && i != 6 && j != 0 && j != 6) continue;
        point_t f = robot_to_field(p, -REF_HL + (float)i * (REF_HL / 3.0f), -REF_HW + (float)j * (REF_HW / 3.0f));
        if (!ref_point_ok(f.x, f.y)) return false;
    }
    return true;
}

/* ---------- 障碍物（规则：黑色 φ50×100，数量和位置随机抽签放置在车道上） ---------- */
#define SIM_MAX_OBS 6
typedef struct { float x, y; uint32_t hit_t; } sobs_t;
static sobs_t   OB[SIM_MAX_OBS];
static int      s_nob, s_obs_hits;
static uint32_t s_first_hit;
static uint32_t s_rng2;                   /* 障碍物和测距噪声单独用一个随机数，不打乱其他随机过程 */
static float rnd2(void) { s_rng2 = s_rng2 * 1664525u + 1013904223u; return (float)(s_rng2 >> 8) / 16777216.0f; }

/* ---------- 测距传感器（VL53L0X）模拟 ---------- */
#define SIM_TOF_BLACK_MAX 700.0f          /* 黑色障碍物：最远能测到的距离（模拟值；实车要实测） */
#define SIM_TOF_OTHER_MAX 1200.0f         /* 浅色物体 */
static int      s_tof_mm[TOF_COUNT];
static uint32_t s_tof_t;

/* ---------- 视觉 ---------- */
static int      v_mode;
static uint32_t v_next;

/* ---------- 串口接收队列（模拟器 → 程序） ---------- */
typedef struct { uint8_t b[4096]; int h, t; } q_t;
static q_t RXQ[PORT_UART_COUNT];
static void q_put(int ch, const uint8_t *d, int n) { for (int i = 0; i < n; i++) { RXQ[ch].b[RXQ[ch].h] = d[i]; RXQ[ch].h = (RXQ[ch].h + 1) % 4096; } }
int sim_uart_rx(int ch, uint8_t *o) { q_t *q = &RXQ[ch]; if (q->h == q->t) return 0; *o = q->b[q->t]; q->t = (q->t + 1) % 4096; return 1; }

/* ---------- 统计 ---------- */
typedef struct { int idx; float x, y, z; uint32_t t; } place_ev_t;
static place_ev_t s_ev[64]; static int s_nev;
static int   s_viol; static float s_vx, s_vy;
static bool  s_left_start;
static int   s_grab_tt_ok, s_grab_rough_ok;
/* 规则：车身越出车道（机械臂除外）本轮立即结束。模拟器不让车停下（方便看后面的动作），
   但记下第一次越线的时刻，并把那一刻的物料状态和抓取数存下来，计分只算这之前的。 */
static uint32_t s_viol_t;
static uint32_t s_qr_t;
static mat_t    M_snap[6];
static int      s_grab_at_viol;

/* ---------- 回放记录（只在单场模式 verbose 时记录，用来生成 HTML 回放） ---------- */
#define REC_DT_MS 50
typedef struct { uint32_t t; float x, y, th, q[3], g; float m[6][4]; float tt, w[4]; short tof[TOF_COUNT]; } frame_t;
static frame_t *s_fr; static int s_nfr, s_cfr;
typedef struct { uint32_t t; char s[200]; } logl_t;
static logl_t *s_lg; static int s_nlg, s_clg;
static char s_lbuf[220]; static int s_llen;

static void rec_log(const char *line)
{
    if (!C.verbose) return;
    if (s_nlg == s_clg) { s_clg = s_clg ? s_clg * 2 : 512; s_lg = realloc(s_lg, (size_t)s_clg * sizeof *s_lg); }
    s_lg[s_nlg].t = s_t;
    snprintf(s_lg[s_nlg].s, sizeof s_lg[s_nlg].s, "%s", line);
    s_nlg++;
}

/* 模拟器自己的提示：打印到屏幕，同时记进回放 */
static void sim_note(const char *fmt, ...)
{
    if (!C.verbose) return;
    char buf[200];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
    size_t n = strlen(buf);
    while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = 0;
    printf("    %s\n", buf);
    char line[220];
    snprintf(line, sizeof line, "[%3u.%03u]     %s", (unsigned)(s_t / 1000), (unsigned)(s_t % 1000), buf);
    rec_log(line);
}

static float frand(void) { return (float)rand() / (float)RAND_MAX * 2.0f - 1.0f; }

uint32_t sim_time_ms(void) { return s_t; }
pose_t sim_true_pose(void) { return P; }

/* 模拟陀螺仪：真实朝向 + 每分钟 1° 的漂移（MPU6050 校准零漂后的典型水平） */
bool imu_get_yaw(float *yaw)
{
    if (!C.imu) return false;
    *yaw = wrap_pi(P.th + DEG2RAD(1.0f) * (float)s_t / 60000.0f);
    return true;
}

static void tt_place_batch(int b)
{
    int order[3] = { 0, 1, 2 };
    for (int i = 2; i > 0; i--) { int j = rand() % (i + 1); int t = order[i]; order[i] = order[j]; order[j] = t; }
    for (int k = 0; k < 3; k++) {
        mat_t *m = &M[b * 3 + k];
        m->color = T.color[b][k];
        m->batch = b;
        m->loc = M_TT;
        m->ang = DEG2RAD(120.0f * (float)order[k]) + DEG2RAD(37.0f);  /* 初始不在取料点 */
    }
}

static void obs_generate(void);

void sim_init(const sim_cfg_t *cfg)
{
    C = *cfg;
    srand(C.seed);
    s_t = 0;
    P = field_start_pose(C.start_zone);
    memset(ch_cmd, 0, sizeof ch_cmd); memset(ch_spd, 0, sizeof ch_spd); memset(ch_cnt, 0, sizeof ch_cnt);
    for (int i = 0; i < 4; i++) { st[i].pos = 2048; st[i].goal = 2048; st[i].speed = 0; }
    st[1].pos = FOLD_YAW_RAW; st[2].pos = FOLD_SHOULDER_RAW; st[3].pos = FOLD_ELBOW_RAW;
    for (int i = 1; i <= 3; i++) st[i].goal = (int)st[i].pos;
    sc[1].pos = GRIP_RAW_OPEN; sc[1].goal = GRIP_RAW_OPEN;
    const char *err;
    if (!task_parse(C.task, &T, &err)) { fprintf(stderr, "sim: bad task %s\n", err); exit(2); }
    memset(M, 0, sizeof M);
    tt_ang = 0; tt_dir = (rand() & 1) ? 1.0f : -1.0f; tt_stop_left = 0;
    tt_place_batch(0);
    tt_batch2_loaded = false; tt_refill_timer = -1;
    s_gripped = -1; s_nev = 0; s_viol = 0; s_left_start = false;
    s_grab_tt_ok = s_grab_rough_ok = 0;
    s_viol_t = 0; s_grab_at_viol = 0; s_qr_t = 0;
    v_mode = 0; v_next = 0;
    memset(RXQ, 0, sizeof RXQ);
    s_nfr = 0; s_nlg = 0; s_llen = 0;
    obs_generate();
}

/* ---------- 几何 ---------- */
static point_t tt_mat_point(const mat_t *m)
{
    point_t p = { TT_CX + TT_R * cosf(m->ang + tt_ang), TT_CY + TT_R * sinf(m->ang + tt_ang) };
    return p;
}

static void mat_field_pos(const mat_t *m, float *x, float *y, float *z)
{
    switch (m->loc) {
    case M_TT: { point_t p = tt_mat_point(m); *x = p.x; *y = p.y; *z = TURNTABLE_TOP_Z_MM; break; }
    case M_SLOT: { point_t p = robot_to_field(P, m->x, m->y); *x = p.x; *y = p.y; *z = CAR_TRAY_FLOOR_Z_MM; break; }
    default: *x = m->x; *y = m->y; *z = m->z; break;
    }
}

static int raw_to_q(int raw, int zero, int dir, float *q) { *q = (float)(raw - zero) / ((float)dir * ST_COUNTS_PER_RAD); return 0; }

static void grasp_point(float *x, float *y, float *z, float *rx, float *ry)
{
    float q[3];
    raw_to_q((int)lroundf(st[1].pos), YAW_ZERO_RAW, YAW_DIR, &q[0]);
    raw_to_q((int)lroundf(st[2].pos), SHOULDER_ZERO_RAW, SHOULDER_DIR, &q[1]);
    raw_to_q((int)lroundf(st[3].pos), ELBOW_ZERO_RAW, ELBOW_DIR, &q[2]);
    vec3_t pr;
    arm_fk(q, &pr);
    point_t f = robot_to_field(P, pr.x, pr.y);
    *x = f.x; *y = f.y; *z = pr.z; *rx = pr.x; *ry = pr.y;
}

/* 夹爪合上：看夹持区内有没有物料 */
static void grip_close(void)
{
    float gx, gy, gz, rx, ry;
    grasp_point(&gx, &gy, &gz, &rx, &ry);
    int best = -1; float bd = 1e9f;
    for (int i = 0; i < 6; i++) {
        if (M[i].loc == M_NONE || M[i].loc == M_GRIP || M[i].loc == M_FALLEN) continue;
        /* 码垛时只能夹最上面的：有别的物料叠在它上面就跳过 */
        float x, y, z;
        mat_field_pos(&M[i], &x, &y, &z);
        float dh = hypotf(x - gx, y - gy), dz = fabsf(z + MAT_GRIP_FROM_BASE_MM - gz);
        if (dh < 9.0f && dz < 6.0f && dh < bd) { bd = dh; best = i; }
    }
    if (best >= 0) {
        mat_t *m = &M[best];
        if (m->loc == M_TT) {
            int idx = -1;
            for (int k = 0; k < 3; k++) if (T.color[m->batch][k] == m->color) idx = k;
            (void)idx;
            s_grab_tt_ok++;
        } else if (m->loc == M_FIELD) {
            s_grab_rough_ok++;
        }
        m->loc = M_GRIP;
        s_gripped = best;
        sc[1].goal = GRIP_RAW_CLOSE + 60;                /* 被物料挡住 */
        sim_note("[模拟] 夹住 %s 色物料（偏差 %.1fmm）\n", color_name(m->color), bd);
    } else {
        sim_note("[模拟] 空夹：夹持点(%.0f,%.0f,%.0f)\n", gx, gy, gz);
    }
}

static void grip_open(void)
{
    if (s_gripped < 0) return;
    mat_t *m = &M[s_gripped];
    s_gripped = -1;
    float gx, gy, gz, rx, ry;
    grasp_point(&gx, &gy, &gz, &rx, &ry);
    float base_z = gz - MAT_GRIP_FROM_BASE_MM;

    /* 1) 落在车上载物槽里？ */
    for (int k = 0; k < 3; k++) {
        vec3_t s = arm_slot_base(k);
        if (hypotf(rx - s.x, ry - s.y) < 16.0f && fabsf(base_z - CAR_TRAY_FLOOR_Z_MM) < 8.0f) {
            m->loc = M_SLOT; m->x = rx; m->y = ry; m->z = CAR_TRAY_FLOOR_Z_MM;
            sim_note("[模拟] 放进载物槽%d（偏差 %.1fmm）\n", k, hypotf(rx - s.x, ry - s.y));
            return;
        }
    }
    /* 2) 叠在另一个物料上？ */
    for (int i = 0; i < 6; i++) {
        if (&M[i] == m || M[i].loc != M_FIELD) continue;
        if (hypotf(M[i].x - gx, M[i].y - gy) < 30.0f && fabsf(base_z - (M[i].z + MAT_HEIGHT_MM)) < 8.0f) {
            bool stable = hypotf(M[i].x - gx, M[i].y - gy) < 12.0f;
            m->loc = stable ? M_FIELD : M_FALLEN;
            sim_note("[模拟] 叠放 偏差 %.1fmm %s\n", hypotf(M[i].x - gx, M[i].y - gy), stable ? "稳" : "倒了");
            m->x = gx; m->y = gy; m->z = M[i].z + MAT_HEIGHT_MM;
            if (s_nev < 64) { s_ev[s_nev].idx = (int)(m - M); s_ev[s_nev].x = gx; s_ev[s_nev].y = gy; s_ev[s_nev].z = m->z; s_ev[s_nev].t = s_t; s_nev++; }
            return;
        }
    }
    /* 3) 放在地面上 */
    if (fabsf(base_z) < 8.0f) {
        m->loc = M_FIELD; m->x = gx; m->y = gy; m->z = 0;
        sim_note("[模拟] 放在地面 (%.1f, %.1f)\n", gx, gy);
        if (s_nev < 64) { s_ev[s_nev].idx = (int)(m - M); s_ev[s_nev].x = gx; s_ev[s_nev].y = gy; s_ev[s_nev].z = 0; s_ev[s_nev].t = s_t; s_nev++; }
        return;
    }
    m->loc = M_FALLEN; m->x = gx; m->y = gy; m->z = 0;
    sim_note("[模拟] 物料掉落！松爪高度 %.0f\n", base_z);
}

/* ---------- 驱动板 I2C ---------- */
void sim_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *d, uint16_t n)
{
    if (addr != MOTOR_DRV_I2C_ADDR) return;
    if (reg == 51 && n == 4) for (int i = 0; i < 4; i++) ch_cmd[i] = (float)(int8_t)d[i];
}

int sim_i2c_read(uint8_t addr, uint8_t reg, uint8_t *d, uint16_t n)
{
    if (addr != MOTOR_DRV_I2C_ADDR) return -1;
    if (reg == 60 && n == 16) {
        for (int i = 0; i < 4; i++) {
            int32_t v = (int32_t)llround(ch_cnt[i]);
            uint32_t u = (uint32_t)v;
            d[4 * i] = (uint8_t)u; d[4 * i + 1] = (uint8_t)(u >> 8); d[4 * i + 2] = (uint8_t)(u >> 16); d[4 * i + 3] = (uint8_t)(u >> 24);
        }
        return 0;
    }
    if (reg == 0 && n == 2) { d[0] = (uint8_t)(11800 & 0xFF); d[1] = (uint8_t)(11800 >> 8); return 0; }
    return 0;
}

/* ---------- 舵机总线 ---------- */
static void servo_reply(int ch, uint8_t id, const uint8_t *prm, int n)
{
    uint8_t b[16];
    uint8_t len = (uint8_t)(n + 2), sum = (uint8_t)(id + len);
    b[0] = 0xFF; b[1] = 0xFF; b[2] = id; b[3] = len; b[4] = 0;
    for (int i = 0; i < n; i++) { b[5 + i] = prm[i]; sum = (uint8_t)(sum + prm[i]); }
    b[5 + n] = (uint8_t)~sum;
    q_put(ch, b, n + 6);
}

static void servo_packet(int ch, const uint8_t *b, int n)
{
    uint8_t id = b[2], inst = b[4];
    const uint8_t *p = &b[5];
    (void)n;
    /* 夹爪 SC09 在哪条总线、用哪个 ID 由 robot_config.h 决定（GRIP_ON_ST_BUS） */
    bool scs = (ch == PORT_UART_GRIP && id == SERVO_ID_GRIP);
    if (inst == 0x83) {                         /* 同步写：只给 ST3215 用 */
        if (ch != PORT_UART_SERVO_ST) return;
        int L = p[1], cnt = (b[3] - 4) / (L + 1);
        for (int k = 0; k < cnt; k++) {
            const uint8_t *e = &p[2 + k * (L + 1)];
            int sid = e[0];
            if (sid < 1 || sid > 3) continue;
            st[sid].goal = e[2] | (e[3] << 8);
            st[sid].speed = e[6] | (e[7] << 8);
        }
        return;
    }
    sv_t *sv;
    if (scs) sv = &sc[1];
    else if (ch == PORT_UART_SERVO_ST && id >= 1 && id <= 3) sv = &st[id];
    else return;                                 /* 不存在的舵机：不回复 */
    if (inst == 0x01) { servo_reply(ch, id, NULL, 0); return; }
    if (inst == 0x02 && p[0] == 66) {                       /* 读"是否在运动" */
        uint8_t mv = fabsf((float)sv->goal - sv->pos) > 2.0f ? 1 : 0;
        servo_reply(ch, id, &mv, 1);
        return;
    }
    if (inst == 0x02) {
        int pos = (int)lroundf(sv->pos);
        uint8_t r[2];
        if (scs) { r[0] = (uint8_t)(pos >> 8); r[1] = (uint8_t)pos; } else { r[0] = (uint8_t)pos; r[1] = (uint8_t)(pos >> 8); }
        servo_reply(ch, id, r, 2);
        return;
    }
    if (inst == 0x03) {
        uint8_t reg = p[0];
        if (!scs && reg == 41) { sv->goal = p[2] | (p[3] << 8); sv->speed = p[6] | (p[7] << 8); }
        if (scs && reg == 42) {
            int goal = (p[1] << 8) | p[2];
            sv->speed = (p[5] << 8) | p[6];
            sv->goal = goal;
            if (goal < (GRIP_RAW_OPEN + GRIP_RAW_CLOSE) / 2) grip_close(); else grip_open();
        }
        servo_reply(ch, id, NULL, 0);
    }
}

static uint8_t s_txbuf[PORT_UART_COUNT][128];
static int     s_txlen[PORT_UART_COUNT];

void sim_uart_tx(int ch, const uint8_t *d, uint16_t n)
{
    if (ch == PORT_UART_LOG) {
        if (!C.verbose) return;
        fwrite(d, 1, n, stdout);
        for (int i = 0; i < n; i++) {           /* 按行记进回放 */
            char c = (char)d[i];
            if (c == '\n') { s_lbuf[s_llen] = 0; rec_log(s_lbuf); s_llen = 0; }
            else if (c != '\r' && s_llen < (int)sizeof s_lbuf - 1) s_lbuf[s_llen++] = c;
        }
        return;
    }
    if (ch == PORT_UART_VISION) {
        for (int i = 0; i < n; i++) {
            uint8_t c = d[i];
            if (c == '\n') { s_txbuf[ch][s_txlen[ch]] = 0; int m; if (sscanf((char *)s_txbuf[ch], "$MODE,%d", &m) == 1) v_mode = m; s_txlen[ch] = 0; }
            else if (s_txlen[ch] < 127) s_txbuf[ch][s_txlen[ch]++] = c;
        }
        return;
    }
    /* 舵机：一次写入就是一个完整数据包 */
    if (n >= 6 && d[0] == 0xFF && d[1] == 0xFF) servo_packet(ch, d, n);
}

/* ---------- 视觉 ---------- */
static void vision_emit(const char *s) { q_put(PORT_UART_VISION, (const uint8_t *)s, (int)strlen(s)); }

static void vision_step(void)
{
    if (v_mode == 0 || s_t < v_next) return;
    v_next = s_t + 100;
    char line[64];
    if (v_mode == 1) {
        if (P.x > 1950.0f && fabsf(P.y - 1200.0f) < 150.0f && fabsf(wrap_pi(P.th)) < DEG2RAD(20)) {
            if (!s_qr_t) s_qr_t = s_t;                         /* 第一次让相机看到二维码的时刻 */
            snprintf(line, sizeof line, "$QR,%s\n", T.text);
            vision_emit(line);
        }
    } else if (v_mode == 2) {
        pose_t tp = field_turntable_pose();
        if (dist2d(P.x, P.y, tp.x, tp.y) > 150.0f || fabsf(wrap_pi(P.th - tp.th)) > DEG2RAD(10)) return;
        point_t ideal_f = field_turntable_pick_point();
        point_t ideal = field_to_robot(tp, ideal_f.x, ideal_f.y);       /* 理想取料点（车体坐标） */
        int color = 0; float dx = 0, dy = 0;
        for (int i = 0; i < 6; i++) {
            if (M[i].loc != M_TT) continue;
            point_t mp = tt_mat_point(&M[i]);
            point_t mr = field_to_robot(P, mp.x, mp.y);                  /* 物料真实位置（车体坐标） */
            /* 相机视野内、离理想取料点 60mm 以内、并且转盘停着时正对场地的那个物料 */
            point_t rel = { mp.x - TT_CX, mp.y - TT_CY };
            bool at_front = fabsf(wrap_pi(atan2f(rel.y, rel.x) + PI_F / 2.0f)) < DEG2RAD(8.0f);
            if (at_front && hypotf(mr.x - ideal.x, mr.y - ideal.y) < 60.0f) { color = M[i].color; dx = mr.x - ideal.x; dy = mr.y - ideal.y; }
        }
        snprintf(line, sizeof line, "$TT,%d,%d,%.1f,%.1f\n", color, tt_stop_left > 0.3f ? 1 : 0, dx, dy);
        vision_emit(line);
    } else if (v_mode == 3) {
        if (!C.vision_ring) return;
        /* 找离"理想放置点"最近的圆环 */
        for (int z = 0; z < 2; z++) for (int n = 1; n <= 3; n++) {
            pose_t pp = field_place_pose((place_zone_t)z, n);
            point_t rf = field_ring_point((place_zone_t)z, n);
            point_t ideal = field_to_robot(pp, rf.x, rf.y);       /* 程序认为圆环应该出现在相机画面的哪里 */
            if (dist2d(P.x, P.y, pp.x, pp.y) >= 110.0f) continue;
            /* 相机实际看到的：离这个位置最近的真实圆环 */
            point_t real = { 1e9f, 1e9f };
            for (int zz = 0; zz < 2; zz++) for (int nn = 1; nn <= 3; nn++) {
                point_t tr = true_ring(zz, nn), c = field_to_robot(P, tr.x, tr.y);
                if (hypotf(c.x - ideal.x, c.y - ideal.y) < hypotf(real.x - ideal.x, real.y - ideal.y)) real = c;
            }
            if (hypotf(real.x - ideal.x, real.y - ideal.y) < 75.0f) {
                snprintf(line, sizeof line, "$RING,1,%.1f,%.1f,%.2f\n", real.x - ideal.x, real.y - ideal.y,
                         RAD2DEG(wrap_pi(P.th - pp.th)));
                vision_emit(line);
                return;
            }
        }
        vision_emit("$RING,0,0,0\n");
    }
}

/* 记录一帧：车的位姿、机械臂三个关节角、夹爪开度、6 个物料的位置 */
static void rec_frame(void)
{
    if (s_nfr == s_cfr) { s_cfr = s_cfr ? s_cfr * 2 : 4096; s_fr = realloc(s_fr, (size_t)s_cfr * sizeof *s_fr); }
    frame_t *f = &s_fr[s_nfr++];
    f->t = s_t; f->x = P.x; f->y = P.y; f->th = P.th;
    raw_to_q((int)lroundf(st[1].pos), YAW_ZERO_RAW, YAW_DIR, &f->q[0]);
    raw_to_q((int)lroundf(st[2].pos), SHOULDER_ZERO_RAW, SHOULDER_DIR, &f->q[1]);
    raw_to_q((int)lroundf(st[3].pos), ELBOW_ZERO_RAW, ELBOW_DIR, &f->q[2]);
    float g = (sc[1].pos - (float)GRIP_RAW_EMPTY_CLOSED) / (float)(GRIP_RAW_OPEN - GRIP_RAW_EMPTY_CLOSED);
    f->g = g < 0 ? 0 : g > 1 ? 1 : g;
    for (int i = 0; i < 6; i++) {
        float x = 0, y = 0, z = 0;
        if (M[i].loc == M_GRIP) { float rx, ry; grasp_point(&x, &y, &z, &rx, &ry); z -= MAT_GRIP_FROM_BASE_MM; }
        else if (M[i].loc != M_NONE) mat_field_pos(&M[i], &x, &y, &z);
        f->m[i][0] = x; f->m[i][1] = y; f->m[i][2] = z; f->m[i][3] = (float)M[i].loc;
    }
    /* 三维回放用：转盘转角、四个轮子转过的角度（左前、右前、左后、右后） */
    static const int8_t WCH[4] = WHEEL_CH_INIT, WSG[4] = WHEEL_SIGN_INIT;
    f->tt = tt_ang;
    for (int i = 0; i < 4; i++) f->w[i] = (float)(ch_cnt[WCH[i]] * WSG[i] * 2.0 * PI_F / ENC_COUNTS_PER_WHEEL_REV);
    for (int i = 0; i < TOF_COUNT; i++) f->tof[i] = (short)tof_get_mm(i, 120);
}

/* ======================================================================
 * 障碍物：按规则“数量和位置抽签决定，放在车道上”随机生成
 * ====================================================================== */
static bool obs_spot_ok(float x, float y)
{
    /* 障碍物整个落在灰色车道上 */
    for (int k = 0; k < 8; k++) {
        float a = (float)k * PI_F / 4.0f;
        if (!ref_point_ok(x + 25.0f * cosf(a), y + 25.0f * sinf(a))) return false;
    }
    if (dist2d(x, y, 1200.0f, 2470.0f) < 150.0f + 25.0f + 100.0f) return false;
    /* 不放在任务点上：车要停的位置（读码、取料、6 个放料位）和启停区周围 330mm 内不放。
     * 真比赛里裁判也不会把障碍物摆在必须停车的地方，否则这一项就没法完成了 */
    pose_t keep[10];
    int nk = 0;
    keep[nk++] = field_start_pose(1); keep[nk++] = field_start_pose(2);
    keep[nk++] = field_qr_pose(); keep[nk++] = field_turntable_pose();
    for (int z = 0; z < 2; z++) for (int r = 1; r <= 3; r++) keep[nk++] = field_place_pose((place_zone_t)z, r);
    for (int k = 0; k < nk; k++) if (dist2d(x, y, keep[k].x, keep[k].y) < 330.0f) return false;
    if (x > 2100.0f - 25.0f && (y > 2100.0f - 25.0f || y < 300.0f + 25.0f)) return false;   /* 启停区 */
    for (int k = 0; k < s_nob; k++) if (dist2d(x, y, OB[k].x, OB[k].y) < 450.0f) return false;
    return true;
}

/* 放完之后车道网格还连通吗（用和程序相同的“挡住”判据：离车道中心线 < 车宽一半+障碍物半径+余量） */
static bool obs_graph_connected(void)
{
    bool blk[NODE_COUNT][NODE_COUNT];
    memset(blk, 0, sizeof blk);
    /* 3×3 车道网格：同一行相邻、同一列相邻的节点之间有路（不依赖 field.c 的初始化） */
    #define GRID_ADJ(a, b) (((a) / 3 == (b) / 3 && abs((a) - (b)) == 1) || abs((a) - (b)) == 3)
    for (int a = 0; a < NODE_COUNT; a++) for (int b = a + 1; b < NODE_COUNT; b++) {
        if (!GRID_ADJ(a, b)) continue;
        point_t A = field_node((node_id_t)a), B = field_node((node_id_t)b);
        for (int k = 0; k < s_nob; k++) {
            float vx = B.x - A.x, vy = B.y - A.y, L2 = vx * vx + vy * vy;
            float t = ((OB[k].x - A.x) * vx + (OB[k].y - A.y) * vy) / L2;
            t = t < 0 ? 0 : t > 1 ? 1 : t;
            if (dist2d(OB[k].x, OB[k].y, A.x + vx * t, A.y + vy * t) < REF_HW + 25.0f + 35.0f) blk[a][b] = blk[b][a] = true;
        }
    }
    bool seen[NODE_COUNT] = { false };
    int stack[NODE_COUNT], sp = 0, cnt = 0;
    stack[sp++] = 0; seen[0] = true;
    while (sp) {
        int a = stack[--sp]; cnt++;
        for (int b = 0; b < NODE_COUNT; b++)
            if (!seen[b] && GRID_ADJ(a, b) && !blk[a][b]) { seen[b] = true; stack[sp++] = b; }
    }
    return cnt == NODE_COUNT;
}

static void obs_generate(void)
{
    s_rng2 = (C.obs_seed ? C.obs_seed : C.seed) * 2654435761u + (unsigned)C.start_zone * 40503u + 12345u;
    for (int i = 0; i < 8; i++) (void)rnd2();
    s_nob = 0; s_obs_hits = 0; s_first_hit = 0;
    memset(s_tof_mm, 0, sizeof s_tof_mm); s_tof_t = 0;
    int want = C.n_obs < 0 ? 1 + (int)(rnd2() * 3.0f) : C.n_obs;
    if (want > SIM_MAX_OBS) want = SIM_MAX_OBS;
    static const float LANE_C[3] = { 275.0f, 1200.0f, 2125.0f };   /* 三条横道、三条竖道的中线（灰色区域的正中） */
    for (int tries = 0; s_nob < want && tries < 5000; tries++) {
        int vert = rnd2() < 0.5f;
        float c = LANE_C[(int)(rnd2() * 3.0f) % 3] + (rnd2() * 2.0f - 1.0f) * 100.0f;
        float a = 300.0f + rnd2() * 1800.0f;
        float x = vert ? c : a, y = vert ? a : c;
        if (!obs_spot_ok(x, y)) continue;
        OB[s_nob].x = x; OB[s_nob].y = y; OB[s_nob].hit_t = 0;
        s_nob++;
        if (!obs_graph_connected()) s_nob--;          /* 把路全堵死的摆法不要 */
    }
}

int sim_obstacle_count(void) { return s_nob; }
void sim_obstacle_get(int i, float *x, float *y) { *x = OB[i].x; *y = OB[i].y; }

/* ======================================================================
 * 测距传感器（VL53L0X）模型
 *   每个传感器在 25° 视场里打 7 条射线，取最近的回波；黑色障碍物反射弱，最远只到 SIM_TOF_BLACK_MAX。
 *   能被看到的东西（离地 50mm 这一高度上）：障碍物、地上的物料、原料转盘、场地四周围挡（含二维码板）。
 * ====================================================================== */
static float ray_circle(point_t o, float ux, float uy, float cx, float cy, float r)
{
    float vx = cx - o.x, vy = cy - o.y, tca = vx * ux + vy * uy;
    if (tca < 0) return 1e9f;
    float d2 = vx * vx + vy * vy - tca * tca;
    if (d2 > r * r) return 1e9f;
    float t = tca - sqrtf(r * r - d2);
    return t < 0 ? 0 : t;
}

static float ray_walls(point_t o, float ux, float uy)
{
    float t = 1e9f;
    if (ux > 1e-6f) t = fminf(t, (2400.0f - o.x) / ux);
    if (ux < -1e-6f) t = fminf(t, (0.0f - o.x) / ux);
    if (uy > 1e-6f) t = fminf(t, (2400.0f - o.y) / uy);
    if (uy < -1e-6f) t = fminf(t, (0.0f - o.y) / uy);
    return t;
}

static float gauss2(void) { return (rnd2() + rnd2() + rnd2() - 1.5f) * 2.0f; }   /* 近似正态，σ≈1 */

static void tof_sim_step(void)
{
    for (int i = 0; i < TOF_COUNT; i++) {
        const tof_mount_t *m = tof_mount(i);
        point_t o = robot_to_field(P, m->x, m->y);
        float best = 1e9f;
        for (int r = 0; r < 7; r++) {
            float a = P.th + DEG2RAD(m->yaw_deg) + DEG2RAD(TOF_FOV_DEG) * ((float)r / 6.0f - 0.5f);
            float ux = cosf(a), uy = sinf(a), t;
            for (int k = 0; k < s_nob; k++) {
                t = ray_circle(o, ux, uy, OB[k].x, OB[k].y, 25.0f);
                if (t < SIM_TOF_BLACK_MAX && t < best) best = t;
            }
            for (int k = 0; k < 6; k++) {
                if (M[k].loc != M_FIELD || M[k].z > TOF_MOUNT_Z_MM) continue;
                t = ray_circle(o, ux, uy, M[k].x, M[k].y, 25.0f);
                if (t < SIM_TOF_OTHER_MAX && t < best) best = t;
            }
            t = ray_circle(o, ux, uy, TT_CX, TT_CY, 150.0f);
            if (t < SIM_TOF_OTHER_MAX && t < best) best = t;
            t = ray_walls(o, ux, uy);
            if (t < SIM_TOF_OTHER_MAX && t < best) best = t;
        }
        int mm = 0;
        if (best < 1e8f && rnd2() > 0.01f) {                         /* 1% 的概率这次没回波 */
            float d = best * (1.0f + 0.015f * gauss2()) + 3.0f * gauss2();
            mm = (int)lroundf(d < 0 ? 0 : d);
        }
        s_tof_mm[i] = mm;
    }
    s_tof_t = s_t;
}

/* 下面这几个函数替换 tof_vl53l0x.c 里的硬件驱动 */
int  tof_init(void) { return TOF_COUNT; }
void tof_poll(void) { }
bool tof_ok(int i) { return i >= 0 && i < TOF_COUNT; }
int  tof_get_mm(int i, uint32_t max_age)
{
    if (i < 0 || i >= TOF_COUNT || s_t - s_tof_t > max_age) return 0;
    int d = s_tof_mm[i];
    return (d < 20 || d > TOF_MAX_RANGE_MM) ? 0 : d;
}

/* 车身（真实外形）碰到障碍物了吗 */
static void obs_collide_step(void)
{
    for (int k = 0; k < s_nob; k++) {
        point_t o = field_to_robot(P, OB[k].x, OB[k].y);
        float dx = fabsf(o.x) - REF_HL, dy = fabsf(o.y) - REF_HW;
        dx = dx > 0 ? dx : 0; dy = dy > 0 ? dy : 0;
        if (hypotf(dx, dy) < 25.0f && !OB[k].hit_t) {
            OB[k].hit_t = s_t;
            s_obs_hits++;
            if (!s_first_hit) s_first_hit = s_t;
            sim_note("[模拟] 第 %.1f 秒车身撞到障碍物 #%d (%.0f, %.0f)", s_t / 1000.0, k + 1, OB[k].x, OB[k].y);
        }
    }
}

/* ---------- 每 1ms 推进一次 ---------- */
void sim_step_1ms(void)
{
    const float dt = 0.001f;
    s_t++;

    /* 电机：一阶惯性，时间常数 60ms */
    static const int8_t WCH[4] = WHEEL_CH_INIT, WSG[4] = WHEEL_SIGN_INIT;
    for (int i = 0; i < 4; i++) {
        ch_spd[i] += (ch_cmd[i] - ch_spd[i]) * dt / 0.06f;
        ch_cnt[i] += (double)ch_spd[i] * 100.0 * dt;
    }
    float w[4];
    for (int i = 0; i < 4; i++) w[i] = ch_spd[WCH[i]] * (float)WSG[i] * 100.0f * MM_PER_COUNT;
    float vx = (w[0] + w[1] + w[2] + w[3]) * 0.25f;
    float vy = (-w[0] + w[1] + w[2] - w[3]) * 0.25f;
    float om = (-w[0] + w[1] - w[2] + w[3]) * 0.25f / K_ROT;
    float k = 1.0f - C.slip;
    vx *= k; vy *= k; om *= k;
    float thm = P.th + om * dt * 0.5f;
    P.x += (cosf(thm) * vx - sinf(thm) * vy) * dt;
    P.y += (sinf(thm) * vx + cosf(thm) * vy) * dt;
    P.th = wrap_pi(P.th + om * dt);
    if (C.noise_mm > 0 && (fabsf(vx) + fabsf(vy)) > 1.0f) { P.x += frand() * C.noise_mm; P.y += frand() * C.noise_mm; }

    /* 舵机 */
    for (int i = 1; i <= 3; i++) {               /* 梯形速度：按 ARM_ACC 加速、提前减速 */
        float vmax = st[i].speed > 0 ? (float)st[i].speed : 3000.0f, acc = (float)ARM_ACC * 100.0f;
        float d = (float)st[i].goal - st[i].pos;
        float vstop = sqrtf(2.0f * acc * fabsf(d));
        float vt = copysignf(fminf(vmax, vstop), d);
        float dv = vt - st[i].vel, mdv = acc * dt;
        st[i].vel += fabsf(dv) <= mdv ? dv : copysignf(mdv, dv);
        float step = st[i].vel * dt;
        if (fabsf(d) <= fabsf(step) || fabsf(d) < 0.5f) { st[i].pos = (float)st[i].goal; st[i].vel = 0; }
        else st[i].pos += step;
    }
    { float d = (float)sc[1].goal - sc[1].pos, step = 1500.0f * dt; sc[1].pos += fabsf(d) <= step ? d : copysignf(step, d); }

    /* 转盘：匀速转，物料转到取料点（正对场地，角度 -90°）时停 2.5s */
    if (tt_stop_left > 0) {
        tt_stop_left -= dt;
    } else {
        float prev = tt_ang;
        tt_ang += tt_dir * (2.0f * PI_F / 8.0f) * dt;
        static int last_stop = -1;
        for (int i = 0; i < 6; i++) {
            if (M[i].loc != M_TT || i == last_stop) continue;
            float a0 = wrap_pi(M[i].ang + prev + PI_F / 2.0f), a1 = wrap_pi(M[i].ang + tt_ang + PI_F / 2.0f);
            if ((a0 < 0) != (a1 < 0) && fabsf(a0) < 0.5f) { tt_stop_left = 2.5f; tt_ang -= a1; last_stop = i; break; }   /* 对正后停 */
        }
    }
    /* 第一批取完后裁判补上第二批 */
    bool any_tt = false;
    for (int i = 0; i < 6; i++) if (M[i].loc == M_TT) any_tt = true;
    if (!any_tt && !tt_batch2_loaded) {
        if (tt_refill_timer < 0) tt_refill_timer = 4.0f;
        tt_refill_timer -= dt;
        if (tt_refill_timer <= 0) { tt_place_batch(1); tt_batch2_loaded = true; }
    }

    vision_step();
    if (s_t % 33 == 0) tof_sim_step();
    if (s_t % 5 == 0) obs_collide_step();

    /* 可选：环境变量 SIM_TRACE=文件名 时，每 100ms 记录一次真实位姿（画轨迹图用） */
    static FILE *trace;
    static bool trace_init;
    if (!trace_init) { trace_init = true; const char *fn = getenv("SIM_TRACE"); if (fn) trace = fopen(fn, "w"); }
    if (trace && s_t % 100 == 0) fprintf(trace, "%u,%.1f,%.1f,%.2f\n", s_t, P.x, P.y, RAD2DEG(P.th));

    if (C.verbose && s_t % REC_DT_MS == 0) rec_frame();

    /* 车道检查：每 20ms 采样一次真实位姿 */
    if (s_t % 20 == 0) {
        pose_t sp = field_start_pose(C.start_zone);
        bool in_start = fabsf(P.x - sp.x) < 150.0f && fabsf(P.y - sp.y) < 150.0f;
        if (!in_start) s_left_start = true;
        if (s_left_start && !in_start && !ref_pose_on_lane(P)) {
            if (s_viol == 0) {
                s_vx = P.x; s_vy = P.y;
                s_viol_t = s_t;
                memcpy(M_snap, M, sizeof M);
                s_grab_at_viol = s_grab_tt_ok + s_grab_rough_ok;
                sim_note("[模拟] 第 %.1f 秒车身越出车道！按规则本轮到此结束，之后的动作不再计分（仿真继续跑完只是方便你看后面的问题）", s_t / 1000.0);
            }
            if ((s_viol % 25) == 0) sim_note("[模拟] 压线 t=%.2f 位姿(%.0f,%.0f,%.0f°)", s_t / 1000.0, P.x, P.y, RAD2DEG(P.th));
            s_viol++;
        }
    }
}

/* ---------- 评分（按规则表 3 与评分摘要近似计算） ---------- */
static int ring_score(float err)
{
    /* 物料最大直径 φ=50：1环外径 53 → 中心偏差 ≤1.5 得 15 分，依次类推 */
    if (err <= 1.5f) return 15;
    if (err <= 4.0f) return 10;
    if (err <= 7.5f) return 7;
    if (err <= 12.5f) return 5;
    if (err <= 17.5f) return 3;
    if (err <= 22.5f) return 1;
    return 0;
}

void sim_report(sim_report_t *r, int dg, int dp)
{
    (void)dg; (void)dp;
    memset(r, 0, sizeof *r);
    r->lane_violations = s_viol; r->worst_violation_x = s_vx; r->worst_violation_y = s_vy;
    r->correct_grabs = s_grab_tt_ok + s_grab_rough_ok;
    r->ended_ms = s_viol ? s_viol_t : 0;
    r->qr_before_end = s_qr_t && (!s_viol || s_qr_t < s_viol_t);
    if (s_viol) r->correct_grabs = s_grab_at_viol;          /* 越线之后的抓取不算 */
    const mat_t *MM = s_viol ? M_snap : M;                    /* 平放/码垛按越线那一刻的状态算 */

    /* 粗加工区：每次放下时记分（之后会被取回） */
    for (int e = 0; e < s_nev; e++) {
        mat_t *m = &M[s_ev[e].idx];
        if (s_ev[e].y > 200.0f || s_ev[e].z > 1.0f) continue;     /* 只看粗加工区的平放 */
        if (s_viol && s_ev[e].t >= s_viol_t) continue;            /* 越线之后放的不算 */
        int idx = -1;
        for (int k = 0; k < 3; k++) if (T.color[m->batch][k] == m->color) idx = k;
        point_t rp = true_ring(0, T.rough_ring[m->batch][idx]);   /* 任务码要求的那个圆环（真实位置） */
        float err = dist2d(s_ev[e].x, s_ev[e].y, rp.x, rp.y);
        if (err > r->max_place_err_mm) r->max_place_err_mm = err;
        r->rough_ring_score += ring_score(err);
    }
    /* 暂存区：看最终状态 */
    int base_score[4] = { 0, 0, 0, 0 };
    for (int i = 0; i < 3; i++) {
        const mat_t *m = &MM[i];
        if (m->loc != M_FIELD || m->z > 1.0f) continue;
        int ring = T.stash_ring[0][i];
        point_t rp = true_ring(1, ring);
        float err = dist2d(m->x, m->y, rp.x, rp.y);
        if (err > r->max_place_err_mm) r->max_place_err_mm = err;
        base_score[ring] = ring_score(err);
        r->stash_ring_score += base_score[ring];
    }
    for (int i = 3; i < 6; i++) {
        const mat_t *m = &MM[i];
        if (m->loc != M_FIELD || m->z < 50.0f) continue;
        int k = -1;
        for (int j = 0; j < 3; j++) if (T.color[1][j] == m->color) k = j;
        int ring = T.stash_ring[1][k];
        /* 下面那个必须是同色第一批 */
        for (int b = 0; b < 3; b++) {
            if (MM[b].color == m->color && MM[b].loc == M_FIELD && hypotf(MM[b].x - m->x, MM[b].y - m->y) < 12.0f)
                r->stack_score += base_score[ring];
        }
    }
    /* 回区：车身投影的四个角都在 300×300 启停区内 */
    pose_t sp = field_start_pose(C.start_zone);
    r->in_start_zone = true;
    for (int sx = -1; sx <= 1; sx += 2) for (int sy = -1; sy <= 1; sy += 2) {
        point_t c = robot_to_field(P, sx * ROBOT_HALF_LEN_MM, sy * ROBOT_HALF_WID_MM);
        if (fabsf(c.x - sp.x) > 150.0f || fabsf(c.y - sp.y) > 150.0f) r->in_start_zone = false;
    }
    if (s_viol) r->in_start_zone = false;                     /* 本轮已提前结束，没有返程分 */

    /* 障碍物 */
    r->n_obs = s_nob; r->obs_hits = s_obs_hits; r->first_hit_ms = s_first_hit;
    for (int k = 0; k < s_nob; k++)
        for (int j = 0; j < field_obs_count(); j++) {
            point_t q = field_obs_get(j);
            if (dist2d(q.x, q.y, OB[k].x, OB[k].y) < 60.0f) { r->obs_found++; break; }
        }
    for (int j = 0; j < field_obs_count(); j++) {
        point_t q = field_obs_get(j);
        bool real = false;
        for (int k = 0; k < s_nob; k++) if (dist2d(q.x, q.y, OB[k].x, OB[k].y) < 80.0f) real = true;
        if (!real) r->obs_false++;
    }
}

/* ---------- 写出 HTML 回放（浏览器打开即可播放） ---------- */
#include "replay_template.h"

static void json_str(FILE *o, const char *t)
{
    fputc('"', o);
    for (; *t; t++) {
        unsigned char c = (unsigned char)*t;
        if (c == '"' || c == '\\') { fputc('\\', o); fputc(c, o); }
        else if (c == '<') fputs("\\u003c", o);
        else if (c < 0x20) fputc(' ', o);
        else fputc(c, o);
    }
    fputc('"', o);
}

/* 把一场的全部回放数据写成一个 JSON 对象（二维回放页和三维回放页共用） */
static void write_data(FILE *o, const char *summary, const char *result_json)
{
    char when[32] = "";
    time_t now = time(NULL);
    struct tm *lt = localtime(&now);
    if (lt) strftime(when, sizeof when, "%Y-%m-%d %H:%M:%S", lt);
    fprintf(o, "{\"version\":2,\"run_at\":");
    json_str(o, when);
    fprintf(o, ",\"task\":");
    json_str(o, C.task);
    fprintf(o, ",\"zone\":%d,\"slip\":%.3f,\"imu\":%d,\"summary\":", C.start_zone, C.slip, C.imu ? 1 : 0);
    json_str(o, summary);
    fprintf(o, ",\"result\":%s", result_json && result_json[0] ? result_json : "null");
    fprintf(o, ",\"config\":{\"nav_vmax\":%.0f,\"nav_acc\":%.0f,\"arm_speed\":%d,\"arm_safe_z\":%.0f,\"tool_side\":%.1f,\"shoulder_r0\":%.1f,\"return_reserve_ms\":%d,\"lane_edge\":%.0f}",
            NAV_VMAX_MMPS, NAV_ACC_MMPS2, (int)ARM_SPEED_RAW_PER_S, ARM_SAFE_Z_MM, ARM_TOOL_SIDE_MM, ARM_SHOULDER_R0_MM, (int)MATCH_RETURN_RESERVE_MS,
            field_node(NODE_TL).x);
    fprintf(o, ",\"robot\":{\"hl\":%.1f,\"hw\":%.1f,\"bx\":%.1f,\"by\":%.1f,\"r0\":%.1f,\"side\":%.1f,\"l1\":%.1f,\"l2\":%.1f,\"shz\":%.1f,"
               "\"slots\":[[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]],\"tray\":%.1f,\"tt\":%.1f,\"mh\":%.1f,\"grip_from_base\":%.1f}",
            ROBOT_HALF_LEN_MM, ROBOT_HALF_WID_MM, ARM_BASE_X_MM, ARM_BASE_Y_MM, ARM_SHOULDER_R0_MM, ARM_TOOL_SIDE_MM, ARM_L1_MM, ARM_L2_MM, ARM_SHOULDER_Z_MM,
            SLOT0_X_MM, SLOT0_Y_MM, SLOT1_X_MM, SLOT1_Y_MM, SLOT2_X_MM, SLOT2_Y_MM, CAR_TRAY_FLOOR_Z_MM, TURNTABLE_TOP_Z_MM, MAT_HEIGHT_MM, MAT_GRIP_FROM_BASE_MM);
    fprintf(o, ",\"field\":{\"tt\":[%.1f,%.1f],\"nodes\":[", TT_CX, TT_CY);
    for (int i = 0; i < NODE_COUNT; i++) { point_t p = field_node((node_id_t)i); fprintf(o, "%s[%.1f,%.1f]", i ? "," : "", p.x, p.y); }
    fprintf(o, "],\"rings\":[");
    for (int z = 0; z < 2; z++) for (int n = 1; n <= 3; n++) { point_t p = true_ring(z, n); fprintf(o, "%s[%d,%d,%.1f,%.1f]", (z || n > 1) ? "," : "", z, n, p.x, p.y); }
    fprintf(o, "],\"obstacles\":[");
    for (int k = 0; k < s_nob; k++) fprintf(o, "%s[%.0f,%.0f,%u]", k ? "," : "", OB[k].x, OB[k].y, (unsigned)OB[k].hit_t);
    fprintf(o, "],\"obs_found\":[");
    for (int j = 0; j < field_obs_count(); j++) { point_t q = field_obs_get(j); fprintf(o, "%s[%.0f,%.0f]", j ? "," : "", q.x, q.y); }
    fprintf(o, "]},\"tof\":{\"z\":%.0f,\"fov\":%.1f,\"max\":%d,\"mounts\":[", TOF_MOUNT_Z_MM, TOF_FOV_DEG, TOF_MAX_RANGE_MM);
    for (int i = 0; i < TOF_COUNT; i++) { const tof_mount_t *m = tof_mount(i); fprintf(o, "%s[%.1f,%.1f,%.1f]", i ? "," : "", m->x, m->y, m->yaw_deg); }
    fprintf(o, "]},\"colors\":[");
    for (int i = 0; i < 6; i++) fprintf(o, "%s%d", i ? "," : "", M[i].color);
    /* 每帧：t,x,y,th,q0,q1,q2,夹爪开度, 6×(x,y,z,位置类型), 转盘角, 4个轮子角, 各测距传感器读数(mm，0=没看到) */
    fprintf(o, "],\"frames\":[");
    for (int k = 0; k < s_nfr; k++) {
        frame_t *f = &s_fr[k];
        fprintf(o, "%s[%u,%.0f,%.0f,%.3f,%.3f,%.3f,%.3f,%.2f", k ? "," : "", f->t, f->x, f->y, f->th, f->q[0], f->q[1], f->q[2], f->g);
        for (int i = 0; i < 6; i++) fprintf(o, ",%.0f,%.0f,%.0f,%d", f->m[i][0], f->m[i][1], f->m[i][2], (int)f->m[i][3]);
        fprintf(o, ",%.3f,%.2f,%.2f,%.2f,%.2f", f->tt, f->w[0], f->w[1], f->w[2], f->w[3]);
        for (int i = 0; i < TOF_COUNT; i++) fprintf(o, ",%d", f->tof[i]);
        fputc(']', o);
        if (k % 8 == 7) fputc('\n', o);
    }
    fprintf(o, "],\"logs\":[");
    for (int k = 0; k < s_nlg; k++) { fprintf(o, "%s[%u,", k ? "," : "", s_lg[k].t); json_str(o, s_lg[k].s); fputs("]\n", o); }
    fprintf(o, "]}");
}

/* 二维回放：一个独立的 HTML 文件（数据嵌在里面） */
int sim_write_replay(const char *path, const char *summary, const char *result_json)
{
    FILE *o = fopen(path, "wb");
    if (!o) return -1;
    fputs(REPLAY_HEAD, o);
    fprintf(o, "const DATA=");
    write_data(o, summary, result_json);
    fprintf(o, ";\n");
    fputs(REPLAY_TAIL, o);
    fclose(o);
    return 0;
}

/* 三维回放用的数据文件：window.SIM_REPLAY = {...}; 由"三维仿真回放.html"读取 */
int sim_write_replay_js(const char *path, const char *summary, const char *result_json)
{
    FILE *o = fopen(path, "wb");
    if (!o) return -1;
    fprintf(o, "window.SIM_REPLAY=");
    write_data(o, summary, result_json);
    fprintf(o, ";\n");
    fclose(o);
    return 0;
}
