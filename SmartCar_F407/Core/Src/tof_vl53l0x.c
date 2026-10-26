/**
 * tof_vl53l0x.c —— VL53L0X 驱动（多路开关 TCA9548A 版）
 *
 * 初始化流程移植自 Pololu 的 VL53L0X Arduino 库（MIT 许可，https://github.com/pololu/vl53l0x-arduino），
 * 它是把 ST 官方 API 精简成“直接读写寄存器”的版本。改动：C 语言、非阻塞读取、每个传感器接多路开关的一个通道。
 * 【上车验证】这部分只在真实硬件上运行（模拟器不编译它），装好后用串口命令 tof 检查 8 个读数。
 */
#include "tof.h"
#include "port.h"
#include "sys.h"
#include "robot_config.h"
#include <string.h>

static const tof_mount_t MOUNT[TOF_COUNT] = TOF_LAYOUT_INIT;
int tof_count(void) { return TOF_COUNT; }
const tof_mount_t *tof_mount(int i) { return (i >= 0 && i < TOF_COUNT) ? &MOUNT[i] : NULL; }

#if !defined(SIM_HAS_TOF)

#define ADDR 0x29

enum {
    SYSRANGE_START = 0x00, SYSTEM_SEQUENCE_CONFIG = 0x01, SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A, SYSTEM_INTERRUPT_CLEAR = 0x0B,
    RESULT_INTERRUPT_STATUS = 0x13, RESULT_RANGE_STATUS = 0x14,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44, MSRC_CONFIG_TIMEOUT_MACROP = 0x46,
    PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51, MSRC_CONFIG_CONTROL = 0x60,
    FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
    GPIO_HV_MUX_ACTIVE_HIGH = 0x84, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0, GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E, DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
    IDENTIFICATION_MODEL_ID = 0xC0,
};

typedef struct { bool ok; uint8_t stop_var; uint16_t mm; uint32_t t; } tof_t;
static tof_t   s_tof[TOF_COUNT];
static int     s_cur = -1;          /* 多路开关当前选中的通道 */
static bool    s_err;               /* 本次操作中有 I2C 错误 */
static int     s_rr;                /* 轮询起点（每次只查几个，免得占满 I2C） */

static void sel(int ch)
{
    if (ch == s_cur) return;
    uint8_t m = (uint8_t)(1u << ch);
    /* TCA9548A 没有寄存器：直接写一个字节（通道位图）。port_i2c_write 的 reg 参数就是这个字节 */
    if (port_i2c_write(TOF_MUX_ADDR, m, NULL, 0) != 0) { s_err = true; s_cur = -1; return; }
    s_cur = ch;
}
static void wr8(uint8_t r, uint8_t v) { if (port_i2c_write(ADDR, r, &v, 1) != 0) s_err = true; }
static void wr16(uint8_t r, uint16_t v) { uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v }; if (port_i2c_write(ADDR, r, b, 2) != 0) s_err = true; }
static uint8_t rd8(uint8_t r) { uint8_t v = 0; if (port_i2c_read(ADDR, r, &v, 1) != 0) s_err = true; return v; }
static uint16_t rd16(uint8_t r) { uint8_t b[2] = { 0, 0 }; if (port_i2c_read(ADDR, r, b, 2) != 0) s_err = true; return (uint16_t)((b[0] << 8) | b[1]); }
static void wrn(uint8_t r, const uint8_t *d, uint16_t n) { if (port_i2c_write(ADDR, r, d, n) != 0) s_err = true; }
static void rdn(uint8_t r, uint8_t *d, uint16_t n) { if (port_i2c_read(ADDR, r, d, n) != 0) s_err = true; }

/* ---------- 计时换算（来自 ST API） ---------- */
static uint16_t decode_timeout(uint16_t v) { return (uint16_t)(((v & 0x00FF) << ((v & 0xFF00) >> 8)) + 1); }
static uint16_t encode_timeout(uint32_t mclks)
{
    if (mclks == 0) return 0;
    uint32_t ls = mclks - 1; uint16_t ms = 0;
    while ((ls & 0xFFFFFF00u) > 0) { ls >>= 1; ms++; }
    return (uint16_t)((ms << 8) | (ls & 0xFF));
}
static uint32_t macro_period_ns(uint8_t vcsel_pclks) { return ((2304u * vcsel_pclks * 1655u) + 500u) / 1000u; }
static uint32_t mclks_to_us(uint16_t mclks, uint8_t vcsel) { uint32_t m = macro_period_ns(vcsel); return ((mclks * m) + 500u) / 1000u; }
static uint32_t us_to_mclks(uint32_t us, uint8_t vcsel) { uint32_t m = macro_period_ns(vcsel); return ((us * 1000u) + (m / 2u)) / m; }
static uint8_t vcsel_pclks(uint8_t reg) { return (uint8_t)((rd8(reg) + 1) << 1); }

typedef struct { bool tcc, msrc, dss, pre, fin; } steps_t;
typedef struct { uint8_t pre_vcsel, fin_vcsel; uint16_t msrc_mclks, pre_mclks, fin_mclks; uint32_t msrc_us, pre_us, fin_us; } tmo_t;

static void get_steps(steps_t *s)
{
    uint8_t c = rd8(SYSTEM_SEQUENCE_CONFIG);
    s->tcc = (c >> 4) & 1; s->dss = (c >> 3) & 1; s->msrc = (c >> 2) & 1; s->pre = (c >> 6) & 1; s->fin = (c >> 7) & 1;
}
static void get_timeouts(const steps_t *s, tmo_t *t)
{
    t->pre_vcsel = vcsel_pclks(PRE_RANGE_CONFIG_VCSEL_PERIOD);
    t->msrc_mclks = (uint16_t)(rd8(MSRC_CONFIG_TIMEOUT_MACROP) + 1);
    t->msrc_us = mclks_to_us(t->msrc_mclks, t->pre_vcsel);
    t->pre_mclks = decode_timeout(rd16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    t->pre_us = mclks_to_us(t->pre_mclks, t->pre_vcsel);
    t->fin_vcsel = vcsel_pclks(FINAL_RANGE_CONFIG_VCSEL_PERIOD);
    t->fin_mclks = decode_timeout(rd16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    if (s->pre) t->fin_mclks = (uint16_t)(t->fin_mclks - t->pre_mclks);
    t->fin_us = mclks_to_us(t->fin_mclks, t->fin_vcsel);
}
static uint32_t get_budget_us(void)
{
    steps_t s; tmo_t t; get_steps(&s); get_timeouts(&s, &t);
    uint32_t b = 1910 + 960;
    if (s.tcc) b += t.msrc_us + 590;
    if (s.dss) b += 2 * (t.msrc_us + 690); else if (s.msrc) b += t.msrc_us + 660;
    if (s.pre) b += t.pre_us + 660;
    if (s.fin) b += t.fin_us + 550;
    return b;
}
static void set_budget_us(uint32_t budget)
{
    steps_t s; tmo_t t; get_steps(&s); get_timeouts(&s, &t);
    uint32_t used = 1320 + 960;
    if (s.tcc) used += t.msrc_us + 590;
    if (s.dss) used += 2 * (t.msrc_us + 690); else if (s.msrc) used += t.msrc_us + 660;
    if (s.pre) used += t.pre_us + 660;
    if (!s.fin) return;
    used += 550;
    if (used > budget) return;
    uint32_t fin = us_to_mclks(budget - used, t.fin_vcsel);
    if (s.pre) fin += t.pre_mclks;
    wr16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(fin));
}

static bool wait_flag(uint8_t reg, uint8_t mask, bool want_set, uint32_t ms)
{
    uint32_t t0 = sys_now();
    while ((uint32_t)(sys_now() - t0) < ms) {
        uint8_t v = rd8(reg);
        if (s_err) return false;
        if (((v & mask) != 0) == want_set) return true;
    }
    return false;
}

static bool single_ref_cal(uint8_t vhv_init)
{
    wr8(SYSRANGE_START, (uint8_t)(0x01 | vhv_init));
    if (!wait_flag(RESULT_INTERRUPT_STATUS, 0x07, true, 100)) return false;
    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    wr8(SYSRANGE_START, 0x00);
    return true;
}

static bool get_spad_info(uint8_t *count, bool *aperture)
{
    wr8(0x80, 0x01); wr8(0xFF, 0x01); wr8(0x00, 0x00);
    wr8(0xFF, 0x06); wr8(0x83, (uint8_t)(rd8(0x83) | 0x04));
    wr8(0xFF, 0x07); wr8(0x81, 0x01);
    wr8(0x80, 0x01);
    wr8(0x94, 0x6b); wr8(0x83, 0x00);
    if (!wait_flag(0x83, 0xFF, true, 100)) return false;
    wr8(0x83, 0x01);
    uint8_t tmp = rd8(0x92);
    *count = tmp & 0x7f; *aperture = (tmp >> 7) & 0x01;
    wr8(0x81, 0x00); wr8(0xFF, 0x06); wr8(0x83, (uint8_t)(rd8(0x83) & ~0x04));
    wr8(0xFF, 0x01); wr8(0x00, 0x01);
    wr8(0xFF, 0x00); wr8(0x80, 0x00);
    return !s_err;
}

/* 厂家推荐的默认调校参数（ST API 的 DefaultTuningSettings） */
static const uint8_t TUNING[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},{0x24,0x01},{0x25,0xFF},{0x75,0x00},
    {0xFF,0x01},{0x4E,0x2C},{0x48,0x00},{0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},{0x52,0x96},{0x56,0x08},{0x57,0x30},
    {0x61,0x00},{0x62,0x00},{0x64,0x00},{0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},{0x23,0x34},{0x42,0x00},{0x44,0xFF},
    {0x45,0x26},{0x46,0x05},{0x40,0x40},{0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},{0x44,0x00},{0x45,0x20},{0x47,0x08},
    {0x48,0x28},{0x67,0x00},{0x70,0x04},{0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},{0xFF,0x00},{0x80,0x00},
};

static bool init_one(tof_t *t)
{
    s_err = false;
    if (rd8(IDENTIFICATION_MODEL_ID) != 0xEE || s_err) return false;
    wr8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, (uint8_t)(rd8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01));   /* 2.8V I/O */
    wr8(0x88, 0x00);
    wr8(0x80, 0x01); wr8(0xFF, 0x01); wr8(0x00, 0x00);
    t->stop_var = rd8(0x91);
    wr8(0x00, 0x01); wr8(0xFF, 0x00); wr8(0x80, 0x00);
    wr8(MSRC_CONFIG_CONTROL, (uint8_t)(rd8(MSRC_CONFIG_CONTROL) | 0x12));
    wr16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * (1 << 7)));   /* 信号下限 0.25 MCPS */
    wr8(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    uint8_t spad_count; bool aperture;
    if (!get_spad_info(&spad_count, &aperture)) return false;
    uint8_t map[6];
    rdn(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6);
    wr8(0xFF, 0x01); wr8(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00); wr8(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    wr8(0xFF, 0x00); wr8(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
    uint8_t first = aperture ? 12 : 0, enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first || enabled == spad_count) map[i / 8] &= (uint8_t)~(1u << (i % 8));
        else if ((map[i / 8] >> (i % 8)) & 0x1) enabled++;
    }
    wrn(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6);

    for (unsigned k = 0; k < sizeof TUNING / sizeof TUNING[0]; k++) wr8(TUNING[k][0], TUNING[k][1]);

    wr8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    wr8(GPIO_HV_MUX_ACTIVE_HIGH, (uint8_t)(rd8(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10));
    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    uint32_t budget = get_budget_us();
    wr8(SYSTEM_SEQUENCE_CONFIG, 0xE8);
    set_budget_us(budget);
    wr8(SYSTEM_SEQUENCE_CONFIG, 0x01); if (!single_ref_cal(0x40)) return false;
    wr8(SYSTEM_SEQUENCE_CONFIG, 0x02); if (!single_ref_cal(0x00)) return false;
    wr8(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* 连续测距（背靠背，每次约 30ms） */
    wr8(0x80, 0x01); wr8(0xFF, 0x01); wr8(0x00, 0x00);
    wr8(0x91, t->stop_var);
    wr8(0x00, 0x01); wr8(0xFF, 0x00); wr8(0x80, 0x00);
    wr8(SYSRANGE_START, 0x02);
    return !s_err;
}

int tof_init(void)
{
    memset(s_tof, 0, sizeof s_tof);
    s_cur = -1;
    if (!USE_TOF) return 0;
    if (port_i2c_probe(TOF_MUX_ADDR) != 0) { LOG("tof: 没找到多路开关 TCA9548A(0x%02X)，不避障", TOF_MUX_ADDR); return 0; }
    int n = 0;
    for (int i = 0; i < TOF_COUNT; i++) {
        s_err = false;
        sel(i);
        s_tof[i].ok = !s_err && init_one(&s_tof[i]);
        if (s_tof[i].ok) n++;
        else LOG("tof: 第 %d 个传感器（通道 %d）无应答或初始化失败", i, i);
    }
    LOG("tof: %d/%d 个测距传感器就绪", n, TOF_COUNT);
    return n;
}

void tof_poll(void)
{
    /* 每次最多查 3 个，避免一次占 I2C 太久（每个约 0.5ms，有新数据再多约 1ms） */
    for (int k = 0; k < 3; k++) {
        int i = s_rr; s_rr = (s_rr + 1) % TOF_COUNT;
        if (!s_tof[i].ok) continue;
        s_err = false;
        sel(i);
        uint8_t st = rd8(RESULT_INTERRUPT_STATUS);
        if (s_err) { s_tof[i].ok = false; LOG("tof: 第 %d 个传感器掉线", i); continue; }
        if ((st & 0x07) == 0) continue;                              /* 还没测完 */
        uint16_t mm = rd16((uint8_t)(RESULT_RANGE_STATUS + 10));
        wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);
        s_tof[i].mm = mm;
        s_tof[i].t = sys_now();
    }
}

bool tof_ok(int i) { return i >= 0 && i < TOF_COUNT && s_tof[i].ok; }

int tof_get_mm(int i, uint32_t max_age)
{
    if (!tof_ok(i) || s_tof[i].t == 0 || (uint32_t)(sys_now() - s_tof[i].t) > max_age) return 0;
    int mm = s_tof[i].mm;
    if (mm < 20 || mm > TOF_MAX_RANGE_MM) return 0;                  /* 8190/8191 = 没看到东西 */
    return mm;
}

#endif /* !SIM_HAS_TOF */
