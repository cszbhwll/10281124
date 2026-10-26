/**
 * vision.c —— 视觉串口协议解析
 */
#include "vision.h"
#include "port.h"
#include "sys.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static char     s_line[96];
static int      s_len;
static char     s_qr[24];
static uint32_t s_qr_t;
static bool     s_qr_valid;
static vision_tt_t   s_tt;   static bool s_tt_valid;
static vision_ring_t s_ring; static bool s_ring_valid;
static int      s_obs;  static uint32_t s_obs_t;
static uint32_t s_last_rx;
static vision_mode_t s_mode;

void vision_init(void) { vision_clear(); s_len = 0; s_mode = VMODE_IDLE; }

void vision_clear(void) { s_qr_valid = s_tt_valid = s_ring_valid = false; s_obs = 0; }

uint32_t vision_last_rx_ms(void) { return s_last_rx; }

void vision_set_mode(vision_mode_t m)
{
    char b[16];
    int n = snprintf(b, sizeof b, "$MODE,%d\n", (int)m);
    port_uart_write(PORT_UART_VISION, (const uint8_t *)b, (uint16_t)n);
    s_mode = m;
    vision_clear();
}

static int hexv(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* 把 "a,b,c" 拆成字段 */
static int split(char *s, char *f[], int maxf)
{
    int n = 0;
    f[n++] = s;
    for (; *s && n < maxf; s++) if (*s == ',') { *s = 0; f[n++] = s + 1; }
    return n;
}

static void handle_line(char *line)
{
    if (line[0] != '$') return;
    char *star = strchr(line, '*');
    if (star) {                                   /* 有校验就校验 */
        uint8_t x = 0;
        for (char *p = line + 1; p < star; p++) x ^= (uint8_t)*p;
        int h1 = hexv(star[1]), h2 = star[1] ? hexv(star[2]) : -1;
        if (h1 < 0 || h2 < 0 || x != (uint8_t)(h1 * 16 + h2)) return;
        *star = 0;
    }
    char *f[8];
    int n = split(line + 1, f, 8);
    uint32_t now = sys_now();
    s_last_rx = now;

    if (strcmp(f[0], "QR") == 0 && n >= 2) {
        strncpy(s_qr, f[1], sizeof s_qr - 1);
        s_qr[sizeof s_qr - 1] = 0;
        s_qr_t = now; s_qr_valid = true;
    } else if (strcmp(f[0], "TT") == 0 && n >= 5) {
        s_tt.color = atoi(f[1]);
        s_tt.stable = atoi(f[2]) != 0;
        s_tt.dx = (float)atof(f[3]);
        s_tt.dy = (float)atof(f[4]);
        s_tt.stamp = now; s_tt_valid = true;
    } else if (strcmp(f[0], "RING") == 0 && n >= 4) {
        s_ring.found = atoi(f[1]) != 0;
        s_ring.dx = (float)atof(f[2]);
        s_ring.dy = (float)atof(f[3]);
        s_ring.has_dth = (n >= 5);
        s_ring.dth_deg = s_ring.has_dth ? (float)atof(f[4]) : 0.0f;
        s_ring.stamp = now; s_ring_valid = true;
    } else if (strcmp(f[0], "OBS") == 0 && n >= 2) {
        s_obs = atoi(f[1]); s_obs_t = now;
    }
}

void vision_poll(void)
{
    uint8_t c;
    while (port_uart_read_byte(PORT_UART_VISION, &c)) {
        if (c == '\r') continue;
        if (c == '\n') {
            s_line[s_len] = 0;
            if (s_len > 0) handle_line(s_line);
            s_len = 0;
        } else if (s_len < (int)sizeof s_line - 1) {
            s_line[s_len++] = (char)c;
        } else {
            s_len = 0;                           /* 太长，丢掉 */
        }
    }
}

static bool fresh(bool valid, uint32_t stamp, uint32_t max_age)
{
    return valid && (uint32_t)(sys_now() - stamp) <= max_age;
}

bool vision_get_qr(char *out, int maxlen, uint32_t max_age)
{
    if (!fresh(s_qr_valid, s_qr_t, max_age)) return false;
    strncpy(out, s_qr, (size_t)maxlen - 1);
    out[maxlen - 1] = 0;
    return true;
}

bool vision_get_tt(vision_tt_t *out, uint32_t max_age)
{
    if (!fresh(s_tt_valid, s_tt.stamp, max_age)) return false;
    *out = s_tt;
    return true;
}

bool vision_get_ring(vision_ring_t *out, uint32_t max_age)
{
    if (!fresh(s_ring_valid, s_ring.stamp, max_age)) return false;
    *out = s_ring;
    return true;
}

int vision_obstacle_mm(uint32_t max_age)
{
    if ((uint32_t)(sys_now() - s_obs_t) > max_age) return 0;
    return s_obs;
}
