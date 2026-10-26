/**
 * bus_servo.c —— 飞特总线舵机通信（ST3215 / SC09）
 */
#include "bus_servo.h"
#include "sys.h"
#include <string.h>

#define INST_PING        0x01
#define INST_READ        0x02
#define INST_WRITE       0x03
#define INST_SYNC_WRITE  0x83
#define BROADCAST_ID     0xFE
#define REPLY_TIMEOUT_MS 4

static uint8_t s_last_tx[64];
static uint16_t s_last_tx_len;

static void send_packet(port_uart_t uart, uint8_t id, uint8_t inst, const uint8_t *prm, uint8_t n)
{
    uint8_t b[64];
    uint8_t len = (uint8_t)(n + 2);
    uint8_t sum = (uint8_t)(id + len + inst);
    b[0] = 0xFF; b[1] = 0xFF; b[2] = id; b[3] = len; b[4] = inst;
    for (uint8_t i = 0; i < n; i++) { b[5 + i] = prm[i]; sum = (uint8_t)(sum + prm[i]); }
    b[5 + n] = (uint8_t)~sum;
    port_uart_flush_rx(uart);
    port_uart_write(uart, b, (uint16_t)(n + 6));
    memcpy(s_last_tx, b, (size_t)(n + 6));
    s_last_tx_len = (uint16_t)(n + 6);
}

/*
 * 等待舵机回复：FF FF ID LEN ERR 参数... 校验
 * 有些舵机驱动板会把我们发出去的数据也"回声"回来，所以收到和刚发出的包一模一样的数据要跳过。
 */
static int read_status(port_uart_t uart, uint8_t id, uint8_t *prm, uint8_t nprm)
{
    uint8_t f[64];
    int fl = 0;
    uint32_t t0 = sys_now();
    while ((uint32_t)(sys_now() - t0) <= REPLY_TIMEOUT_MS) {
        uint8_t c;
        if (!port_uart_read_byte(uart, &c)) { sys_poll(); continue; }
        if (fl < 2) { if (c == 0xFF) f[fl++] = c; else fl = 0; continue; }
        if (fl == 2 && c == 0xFF) continue;                 /* 多余的 FF */
        f[fl++] = c;
        if (fl >= 4 && fl == 4 + f[3]) {                    /* 收齐一个包 */
            if (fl == s_last_tx_len && memcmp(f, s_last_tx, (size_t)fl) == 0) { fl = 0; continue; }   /* 回声 */
            uint8_t sum = 0;
            for (int i = 2; i < fl - 1; i++) sum = (uint8_t)(sum + f[i]);
            if ((uint8_t)~sum != f[fl - 1] || f[2] != id) { fl = 0; continue; }
            if (f[3] < (uint8_t)(nprm + 2)) return -3;
            if (prm && nprm) memcpy(prm, &f[5], nprm);
            return f[4];                                    /* 舵机错误码，0 表示正常 */
        }
        if (fl >= (int)sizeof f) fl = 0;
    }
    return -1;   /* 超时 */
}

int servo_ping(const servo_t *s)
{
    send_packet(s->uart, s->id, INST_PING, NULL, 0);
    return read_status(s->uart, s->id, NULL, 0) >= 0 ? 0 : -1;
}

static int write_bytes(const servo_t *s, uint8_t reg, const uint8_t *d, uint8_t n)
{
    uint8_t p[16];
    p[0] = reg;
    memcpy(&p[1], d, n);
    send_packet(s->uart, s->id, INST_WRITE, p, (uint8_t)(n + 1));
    /* 等舵机的应答包收完再返回：不等的话，紧接着发的下一条命令会和舵机的应答在单线总线上撞车，
       舵机就收不到下一条命令（实测：先上力再发位置，位置命令偶尔被丢掉） */
    if (s->id == BROADCAST_ID) return 0;
    return read_status(s->uart, s->id, NULL, 0) >= 0 ? 0 : -1;
}

int servo_torque(const servo_t *s, bool on)
{
    uint8_t v = on ? 1 : 0;
    return write_bytes(s, SREG_TORQUE, &v, 1);
}

static void put16(uint8_t *d, int v, servo_proto_t proto)
{
    uint16_t u = (uint16_t)v;
    if (proto == SERVO_STS) { d[0] = (uint8_t)(u & 0xFF); d[1] = (uint8_t)(u >> 8); }
    else                    { d[0] = (uint8_t)(u >> 8);   d[1] = (uint8_t)(u & 0xFF); }
}

static int get16(const uint8_t *d, servo_proto_t proto)
{
    if (proto == SERVO_STS) {
        int v = d[0] | (d[1] << 8);
        if (v & 0x8000) v = -(v & 0x7FFF);    /* STS 用最高位表示负数 */
        return v;
    }
    return (d[0] << 8) | d[1];
}

int servo_write_pos(const servo_t *s, int pos, int speed, int acc)
{
    uint8_t d[7];
    if (s->proto == SERVO_STS) {
        if (pos < 0) pos = 0;
        if (pos > 4095) pos = 4095;
        d[0] = (uint8_t)acc;
        put16(&d[1], pos, SERVO_STS);
        put16(&d[3], 0, SERVO_STS);
        put16(&d[5], speed, SERVO_STS);
        return write_bytes(s, SREG_ACC, d, 7);
    } else {
        if (pos < 0) pos = 0;
        if (pos > 1023) pos = 1023;
        put16(&d[0], pos, SERVO_SCS);
        put16(&d[2], 0, SERVO_SCS);
        put16(&d[4], speed, SERVO_SCS);
        return write_bytes(s, SREG_GOAL_POS, d, 6);
    }
}

int servo_read_pos(const servo_t *s, int *pos)
{
    uint8_t p[2] = { SREG_PRESENT_POS, 2 }, r[2];
    send_packet(s->uart, s->id, INST_READ, p, 2);
    int e = read_status(s->uart, s->id, r, 2);
    if (e < 0) return -1;
    *pos = get16(r, s->proto);
    return 0;
}

int servo_read_byte(const servo_t *s, uint8_t reg, uint8_t *val)
{
    uint8_t p[2] = { reg, 1 };
    send_packet(s->uart, s->id, INST_READ, p, 2);
    return read_status(s->uart, s->id, val, 1) < 0 ? -1 : 0;
}

int servo_set_id(const servo_t *s, uint8_t new_id)
{
    uint8_t lock_reg = (s->proto == SERVO_STS) ? 55 : 48;
    uint8_t zero = 0, one = 1;
    write_bytes(s, lock_reg, &zero, 1);           /* 解锁 EEPROM */
    sys_wait_ms(5);
    write_bytes(s, SREG_ID, &new_id, 1);
    sys_wait_ms(5);
    servo_t n = *s;
    n.id = new_id;
    write_bytes(&n, lock_reg, &one, 1);           /* 重新上锁，掉电保存 */
    sys_wait_ms(5);
    return servo_ping(&n);
}

/* 中位校准：飞特 STS 系列向扭矩开关寄存器(40)写 128 = “任意当前位置校正为 2048”。
 * 校正量存在舵机的 EEPROM 里，所以前后解锁/上锁 EEPROM（多解锁一次没有坏处）。 */
int servo_define_middle(const servo_t *s)
{
    if (s->proto != SERVO_STS) return -1;
    uint8_t zero = 0, one = 1, cal = 128;
    write_bytes(s, 55, &zero, 1);
    sys_wait_ms(5);
    write_bytes(s, SREG_TORQUE, &cal, 1);
    sys_wait_ms(50);
    write_bytes(s, 55, &one, 1);
    sys_wait_ms(5);
    return servo_ping(s);
}

int servo_sync_write_pos(port_uart_t uart, const uint8_t *ids, const int *pos,
                         const int *speed, int acc, int n)
{
    uint8_t p[2 + 4 * 8];
    if (n > 4) n = 4;
    p[0] = SREG_ACC;
    p[1] = 7;
    int k = 2;
    for (int i = 0; i < n; i++) {
        int v = pos[i] < 0 ? 0 : (pos[i] > 4095 ? 4095 : pos[i]);
        p[k++] = ids[i];
        p[k++] = (uint8_t)acc;
        put16(&p[k], v, SERVO_STS);        k += 2;
        put16(&p[k], 0, SERVO_STS);        k += 2;
        put16(&p[k], speed[i], SERVO_STS); k += 2;
    }
    send_packet(uart, BROADCAST_ID, INST_SYNC_WRITE, p, (uint8_t)k);
    return 0;
}
