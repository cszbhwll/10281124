/* port_sim.c —— 硬件抽象层的"模拟器版本"：把对硬件的操作转给 world.c */
#include "port.h"
#include "sim.h"

static bool s_key[PORT_KEY_COUNT];

void port_init(void) {}
uint32_t port_millis(void) { return sim_time_ms(); }
void port_idle(void) { sim_step_1ms(); }

int port_i2c_write(uint8_t a, uint8_t r, const uint8_t *d, uint16_t n) { sim_i2c_write(a, r, d, n); return a == 0x34 ? 0 : -1; }
int port_i2c_read(uint8_t a, uint8_t r, uint8_t *d, uint16_t n) { return sim_i2c_read(a, r, d, n); }
int port_i2c_probe(uint8_t a) { return a == 0x34 ? 0 : -1; }

void port_uart_write(port_uart_t ch, const uint8_t *d, uint16_t n) { sim_uart_tx((int)ch, d, n); }
int port_uart_read_byte(port_uart_t ch, uint8_t *o) { return sim_uart_rx((int)ch, o); }
void port_uart_flush_rx(port_uart_t ch) { uint8_t c; while (sim_uart_rx((int)ch, &c)) {} }

bool port_key_down(port_key_t k) { return s_key[k]; }
void port_led(int i, bool on) { (void)i; (void)on; }
void port_beep(bool on) { (void)on; }
