/* Real ground_drive.c + motor_drv.c with mock hardware, independent channel expectations. */
#include "ground_drive.h"
#include "ground_tuning.h"
#include "motor_drv.h"
#include "port.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static bool key0,key1,key2,wakeup,rx_error,bad_probe,bad_read,bad_write;
static bool led[2];
static bool encoder_enabled[4];
static uint32_t encoder[4];
static int8_t output[4];
static unsigned writes,nonzero_writes;
static char rx[4096],logs[60000];
static unsigned rx_head,rx_tail,log_len;
typedef struct { uint32_t t; int8_t out[4]; } record_t;
static record_t record[10000];
static unsigned records;

void port_init(void) {}
uint32_t port_millis(void) { return tick; }
void port_idle(void) { ++tick; }
bool port_key_down(port_key_t k)
{
    if (k==PORT_KEY_START) return key0;
    if (k==PORT_KEY_ZONE) return key1;
    if (k==PORT_KEY_WKUP) return wakeup;
    return k==PORT_KEY_AUX && key2;
}
void port_led(int n,bool x) { assert(n==0 || n==1);led[n]=x; }
void port_beep(bool x) { (void)x; }
int port_i2c_probe(uint8_t a) { assert(a==0x34); return bad_probe ? -1 : 0; }
int port_i2c_write(uint8_t a,uint8_t reg,const uint8_t *d,uint16_t n)
{
    assert(a==0x34); ++writes;
    if (bad_write) return -1;
    if (reg==MDRV_REG_SPEED || reg==MDRV_REG_PWM) {
        assert(n==4); memcpy(output,d,4);
        for (int i=0;i<4;++i) {
            assert(output[i]>=-MOTOR_SPEED_LIMIT && output[i]<=MOTOR_SPEED_LIMIT);
            if (reg==MDRV_REG_PWM) assert(output[i]==0); /* Ground firmware never requests open-loop motion. */
        }
        if (output[0] || output[1] || output[2] || output[3]) ++nonzero_writes;
        if (reg==MDRV_REG_SPEED) {
            assert(records<sizeof(record)/sizeof(*record));
            record[records].t=tick;
            memcpy(record[records++].out,output,4);
        }
    } else if (reg==MDRV_REG_TYPE) assert(n==1 && d[0]==3);
    else if (reg==MDRV_REG_POLARITY) assert(n==1 && d[0]==0);
    else assert(0);
    return 0;
}
int port_i2c_read(uint8_t a,uint8_t reg,uint8_t *d,uint16_t n)
{
    assert(a==0x34);
    if (bad_read) return -1;
    if (reg==MDRV_REG_ENC_TOTAL) {
        assert(n==16);
        for (int i=0;i<4;++i) for (int j=0;j<4;++j) d[4*i+j]=(uint8_t)(encoder[i]>>(8*j));
    } else { assert(reg==MDRV_REG_BATTERY && n==2); d[0]=0xE0;d[1]=0x2E; }
    return 0;
}
void port_uart_write(port_uart_t ch,const uint8_t *d,uint16_t n)
{
    assert(ch==PORT_UART_LOG);
    if (log_len+n<sizeof(logs)) { memcpy(logs+log_len,d,n); log_len+=n;logs[log_len]=0; }
}
int port_uart_read_byte(port_uart_t ch,uint8_t *b)
{
    assert(ch==PORT_UART_LOG);
    if (rx_tail==rx_head) return 0;
    *b=(uint8_t)rx[rx_tail++];return 1;
}
void port_uart_flush_rx(port_uart_t ch) { assert(ch==PORT_UART_LOG);rx_tail=rx_head; }
bool port_uart_rx_fault(void) { bool x=rx_error;rx_error=false;return x; }
static void cmd(const char *s)
{
    char b[300];assert(strlen(s)<sizeof(b));strcpy(b,s);ground_drive_command(b);
}
static void feed(const char *s)
{
    unsigned n=(unsigned)strlen(s);
    assert(rx_head+n<sizeof(rx));memcpy(rx+rx_head,s,n);rx_head+=n;
}
static void drain(void)
{
    while (rx_tail<rx_head) ground_drive_poll();
}
static void expect(int a,int b,int c,int d)
{
    assert(output[0]==a && output[1]==b && output[2]==c && output[3]==d);
}
static void advance(unsigned ms)
{
    for (unsigned t=0;t<ms;++t) {
        ++tick;
        if (tick%10U==0) for (int i=0;i<4;++i)
            if (encoder_enabled[i]) encoder[i]+=(uint32_t)(int32_t)output[i];
        ground_drive_poll();
    }
}
static void fresh(void)
{
    tick=0;key0=key1=key2=wakeup=rx_error=bad_probe=bad_read=bad_write=false;
    writes=nonzero_writes=records=rx_head=rx_tail=log_len=0;logs[0]=0;
    memset(encoder,0,sizeof(encoder));memset(output,0,sizeof(output));
    for (int i=0;i<4;++i) encoder_enabled[i]=true;
    ground_drive_init();
    assert(!ground_drive_faulted() && !ground_drive_running() && !ground_drive_enabled());
    assert(nonzero_writes==0);expect(0,0,0,0);
}

static void release_buttons(void)
{
    key0=key1=false;advance(GROUND_KEY_DEBOUNCE_MS+2U);
}
static void press_button(unsigned n)
{
    if (n==0) key0=true;else key1=true;
    advance(GROUND_KEY_DEBOUNCE_MS+2U);
}

static void wait_stopped(void)
{
    for (unsigned t=0;t<5000U && ground_drive_running();++t) advance(1);
    assert(!ground_drive_running() && !ground_drive_faulted());
    expect(0,0,0,0);
}

static void test_distance(void)
{
    fresh();cmd("cal fwd 100");assert(!nonzero_writes);
    cmd("enable");cmd("cal fwd 29");assert(!nonzero_writes);
    cmd("cal fwd 100");expect(-2,-2,-2,-2);assert(ground_drive_running());
    wait_stopped();
    assert(encoder[0]>=(uint32_t)-120 && encoder[0]<=(uint32_t)-100);
    assert(strstr(logs,"DIST DONE fwd target=100 counts="));
    assert(strstr(logs,"STOP: calibration complete"));

    fresh();cmd("enable");cmd("cal left 100");expect(-2,2,2,-2);
    wait_stopped();assert(encoder[1]>=100 && encoder[1]<=120);
    assert(strstr(logs,"DIST DONE left target=100 counts="));

    fresh();cmd("enable");cmd("cal fwd 100");
    for (unsigned i=0;i<4;++i) encoder_enabled[i]=false;
    for (unsigned t=0;t<120 && !ground_drive_faulted();++t) {
        for (unsigned i=0;i<4;++i) encoder[i]=(t%2U==0) ? (uint32_t)-3 : 0;
        advance(GROUND_ENCODER_MS);
    }
    assert(ground_drive_faulted() && strstr(logs,"distance segment timed out"));
    expect(0,0,0,0);

    fresh();cmd("enable");cmd("cal fwd 100");
    encoder_enabled[0]=false;encoder[0]=20;
    advance(GROUND_ENCODER_MS);
    assert(ground_drive_faulted() && strstr(logs,"encoder moved opposite"));
    expect(0,0,0,0);

    fresh();cmd("enable");cmd("cal fwd 100");
    encoder_enabled[0]=false;
    advance(600);
    assert(ground_drive_faulted());
    expect(0,0,0,0);
    puts("PASS forward/left calibration, signed counts, timeout, wrong-way feedback and single-wheel stall");
}

static void test_bench(void)
{
    const char *second_log=BENCH_TURN_IN_PLACE ? "RUN BENCH ccw" : "RUN BENCH left";
    const int second_sign=BENCH_TURN_IN_PLACE ? 1 : -1;
    fresh();release_buttons();
    wakeup=true;ground_drive_init();advance(100);
    assert(!ground_drive_running() && !nonzero_writes);
    wakeup=false;release_buttons();cmd("route");assert(!nonzero_writes);
    wakeup=true;advance(GROUND_KEY_DEBOUNCE_MS+2U);
    assert(ground_drive_running());expect(-2,-2,-2,-2);
    uint32_t start=record[records-1].t;
    advance(BENCH_FORWARD_MS-1U-(tick-start));
    assert(output[0]<0 && output[1]<0);
    advance(1);expect(0,0,0,0);
    assert(tick-start==BENCH_FORWARD_MS && ground_drive_running());

    advance(ROUTE_SETTLE_MIN_MS-1U);expect(0,0,0,0);
    advance(1);expect(second_sign*2,-second_sign*2,2,-2);
    char expected_second[80];
    snprintf(expected_second,sizeof(expected_second),"%s time=3000ms",second_log);
    assert(strstr(logs,"RUN BENCH fwd time=5000ms") && strstr(logs,expected_second));
    advance(BENCH_LEFT_MS-1U);
    assert(output[0]*second_sign>0 && output[1]*second_sign<0);
    advance(1);expect(0,0,0,0);
    assert(!ground_drive_running() && !ground_drive_faulted());
    assert(strstr(logs,"STOP: bench complete"));
    unsigned before=nonzero_writes;advance(500);assert(before==nonzero_writes);

    fresh();cmd("enable");cmd("route");advance(BENCH_FORWARD_MS);
    assert(ground_drive_running());expect(0,0,0,0);
    key2=true;advance(1);assert(ground_drive_faulted());
    key2=false;advance(ROUTE_SETTLE_LIMIT_MS);assert(!strstr(logs,second_log));
    expect(0,0,0,0);

    fresh();cmd("enable");cmd("route");advance(BENCH_FORWARD_MS+ROUTE_SETTLE_MIN_MS);
    assert(strstr(logs,second_log));
    key2=true;advance(1);assert(ground_drive_faulted());expect(0,0,0,0);

    fresh();cmd("enable");cmd("route");advance(BENCH_FORWARD_MS);
    for (unsigned t=0;t<40 && !ground_drive_faulted();++t) {
        --encoder[0];
        advance(GROUND_ENCODER_MS);
    }
    assert(ground_drive_faulted() && strstr(logs,"did not settle"));
    assert(!strstr(logs,second_log));expect(0,0,0,0);

    fresh();cmd("enable");cmd("route");advance(120);bad_read=true;advance(GROUND_ENCODER_MS);
    assert(ground_drive_faulted() && !strstr(logs,second_log));expect(0,0,0,0);
    fresh();cmd("enable");cmd("route");
    advance(BENCH_FORWARD_MS-1U);bad_write=true;advance(1);
    assert(ground_drive_faulted() && strstr(logs,"STOP NOT CONFIRMED"));
    bad_write=false;advance(ROUTE_SETTLE_LIMIT_MS);expect(0,0,0,0);
    assert(!strstr(logs,second_log));

    fresh();bad_probe=true;ground_drive_init();release_buttons();
    wakeup=true;advance(100);cmd("enable");cmd("route");
    assert(ground_drive_faulted() && !nonzero_writes && strstr(logs,"I2C 0x34 no ACK"));
    bad_probe=false;cmd("reset");advance(100);
    assert(!ground_drive_faulted() && !ground_drive_running() && !nonzero_writes);

    fresh();cmd("enable");cmd("route");advance(100);
    ground_drive_init();advance(BENCH_FORWARD_MS+ROUTE_SETTLE_LIMIT_MS);
    assert(!ground_drive_running() && !strstr(logs,second_log));expect(0,0,0,0);
    puts("PASS WK_UP 5s forward, 500ms settle, 3s second leg, no repeat; KEY2, reset, and I2C faults cancel");
}

int main(void)
{
    fresh();cmd("move fwd 3 400");assert(!ground_drive_running());assert(!nonzero_writes);
    cmd("enable");assert(ground_drive_enabled());expect(0,0,0,0);
    cmd("move fwd 3 400");expect(-1,-1,-1,-1);
    advance(80);expect(-2,-2,-2,-2);
    advance(80);expect(-3,-3,-3,-3);
    advance(80);expect(-2,-2,-2,-2);
    advance(80);expect(-1,-1,-1,-1);
    advance(79);assert(ground_drive_running());
    advance(1);expect(0,0,0,0);assert(!ground_drive_running() && !ground_drive_enabled());
    unsigned before=nonzero_writes;cmd("move fwd 3 400");assert(nonzero_writes==before);
    puts("PASS boot stopped, one-enable/one-motion, finite deadline, ramp up/down, auto-disable");

    fresh();
    const char *dirs[]={"fwd","back","left","right","ccw","cw"};
    const int expected[6][4]={{-1,-1,-1,-1},{1,1,1,1},{-1,1,1,-1},
        {1,-1,-1,1},{1,-1,1,-1},{-1,1,-1,1}};
    for (int i=0;i<6;++i) {
        char b[80];cmd("enable");snprintf(b,sizeof(b),"move %s 1 300",dirs[i]);cmd(b);
        expect(expected[i][0],expected[i][1],expected[i][2],expected[i][3]);
        advance(300);expect(0,0,0,0);assert(!ground_drive_faulted());
    }
    cmd("status");assert(strstr(logs,"fl=M3 sign=-1") && strstr(logs,"fr=M4 sign=-1"));
    assert(strstr(logs,"rl=M1 sign=-1") && strstr(logs,"rr=M2 sign=-1"));
    puts("PASS six direction outputs against confirmed FL=M3 FR=M4 RL=M1 RR=M2 layout");

    fresh();cmd("enable");cmd("move fwd 3 600");advance(100);
    cmd("move back 6 1500");cmd("enable");assert(ground_drive_running());
    advance(500);assert(!ground_drive_running());assert(!ground_drive_enabled());expect(0,0,0,0);
    cmd("enable");advance(GROUND_ENABLE_WINDOW_MS);assert(!ground_drive_enabled());
    cmd("move fwd 1 100");assert(!ground_drive_running());
    cmd("enable");cmd("move fwd");advance(GROUND_DEFAULT_TIME_MS);expect(0,0,0,0);
    puts("PASS busy command cannot restart/extend timer, enable lease expires, defaults");

    fresh();
    const char *bad[]={"move fwd 0 400","move fwd 7 400","move fwd -1 400","move fwd 3 99",
        "move fwd 3 1501","move fwd 3 99999999999999999999999","move fwd 3x 400", "move fwd 3.5 400",
        "move nowhere 3 400","move fwd 1 100 extra","move fwd 1 100 a b c", "move",
        "pwm 1 15 500","ch 1 3 500","wheel fl 3 500","wheels 3 3 3 3 500"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(*bad);++i) {
        before=nonzero_writes;cmd(bad[i]);assert(nonzero_writes==before && !ground_drive_running());
    }
    cmd("enable");cmd("move fwd 1 100");advance(100);assert(!ground_drive_faulted());
    cmd("enable");cmd("move fwd 6 1500");advance(1500);assert(!ground_drive_faulted());expect(0,0,0,0);
    cmd("enable");cmd("move fwd 3 400");cmd("wrong");assert(ground_drive_faulted());expect(0,0,0,0);
    puts("PASS strict ranges and parsing, no raw/wheel commands, bad command during motion stops");

    fresh();cmd("enable");cmd("move fwd");feed("!");ground_drive_poll();
    assert(ground_drive_faulted());expect(0,0,0,0);
    cmd("reset");assert(!ground_drive_enabled() && !ground_drive_faulted());
    cmd("enable");cmd("move fwd");key2=true;ground_drive_poll();expect(0,0,0,0);
    assert(ground_drive_faulted());cmd("reset");assert(ground_drive_faulted());key2=false;cmd("reset");
    cmd("enable");cmd("move fwd");feed("\3");ground_drive_poll();assert(ground_drive_faulted());expect(0,0,0,0);
    cmd("reset");cmd("enable");cmd("move fwd");cmd("stop");assert(!ground_drive_enabled());expect(0,0,0,0);
    puts("PASS !, Ctrl-C, KEY2, stop, reset never resumes a previous command");

    fresh();feed("enable\r\nmove\tfwd\t1\t100\r\n");drain();assert(ground_drive_running());
    advance(100);assert(!ground_drive_running());expect(0,0,0,0);
    cmd("enable");cmd("move fwd 3 400");rx_error=true;ground_drive_poll();
    assert(ground_drive_faulted());expect(0,0,0,0);
    cmd("reset");cmd("enable");cmd("move fwd");
    char longline[200];memset(longline,'x',sizeof(longline));longline[199]=0;feed(longline);drain();
    assert(ground_drive_faulted());expect(0,0,0,0);
    cmd("reset");feed(longline);feed("\nenable\nmove fwd 1 100\n");drain();
    assert(ground_drive_running());advance(100);assert(!ground_drive_faulted());
    puts("PASS CRLF/tabs, UART overflow/error lock, long-line discard/recovery");

    fresh();cmd("enable");cmd("move fwd 3 1000");encoder_enabled[2]=false;
    advance(GROUND_STALL_TIMEOUT_MS+GROUND_ENCODER_MS);
    assert(ground_drive_faulted() && !ground_drive_running());expect(0,0,0,0);
    assert(strstr(logs,"M3 has no encoder changes")!=NULL);
    puts("PASS one stationary wheel detected while other three encoder channels move");

    fresh();cmd("enable");cmd("move fwd 3 1000");bad_read=true;advance(GROUND_ENCODER_MS);
    assert(ground_drive_faulted());expect(0,0,0,0);bad_read=false;cmd("reset");
    cmd("enable");cmd("move fwd");bad_write=true;advance(GROUND_REFRESH_MS);
    assert(ground_drive_faulted());before=writes;advance(100);assert(writes>before);
    bad_write=false;advance(100);expect(0,0,0,0);assert(ground_drive_faulted());
    cmd("reset");assert(!ground_drive_enabled() && !ground_drive_running());
    bad_probe=true;cmd("reset");assert(ground_drive_faulted());
    puts("PASS I2C read/write/init errors, zero retries, reconnection never auto-resumes");

    fresh();tick=0xFFFFFF00U;cmd("enable");cmd("move fwd 3 1000");advance(999);
    assert(ground_drive_running());advance(1);assert(!ground_drive_running());expect(0,0,0,0);
    encoder[0]=0x7FFFFFFFU;advance(GROUND_ENCODER_MS);encoder[0]=0x80000001U;advance(GROUND_ENCODER_MS);
    log_len=0;logs[0]=0;cmd("enc");assert(strstr(logs,"d=2,")!=NULL);
    cmd("zero");cmd("status");assert(strstr(logs,"relative M1..M4: 0 0 0 0")!=NULL);
    puts("PASS time wrap, signed encoder wrap, relative-zero baseline");

    fresh();cmd("enable");cmd("move fwd 3 400");
    /* Lost host traffic while the MCU remains powered cannot extend the finite jog. */
    advance(400);assert(!ground_drive_running());expect(0,0,0,0);
    puts("PASS finite move stops without further serial input (MCU power retained)");

    fresh();release_buttons();press_button(0);
    assert(ground_drive_running());expect(-1,-1,-1,-1);
    advance(GROUND_BUTTON_TIME_MS);assert(!ground_drive_running() && !ground_drive_enabled());
    before=nonzero_writes;advance(1200);assert(nonzero_writes==before);expect(0,0,0,0);
    release_buttons();press_button(1);assert(ground_drive_running());expect(1,1,1,1);
    advance(GROUND_BUTTON_TIME_MS);expect(0,0,0,0);
    puts("PASS KEY0 forward, KEY1 backward, 400ms stop, no repeat while held, release-to-rearm");

    fresh();key0=true;ground_drive_init();advance(1000);
    assert(nonzero_writes==0 && !ground_drive_running());
    release_buttons();press_button(0);assert(ground_drive_running());cmd("stop");
    before=nonzero_writes;advance(800);assert(nonzero_writes==before && !ground_drive_running());
    release_buttons();press_button(1);assert(ground_drive_running());cmd("stop");
    puts("PASS held-on-boot does not start, software stop suppresses held motion key");

    fresh();release_buttons();
    key0=true;advance(10);key0=false;advance(10);key0=true;advance(10);key0=false;advance(10);
    assert(!ground_drive_running() && nonzero_writes==0);
    key0=key1=true;advance(100);assert(!ground_drive_running() && nonzero_writes==0);
    key1=false;advance(100);assert(!ground_drive_running());
    release_buttons();press_button(0);assert(ground_drive_running());
    key0=false;key1=true;advance(GROUND_BUTTON_TIME_MS+500U);
    assert(!ground_drive_running());before=nonzero_writes;advance(500);assert(before==nonzero_writes);
    release_buttons();press_button(1);assert(ground_drive_running());cmd("stop");
    puts("PASS bounce rejection, simultaneous opposite keys ignored, busy-time press never queued");

    fresh();release_buttons();press_button(0);key2=true;advance(1);
    assert(ground_drive_faulted());expect(0,0,0,0);assert(led[0] && !led[1]);
    key2=false;advance(100);assert(ground_drive_faulted());
    cmd("reset");advance(500);assert(!ground_drive_faulted() && !ground_drive_running());
    expect(0,0,0,0);release_buttons();press_button(1);assert(ground_drive_running());cmd("stop");
    puts("PASS KEY2 immediate lock, releasing stop key alone never unlocks, reset with held key never starts");

    fresh();tick=0xFFFFFFF0U;release_buttons();press_button(0);
    assert(ground_drive_running());advance(GROUND_BUTTON_TIME_MS);assert(!ground_drive_running());
    release_buttons();advance(600);bool g=led[1];advance(500);assert(led[1]!=g && !led[0]);
    puts("PASS button debounce across tick wrap, ready slow blink, no PC commands needed for button motion");
    puts("ALL GROUND REGRESSIONS PASSED (mock hardware, no floor-distance claim)");
    puts("ALL BUTTON REGRESSIONS PASSED (mock KEY0/KEY1/KEY2)");
    test_distance();
    test_bench();
    puts("ALL BENCH REGRESSIONS PASSED (mock encoders)");
    return 0;
}
