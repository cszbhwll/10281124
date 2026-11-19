/* Timed ground jogs, with one explicit enable per motion. No position claim. */
#include "ground_drive.h"
#include "ground_tuning.h"
#include "motor_drv.h"
#include "port.h"
#include "sys.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

static const int8_t channel[4] = WHEEL_CH_INIT;
static const int8_t forward_sign[4] = WHEEL_SIGN_INIT;
static const unsigned trim[4] = WHEEL_TRIM_PCT_INIT;
static const char *const direction_name[6] = {"fwd", "back", "left", "right", "ccw", "cw"};
/* Original chassis convention: X forward, Y left, CCW positive. FL FR RL RR. */
static const int8_t pattern[6][4] = {{1,1,1,1}, {-1,-1,-1,-1}, {-1,1,1,-1},
    {1,-1,-1,1}, {-1,1,-1,1}, {1,-1,1,-1}};
static bool ready, enabled, locked, running, watching, have_counts;
static unsigned motion, goal, level;
static int8_t out[4];
static int32_t counts[4], previous[4], baseline[4], delta[4];
static uint32_t started, duration, enabled_at, last_step, last_write;
static uint32_t last_sample, sample_at, sample_dt, last_watch, last_retry;
static uint32_t last_pulse[4];
static char line_buf[96];
static unsigned line_len;
static bool discard_line;
static const port_key_t motion_keys[3] = {PORT_KEY_START,PORT_KEY_ZONE,PORT_KEY_WKUP};
static const unsigned motion_dir[3] = {0U,4U,5U};
static const char *const motion_label[3] = {"forward","turn-left","turn-right"};
static bool keys_armed,release_tracking,key_raw[3],key_stable[3];
static uint32_t release_since,key_changed_at[3];

/* Fresh release of BOTH motion buttons is required after boot, stop or fault. */
static void inhibit_keys(void)
{
    keys_armed=false;release_tracking=false;
    uint32_t now=port_millis();
    for (int i=0;i<3;++i) {
        key_raw[i]=key_stable[i]=port_key_down(motion_keys[i]);
        key_changed_at[i]=now;
    }
}

bool ground_drive_running(void) { return running; }
bool ground_drive_faulted(void) { return locked; }
bool ground_drive_enabled(void) { return enabled; }

static int zero_outputs(void)
{
    int8_t z[4] = {0,0,0,0};
    int a = motor_drv_set_pwm(z);
    int b = motor_drv_set_speed(z);
    return a || b ? -1 : 0;
}

static void clear_input(void)
{
    port_uart_flush_rx(PORT_UART_LOG);
    line_len = 0; discard_line = false;
}

static void fault(const char *reason)
{
    running = enabled = false; locked = true; level = 0;
    inhibit_keys();
    memset(out,0,sizeof(out));
    (void)zero_outputs();
    clear_input();
    last_retry = port_millis();
    LOG("LOCKED: %s. Release keys; press RESET or send reset to unlock.",reason);
}

static void stop(const char *reason)
{
    running = enabled = false; level = 0;
    inhibit_keys();
    memset(out,0,sizeof(out));
    clear_input();
    if (zero_outputs()) fault("stop write failed; cut motor power if needed");
    else LOG("STOP: %s; disabled. Next: fresh KEY0/KEY1 press, or enable + move.",reason);
}

static int32_t difference(int32_t a,int32_t b)
{
    uint32_t u = (uint32_t)a-(uint32_t)b;
    return u <= 0x7FFFFFFFU ? (int32_t)u : -1-(int32_t)(0xFFFFFFFFU-u);
}

static bool valid_config(void)
{
    unsigned mask=0;
    for (int i=0;i<4;++i) {
        if (channel[i]<0 || channel[i]>3 || (mask&(1U<<channel[i]))) return false;
        mask |= 1U<<channel[i];
        if (forward_sign[i]!=1 && forward_sign[i]!=-1) return false;
        if (trim[i]<50 || trim[i]>150) return false;
    }
    return true;
}

static bool sample_counts(void)
{
    int32_t fresh[4];
    if (motor_drv_read_counts(fresh)) {
        have_counts=false; fault("encoder I2C read failed"); return false;
    }
    uint32_t now=port_millis();
    sample_dt=have_counts ? now-sample_at : 0;
    for (int i=0;i<4;++i) {
        delta[i]=have_counts ? difference(fresh[i],previous[i]) : 0;
        if (delta[i]!=0) last_pulse[i]=now;
        counts[i]=previous[i]=fresh[i];
    }
    sample_at=now; have_counts=true;
    if (running && (uint32_t)(now-started)>=GROUND_STALL_GRACE_MS) {
        for (int i=0;i<4;++i) {
            if (out[i]!=0 && (uint32_t)(now-last_pulse[i])>=GROUND_STALL_TIMEOUT_MS) {
                LOG("M%d has no encoder changes",i+1);
                fault("wheel stalled or encoder disconnected"); return false;
            }
        }
    }
    return true;
}

static void setup_board(void)
{
    running=enabled=ready=have_counts=false; level=0;
    inhibit_keys();
    memset(out,0,sizeof(out));
    if (!valid_config()) { fault("invalid mapping, signs or wheel trim"); return; }
    if (motor_drv_init()) { fault("motor board init failed"); return; }
    ready=true;
    if (!sample_counts()) return;
    memcpy(baseline,counts,sizeof(baseline));
    if (port_key_down(PORT_KEY_AUX)) { fault("KEY2 pressed"); return; }
    locked=false;
    clear_input();
    LOG("READY GroundButtons. KEY0=fwd KEY1=turn-left WK_UP=turn-right KEY2=stop.");
}

static void status(void)
{
    LOG("GroundButtons ready=%d enabled=%d locked=%d running=%d goal=%u out_level=%u I2C_errors=%lu",
        ready,enabled,locked,running,goal,level,(unsigned long)motor_drv_error_count());
    LOG("output M1..M4: %d %d %d %d",out[0],out[1],out[2],out[3]);
    if (!running) LOG("KEY0/KEY1/WK_UP speed=%d time=%lums release_ready=%d",
        GROUND_BUTTON_SPEED,(unsigned long)GROUND_BUTTON_TIME_MS,keys_armed);
    if (have_counts) LOG("ENC raw=%ld,%ld,%ld,%ld d=%ld,%ld,%ld,%ld dt=%lums age=%lums",
        (long)counts[0],(long)counts[1],(long)counts[2],(long)counts[3],
        (long)delta[0],(long)delta[1],(long)delta[2],(long)delta[3],
        (unsigned long)sample_dt,(unsigned long)(port_millis()-sample_at));
    else LOG("ENC unavailable");
    if (!running) {
        static const char *const wheels[4]={"fl","fr","rl","rr"};
        for (int i=0;i<4;++i) LOG("%s=M%d sign=%d trim=%u%%",wheels[i],channel[i]+1,forward_sign[i],trim[i]);
        if (have_counts) LOG("relative M1..M4: %ld %ld %ld %ld",
            (long)difference(counts[0],baseline[0]),(long)difference(counts[1],baseline[1]),
            (long)difference(counts[2],baseline[2]),(long)difference(counts[3],baseline[3]));
    }
}

static void help(void)
{
    LOG("GroundButtons: help | status | enc | enable | stop/s | reset | zero | watch 0/1");
    LOG("KEY0=fwd KEY1=in-place left WK_UP=in-place right (%d/%lums), KEY2=stop+lock.",
        GROUND_BUTTON_SPEED,(unsigned long)GROUND_BUTTON_TIME_MS);
    LOG("move fwd/back/left/right/ccw/cw [speed] [ms]");
    LOG("Serial: one enable permits ONE move. Buttons auto-enable one timed move.");
    LOG("speed=1..%d pulse/10ms, time=100..%lums; defaults=%d/%lums",
        MOTOR_SPEED_LIMIT,(unsigned long)GROUND_MAX_TIME_MS,GROUND_DEFAULT_SPEED,
        (unsigned long)GROUND_DEFAULT_TIME_MS);
    LOG("! or Ctrl-C / KEY2: software stop + lock. Not a motor power switch.");
}

static bool number(const char *s,long lo,long hi,long *v)
{
    char *end;
    errno=0;
    long x=strtol(s,&end,10);
    if (errno || !*s || *end || x<lo || x>hi) return false;
    *v=x; return true;
}

static bool send_level(void)
{
    if (port_key_down(PORT_KEY_AUX)) { fault("KEY2 software stop"); return false; }
    if ((uint32_t)(port_millis()-started)>=duration) { stop("time complete"); return false; }
    memset(out,0,sizeof(out));
    for (int i=0;i<4;++i) {
        unsigned x=(level*trim[i]+50U)/100U;
        if (level && !x) x=1;
        if (x>MOTOR_SPEED_LIMIT) x=MOTOR_SPEED_LIMIT;
        out[channel[i]]=(int8_t)((int)x*pattern[motion][i]*forward_sign[i]);
    }
    if (motor_drv_set_speed(out)) { fault("speed I2C write failed"); return false; }
    last_write=port_millis();
    return true;
}

static void move(unsigned dir,unsigned speed,uint32_t ms)
{
    if (running) { LOG("BUSY: current timed move unchanged; stop before a new move"); return; }
    if (!ready || locked || !enabled) { LOG("DENIED: reset if locked; enable before each move"); return; }
    if ((uint32_t)(port_millis()-enabled_at)>=GROUND_ENABLE_WINDOW_MS) {
        enabled=false; LOG("DENIED: enable expired"); return;
    }
    if (port_key_down(PORT_KEY_AUX)) { fault("KEY2 pressed"); return; }
    if (!sample_counts()) return;
    if (zero_outputs()) { fault("zero-before-move write failed"); return; }
    motion=dir; goal=speed; duration=ms;
    started=last_step=port_millis();
    for (int i=0;i<4;++i) last_pulse[i]=started;
    level=1; running=true;
    if (send_level()) LOG("RUN %s max_speed=%u time=%lums (ramped, includes ramps)",
        direction_name[dir],speed,(unsigned long)ms);
}

static bool arm_once(void)
{
    if (running) { LOG("BUSY: enable refused while moving"); return false; }
    if (!ready || locked || port_key_down(PORT_KEY_AUX)) {
        LOG("DENIED: release keys and reset if locked"); return false;
    }
    if (!sample_counts()) return false;
    if (zero_outputs()) { fault("enable stop write failed"); return false; }
    enabled=true;enabled_at=port_millis();
    LOG("ENABLED, stopped; one move within %lus",(unsigned long)(GROUND_ENABLE_WINDOW_MS/1000U));
    return true;
}

static void poll_buttons(void)
{
    if (!ready || locked || running || port_key_down(PORT_KEY_AUX)) {
        inhibit_keys();return;
    }
    bool raw[3];
    unsigned down=0;
    for (int i=0;i<3;++i) {
        raw[i]=port_key_down(motion_keys[i]);
        down+=(unsigned)raw[i];
    }
    uint32_t now=port_millis();
    if (!keys_armed) {
        if (down) release_tracking=false;
        else if (!release_tracking) { release_tracking=true;release_since=now; }
        else if ((uint32_t)(now-release_since)>=GROUND_KEY_DEBOUNCE_MS) {
            keys_armed=true;
            for (int i=0;i<3;++i) { key_raw[i]=key_stable[i]=false;key_changed_at[i]=now; }
        }
        return;
    }
    if (down>1) { inhibit_keys();return; }
    int pressed=-1;
    for (int i=0;i<3;++i) {
        if (raw[i]!=key_raw[i]) { key_raw[i]=raw[i];key_changed_at[i]=now; }
        if (key_stable[i]!=raw[i] && (uint32_t)(now-key_changed_at[i])>=GROUND_KEY_DEBOUNCE_MS) {
            key_stable[i]=raw[i];
            if (raw[i]) pressed=i;
        }
    }
    if (pressed>=0) {
        inhibit_keys();
        if (arm_once()) {
            LOG("KEY %s: one %s jog",pressed==0 ? "KEY0" : pressed==1 ? "KEY1" : "WK_UP",
                motion_label[pressed]);
            move(motion_dir[pressed],GROUND_BUTTON_SPEED,GROUND_BUTTON_TIME_MS);
        }
    }
}

static void reject(void)
{
    if (running) fault("invalid command while moving");
    else LOG("ERR command/range. Type help.");
}

void ground_drive_command(char *line)
{
    char *a[6]; int n=0;
    for (char *s=strtok(line," \t");s;s=strtok(NULL," \t")) {
        if (n==6) { reject(); return; }
        a[n++]=s;
    }
    if (!n) return;
    if (!strcmp(a[0],"stop") || !strcmp(a[0],"s")) { stop("command"); return; }
    if (n==1 && !strcmp(a[0],"help")) {
        if (running) LOG("BUSY: stop before reading help"); else help();
        return;
    }
    if (n==1 && (!strcmp(a[0],"status") || !strcmp(a[0],"enc"))) { status(); return; }
    if (n==1 && !strcmp(a[0],"reset")) {
        stop("reset");
        if (zero_outputs()) { fault("reset stop write failed"); return; }
        setup_board(); return;
    }
    if (n==1 && !strcmp(a[0],"enable")) {
        (void)arm_once();return;
    }
    if (n==1 && !strcmp(a[0],"zero")) {
        if (running) { LOG("BUSY: stop before zero"); return; }
        if (!ready || locked) { LOG("DENIED: reset first"); return; }
        if (sample_counts()) { memcpy(baseline,counts,sizeof(baseline)); LOG("relative count baseline reset"); }
        return;
    }
    long v;
    if (n==2 && !strcmp(a[0],"watch") && number(a[1],0,1,&v)) {
        watching=v!=0; LOG("watch=%d",watching); return;
    }
    if (n>=2 && n<=4 && !strcmp(a[0],"move")) {
        long speed=GROUND_DEFAULT_SPEED,ms=GROUND_DEFAULT_TIME_MS;
        if (n>=3 && !number(a[2],1,MOTOR_SPEED_LIMIT,&speed)) { reject(); return; }
        if (n==4 && !number(a[3],100,GROUND_MAX_TIME_MS,&ms)) { reject(); return; }
        unsigned k;
        for (k=0;k<6 && strcmp(a[1],direction_name[k]);++k) {}
        if (k==6) { reject(); return; }
        move(k,(unsigned)speed,(uint32_t)ms); return;
    }
    reject();
}

static void safety(void)
{
    if (port_key_down(PORT_KEY_AUX) && !locked) fault("KEY2 software stop");
    if (port_uart_rx_fault()) fault("UART RX overflow or framing error");
    uint32_t now=port_millis();
    if (running && (uint32_t)(now-started)>=duration) stop("time complete");
    if (enabled && !running && (uint32_t)(now-enabled_at)>=GROUND_ENABLE_WINDOW_MS) {
        enabled=false; LOG("DISABLED: enable window expired");
    }
}

void ground_drive_init(void)
{
    ready=enabled=locked=running=watching=have_counts=false;
    motion=goal=level=0; line_len=0; discard_line=false;
    inhibit_keys();
    memset(counts,0,sizeof(counts)); memset(delta,0,sizeof(delta));
    LOG("GroundButtons F407, UART1 115200, I2C1 PB8/PB9");
    LOG("layout FL=M3 FR=M4 RL=M1 RR=M2, all forward signs=-1");
    setup_board();
    last_sample=last_watch=last_retry=port_millis();
    help();
}

void ground_drive_poll(void)
{
    safety();
    uint32_t now=port_millis();
    if (locked && (uint32_t)(now-last_retry)>=100U) {
        last_retry=now; (void)zero_outputs();
    }
    uint8_t c;
    for (unsigned budget=0;budget<24 && port_uart_read_byte(PORT_UART_LOG,&c);++budget) {
        if (c=='!' || c==3) { fault("serial software stop"); break; }
        if (c=='\r' || c=='\n') {
            if (!discard_line && line_len) { line_buf[line_len]=0; ground_drive_command(line_buf); }
            line_len=0; discard_line=false;
        } else if (discard_line) {
            /* Ignore remainder of this invalid line. */
        } else if (c==8 || c==127) {
            if (line_len) --line_len;
        } else if ((c>=32 && c<127) || c=='\t') {
            if (line_len<sizeof(line_buf)-1) line_buf[line_len++]=(char)c;
            else {
                line_len=0;
                if (running) { fault("serial command too long"); break; }
                discard_line=true; LOG("ERR line too long; discarded");
            }
        } else if (c!='\t') {
            if (running) { fault("invalid serial byte"); break; }
            line_len=0; discard_line=true;
        }
    }
    safety();
    poll_buttons();
    safety();
    now=port_millis();
    if (ready && !locked && (uint32_t)(now-last_sample)>=GROUND_ENCODER_MS) {
        last_sample=now; (void)sample_counts();
    }
    safety();
    now=port_millis();
    if (running && (uint32_t)(now-last_step)>=GROUND_SLEW_STEP_MS) {
        /* Do not jump multiple steps after a delayed poll. */
        last_step=now;
        uint32_t remaining=duration-(uint32_t)(now-started);
        unsigned braking=(unsigned)((remaining+GROUND_SLEW_STEP_MS-1U)/GROUND_SLEW_STEP_MS);
        unsigned wanted=goal<braking ? goal : braking;
        if (level<wanted) ++level;
        else if (level>wanted) --level;
    }
    if (running && (uint32_t)(now-last_write)>=GROUND_REFRESH_MS) (void)send_level();
    if (watching && (uint32_t)(now-last_watch)>=GROUND_WATCH_MS) {
        last_watch=now;
        if (have_counts && !locked) LOG("ENC %ld,%ld,%ld,%ld d=%ld,%ld,%ld,%ld dt=%lums level=%u",
            (long)counts[0],(long)counts[1],(long)counts[2],(long)counts[3],
            (long)delta[0],(long)delta[1],(long)delta[2],(long)delta[3],(unsigned long)sample_dt,level);
        else LOG("ENC unavailable or locked");
    }
    safety();
    port_led(0,locked);
    bool green=false;
    if (ready && !locked) {
        if (running) green=(port_millis()/120U)%2U!=0;
        else if (enabled) green=true;
        else green=(port_millis()/500U)%2U!=0;
    }
    port_led(1,green);
}
