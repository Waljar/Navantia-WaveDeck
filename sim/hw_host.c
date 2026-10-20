/**
 * hw_host.c - plat_hw.h on the PC, with a virtual clock and a model of the
 * three legs (a STEP edge moves the leg by one step in the DIR direction).
 * Used by the test programs; never linked into the firmware.
 */
#include "plat_hw.h"
#include "plat_config.h"
#include "hw_host.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#define CPU_MHZ 170u

static uint64_t vclk = 0;                       /* virtual CPU cycles */
uint32_t sim_cycles_per_call = 85;              /* time that passes per hw_cycles() call (0.5 us) */

static int step_level[NUM_LEGS];
static int dir_level[NUM_LEGS];
long sim_real[NUM_LEGS];                        /* actual leg position, steps from the switch */
long sim_steps_emitted[NUM_LEGS];
long sim_pulses_bad[NUM_LEGS];                  /* pulses narrower than 2.5 us */
static uint64_t pulse_start[NUM_LEGS];
double sim_last_pulse_us[NUM_LEGS];
static const int HOME_LEVEL[NUM_LEGS] = { HOME_DIR_LEG0, HOME_DIR_LEG1, HOME_DIR_LEG2 };

/* time series of every pulse for the speed / accel checks */
SimPulse *sim_log = 0;
long sim_log_n = 0, sim_log_cap = 0;
int sim_log_on = 0;

long sim_seen_min[NUM_LEGS], sim_seen_max[NUM_LEGS];
long sim_spread_max = 0;                         /* largest (highest leg - lowest leg) since sim_track_reset() */
static long spread_now(void)
{
    long lo = sim_real[0], hi = sim_real[0];
    for (int i = 1; i < NUM_LEGS; i++) { if (sim_real[i] < lo) lo = sim_real[i]; if (sim_real[i] > hi) hi = sim_real[i]; }
    return hi - lo;
}
void sim_track_reset(void) { for (int i = 0; i < NUM_LEGS; i++) { sim_seen_min[i] = sim_seen_max[i] = sim_real[i]; } sim_spread_max = spread_now(); }

/* scripted keys */
typedef struct { double t_ms; int key; } Key;
static Key keys[8192];
static int nkeys = 0, keyi = 0;
int sim_echo = 1;                                /* print console output */
char sim_out[1 << 16];
size_t sim_out_n = 0;
int sim_exit_on_keys_done = 1;
double sim_end_ms = 1e12;

jmp_buf sim_exit_jmp; int sim_exit_jmp_set = 0;
static void sim_finish(void) { if (sim_exit_jmp_set) longjmp(sim_exit_jmp, 1); exit(0); }

/* output-driven script state */
static const SimStep *steps_p = 0;
static int steps_n = 0, step_i = 0, armed = 0;
static size_t scan_from = 0;
static double armed_at = 0, script_done_ms = -1;

void sim_script(const SimStep *steps, int n) { steps_p = steps; steps_n = n; step_i = 0; scan_from = 0; armed = 0; script_done_ms = -1; }

static void script_tick(void)
{
    if (!steps_p) return;
    if (step_i >= steps_n) { if (script_done_ms < 0) script_done_ms = sim_now_ms(); return; }
    const SimStep *st = &steps_p[step_i];
    if (!armed && !st->expect) { armed = 1; armed_at = sim_now_ms(); return; }
    if (!armed)
    {
        if (scan_from > sim_out_n) scan_from = sim_out_n;
        char *hit = strstr(sim_out + scan_from, st->expect);
        if (!hit) return;
        scan_from = (size_t)(hit - sim_out) + strlen(st->expect);
        armed = 1; armed_at = sim_now_ms();
        return;
    }
    if (sim_now_ms() - armed_at < st->delay_ms) return;
    if (st->check) st->check();
    if (st->keys) sim_keys_str(sim_now_ms(), st->keys);
    armed = 0; step_i++;
}

void sim_reset(long start0, long start1, long start2)
{
    vclk = 0; nkeys = keyi = 0;
    sim_real[0] = start0; sim_real[1] = start1; sim_real[2] = start2;
    for (int i = 0; i < NUM_LEGS; i++) { step_level[i] = 0; dir_level[i] = 0; sim_steps_emitted[i] = 0; sim_pulses_bad[i] = 0; }
    sim_out_n = 0; sim_out[0] = 0;
    sim_log_n = 0;
    steps_p = 0; steps_n = 0; scan_from = 0; armed = 0; script_done_ms = -1;
    sim_track_reset();
}

void sim_key(double t_ms, int key) { if (nkeys < 8192) { keys[nkeys].t_ms = t_ms; keys[nkeys].key = key; nkeys++; } }
double sim_key_gap_ms = 5.0;
void sim_keys_str(double t_ms, const char *s) { while (*s) { sim_key(t_ms, (unsigned char)*s++); t_ms += sim_key_gap_ms; } }
double sim_now_ms(void) { return (double)vclk / (CPU_MHZ * 1000.0); }

static void emit(const char *s)
{
    size_t n = strlen(s);
    if (sim_out_n + n < sizeof(sim_out) - 1) { memcpy(sim_out + sim_out_n, s, n); sim_out_n += n; sim_out[sim_out_n] = 0; }
    if (sim_echo) fputs(s, stdout);
}

void hw_init(void) { }
uint32_t hw_cycles(void) { vclk += sim_cycles_per_call; return (uint32_t)vclk; }
uint32_t hw_cycles_per_us(void) { return CPU_MHZ; }
uint32_t hw_ms(void) { return (uint32_t)(vclk / (CPU_MHZ * 1000u)); }
void hw_delay_us(uint32_t us) { vclk += (uint64_t)us * CPU_MHZ; }
void hw_delay_ms(uint32_t ms) { vclk += (uint64_t)ms * CPU_MHZ * 1000u; }

void hw_step_pin(int leg, int on)
{
    if (on && !step_level[leg])                  /* rising edge = one step */
    {
        pulse_start[leg] = vclk;
        int toward = (dir_level[leg] == HOME_LEVEL[leg]);
        sim_real[leg] += toward ? -1 : 1;
        sim_steps_emitted[leg]++;
        if (sim_real[leg] < sim_seen_min[leg]) sim_seen_min[leg] = sim_real[leg];
        if (sim_real[leg] > sim_seen_max[leg]) sim_seen_max[leg] = sim_real[leg];
        { long sp = spread_now(); if (sp > sim_spread_max) sim_spread_max = sp; }
        if (sim_log_on)
        {
            if (sim_log_n >= sim_log_cap) { sim_log_cap = sim_log_cap ? sim_log_cap * 2 : 1 << 16; sim_log = realloc(sim_log, sim_log_cap * sizeof(SimPulse)); }
            sim_log[sim_log_n++] = (SimPulse){ (double)vclk / (CPU_MHZ * 1000.0), (short)leg, (short)(toward ? -1 : 1), sim_real[leg] };
        }
    }
    if (!on && step_level[leg])
    {
        double us = (double)(vclk - pulse_start[leg]) / CPU_MHZ;
        sim_last_pulse_us[leg] = us;
        if (us < 2.5) sim_pulses_bad[leg]++;
    }
    step_level[leg] = on;
}

void hw_dir_pin(int leg, int level) { dir_level[leg] = level; }
int  hw_limit_raw(int leg) { return sim_real[leg] <= 0; }
int  hw_limit_pressed(int leg) { if (!hw_limit_raw(leg)) return 0; hw_delay_us(500); return hw_limit_raw(leg); }

void con_init(void) { }
void con_print(const char *s) { emit(s); }
void con_printf(const char *fmt, ...) { char b[1024]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap); emit(b); }
void con_queue(const char *s) { emit(s); }
void con_queuef(const char *fmt, ...) { char b[1024]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap); emit(b); }
void con_pump(void) { }

void (*sim_poll_hook)(void) = 0;
int con_poll(void)
{
    if (sim_poll_hook) sim_poll_hook();
    if (sim_now_ms() > sim_end_ms) { fprintf(stderr, "[sim] reached end time\n"); sim_finish(); }
    script_tick();
    /* script finished, all keys typed, and the platform has been idle at the menu for 3 s */
    if (script_done_ms >= 0 && keyi >= nkeys && sim_exit_on_keys_done && sim_now_ms() - script_done_ms > 3000.0) sim_finish();
    if (keyi < nkeys && sim_now_ms() >= keys[keyi].t_ms) return keys[keyi++].key;
    return -1;
}

int con_getc(void)
{
    int c;
    while ((c = con_poll()) < 0)
    {
        if (keyi >= nkeys && sim_exit_on_keys_done && (!steps_p || step_i >= steps_n)) { fprintf(stderr, "[sim] no more scripted keys (t=%.1f s)\n", sim_now_ms() / 1000.0); sim_finish(); }
        vclk += CPU_MHZ * 100u;                 /* 100 us */
    }
    return c;
}

int sim_rx_lost = 0;
int con_rx_lost(void) { int v = sim_rx_lost; sim_rx_lost = 0; return v; }
void con_flush_rx(void) { while (keyi < nkeys && keys[keyi].t_ms <= sim_now_ms()) keyi++; }

static uint8_t flash[64];
static int flash_valid = 0;
int hw_flash_read(void *buf, uint32_t len) { if (!flash_valid) { memset(buf, 0xFF, len); return 0; } memcpy(buf, flash, len); return 0; }
int hw_flash_write(const void *buf, uint32_t len) { memcpy(flash, buf, len); flash_valid = 1; return 0; }
