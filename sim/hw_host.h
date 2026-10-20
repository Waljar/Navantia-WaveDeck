#ifndef HW_HOST_H
#define HW_HOST_H
#include <stddef.h>
#include <setjmp.h>
typedef struct { double t_ms; short leg; short dir; long pos; } SimPulse;
extern long sim_real[3];
extern long sim_steps_emitted[3];
extern long sim_pulses_bad[3];
extern double sim_last_pulse_us[3];
extern SimPulse *sim_log; extern long sim_log_n; extern int sim_log_on;
extern int sim_rx_lost;
extern double sim_key_gap_ms;
extern int sim_echo; extern char sim_out[]; extern size_t sim_out_n;
extern int sim_exit_on_keys_done; extern double sim_end_ms;
extern unsigned sim_cycles_per_call;
void sim_reset(long s0, long s1, long s2);
/* Output-driven script: wait until `expect` appears in the console output (searched
 * after the previous match), wait delay_ms of virtual time, run check(), then type keys. */
typedef struct { const char *expect; double delay_ms; void (*check)(void); const char *keys; } SimStep;
void sim_script(const SimStep *steps, int n);
extern long sim_seen_min[3], sim_seen_max[3];     /* extreme leg positions since sim_track_reset() */
extern long sim_spread_max;                         /* biggest difference between the highest and lowest leg since sim_track_reset() */
void sim_track_reset(void);
extern void (*sim_poll_hook)(void);                /* called on every console poll (debug sampling) */
extern jmp_buf sim_exit_jmp; extern int sim_exit_jmp_set;   /* scenario end: longjmp here if set */
void sim_key(double t_ms, int key);
void sim_keys_str(double t_ms, const char *s);
double sim_now_ms(void);
#endif
