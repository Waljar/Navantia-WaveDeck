/**
 * plat_app.h - shared helpers used by the menu and by every mode.
 */
#ifndef PLAT_APP_H
#define PLAT_APP_H

#include <stdint.h>
#include "plat_config.h"
#include "plat_kin.h"

enum { MODE_MENU = 0, MODE_SETUP = 1, MODE_HEAVE = 2, MODE_CIRCLES = 3,
       MODE_WAVES = 4, MODE_MAX = 5, MODE_MANUAL = 6 };

extern int g_homed;       /* 1 once all legs are homed and the platform is trusted */
extern int g_locked;      /* 1 = presentation lock on                                */
extern int g_tele;        /* 1 = send "T,..." telemetry lines to the visualiser     */
extern int g_kin_ok;      /* 1 if the geometry in plat_config.h is usable           */

#define FIT_MARGIN_STEPS  30.0f      /* keep poses this far inside every leg limit */

void app_init(void);
void app_run(void);                           /* menu loop, never returns */

/* Keep the step engine and the console moving. Call every loop pass. */
void app_idle(void);
int  app_poll(void);                          /* app_idle() + next key or -1 */
int  app_getc(void);                          /* blocks, but keeps the engine alive */
int  app_check_fault(void);                   /* 1 (and message) if a home switch fired */

int  app_ensure_homed(void);                  /* home + go to the minimums if needed; 0 = cancelled */
void app_apply_cal(void);                     /* push g_cal into the engine and kinematics */
void app_apply_cal_ex(int with_limits);       /* 0 = leave the tilt and leg-difference limits off (Setup uses this
                                                 while it is finding them) */
int  app_lower_to_rest(int announce);         /* all legs to their minimums together; 1 = arrived, 0 = stopped by the user */
int  app_rehome(void);                        /* lower to rest if the position is known, then home. 0 = cancelled */
void app_settle(uint32_t timeout_ms);         /* stop smoothly where the legs are and wait until still (cancels a move in progress) */
int  app_arrive(uint32_t timeout_ms, int abortable);   /* wait for the legs to reach the target already set; 0 = stopped by x / q or a fault */
void app_tele(int mode);                      /* rate-limited telemetry line */

/* Format a float with a fixed number of decimals (the embedded printf has no %f). */
int  fmt_f(char *out, int n, float v, int decimals);

/* A fixed-rate timer for a mode's control loop */
typedef struct { uint32_t period, next; } CtlTimer;
void ctl_init(CtlTimer *t, uint32_t hz);
int  ctl_due(CtlTimer *t);

/* Drive the platform to (center + dev) shrunk to fit the leg limits. `prev` holds
 * the previous leg targets so the engine gets a feed-forward velocity.
 * Returns the fraction of dev kept (1 = all, <0 = failure). */
float app_drive_pose(const Pose *center, const Pose *dev, float prev[NUM_LEGS], int *have_prev);

/* Modes */
void mode_setup(void);
void mode_heave(void);
void mode_circles(void);
void mode_waves(int max_mode);
void mode_manual(void);

#endif
