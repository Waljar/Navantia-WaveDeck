/**
 * plat_engine.h - the step generator.
 *
 * Modes never touch the pins. They hand the engine a target position (and
 * optionally the target's velocity) for each leg, and the engine follows it
 * with a velocity- and acceleration-limited profile, emitting STEP pulses on a
 * fixed ENGINE_TICK_HZ clock. engine_service() must be called continuously
 * (every main-loop pass); it never blocks.
 */
#ifndef PLAT_ENGINE_H
#define PLAT_ENGINE_H

#include <stdint.h>
#include "plat_config.h"

void  engine_init(void);
/* Declare where the legs are right now (after homing) and enable stepping. */
void  engine_sync(const int32_t pos[NUM_LEGS]);
void  engine_disable(void);                       /* stop generating pulses (homing uses its own) */
void  engine_set_limits(const int32_t mn[NUM_LEGS], const int32_t mx[NUM_LEGS]);
void  engine_set_motion(float vmax, float amax);  /* steps/s, steps/s^2 */
void  engine_guard(int on);                       /* watch the home switches while stepping toward them */
/* Leg difference guard: heights above ref[] (the level minimums) may not differ by more than
 * maxd steps. A leg that would step further ahead is held where it is (its command is reset to
 * its position) until the others catch up. maxd = 0 turns the guard off. */
void  engine_set_spread(const int32_t ref[NUM_LEGS], int32_t maxd);

/* Targets are clamped to the limits. tv = target velocity (steps/s) or NULL. */
void  engine_set_target(const float tgt[NUM_LEGS], const float tv[NUM_LEGS]);
void  engine_hold(void);                          /* stop smoothly where the legs would come to rest */
void  engine_estop(void);                         /* stop pulses at once */

void  engine_service(void);                       /* call very often */

int32_t engine_pos(int leg);                      /* actual step count */
float   engine_cmd(int leg);                      /* commanded position (float) */
float   engine_vel(int leg);                      /* current velocity, steps/s */
float   engine_target(int leg);
int     engine_settled(void);                     /* every leg on target and still */
int     engine_fault(void);                       /* 0, or 1 + leg index of the switch that fired */
void    engine_clear_fault(void);
uint32_t engine_spread_holds(void);                /* ticks in which a leg was held back by the guard */
uint32_t engine_late(void);                       /* times the main loop fell more than 0.8 ms behind the step clock */

#endif /* PLAT_ENGINE_H */
