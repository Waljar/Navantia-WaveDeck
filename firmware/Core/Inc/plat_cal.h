/**
 * plat_cal.h - calibration (leg minimums / maximums, speed limits) kept in the
 * last page of the STM32's flash, so nothing has to be copied back into the code.
 */
#ifndef PLAT_CAL_H
#define PLAT_CAL_H

#include <stdint.h>
#include "plat_config.h"

typedef struct {
    int32_t min[NUM_LEGS];      /* level minimum, steps from the home switch       */
    int32_t max[NUM_LEGS];      /* maximum heave, steps                            */
    int32_t vmax;               /* steps/s                                         */
    int32_t amax;               /* steps/s^2                                       */
    /* safety limits found by hand in Setup; 0 = not set (the geometry decides) */
    int32_t pitch_pos_cdeg;     /* largest pitch in the + direction, 0.01 deg      */
    int32_t pitch_neg_cdeg;     /* largest pitch in the - direction                */
    int32_t roll_pos_cdeg;
    int32_t roll_neg_cdeg;
    int32_t max_diff;           /* largest difference between two legs' heights above their minimums, steps */
    int     from_flash;         /* 1 if these values were read from flash          */
} Cal;

extern Cal g_cal;

void cal_defaults(void);        /* compile-time values from plat_config.h */
void cal_load(void);            /* defaults, then flash if it holds a valid record */
int  cal_save(void);            /* 0 = saved */
int  cal_valid(const Cal *c);   /* sanity check */

#endif
