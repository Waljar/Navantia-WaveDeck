/**
 * plat_homing.h - blocking homing sequence: seek toward the switch, pull back,
 * creep up again; that trigger point is 0.
 * The step engine must be disabled while this runs.
 */
#ifndef PLAT_HOMING_H
#define PLAT_HOMING_H

/* Lower all three legs to their switches together (short bursts in turn, so no leg gets far ahead
 * of the others), then finish each: pull back, creep up again, that trigger point is 0.
 * On a failure the user may retry (r) or abort (q). Returns 1 when all three are homed, 0 if aborted. */
int home_all(void);

#endif
