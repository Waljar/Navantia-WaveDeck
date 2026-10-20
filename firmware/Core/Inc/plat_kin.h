/**
 * plat_kin.h - inverse / forward kinematics for the 3-crank platform.
 *
 * Pose: heave (mm above the level minimum), pitch and roll (degrees).
 * Positive pitch raises the front (X) leg, positive roll raises the back-left (Z) side.
 */
#ifndef PLAT_KIN_H
#define PLAT_KIN_H

#include <stdint.h>
#include "plat_config.h"

typedef struct { float heave_mm, pitch_deg, roll_deg; } Pose;

typedef struct {
    float r, L, Rb, Rp;          /* mm */
    float theta_min_deg;         /* crank angle above horizontal at the level minimum */
    float crank_az_deg;          /* 0 = crank points outward, 180 = inward, +-90 sideways */
    int   sense;                 /* +1: more steps raises the crank pin */
    float leg_az_deg[NUM_LEGS];
    int   configured;
} KinGeom;

typedef struct {
    float heave_range_mm;        /* full travel: level minimum to level maximum */
    float heave_mid_mm;
    float pitch_cap_deg;         /* largest tilt at mid-heave that keeps every leg in range */
    float roll_cap_deg;
    float J[NUM_LEGS][3];        /* steps per mm / per deg pitch / per deg roll at mid-heave */
} KinCaps;

/* Optional hook called inside long loops (the app points it at the step engine
 * so pulses keep their timing while the maths runs). */
extern void (*kin_yield_hook)(void);

/* Safety limits found by hand in Setup (0 = none). They make kin_feasible() say "no" for a pose
 * that tilts further than measured, or lets one leg get more than max_diff steps ahead of another
 * (heights above each leg's minimum). Call this BEFORE kin_init(), which then works the
 * capability numbers out inside the limits. */
void kin_set_limits(float pitch_pos_deg, float pitch_neg_deg, float roll_pos_deg, float roll_neg_deg, int32_t max_diff);

/* Set geometry (use kin_default_geom() for the values in plat_config.h) and the
 * leg limits (steps). Recomputes the capability numbers. Returns 0 on success. */
void kin_default_geom(KinGeom *g);
int  kin_init(const KinGeom *g, const int32_t min_steps[NUM_LEGS], const int32_t max_steps[NUM_LEGS]);
const KinGeom *kin_geom(void);
const KinCaps *kin_caps(void);

/* Pose -> leg positions in steps. Returns 0, or -1 if the pose is geometrically
 * impossible. Does NOT check the leg limits. */
int kin_pose_to_steps(const Pose *p, float steps[NUM_LEGS]);

/* Are all legs inside [min + margin, max - margin] for this pose? */
int kin_feasible(const Pose *p, float margin_steps);

/* Leg positions -> pose (Newton iteration). Returns 0 on success. */
int kin_steps_to_pose(const float steps[NUM_LEGS], Pose *p);

/* Shrink the deviation `dev` (applied about `center`) until every leg is inside
 * its limits less `margin_steps`. Writes the resulting leg targets. Returns the
 * fraction of dev that was kept (1.0 = untouched), or -1 on failure. */
float kin_fit(const Pose *center, const Pose *dev, float margin_steps,
              Pose *out, float steps[NUM_LEGS]);

#endif /* PLAT_KIN_H */
