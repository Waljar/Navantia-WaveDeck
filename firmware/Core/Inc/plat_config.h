/**
 * plat_config.h - every tunable number for the 3-leg motion platform.
 *
 * Wiring, pin names and the CL86T driver settings are documented in the README.
 * Only the geometry block (bottom of this file) must be filled in by hand;
 * everything else already matches the hardware as tested.
 */
#ifndef PLAT_CONFIG_H
#define PLAT_CONFIG_H

#include <stdint.h>

/* ---- Legs ------------------------------------------------------------------ */
#define NUM_LEGS             3          /* leg 0 = X, leg 1 = Y, leg 2 = Z */

#define STEPS_PER_MOTOR_REV  2000       /* CL86T DIP switches: 2000 pulses/rev */
#define GEAR_RATIO           10
#define STEPS_PER_REV        (STEPS_PER_MOTOR_REV * GEAR_RATIO)      /* 20000 = 1 crank rev */
#define STEPS_PER_DEG        (20000.0f / 360.0f)                     /* 55.56 steps per crank degree */

/* DIR pin level that moves a crank TOWARD its home switch (1 = high). */
#define HOME_DIR_LEG0        1
#define HOME_DIR_LEG1        1
#define HOME_DIR_LEG2        1

/* ---- Homing ------------------------------------------------------------------ */
#define SEEK_HZ              400u       /* first approach to the switch              */
#define BACKOFF_HZ           400u
#define BACKOFF_STEPS        500u       /* pull back 9 deg of crank (was 1000)       */
#define HOME_CHUNK_STEPS     40u        /* the three legs lower together in bursts of this many steps
                                           (0.7 deg of crank), so no leg is ever far ahead of the others */
#define LATCH_HZ             100u       /* slow second approach: this point is 0     */
#define MAX_SEEK_STEPS       (STEPS_PER_REV * 5u / 4u)               /* give up after 1.25 revs */
#define DIR_SETUP_US         20u

/* Switches are normally closed (COM to GND, NC to the pin, internal pull-up):
 * a pressed or broken switch reads HIGH. */
#define LIMIT_PRESSED_LEVEL  1

/* ---- Travel limits --------------------------------------------------------- */
#define TRAVEL_FLOOR_STEPS   150        /* never jog lower than this (switch is at 0)   */
#define TRAVEL_CEIL_STEPS    9500       /* never jog higher than this during setup      */

/* Calibration defaults, used until the Setup mode has saved real values to flash.
 * (Level minimum / maximum heave, measured on the rig.) */
#define DEF_MIN_LEG0         672
#define DEF_MIN_LEG1         700
#define DEF_MIN_LEG2         798
#define DEF_MAX_LEG0         8567
#define DEF_MAX_LEG1         8595
#define DEF_MAX_LEG2         8693

/* ---- Safety limits ------------------------------------------------------------- */
#define JOG_LEAD_STEPS       600        /* a jog key can never put a leg's target more than this
                                           far ahead of where the leg really is (stops key bursts piling up) */
#define SPREAD_TOL_STEPS     40         /* the step engine lets the legs differ this much more than
                                           the saved limit before it holds a leg back (tracking lag)  */
#define MIN_TILT_CDEG        100        /* smallest tilt limit Setup will accept: 1.00 deg            */
#define MIN_SPREAD_STEPS     200        /* smallest leg difference limit Setup will accept            */

/* ---- Motion engine --------------------------------------------------------- */
#define ENGINE_TICK_HZ       10000u     /* step-generation clock                        */
#define STEP_PULSE_US        20u        /* STEP pulse width (CL86T needs >= 2.5 us)     */
#define CTRL_HZ              100u       /* how often modes update their target          */

#define VMAX_DEFAULT         1500       /* steps/s  (27 deg/s of crank) - tested        */
#define AMAX_DEFAULT         1500       /* steps/s^2                    - tested        */
#define VMAX_LIMIT           4000       /* the Setup menu can never set more than this  */
#define AMAX_LIMIT           8000

/* ---- Heave cycle ----------------------------------------------------------- */
#define CYCLE_PAUSE_MS       1500u      /* pause at each end (adjust live with [ and ]) */

/* ---- Waves ----------------------------------------------------------------- */
#define WAVE_COMPONENTS      24         /* sinusoids per spectrum                       */
#define WAVE_SEED            1u         /* same seed -> same sea in firmware and visualiser */
#define WAVE_HS_EXPONENT     0.75f      /* Hs shown = Hs4 * (Hs/Hs4)^exp  (1.0 = true proportions) */
#define WAVE_VESSEL_WC       2.0f       /* rad/s: vessel stops following waves faster than this  */
#define WAVE_PITCH_GAIN      1.0f       /* vessel follows the wave slope this much      */
#define WAVE_ROLL_GAIN       0.8f
#define WAVE_USE_NORMAL      0.65f      /* SS4 p98 reaches 65 % of the platform's range */
#define WAVE_USE_MAX         0.95f      /* "max platform" mode reaches 95 %             */
#define WAVE_SPEED_USE       0.85f      /* never plan above 85 % of vmax / amax         */
/* When the motors cannot follow the waves at full size and real-time speed, the
 * wave clock is slowed down (rather than the waves being shrunk) until the
 * wanted size is reached, but never below these factors (1.0 = real time).
 * Only after that is the amplitude reduced. */
#define WAVE_TIME_FLOOR_NORMAL 0.5f     /* sea states 1-4: at most 2x slower than real time  */
#define WAVE_TIME_FLOOR_MAX    0.2f     /* MAX platform:   up to 5x slower                   */

/* ---- Geometry: FILL THESE IN (mm and degrees) ------------------------------
 * The numbers below are PLACEHOLDERS so that everything builds and runs.
 * Until GEOMETRY_CONFIGURED is 1 the console prints "geometry NOT set": the
 * mm / degree values are then only indicative. Leg limits are always enforced
 * whatever the geometry says.
 *
 *  GEOM_CRANK_R          crank arm: motor shaft centre -> pin joint centre
 *  GEOM_LINK_L           link (carbon rod): joint centre -> joint centre
 *  GEOM_BASE_R           centre-line -> motor shaft, horizontal
 *  GEOM_PLATFORM_R       centre-line -> link top joint, horizontal
 *  GEOM_THETA_MIN_DEG    crank angle above horizontal when the platform is at
 *                        its level minimum (negative if below horizontal)
 *  GEOM_CRANK_AZ_DEG     direction the crank points at 0 deg, relative to the leg's
 *                        outward direction: 0 = outward, 180 = inward, +-90 = sideways
 *  GEOM_SENSE            +1 if more steps raises the crank pin, -1 if it lowers it
 *  GEOM_AZ_LEG0/1/2      azimuth of each leg seen from above, counter-clockwise
 *                        from the front (X leg = 0, Z = 120 back left, Y = 240 back right).
 */
#define GEOMETRY_CONFIGURED  1
#define GEOM_CRANK_R         500.0f
#define GEOM_LINK_L          620.0f
#define GEOM_BASE_R          554.3f
#define GEOM_PLATFORM_R      207.8f
#define GEOM_THETA_MIN_DEG   -68.0f
#define GEOM_CRANK_AZ_DEG    180.0f
#define GEOM_SENSE           1
#define GEOM_AZ_LEG0         0.0f
#define GEOM_AZ_LEG1         240.0f
#define GEOM_AZ_LEG2         120.0f

#endif /* PLAT_CONFIG_H */
