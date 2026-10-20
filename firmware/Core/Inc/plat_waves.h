/**
 * plat_waves.h - irregular sea generator (Bretschneider / Pierson-Moskowitz
 * spectrum, WMO sea states 1-4) producing heave, pitch and roll of a vessel.
 *
 * A sea is a sum of WAVE_COMPONENTS sinusoids with random phases. Heave is the
 * wave elevation; pitch and roll follow the wave slope (short-crested sea: the
 * roll uses independent phases). The same maths runs in the web visualiser
 * (docs/visualiser.html) with the same seed, so both show the same sea.
 */
#ifndef PLAT_WAVES_H
#define PLAT_WAVES_H

#include <stdint.h>

typedef struct { float hs_m, tp_s; const char *name; } SeaState;

/* Sea states 1..4 (index 1..4). 0 is unused. */
const SeaState *waves_sea_state(int ss);

/* Statistics of one sea (analytic, no simulation needed) */
typedef struct {
    float rms_h_m, rms_p_deg, rms_r_deg;          /* position RMS               */
    float rms_hv, rms_pv, rms_rv;                 /* velocity RMS  (m/s, deg/s) */
    float rms_ha, rms_pa, rms_ra;                 /* acceleration RMS           */
} WaveStats;

/* Build the sea for state ss (1..4). The displayed height follows
 * Hs4 * (Hs / Hs4)^WAVE_HS_EXPONENT so the small seas stay visible. */
void waves_init(int ss, uint32_t seed);
void waves_reset(void);                              /* back to t = 0               */
void waves_advance(float dt_s);
void waves_sample(float *heave_m, float *pitch_deg, float *roll_deg);
const WaveStats *waves_stats(void);

#endif /* PLAT_WAVES_H */
