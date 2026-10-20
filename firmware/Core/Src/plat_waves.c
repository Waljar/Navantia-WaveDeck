/**
 * plat_waves.c - see plat_waves.h.
 *
 * Spectrum: Bretschneider  S(w) = 5/16 Hs^2 wp^4 w^-5 exp(-1.25 (wp/w)^4)
 * Components are spread over 0.5 wp .. 3.5 wp, each with amplitude sqrt(2 S dw),
 * then renormalised so that the variance equals (Hs/4)^2 exactly.
 */
#include "plat_waves.h"
#include "plat_config.h"
#include <math.h>

#define TWO_PI   6.283185307179586f
#define RAD2DEG  57.29577951308232f
#define GRAV     9.81f

static const SeaState SEA[5] = {
    { 0.00f, 1.0f, "none" },
    { 0.05f, 2.5f, "Sea state 1 - calm (rippled)" },
    { 0.30f, 4.0f, "Sea state 2 - smooth" },
    { 0.90f, 6.0f, "Sea state 3 - slight" },
    { 1.90f, 8.0f, "Sea state 4 - moderate" },
};

const SeaState *waves_sea_state(int ss)
{
    if (ss < 1) ss = 1;
    if (ss > 4) ss = 4;
    return &SEA[ss];
}

static float    W[WAVE_COMPONENTS];     /* rad/s                         */
static float    AH[WAVE_COMPONENTS];    /* heave amplitude (m)           */
static float    AP[WAVE_COMPONENTS];    /* pitch amplitude (deg)         */
static float    AR[WAVE_COMPONENTS];    /* roll amplitude (deg)          */
static float    PH0[WAVE_COMPONENTS], PR0[WAVE_COMPONENTS];   /* start phases */
static float    PH[WAVE_COMPONENTS],  PR[WAVE_COMPONENTS];    /* running phases */
static WaveStats ST;

static uint32_t rng;
static float rand01(void)
{
    rng = rng * 1664525u + 1013904223u;
    return (float)(rng >> 8) / 16777216.0f;
}

void waves_init(int ss, uint32_t seed)
{
    const SeaState *s = waves_sea_state(ss);
    const SeaState *s4 = waves_sea_state(4);
    float hs = s4->hs_m * powf(s->hs_m / s4->hs_m, WAVE_HS_EXPONENT);
    float wp = TWO_PI / s->tp_s;
    float wmin = 0.5f * wp, wmax = 3.5f * wp;
    float dw = (wmax - wmin) / (float)WAVE_COMPONENTS;
    float sum = 0.0f;

    rng = seed ? seed : 1u;
    for (int k = 0; k < WAVE_COMPONENTS; k++)
    {
        float jit = rand01() - 0.5f;
        float w = wmin + ((float)k + 0.5f + 0.8f * jit) * dw;
        float spec = (5.0f / 16.0f) * hs * hs * powf(wp, 4.0f) * powf(w, -5.0f)
                   * expf(-1.25f * powf(wp / w, 4.0f));
        W[k]   = w;
        AH[k]  = sqrtf(2.0f * spec * dw);
        PH0[k] = TWO_PI * rand01();
        PR0[k] = TWO_PI * rand01();
        sum += 0.5f * AH[k] * AH[k];
    }

    float norm = (sum > 0.0f) ? sqrtf((hs / 4.0f) * (hs / 4.0f) / sum) : 1.0f;
    ST = (WaveStats){0};
    for (int k = 0; k < WAVE_COMPONENTS; k++)
    {
        float w   = W[k];
        float rao = 1.0f / sqrtf(1.0f + powf(w / WAVE_VESSEL_WC, 4.0f));
        float a   = AH[k] * norm;
        float slope = a * (w * w / GRAV) * rao * RAD2DEG;     /* deg */

        AH[k] = a * rao;
        AP[k] = WAVE_PITCH_GAIN * slope;
        AR[k] = WAVE_ROLL_GAIN * slope;

        float w2 = w * w, w4 = w2 * w2;
        ST.rms_h_m   += 0.5f * AH[k] * AH[k];
        ST.rms_p_deg += 0.5f * AP[k] * AP[k];
        ST.rms_r_deg += 0.5f * AR[k] * AR[k];
        ST.rms_hv    += 0.5f * w2 * AH[k] * AH[k];
        ST.rms_pv    += 0.5f * w2 * AP[k] * AP[k];
        ST.rms_rv    += 0.5f * w2 * AR[k] * AR[k];
        ST.rms_ha    += 0.5f * w4 * AH[k] * AH[k];
        ST.rms_pa    += 0.5f * w4 * AP[k] * AP[k];
        ST.rms_ra    += 0.5f * w4 * AR[k] * AR[k];
    }
    ST.rms_h_m = sqrtf(ST.rms_h_m);  ST.rms_p_deg = sqrtf(ST.rms_p_deg);  ST.rms_r_deg = sqrtf(ST.rms_r_deg);
    ST.rms_hv  = sqrtf(ST.rms_hv);   ST.rms_pv    = sqrtf(ST.rms_pv);     ST.rms_rv    = sqrtf(ST.rms_rv);
    ST.rms_ha  = sqrtf(ST.rms_ha);   ST.rms_pa    = sqrtf(ST.rms_pa);     ST.rms_ra    = sqrtf(ST.rms_ra);
    waves_reset();
}

void waves_reset(void)
{
    for (int k = 0; k < WAVE_COMPONENTS; k++) { PH[k] = PH0[k]; PR[k] = PR0[k]; }
}

void waves_advance(float dt)
{
    for (int k = 0; k < WAVE_COMPONENTS; k++)
    {
        float d = W[k] * dt;
        PH[k] += d;  PR[k] += d;
        while (PH[k] > TWO_PI) PH[k] -= TWO_PI;
        while (PR[k] > TWO_PI) PR[k] -= TWO_PI;
    }
}

void waves_sample(float *h, float *p, float *r)
{
    float sh = 0.0f, sp = 0.0f, sr = 0.0f;
    for (int k = 0; k < WAVE_COMPONENTS; k++)
    {
        sh += AH[k] * cosf(PH[k]);
        sp += AP[k] * sinf(PH[k]);
        sr += AR[k] * sinf(PR[k]);
    }
    *h = sh;  *p = sp;  *r = sr;
}

const WaveStats *waves_stats(void) { return &ST; }
