/**
 * mode_waves.c - Mode 4 (sea states 1-4) and Mode 5 (MAX platform).
 *
 * A sea (heave, pitch, roll of a vessel) comes from plat_waves.c. It is mapped
 * onto the platform like this:
 *   - every degree of freedom is divided by its sea-state-4 "p98" size, so SS4
 *     peaks are about 1.0 and the smaller seas are proportionally smaller;
 *   - that is multiplied by the platform's capability (half the heave range,
 *     the pitch cap, the roll cap) and by a scale sigma;
 *   - sigma = 65 % normally, 95 % in MAX mode. If the legs could not follow that
 *     at real-time speed (85 % of the speed / acceleration limit), the wave clock
 *     is slowed first (floors in plat_config.h) and only then is sigma reduced.
 * In MAX mode every sea is normalised to its own size, so even a calm sea fills
 * the whole workspace; key d switches to a slow demo that sweeps heave, then
 * pitch, then roll through their full range.
 */
#include "plat_app.h"
#include "plat_hw.h"
#include "plat_engine.h"
#include "plat_cal.h"
#include "plat_waves.h"
#include <math.h>

#define TWO_PI 6.283185307179586f
#define K98    2.33f            /* p98 of |x| for a Gaussian = 2.33 sigma */
#define PEAK   3.3f             /* peak / rms for speed and acceleration estimates */

typedef struct {
    float sigma;                /* overall scale, 0..1 of the platform's range */
    float ts;                   /* effective time scale (1 = real time)        */
    float refH, refP, refR;     /* sea size the outputs are divided by */
    const char *limit;          /* what limited sigma */
} Scale;

static int      ss = 3;
static uint32_t seed = WAVE_SEED;
static float    trim = 1.0f;    /* amplitude trim (+ / -) */
static float    tscale = 1.0f;  /* time scale ([ / ]) */

static float smooth(float x) { x = x < 0 ? 0 : (x > 1 ? 1 : x); return x * x * (3.0f - 2.0f * x); }
static float maxf(float a, float b) { return a > b ? a : b; }
static float minf(float a, float b) { return a < b ? a : b; }

static void prepare(int max_mode, Scale *sc)
{
    const KinCaps *c = kin_caps();
    WaveStats ref;

    waves_init(max_mode ? ss : 4, seed);
    ref = *waves_stats();
    waves_init(ss, seed);

    sc->refH = K98 * ref.rms_h_m;
    sc->refP = K98 * ref.rms_p_deg;
    sc->refR = K98 * ref.rms_r_deg;

    float capH = 0.5f * c->heave_range_mm, capP = c->pitch_cap_deg, capR = c->roll_cap_deg;

    /* pose-space velocity / acceleration RMS at sigma = 1, real-time speed */
    float vH = capH * ref.rms_hv / sc->refH, vP = capP * ref.rms_pv / sc->refP, vR = capR * ref.rms_rv / sc->refR;
    float aH = capH * ref.rms_ha / sc->refH, aP = capP * ref.rms_pa / sc->refP, aR = capR * ref.rms_ra / sc->refR;

    float maxv = 1e-3f, maxa = 1e-3f;
    for (int i = 0; i < NUM_LEGS; i++)
    {
        float sv = sqrtf(powf(c->J[i][0] * vH, 2) + powf(c->J[i][1] * vP, 2) + powf(c->J[i][2] * vR, 2));
        float sa = sqrtf(powf(c->J[i][0] * aH, 2) + powf(c->J[i][1] * aP, 2) + powf(c->J[i][2] * aR, 2));
        maxv = maxf(maxv, sv);
        maxa = maxf(maxa, sa);
    }

    /* the size the legs can follow is  k_v / ts  (speed)  and  k_a / ts^2  (acceleration) */
    float s_amp = max_mode ? WAVE_USE_MAX : WAVE_USE_NORMAL;
    float k_v = WAVE_SPEED_USE * (float)g_cal.vmax / (PEAK * maxv);
    float k_a = WAVE_SPEED_USE * (float)g_cal.amax / (PEAK * maxa);

    /* slow the wave clock just enough to reach the wanted size, within the floor */
    float ts_lim = minf(k_v / s_amp, sqrtf(k_a / s_amp));
    float floor_ = max_mode ? WAVE_TIME_FLOOR_MAX : WAVE_TIME_FLOOR_NORMAL;
    float autos = ts_lim > 1.0f ? 1.0f : (ts_lim < floor_ ? floor_ : ts_lim);
    sc->ts = autos * tscale;

    float s_v = k_v / sc->ts;
    float s_a = k_a / (sc->ts * sc->ts);
    sc->sigma = s_amp; sc->limit = "amplitude";
    if (s_v < sc->sigma) { sc->sigma = s_v; sc->limit = "speed limit"; }
    if (s_a < sc->sigma) { sc->sigma = s_a; sc->limit = "acceleration limit"; }
}

static void show(int max_mode, const Scale *sc)
{
    const SeaState *s = waves_sea_state(ss);
    int pct = (int)(sc->sigma * trim * 100.0f + 0.5f);
    con_queuef("%s | wave scale %d%% of range (set by %s) | time x%ld.%02ld | trim %d%%%s\r\n",
               s->name, pct, sc->limit, (long)sc->ts, (long)(sc->ts * 100.0f) % 100,
               (int)(trim * 100.0f + 0.5f), max_mode ? " | MAX" : "");
}

void mode_waves(int max_mode)
{
    if (!g_kin_ok) { con_print("The geometry in plat_config.h is not usable - cannot run pose modes.\r\n"); return; }
    if (!app_ensure_homed()) return;

    const KinCaps *caps = kin_caps();
    Pose center = { caps->heave_mid_mm, 0.0f, 0.0f };
    float prev[NUM_LEGS];
    int have_prev = 0, paused = 0, sweep = 0, seg = 0;
    float capH = 0.5f * caps->heave_range_mm, capP = caps->pitch_cap_deg, capR = caps->roll_cap_deg;
    Scale sc;

    if (max_mode) ss = 4;
    trim = 1.0f;
    tscale = 1.0f;

    con_printf("\r\n=== %s ===\r\n", max_mode ? "MAX PLATFORM" : "WAVES");
    con_print("1 / 2 / 3 / 4 = sea state     r = new random sea     p or space = pause / resume\r\n"
              "+ / - = bigger / smaller waves   ] / [ = faster / slower time\r\n");
    if (max_mode) con_print("d = switch between exaggerated waves and the full-range demo sweep\r\n");
    con_print("q = menu     x = emergency stop\r\n");

    prepare(max_mode, &sc);
    show(max_mode, &sc);

    Pose zero = { 0, 0, 0 };
    app_drive_pose(&center, &zero, prev, &have_prev);
    if (!app_arrive(30000u, 1)) return;
    have_prev = 0;

    const float dt = 1.0f / (float)CTRL_HZ;
    float ramp_t = 0.0f, sweep_t = 0.0f;
    CtlTimer ct;
    ctl_init(&ct, CTRL_HZ);
    con_flush_rx();

    while (1)
    {
        int k = app_poll();
        if (app_check_fault()) return;
        app_tele(max_mode ? MODE_MAX : MODE_WAVES);

        if (k >= 0)
        {
            switch (k)
            {
                case 'q': case 'Q': return;
                case 'x': case 'X': engine_estop(); con_queue("EMERGENCY STOP\r\n"); return;
                case 'p': case 'P': case ' ':
                    paused = !paused;
                    if (paused) engine_hold(); else have_prev = 0;
                    con_queue(paused ? "Paused.\r\n" : "Resumed.\r\n");
                    break;
                case '1': case '2': case '3': case '4':
                    ss = k - '0'; prepare(max_mode, &sc); ramp_t = 0.0f; show(max_mode, &sc); break;
                case 'r': case 'R':
                    seed = seed * 1664525u + 1013904223u; prepare(max_mode, &sc); ramp_t = 0.0f; show(max_mode, &sc); break;
                case '+': case '=': if (trim < 1.3f) trim += 0.1f; show(max_mode, &sc); break;
                case '-': case '_': if (trim > 0.25f) trim -= 0.1f; show(max_mode, &sc); break;
                case ']': if (tscale < 2.5f) tscale *= 1.25f; prepare(max_mode, &sc); show(max_mode, &sc); break;
                case '[': if (tscale > 0.4f) tscale /= 1.25f; prepare(max_mode, &sc); show(max_mode, &sc); break;
                case 'd': case 'D':
                    if (max_mode)
                    {
                        sweep = !sweep; sweep_t = 0.0f; seg = 0; ramp_t = 0.0f;
                        con_queue(sweep ? "Demo sweep: heave, then pitch, then roll, each through its full range.\r\n"
                                        : "Back to exaggerated waves.\r\n");
                    }
                    break;
                default: break;
            }
        }

        if (paused || !ctl_due(&ct)) continue;

        Pose dev = { 0, 0, 0 };
        ramp_t += dt;
        float ramp = smooth(ramp_t / 4.0f);

        if (!sweep)
        {
            float h, p, r;
            waves_advance(dt * sc.ts);
            waves_sample(&h, &p, &r);
            float g = sc.sigma * trim * ramp;
            dev.heave_mm  = g * capH * h / sc.refH;
            dev.pitch_deg = g * capP * p / sc.refP;
            dev.roll_deg  = g * capR * r / sc.refR;
        }
        else
        {
            /* one degree of freedom at a time, two full oscillations each */
            float cap = (seg == 0) ? capH : (seg == 1 ? capP : capR);
            float amp = 0.95f * cap;
            float steps = 1.0f;
            for (int i = 0; i < NUM_LEGS; i++) steps = maxf(steps, fabsf(caps->J[i][seg]) * amp);
            float om = TWO_PI / 12.0f;
            float om_v = WAVE_SPEED_USE * (float)g_cal.vmax / steps;
            float om_a = sqrtf(WAVE_SPEED_USE * (float)g_cal.amax / steps);
            if (om_v < om) om = om_v;
            if (om_a < om) om = om_a;
            sweep_t += dt;
            if (om * sweep_t >= 2.0f * TWO_PI)
            {
                sweep_t = 0.0f; seg = (seg + 1) % 3;
                con_queuef("sweep: %s\r\n", seg == 0 ? "heave" : (seg == 1 ? "pitch" : "roll"));
            }
            float v = amp * sinf(om * sweep_t);
            if (seg == 0) dev.heave_mm = v; else if (seg == 1) dev.pitch_deg = v; else dev.roll_deg = v;
        }
        app_drive_pose(&center, &dev, prev, &have_prev);
    }
}
