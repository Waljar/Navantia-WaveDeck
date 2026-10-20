/**
 * mode_circles.c - Mode 3: the platform tilts in a slow circle (pitch and roll
 * together), optionally while also heaving slowly. Amplitude and period are
 * adjustable live.
 */
#include "plat_app.h"
#include "plat_hw.h"
#include "plat_engine.h"
#include "plat_cal.h"
#include <math.h>

#define TWO_PI 6.283185307179586f

static float amp_deg = 1.0f;       /* tilt amplitude */
static float period_s = 8.0f;      /* seconds per circle */
static int   heave_on = 0;         /* also oscillate the heave */
static int   dir = 1;

static float smooth(float x) { x = x < 0 ? 0 : (x > 1 ? 1 : x); return x * x * (3.0f - 2.0f * x); }

static void show(void)
{
    const KinCaps *c = kin_caps();
    float cap = c->pitch_cap_deg < c->roll_cap_deg ? c->pitch_cap_deg : c->roll_cap_deg;
    con_queuef("tilt circle %ld.%02ld deg (max about %ld.%ld) | one circle every %ld.%ld s | heave motion %s | %s\r\n",
               (long)amp_deg, (long)(amp_deg * 100.0f) % 100, (long)cap, (long)(cap * 10.0f) % 10,
               (long)period_s, (long)(period_s * 10.0f) % 10, heave_on ? "ON" : "off",
               dir > 0 ? "forward" : "reversed");
}

void mode_circles(void)
{
    if (!g_kin_ok) { con_print("The geometry in plat_config.h is not usable - cannot run pose modes.\r\n"); return; }
    if (!app_ensure_homed()) return;

    const KinCaps *caps = kin_caps();
    Pose center = { caps->heave_mid_mm, 0.0f, 0.0f };
    float prev[NUM_LEGS];
    int have_prev = 0, paused = 0;

    con_print("\r\n=== PITCH / ROLL CIRCLES ===\r\n"
              "+ / - = bigger / smaller tilt     ] / [ = slower / faster circle\r\n"
              "h = heave motion on/off           r = reverse direction\r\n"
              "p or space = pause / resume       q = menu       x = emergency stop\r\n");
    show();

    /* ease to the centre pose first */
    Pose zero = { 0, 0, 0 };
    app_drive_pose(&center, &zero, prev, &have_prev);
    if (!app_arrive(30000u, 1)) return;
    have_prev = 0;

    float cap_amp = caps->pitch_cap_deg < caps->roll_cap_deg ? caps->pitch_cap_deg : caps->roll_cap_deg;
    float t = 0.0f, tilt_t = 0.0f;
    const float dt = 1.0f / (float)CTRL_HZ;
    CtlTimer ct;
    ctl_init(&ct, CTRL_HZ);
    con_flush_rx();

    while (1)
    {
        int k = app_poll();
        if (app_check_fault()) return;
        app_tele(MODE_CIRCLES);

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
                case '+': case '=': amp_deg += 0.25f; if (amp_deg > 0.9f * cap_amp) amp_deg = 0.9f * cap_amp; show(); break;
                case '-': case '_': amp_deg -= 0.25f; if (amp_deg < 0.25f) amp_deg = 0.25f; show(); break;
                case ']': period_s += 1.0f; if (period_s > 60.0f) period_s = 60.0f; show(); break;
                case '[': period_s -= 1.0f; if (period_s < 2.0f) period_s = 2.0f; show(); break;
                case 'h': case 'H': heave_on = !heave_on; show(); break;
                case 'r': case 'R': dir = -dir; show(); break;
                default: break;
            }
        }

        if (paused || !ctl_due(&ct)) continue;

        t += dt;
        tilt_t += dt;
        float ramp = smooth(t / 3.0f);                          /* ease the circle in */
        float ph = dir * TWO_PI * tilt_t / period_s;
        Pose dev;
        dev.pitch_deg = amp_deg * ramp * cosf(ph);
        dev.roll_deg  = amp_deg * ramp * sinf(ph);
        dev.heave_mm  = heave_on ? 0.35f * 0.5f * caps->heave_range_mm * ramp * sinf(TWO_PI * t / 20.0f) : 0.0f;
        app_drive_pose(&center, &dev, prev, &have_prev);
    }
}
