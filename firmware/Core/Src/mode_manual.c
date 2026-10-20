/**
 * mode_manual.c - Mode 6: set heave, pitch and roll by hand, with keys or by
 * typing a number (h12.5 + Enter, p-3 + Enter, r2 + Enter).
 * Heave is measured from the middle of the stroke (0 = mid, + = up).
 */
#include "plat_app.h"
#include "plat_hw.h"
#include "plat_engine.h"
#include "plat_cal.h"
#include <stdlib.h>
#include <math.h>

static const float H_STEPS[] = { 1.0f, 2.0f, 5.0f, 10.0f };
static const float A_STEPS[] = { 0.25f, 0.5f, 1.0f, 2.0f };
static int step_idx = 1;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void report(const Pose *got, float keep)
{
    char h[16], p[16], r[16];
    fmt_f(h, sizeof(h), got->heave_mm, 1);
    fmt_f(p, sizeof(p), got->pitch_deg, 2);
    fmt_f(r, sizeof(r), got->roll_deg, 2);
    con_queuef("heave %s mm (0 = mid) | pitch %s deg | roll %s deg%s\r\n", h, p, r,
               (keep >= 0.0f && keep < 0.999f) ? "  <-- LIMITED by the leg ranges" : "");
}

void mode_manual(void)
{
    if (!g_kin_ok) { con_print("The geometry in plat_config.h is not usable - cannot run pose modes.\r\n"); return; }
    if (!app_ensure_homed()) return;

    const KinCaps *caps = kin_caps();
    Pose center = { caps->heave_mid_mm, 0.0f, 0.0f };
    Pose want = { 0, 0, 0 };
    float prev[NUM_LEGS];
    int have_prev = 0;
    char buf[16];
    int entering = 0, nbuf = 0;
    uint8_t which = 0;

    con_print("\r\n=== MANUAL ===\r\n"
              "u / j = heave up / down     w / s = pitch + / -     d / a = roll + / -\r\n"
              "+ / - = bigger / smaller steps      z = zero everything (mid-height, level)\r\n"
              "type  h12.5 [Enter]  /  p-3 [Enter]  /  r2 [Enter]  to set a value directly\r\n"
              "q = menu     x = emergency stop\r\n");

    float keep = app_drive_pose(&center, &want, prev, &have_prev);
    report(&want, keep);
    con_flush_rx();

    while (1)
    {
        int k = app_poll();
        if (app_check_fault()) return;
        app_tele(MODE_MANUAL);
        /* a character was lost on the way in: a half-typed number may have changed meaning ("12.5" -> "125"),
           so throw it away instead of acting on it */
        if (con_rx_lost() && entering)
        {
            entering = 0;
            con_queue("Input lost - that value was NOT used. Type it again.\r\n");
            if (k == '\r' || k == '\n') continue;
        }
        if (k < 0) continue;

        if (entering)
        {
            if (k == '\r' || k == '\n')
            {
                buf[nbuf] = 0;
                float v = (float)atof(buf);
                if (which == 'h') want.heave_mm = v; else if (which == 'p') want.pitch_deg = v; else want.roll_deg = v;
                entering = 0;
            }
            else if (k == 27) { entering = 0; con_queue("cancelled\r\n"); continue; }
            else if ((k == 8 || k == 127) && nbuf > 0) { nbuf--; continue; }
            else if (((k >= '0' && k <= '9') || k == '.' || k == '-') && nbuf < 12) { buf[nbuf++] = (char)k; continue; }
            else continue;
        }
        else
        {
            float hs = H_STEPS[step_idx], as = A_STEPS[step_idx];
            switch (k)
            {
                case 'q': case 'Q': return;
                case 'x': case 'X': engine_estop(); con_queue("EMERGENCY STOP\r\n"); return;
                case 'u': case 'U': want.heave_mm  += hs; break;
                case 'j': case 'J': want.heave_mm  -= hs; break;
                case 'w': case 'W': want.pitch_deg += as; break;
                case 's': case 'S': want.pitch_deg -= as; break;
                case 'd': case 'D': want.roll_deg  += as; break;
                case 'a': case 'A': want.roll_deg  -= as; break;
                case '+': case '=': if (step_idx < 3) step_idx++; con_queuef("step: %ld mm / %ld.%02ld deg\r\n", (long)H_STEPS[step_idx], (long)A_STEPS[step_idx], (long)(A_STEPS[step_idx] * 100.0f) % 100); continue;
                case '-': case '_': if (step_idx > 0) step_idx--; con_queuef("step: %ld mm / %ld.%02ld deg\r\n", (long)H_STEPS[step_idx], (long)A_STEPS[step_idx], (long)(A_STEPS[step_idx] * 100.0f) % 100); continue;
                case 'z': case 'Z': want.heave_mm = want.pitch_deg = want.roll_deg = 0.0f; break;
                case 'h': case 'H': case 'p': case 'P': case 'r': case 'R':
                    which = (uint8_t)((k | 0x20)); entering = 1; nbuf = 0;
                    con_queuef("%s = ", which == 'h' ? "heave (mm)" : (which == 'p' ? "pitch (deg)" : "roll (deg)"));
                    continue;
                default: continue;
            }
        }

        want.heave_mm  = clampf(want.heave_mm,  -0.5f * caps->heave_range_mm, 0.5f * caps->heave_range_mm);
        want.pitch_deg = clampf(want.pitch_deg, -caps->pitch_cap_deg, caps->pitch_cap_deg);
        want.roll_deg  = clampf(want.roll_deg,  -caps->roll_cap_deg,  caps->roll_cap_deg);

        have_prev = 0;
        keep = app_drive_pose(&center, &want, prev, &have_prev);
        Pose got = { want.heave_mm * keep, want.pitch_deg * keep, want.roll_deg * keep };
        if (keep < 0.0f) got = want;
        report(&got, keep);
    }
}
