/**
 * mode_setup.c - Mode 1: homing, setting the minimums (one leg at a time, until
 * the platform is level), the maximum heave (all legs together), speed limits,
 * and saving everything to flash.
 */
#include "plat_app.h"
#include "plat_hw.h"
#include "plat_engine.h"
#include "plat_cal.h"
#include <stdio.h>
#include <math.h>

static const char NAME[NUM_LEGS] = { 'X', 'Y', 'Z' };
static const uint32_t INC_CDEG[] = { 5, 10, 25, 50, 100, 250, 500 };    /* hundredths of a crank degree */
#define NUM_INC ((int)(sizeof(INC_CDEG) / sizeof(INC_CDEG[0])))
static int inc_idx = 3;                                                  /* 0.50 deg */

static int32_t inc_steps(void)
{
    int32_t s = (int32_t)((INC_CDEG[inc_idx] * STEPS_PER_REV + 18000u) / 36000u);
    return (s < 1) ? 1 : s;
}

static void bump_inc(int up)
{
    if (up && inc_idx < NUM_INC - 1) inc_idx++;
    if (!up && inc_idx > 0) inc_idx--;
}

static int32_t tgt_round(int i) { float t = engine_target(i); return (int32_t)(t + (t >= 0 ? 0.5f : -0.5f)); }

/* A jog key moves a target; it may never put the target more than JOG_LEAD_STEPS ahead of where the
 * leg really is. A burst of keys (key repeat, a laggy link) therefore cannot build up a long move
 * that keeps going after the keys stop. */
static float lead_clamp(int leg, float t)
{
    float p = (float)engine_pos(leg);
    if (t > p + (float)JOG_LEAD_STEPS) t = p + (float)JOG_LEAD_STEPS;
    if (t < p - (float)JOG_LEAD_STEPS) t = p - (float)JOG_LEAD_STEPS;
    return t;
}

static void jog_leg(int leg, int32_t delta)
{
    float t[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) t[i] = engine_target(i);
    t[leg] = lead_clamp(leg, t[leg] + (float)delta);
    engine_set_target(t, 0);
}

static void jog_all(int32_t delta)
{
    float t[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) t[i] = lead_clamp(i, engine_target(i) + (float)delta);
    engine_set_target(t, 0);
}

static void goto_steps(const int32_t v[NUM_LEGS])
{
    float t[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) t[i] = (float)v[i];
    engine_set_target(t, 0);
}

static void show(int sel)
{
    uint32_t c = INC_CDEG[inc_idx];
    con_queuef("[%c] X=%ld Y=%ld Z=%ld | step %lu.%02lu deg (%ld steps)\r\n", NAME[sel],
               (long)tgt_round(0), (long)tgt_round(1), (long)tgt_round(2),
               (unsigned long)(c / 100u), (unsigned long)(c % 100u), (long)inc_steps());
}

static void show_cal(void)
{
    con_queuef("Minimums: X=%ld Y=%ld Z=%ld   Maximums: X=%ld Y=%ld Z=%ld\r\n",
               (long)g_cal.min[0], (long)g_cal.min[1], (long)g_cal.min[2],
               (long)g_cal.max[0], (long)g_cal.max[1], (long)g_cal.max[2]);
}

/* ---- 1: minimums, one leg at a time --------------------------------------------------- */
static void phase_min(void)
{
    int sel = 0;
    con_print("\r\n--- Set MINIMUMS (platform level, at the bottom) ---\r\n"
              "x / y / z = select leg      d / a = selected leg up / down\r\n"
              "u / j = all three up / down together     + / - = bigger / smaller step\r\n"
              "m = mark selected leg's MIN here      n = mark all three legs' MIN here\r\n"
              "g = go to the saved minimums          p = print values      q = done\r\n");
    show(sel);
    con_flush_rx();
    engine_set_spread(g_cal.min, 0);            /* the minimums are being redefined: no leg difference guard here */

    while (1)
    {
        int k = app_poll();
        if (app_check_fault()) return;
        app_tele(MODE_SETUP);
        if (k < 0) continue;

        switch (k)
        {
            case 'x': case 'X': sel = 0; show(sel); break;
            case 'y': case 'Y': sel = 1; show(sel); break;
            case 'z': case 'Z': sel = 2; show(sel); break;
            case 'd': case 'D': jog_leg(sel,  inc_steps()); show(sel); break;
            case 'a': case 'A': jog_leg(sel, -inc_steps()); show(sel); break;
            case 'u': case 'U': jog_all( inc_steps()); show(sel); break;
            case 'j': case 'J': jog_all(-inc_steps()); show(sel); break;
            case '+': case '=': bump_inc(1); show(sel); break;
            case '-': case '_': bump_inc(0); show(sel); break;
            case 'm': case 'M': g_cal.min[sel] = tgt_round(sel); show_cal(); break;
            case 'n': case 'N': for (int i = 0; i < NUM_LEGS; i++) g_cal.min[i] = tgt_round(i); show_cal(); break;
            case 'g': case 'G': goto_steps(g_cal.min); show(sel); break;
            case 'p': case 'P': show_cal(); break;
            case 'q': case 'Q': engine_set_spread(g_cal.min, g_cal.max_diff); return;
            default: break;
        }
    }
}

/* ---- 2: maximum, all legs together ------------------------------------------------------ */
static void phase_max(void)
{
    con_print("\r\n--- Set MAXIMUM heave (all three legs together) ---\r\n"
              "First the legs go to the saved minimums so they rise together from level.\r\n"
              "u (or d) / j (or a) = up / down     + / - = bigger / smaller step\r\n"
              "m = mark MAX here (all three legs)  g = go back to the minimums\r\n"
              "p = print values                    q = done\r\n");
    goto_steps(g_cal.min);
    show(0);
    con_flush_rx();

    while (1)
    {
        int k = app_poll();
        if (app_check_fault()) return;
        app_tele(MODE_SETUP);
        if (k < 0) continue;

        switch (k)
        {
            case 'u': case 'U': case 'd': case 'D': jog_all( inc_steps()); show(0); break;
            case 'j': case 'J': case 'a': case 'A': jog_all(-inc_steps()); show(0); break;
            case '+': case '=': bump_inc(1); show(0); break;
            case '-': case '_': bump_inc(0); show(0); break;
            case 'm': case 'M':
            {
                int ok = 1;
                for (int i = 0; i < NUM_LEGS; i++) if (tgt_round(i) < g_cal.min[i] + 500) ok = 0;
                if (!ok) { con_queue("Too close to the minimums - raise the platform first.\r\n"); break; }
                for (int i = 0; i < NUM_LEGS; i++) g_cal.max[i] = tgt_round(i);
                show_cal();
                break;
            }
            case 'g': case 'G': goto_steps(g_cal.min); show(0); break;
            case 'p': case 'P': show_cal(); break;
            case 'q': case 'Q': return;
            default: break;
        }
    }
}

/* ---- 4: tilt limits, found by hand ---------------------------------------------------------- */
static void show_limits(void)
{
    char a[20], b[20], c[20], d[20];
    fmt_f(a, sizeof a, (float)g_cal.pitch_pos_cdeg / 100.0f, 2); fmt_f(b, sizeof b, (float)g_cal.pitch_neg_cdeg / 100.0f, 2);
    fmt_f(c, sizeof c, (float)g_cal.roll_pos_cdeg  / 100.0f, 2); fmt_f(d, sizeof d, (float)g_cal.roll_neg_cdeg  / 100.0f, 2);
    con_queuef("Tilt limits: pitch +%s / -%s deg, roll +%s / -%s deg   (0.00 = not set)\r\n", a, b, c, d);
}

static const float TILT_H[] = { 2.0f, 5.0f, 10.0f, 20.0f };
static const float TILT_A[] = { 0.25f, 0.5f, 1.0f, 2.0f };

static void phase_tilt(void)
{
    if (!g_kin_ok) { con_print("The geometry is not usable - cannot move in pitch and roll.\r\n"); return; }

    app_apply_cal_ex(0);                              /* only the leg ranges limit the pose while searching */
    engine_set_spread(g_cal.min, g_cal.max_diff);     /* but keep the leg difference guard if it is set      */
    const KinCaps *caps = kin_caps();
    Pose center = { caps->heave_mid_mm, 0.0f, 0.0f }, want = { 0.0f, 0.0f, 0.0f };
    float prev[NUM_LEGS];
    int have_prev = 0, si = 1;

    con_print("\r\n--- Find the TILT limits by hand ---\r\n"
              "Tilt the platform until it is as far as you are willing to go, then mark it.\r\n"
              "w / s = pitch + / -     d / a = roll + / -     u / j = heave up / down\r\n"
              "+ / - = bigger / smaller steps     z = level again (keeps the height)\r\n"
              "p = mark the PITCH limit here (+ or - side, whichever way it is tilted now)\r\n"
              "r = mark the ROLL limit here        c = clear all four limits      l = list them\r\n"
              "q = done.   The limits hold at every height, so find them at the height where the\r\n"
              "platform can tilt least, and keep clear of every link and the frame.\r\n");
    show_limits();
    app_drive_pose(&center, &want, prev, &have_prev);
    if (!app_arrive(30000u, 1)) return;               /* up to the middle of the stroke first (q / x = give up) */
    con_flush_rx();

    int ret = 0;
    while (!ret)
    {
        int k = app_poll();
        if (app_check_fault()) return;
        app_tele(MODE_SETUP);
        if (k < 0) continue;

        float hs = TILT_H[si], as = TILT_A[si];
        int moved = 1;
        switch (k)
        {
            case 'w': case 'W': want.pitch_deg += as; break;
            case 's': case 'S': want.pitch_deg -= as; break;
            case 'd': case 'D': want.roll_deg  += as; break;
            case 'a': case 'A': want.roll_deg  -= as; break;
            case 'u': case 'U': want.heave_mm  += hs; break;
            case 'j': case 'J': want.heave_mm  -= hs; break;
            case 'z': case 'Z': want.pitch_deg = want.roll_deg = 0.0f; break;
            case '+': case '=': if (si < 3) si++; con_queuef("step: %ld mm / %ld.%02ld deg\r\n", (long)TILT_H[si], (long)TILT_A[si], (long)(TILT_A[si] * 100.0f) % 100); moved = 0; break;
            case '-': case '_': if (si > 0) si--; con_queuef("step: %ld mm / %ld.%02ld deg\r\n", (long)TILT_H[si], (long)TILT_A[si], (long)(TILT_A[si] * 100.0f) % 100); moved = 0; break;
            case 'l': case 'L': show_limits(); moved = 0; break;
            case 'c': case 'C':
                g_cal.pitch_pos_cdeg = g_cal.pitch_neg_cdeg = g_cal.roll_pos_cdeg = g_cal.roll_neg_cdeg = 0;
                con_queue("Tilt limits cleared (not saved yet).\r\n"); moved = 0; break;
            case 'p': case 'P': case 'r': case 'R':
            {
                int is_pitch = (k == 'p' || k == 'P');
                app_settle(8000u);                    /* let it come to rest, then read where the legs really are */
                float s3[NUM_LEGS];
                for (int i = 0; i < NUM_LEGS; i++) s3[i] = (float)engine_pos(i);
                Pose got = want;
                if (kin_steps_to_pose(s3, &got) != 0) { con_queue("Could not work out the tilt from the legs - nothing marked.\r\n"); moved = 0; break; }
                float v = is_pitch ? got.pitch_deg : got.roll_deg;
                int32_t cd = (int32_t)(fabsf(v) * 100.0f + 0.5f);
                if (cd < MIN_TILT_CDEG)
                {
                    char mv[14];
                    fmt_f(mv, sizeof mv, v, 2);
                    con_queuef("Tilt it further first (at least 1 degree; it measures %s deg now).\r\n", mv);
                    moved = 0; break;
                }
                if (cd > 9000) cd = 9000;
                if (is_pitch) { if (v > 0.0f) g_cal.pitch_pos_cdeg = cd; else g_cal.pitch_neg_cdeg = cd; }
                else          { if (v > 0.0f) g_cal.roll_pos_cdeg  = cd; else g_cal.roll_neg_cdeg  = cd; }
                con_queue(is_pitch ? "Pitch limit marked.\r\n" : "Roll limit marked.\r\n");
                show_limits();
                moved = 0;
                break;
            }
            case 'q': case 'Q': ret = 1; moved = 0; break;
            default: moved = 0; break;
        }
        if (!moved) continue;

        want.heave_mm  = fmaxf(-0.5f * caps->heave_range_mm, fminf(0.5f * caps->heave_range_mm, want.heave_mm));
        want.pitch_deg = fmaxf(-caps->pitch_cap_deg, fminf(caps->pitch_cap_deg, want.pitch_deg));
        want.roll_deg  = fmaxf(-caps->roll_cap_deg,  fminf(caps->roll_cap_deg,  want.roll_deg));
        have_prev = 0;
        float keep = app_drive_pose(&center, &want, prev, &have_prev);
        char h[14], pp[14], rr[14];
        fmt_f(h, sizeof h, want.heave_mm * (keep < 0.0f ? 1.0f : keep), 1);
        fmt_f(pp, sizeof pp, want.pitch_deg * (keep < 0.0f ? 1.0f : keep), 2);
        fmt_f(rr, sizeof rr, want.roll_deg * (keep < 0.0f ? 1.0f : keep), 2);
        con_queuef("heave %s mm (0 = mid) | pitch %s deg | roll %s deg%s\r\n", h, pp, rr,
                   (keep >= 0.0f && keep < 0.999f) ? "  <-- held by the leg ranges" : "");
    }

    /* back to level at the middle before the limits switch on */
    want.pitch_deg = want.roll_deg = 0.0f;
    have_prev = 0;
    app_drive_pose(&center, &want, prev, &have_prev);
    (void)app_arrive(30000u, 0);
    engine_set_spread(g_cal.min, g_cal.max_diff);
}

/* ---- 5: the largest difference between two legs ------------------------------------------------ */
static void show_diff(int sel)
{
    int32_t r[NUM_LEGS], lo = 1000000, hi = -1000000;
    for (int i = 0; i < NUM_LEGS; i++)
    {
        r[i] = tgt_round(i) - g_cal.min[i];
        if (r[i] < lo) lo = r[i];
        if (r[i] > hi) hi = r[i];
    }
    long d = (long)(hi - lo);
    char lim[40];
    if (g_cal.max_diff) snprintf(lim, sizeof lim, "%ld steps", (long)g_cal.max_diff); else snprintf(lim, sizeof lim, "not set");
    con_queuef("[%c] heights above the minimums: X=%ld Y=%ld Z=%ld | difference now %ld steps (%ld.%ld deg) | saved limit: %s\r\n",
               NAME[sel], (long)r[0], (long)r[1], (long)r[2], d, d * 18 / 1000, (d * 18 / 100) % 10, lim);
}

static void phase_diff(void)
{
    int sel = 0;
    con_print("\r\n--- Set the largest DIFFERENCE between two legs ---\r\n"
              "The legs go to the minimums first. Then raise ONE leg (the others stay down) until the\r\n"
              "platform is as twisted as you are willing to allow, and mark it.\r\n"
              "x / y / z = select leg     d / a = selected leg up / down     u / j = all three\r\n"
              "+ / - = bigger / smaller step     m = mark the difference shown as the LIMIT\r\n"
              "c = clear the limit     g = back to the minimums     p = print     q = done (legs return to the minimums)\r\n");
    engine_set_spread(g_cal.min, 0);
    goto_steps(g_cal.min);
    if (!app_arrive(30000u, 1)) { engine_set_spread(g_cal.min, g_cal.max_diff); return; }
    show_diff(sel);
    con_flush_rx();

    while (1)
    {
        int k = app_poll();
        if (app_check_fault()) return;
        app_tele(MODE_SETUP);
        if (k < 0) continue;

        switch (k)
        {
            case 'x': case 'X': sel = 0; show_diff(sel); break;
            case 'y': case 'Y': sel = 1; show_diff(sel); break;
            case 'z': case 'Z': sel = 2; show_diff(sel); break;
            case 'd': case 'D': jog_leg(sel,  inc_steps()); show_diff(sel); break;
            case 'a': case 'A': jog_leg(sel, -inc_steps()); show_diff(sel); break;
            case 'u': case 'U': jog_all( inc_steps()); show_diff(sel); break;
            case 'j': case 'J': jog_all(-inc_steps()); show_diff(sel); break;
            case '+': case '=': bump_inc(1); show_diff(sel); break;
            case '-': case '_': bump_inc(0); show_diff(sel); break;
            case 'g': case 'G': goto_steps(g_cal.min); show_diff(sel); break;
            case 'p': case 'P': show_diff(sel); break;
            case 'c': case 'C': g_cal.max_diff = 0; con_queue("Leg difference limit cleared (not saved yet).\r\n"); show_diff(sel); break;
            case 'm': case 'M':
            {
                app_settle(8000u);
                int32_t lo = 1000000, hi = -1000000;
                for (int i = 0; i < NUM_LEGS; i++)
                {
                    int32_t r = engine_pos(i) - g_cal.min[i];
                    if (r < lo) lo = r;
                    if (r > hi) hi = r;
                }
                if (hi - lo < MIN_SPREAD_STEPS) { con_queue("Raise one leg further first.\r\n"); break; }
                g_cal.max_diff = hi - lo;
                con_queuef("Leg difference limit marked: %ld steps.\r\n", (long)g_cal.max_diff);
                show_diff(sel);
                break;
            }
            case 'q': case 'Q':
                goto_steps(g_cal.min);
                (void)app_arrive(30000u, 0);
                engine_set_spread(g_cal.min, g_cal.max_diff);
                return;
            default: break;
        }
    }
}

/* ---- 3: speed / acceleration ---------------------------------------------------------------- */
static void show_motion(void)
{
    con_queuef("Speed limit %ld steps/s (%ld.%02ld deg/s of crank)   Acceleration limit %ld steps/s^2\r\n",
               (long)g_cal.vmax, (long)(g_cal.vmax * 18 / 1000), (long)((g_cal.vmax * 18 / 10) % 100),
               (long)g_cal.amax);
}

static void phase_motion(void)
{
    con_print("\r\n--- Motion limits ---\r\n"
              "These cap every mode. Tested on the rig: 1500 steps/s, 1500 steps/s^2.\r\n"
              "Raise them in small steps and watch the driver LEDs (red LED 7 blinking = following error).\r\n"
              "+ / - = speed +-100      ] / [ = acceleration +-250      q = done\r\n");
    show_motion();
    con_flush_rx();

    while (1)
    {
        int k = app_poll();
        if (k < 0) continue;
        switch (k)
        {
            case '+': case '=': if (g_cal.vmax + 100 <= VMAX_LIMIT) g_cal.vmax += 100; show_motion(); break;
            case '-': case '_': if (g_cal.vmax - 100 >= 100)        g_cal.vmax -= 100; show_motion(); break;
            case ']':           if (g_cal.amax + 250 <= AMAX_LIMIT) g_cal.amax += 250; show_motion(); break;
            case '[':           if (g_cal.amax - 250 >= 250)        g_cal.amax -= 250; show_motion(); break;
            case 'q': case 'Q': return;
            default: break;
        }
        engine_set_motion((float)g_cal.vmax, (float)g_cal.amax);
    }
}

/* ---- the setup menu ---------------------------------------------------------------------------- */
static void print_setup_menu(void)
{
    con_print("\r\n=== SETUP ===\r\n"
              " 1  set MINIMUMS (jog each leg until the platform is level)\r\n"
              " 2  set MAXIMUM (jog all three legs together)\r\n"
              " 3  motion limits (speed, acceleration)\r\n"
              " 4  TILT limits (find the largest pitch and roll by hand)\r\n"
              " 5  LEG DIFFERENCE limit (largest difference between two legs)\r\n"
              " s  SAVE to flash      d  restore defaults (not saved)      h  re-home      q  back\r\n"
              "Select: ");
}

void mode_setup(void)
{
    if (!app_ensure_homed()) return;

    /* open the limits so the legs can be jogged outside the saved range */
    int32_t lo[NUM_LEGS], hi[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) { lo[i] = TRAVEL_FLOOR_STEPS; hi[i] = TRAVEL_CEIL_STEPS; }
    engine_set_limits(lo, hi);

    show_cal();
    print_setup_menu();
    while (1)
    {
        int k = app_poll();
        if (app_check_fault()) break;
        app_tele(MODE_SETUP);
        if (k < 0) continue;

        if (k == 'q' || k == 'Q') break;
        switch (k)
        {
            case '1': phase_min();    if (!g_homed) goto out; print_setup_menu(); break;
            case '2': phase_max();    if (!g_homed) goto out; print_setup_menu(); break;
            case '3': phase_motion(); print_setup_menu(); break;
            case '4': phase_tilt();   engine_set_limits(lo, hi); if (!g_homed) goto out; print_setup_menu(); break;
            case '5': phase_diff();   if (!g_homed) goto out; print_setup_menu(); break;
            case 's': case 'S':
            {
                if (!cal_valid(&g_cal)) { con_queue("Not saved: the values are not valid (max must be well above min).\r\n"); break; }
                int rc = cal_save();
                con_queuef(rc == 0 ? "Saved to flash. The platform will remember these after a power cycle.\r\n"
                                   : "SAVE FAILED (code %d).\r\n", rc);
                break;
            }
            case 'd': case 'D': cal_defaults(); show_cal(); con_queue("Defaults restored (not saved).\r\n"); break;
            case 'h': case 'H':
                if (g_homed && !app_lower_to_rest(0)) break;         /* lower all legs together first */
                g_homed = 0;
                app_apply_cal();
                if (!app_ensure_homed()) goto out;
                engine_set_limits(lo, hi);
                break;
            default: break;
        }
    }

out:
    /* bring the legs inside the (possibly changed) range before the limits tighten */
    if (g_homed)
    {
        float t[NUM_LEGS];
        for (int i = 0; i < NUM_LEGS; i++)
        {
            t[i] = engine_target(i);
            if (t[i] < (float)g_cal.min[i]) t[i] = (float)g_cal.min[i];
            if (t[i] > (float)g_cal.max[i]) t[i] = (float)g_cal.max[i];
        }
        engine_set_target(t, 0);
        uint32_t t0 = hw_ms();
        while (!engine_settled() && (hw_ms() - t0) < 20000u) app_idle();
    }
    app_apply_cal();
}
