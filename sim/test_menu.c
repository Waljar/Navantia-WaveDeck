/* Drives the whole firmware (menu, homing, setup, every mode) on the PC with the
 * virtual-clock hardware model. Keys are typed when the expected text appears. */
#include "plat_app.h"
#include "plat_cal.h"
#include "plat_kin.h"
#include "plat_engine.h"
#include "plat_hw.h"
#include "hw_host.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("  FAIL @%.1fs: ", sim_now_ms() / 1000.0); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define NOTE(...) do { printf("  [%.1fs] ", sim_now_ms() / 1000.0); printf(__VA_ARGS__); printf("\n"); } while (0)

static void near_steps(const char *what, long a, long b, long b2, long c, long c2, long tol)
{
    (void)c; (void)c2;
    CHECK(labs(sim_real[0] - a) <= tol && labs(sim_real[1] - b) <= tol && labs(sim_real[2] - b2) <= tol,
          "%s: legs at %ld %ld %ld, wanted %ld %ld %ld", what, sim_real[0], sim_real[1], sim_real[2], a, b, b2);
}

/* ---- checks run between key presses ------------------------------------------------ */
static void chk_after_home(void)
{
    near_steps("after homing", 672, 700, 798, 0, 0, 2);
    CHECK(g_homed, "g_homed not set");
    CHECK(sim_seen_min[0] >= -2 && sim_seen_min[1] >= -2 && sim_seen_min[2] >= -2, "went past the switch: %ld %ld %ld", sim_seen_min[0], sim_seen_min[1], sim_seen_min[2]);
    CHECK(sim_pulses_bad[0] + sim_pulses_bad[1] + sim_pulses_bad[2] == 0, "narrow pulses");
    NOTE("homed OK, legs at %ld %ld %ld", sim_real[0], sim_real[1], sim_real[2]);
}
static void chk_z854(void) { near_steps("Z jogged +56", 672, 700, 854, 0, 0, 1); CHECK(g_cal.min[2] == 854, "min[2]=%ld", (long)g_cal.min[2]); }
static void chk_z798(void) { near_steps("Z back", 672, 700, 798, 0, 0, 1); CHECK(g_cal.min[2] == 798, "min[2]=%ld", (long)g_cal.min[2]); }
static void chk_max_start(void) { near_steps("max phase start", 672, 700, 798, 0, 0, 2); sim_key_gap_ms = 320.0; }   /* a person presses keys, they do not burst */
static void chk_max_reached(void)
{
    sim_key_gap_ms = 5.0;
    near_steps("max jogged", 8456, 8484, 8582, 0, 0, 1);
    CHECK(g_cal.max[0] == 8456 && g_cal.max[1] == 8484 && g_cal.max[2] == 8582, "max cal %ld %ld %ld", (long)g_cal.max[0], (long)g_cal.max[1], (long)g_cal.max[2]);
}
static void chk_back_to_menu(void)
{
    for (int i = 0; i < 3; i++) CHECK(sim_real[i] >= g_cal.min[i] - 1 && sim_real[i] <= g_cal.max[i] + 1, "leg %d outside range at menu: %ld", i, sim_real[i]);
}

static void within_limits(const char *what, int margin)
{
    for (int i = 0; i < 3; i++)
    {
        CHECK(sim_seen_min[i] >= g_cal.min[i] + margin - 2, "%s: leg %d went to %ld, minimum %ld (+%d margin)", what, i, sim_seen_min[i], (long)g_cal.min[i], margin);
        CHECK(sim_seen_max[i] <= g_cal.max[i] - margin + 2, "%s: leg %d went to %ld, maximum %ld (-%d margin)", what, i, sim_seen_max[i], (long)g_cal.max[i], margin);
    }
    CHECK(sim_pulses_bad[0] + sim_pulses_bad[1] + sim_pulses_bad[2] == 0, "%s: narrow pulses", what);
}
static void report_ranges(const char *what)
{
    NOTE("%s: leg range seen X %ld..%ld  Y %ld..%ld  Z %ld..%ld  (strokes %ld %ld %ld)", what,
         sim_seen_min[0], sim_seen_max[0], sim_seen_min[1], sim_seen_max[1], sim_seen_min[2], sim_seen_max[2],
         (long)(g_cal.max[0] - g_cal.min[0]), (long)(g_cal.max[1] - g_cal.min[1]), (long)(g_cal.max[2] - g_cal.min[2]));
}
static double frac_used(int leg) { return (double)(sim_seen_max[leg] - sim_seen_min[leg]) / (double)(g_cal.max[leg] - g_cal.min[leg]); }

static void reset_track(void) { sim_track_reset(); }

static long saved_pos[3];
static void save_pos(void) { for (int i = 0; i < 3; i++) saved_pos[i] = sim_real[i]; }
static void chk_still(void) { CHECK(sim_real[0] == saved_pos[0] && sim_real[1] == saved_pos[1] && sim_real[2] == saved_pos[2], "moved while paused: %ld %ld %ld -> %ld %ld %ld", saved_pos[0], saved_pos[1], saved_pos[2], sim_real[0], sim_real[1], sim_real[2]); }
static void chk_moving(void) { CHECK(sim_real[0] != saved_pos[0] || sim_real[1] != saved_pos[1], "did not move after resume"); }

static void chk_heave(void)
{
    within_limits("heave cycle", 0);
    report_ranges("heave cycle");
    for (int i = 0; i < 3; i++) CHECK(frac_used(i) > 0.995, "heave cycle leg %d used only %.3f of the stroke", i, frac_used(i));
    CHECK(strstr(sim_out, "cycle 1 done") != 0, "no 'cycle 1 done' message");
}
static void chk_circles1(void) { within_limits("circles", (int)FIT_MARGIN_STEPS); report_ranges("circles 1.5 deg"); for (int i = 0; i < 3; i++) CHECK(sim_seen_max[i] - sim_seen_min[i] > 20, "circles: leg %d barely moves", i); }
static void chk_circles2(void) { within_limits("circles+heave", (int)FIT_MARGIN_STEPS); report_ranges("circles + heave"); }

static void chk_ss(int ss)
{
    char w[32]; snprintf(w, sizeof w, "sea state %d", ss);
    within_limits(w, (int)FIT_MARGIN_STEPS);
    report_ranges(w);
}
static void chk_ss1(void) { chk_ss(1); reset_track(); }
static void chk_ss2(void) { chk_ss(2); reset_track(); }
static void chk_ss3(void) { chk_ss(3); reset_track(); }
static void chk_ss4(void) { chk_ss(4); reset_track(); }
static void chk_ss4b(void) { chk_ss(4); }
static void chk_max_waves(void)
{
    within_limits("MAX waves", 0);                 /* hard limits only: peaks are clipped by the pose fit */
    report_ranges("MAX waves");
    double best = 0; for (int i = 0; i < 3; i++) if (frac_used(i) > best) best = frac_used(i);
    CHECK(best > 0.55, "MAX waves: most-used leg only used %.2f of its stroke", best);
    reset_track();
}
static void chk_max_sweep(void)
{
    within_limits("MAX sweep", 0);
    report_ranges("MAX full-range sweep");
    for (int i = 0; i < 3; i++) CHECK(frac_used(i) > 0.80, "MAX sweep: leg %d used only %.2f of its stroke", i, frac_used(i));
}

/* manual mode: compare the real leg positions with the kinematics */
static void chk_pose(float h, float p, float r, const char *what)
{
    const KinCaps *c = kin_caps();
    Pose want = { c->heave_mid_mm + h, p, r };
    float st[3];
    int rc = kin_pose_to_steps(&want, st);
    CHECK(rc == 0, "%s: IK failed", what);
    if (rc == 0) CHECK(fabsf((float)sim_real[0] - st[0]) <= 1.01f && fabsf((float)sim_real[1] - st[1]) <= 1.01f && fabsf((float)sim_real[2] - st[2]) <= 1.01f,
                       "%s: legs %ld %ld %ld, IK wants %.1f %.1f %.1f", what, sim_real[0], sim_real[1], sim_real[2], st[0], st[1], st[2]);
    Pose got; float s2[3] = { (float)sim_real[0], (float)sim_real[1], (float)sim_real[2] };
    if (kin_steps_to_pose(s2, &got) == 0) NOTE("%s: FK of the real legs gives heave %.2f (mid+%.2f) pitch %.2f roll %.2f", what, got.heave_mm, got.heave_mm - c->heave_mid_mm, got.pitch_deg, got.roll_deg);
}
static void chk_m1(void) { chk_pose(12.5f, 0, 0, "manual h12.5"); }
static void chk_m2(void) { chk_pose(12.5f, -3, 0, "manual p-3"); }
static void chk_m3(void) { chk_pose(12.5f, -3, 2, "manual r2"); }
static void chk_m4(void) { chk_pose(0, 0, 0, "manual zero"); }
static void chk_m5(void) { within_limits("manual extremes", (int)FIT_MARGIN_STEPS); report_ranges("manual extremes"); }
static void chk_end_menu(void) { chk_back_to_menu(); CHECK(!strstr(sim_out, "FAILED"), "a FAILED message was printed"); CHECK(!strstr(sim_out, "HOME SWITCH"), "a home switch fired"); CHECK(!strstr(sim_out, "SAVE FAILED"), "save failed"); }

static void chk_rest(void) { near_steps("at rest", 672, 700, 798, 0, 0, 1); CHECK(g_homed, "lost the homed state"); }
static void chk_saved(void)
{
    CHECK(strstr(sim_out, "Saved to flash") != 0, "no save confirmation");
}

/* ---- scenario 1: a complete evening ---------------------------------------------------- */
static const SimStep SCENARIO1[] = {
    { "Select: ",                      200, 0,              "1" },
    { "Press any key to start homing", 200, 0,              "g" },
    { "=== SETUP ===",                 500, chk_after_home, "1" },
    { "--- Set MINIMUMS",              300, 0,              "zddm" },
    { "Z=854",                        2000, chk_z854,       "aam" },
    { "Z=798   Maximums",             1500, chk_z798,       "q" },
    { "=== SETUP ===",                 300, 0,              "2" },
    { "--- Set MAXIMUM",              4000, chk_max_start,  "+++uuuuuuuuuuuuuuuuuuuuuuuuuuuum" },
    { "Maximums: X=8456 Y=8484 Z=8582", 12000, chk_max_reached, "q" },
    { "=== SETUP ===",                 300, 0,              "3" },
    { "--- Motion limits",             200, 0,              "+-q" },
    { "=== SETUP ===",                 300, 0,              "s" },
    { "Saved to flash",                300, chk_saved,      "q" },
    { "SEA PLATFORM",                  500, chk_back_to_menu, "2" },

    /* heave cycle */
    { "=== HEAVE CYCLE",              9000, reset_track,    "+++" },     /* the legs first come down from the maximum */
    { "speed 5/7",                    45000, chk_heave,     "p" },
    { "Paused. p",                     4000, save_pos,      0 },
    { 0,                               3000, chk_still,     "p" },
    { "Resumed.",                      6000, chk_moving,    "q" },

    /* circles */
    { "SEA PLATFORM",                  500, chk_back_to_menu, "3" },
    { "=== PITCH / ROLL CIRCLES",     6000, reset_track,    "++" },
    { "tilt circle 1.50",             32000, chk_circles1,  "h" },
    { "heave motion ON",              45000, chk_circles2,  "q" },

    /* waves */
    { "SEA PLATFORM",                  500, chk_back_to_menu, "4" },
    { "=== WAVES",                     6000, reset_track,   "1" },
    { "Sea state 1",                  30000, chk_ss1,       "2" },
    { "Sea state 2",                  40000, chk_ss2,       "3" },
    { "Sea state 3",                  60000, chk_ss3,       "4" },
    { "Sea state 4",                  90000, chk_ss4,       "r" },
    { "Sea state 4",                  90000, chk_ss4b,      "q" },

    /* MAX platform */
    { "SEA PLATFORM",                  500, chk_back_to_menu, "5" },
    { "=== MAX PLATFORM",              6000, reset_track,   0 },
    { 0,                              60000, chk_max_waves, "d" },
    { "Demo sweep",                   85000, chk_max_sweep, "q" },

    /* manual */
    { "SEA PLATFORM",                  500, chk_back_to_menu, "6" },
    { "=== MANUAL",                   3000, 0,              "h12.5\r" },
    { "heave 12.5 mm",                4000, chk_m1,         "p-3\r" },
    { "pitch -3.00 deg",              4000, chk_m2,         "r2\r" },
    { "roll 2.00 deg",                4000, chk_m3,         "z" },
    { "heave 0.0 mm",                 4000, chk_m4,         "h2000\rp80\rr80\r" },
    { "LIMITED",                      5000, chk_m5,         "q" },
    { "SEA PLATFORM",                  500, chk_end_menu,   "g" },
    { "At rest.",                      500, chk_rest,       0 },
};


/* ---- scenario 3: the safety features ----------------------------------------------------------
 * Power-on homing with the legs far apart, a flood of jog keys, finding the tilt and leg-difference
 * limits by hand, saving them, every mode running inside them, a lost character while typing a
 * number, and re-homing from a raised platform. */
static int samp_on = 0;
static double samp_last = 0;
static float s_pmin, s_pmax, s_rmin, s_rmax;
static long s_dmax;
static int s_n;
static void sample_reset(void) { s_pmin = s_rmin = 1e9f; s_pmax = s_rmax = -1e9f; s_dmax = 0; s_n = 0; samp_last = sim_now_ms(); }
static void sampler(void)
{
    if (!samp_on) return;
    double t = sim_now_ms();
    if (t - samp_last < 20.0) return;
    samp_last = t;
    float s3[3] = { (float)sim_real[0], (float)sim_real[1], (float)sim_real[2] };
    Pose p = { kin_caps()->heave_mid_mm, 0.0f, 0.0f };
    if (kin_steps_to_pose(s3, &p) == 0)
    {
        if (p.pitch_deg < s_pmin) s_pmin = p.pitch_deg;
        if (p.pitch_deg > s_pmax) s_pmax = p.pitch_deg;
        if (p.roll_deg < s_rmin) s_rmin = p.roll_deg;
        if (p.roll_deg > s_rmax) s_rmax = p.roll_deg;
        s_n++;
    }
    long lo = 1000000, hi = -1000000;
    for (int i = 0; i < 3; i++) { long r = sim_real[i] - g_cal.min[i]; if (r < lo) lo = r; if (r > hi) hi = r; }
    if (hi - lo > s_dmax) s_dmax = hi - lo;
}
static void sample_start(void) { sample_reset(); sim_track_reset(); samp_on = 1; }
static void sample_stop(void) { samp_on = 0; }

static int real_pose(Pose *p)
{
    float s3[3] = { (float)sim_real[0], (float)sim_real[1], (float)sim_real[2] };
    p->heave_mm = kin_caps()->heave_mid_mm; p->pitch_deg = p->roll_deg = 0.0f;
    return kin_steps_to_pose(s3, p);
}

static long spread0;
static long burst_from[3];
static void slow_keys(void) { sim_key_gap_ms = 320.0; }
static void mid_keys(void) { sim_key_gap_ms = 100.0; }
static void track_reset3(void) { sim_track_reset(); spread0 = sim_spread_max; }

static void chk3_power_on_homing(void)
{
    NOTE("power-on homing from legs 3000/3500/4000: largest leg difference while lowering %ld steps", sim_spread_max);
    CHECK(sim_spread_max <= 1000 + (long)HOME_CHUNK_STEPS + 20, "homing lowered the legs one after the other: leg difference reached %ld (started at 1000)", sim_spread_max);
}
static void chk3_after_home(void)
{
    near_steps("after homing", 672, 700, 798, 0, 0, 2);
    CHECK(g_homed, "g_homed not set");
    CHECK(sim_seen_min[0] >= -2 && sim_seen_min[1] >= -2 && sim_seen_min[2] >= -2, "went past the switch: %ld %ld %ld", sim_seen_min[0], sim_seen_min[1], sim_seen_min[2]);
}
static void chk3_burst_start(void) { for (int i = 0; i < 3; i++) burst_from[i] = sim_real[i]; sim_key_gap_ms = 5.0; }
static void chk3_burst_done(void)
{
    long moved = sim_real[0] - burst_from[0];
    NOTE("40 jog keys in 200 ms with the largest step: legs moved %ld steps (a key would add up to %ld)", moved, 40L * 278L);
    CHECK(moved > 100, "the jog burst did not move the legs at all (%ld)", moved);
    CHECK(moved < 600 + 400, "a burst of jog keys ran the leg %ld steps (lead limit is %d)", moved, (int)JOG_LEAD_STEPS);
}
static void chk3_at_min(void) { near_steps("back at the minimums", 672, 700, 798, 0, 0, 1); }

static void chk3_ppos(void) { NOTE("pitch + marked: %ld", (long)g_cal.pitch_pos_cdeg); CHECK(g_cal.pitch_pos_cdeg >= 780 && g_cal.pitch_pos_cdeg <= 820, "pitch + limit %ld, wanted about 800", (long)g_cal.pitch_pos_cdeg); }
static void chk3_pneg(void) { NOTE("pitch - marked: %ld", (long)g_cal.pitch_neg_cdeg); CHECK(g_cal.pitch_neg_cdeg >= 780 && g_cal.pitch_neg_cdeg <= 820, "pitch - limit %ld, wanted about 800", (long)g_cal.pitch_neg_cdeg); CHECK(g_cal.pitch_pos_cdeg >= 780 && g_cal.pitch_pos_cdeg <= 820, "pitch + limit lost"); }
static void chk3_rpos(void) { NOTE("roll + marked: %ld", (long)g_cal.roll_pos_cdeg); CHECK(g_cal.roll_pos_cdeg >= 780 && g_cal.roll_pos_cdeg <= 820, "roll + limit %ld, wanted about 800", (long)g_cal.roll_pos_cdeg); }
static void chk3_rneg(void) { NOTE("roll - marked: %ld", (long)g_cal.roll_neg_cdeg); CHECK(g_cal.roll_neg_cdeg >= 580 && g_cal.roll_neg_cdeg <= 620, "roll - limit %ld, wanted about 600", (long)g_cal.roll_neg_cdeg); }
static void chk3_tilt_done(void)
{
    CHECK(strstr(sim_out, "Tilt limits: pitch +") != 0, "the limits were not listed");
    CHECK(!strstr(sim_out, "Tilt it further first"), "a mark was refused");
}
static void chk3_diff(void)
{
    long x = sim_real[0] - g_cal.min[0];
    NOTE("leg difference marked: %ld steps (leg X is %ld above its minimum)", (long)g_cal.max_diff, x);
    CHECK(g_cal.max_diff >= 200, "difference limit %ld too small", (long)g_cal.max_diff);
    CHECK(labs((long)g_cal.max_diff - x) <= 2, "difference limit %ld but leg X is %ld above its minimum", (long)g_cal.max_diff, x);
}
static void chk3_saved(void)
{
    CHECK(strstr(sim_out, "Saved to flash") != 0, "no save confirmation");
    CHECK(g_cal.pitch_pos_cdeg > 0 && g_cal.max_diff >= 200, "limits missing after leaving Setup");
}

#define PTOL 0.5f
static void chk3_pose(const char *what, float p_lo, float p_hi, float r_lo, float r_hi)
{
    Pose p;
    CHECK(real_pose(&p) == 0, "%s: could not work out the pose", what);
    NOTE("%s: pitch %.2f roll %.2f heave %.1f", what, p.pitch_deg, p.roll_deg, p.heave_mm - kin_caps()->heave_mid_mm);
    CHECK(p.pitch_deg >= p_lo - PTOL && p.pitch_deg <= p_hi + PTOL, "%s: pitch %.2f outside %.2f..%.2f", what, p.pitch_deg, p_lo, p_hi);
    CHECK(p.roll_deg >= r_lo - PTOL && p.roll_deg <= r_hi + PTOL, "%s: roll %.2f outside %.2f..%.2f", what, p.roll_deg, r_lo, r_hi);
}
static void chk3_m_p(void)  { chk3_pose("manual p80",  7.0f, 8.0f, -0.1f, 0.1f); }
static void chk3_m_pn(void) { chk3_pose("manual p-80", -8.0f, -7.0f, -0.1f, 0.1f); }
static void chk3_m_rp(void) { chk3_pose("manual r80",  -0.1f, 0.1f, 5.0f, 8.0f); }
static void chk3_m_rn(void) { chk3_pose("manual r-80", -0.1f, 0.1f, -6.0f, -5.0f); }
static void check_limits(const char *what, int need_motion, int legs);
static void chk3_m_all(void)
{
    chk3_pose("manual all at once", -8.0f, 8.0f, -6.0f, 8.0f);
    check_limits("manual mode, keys and typed values", 0, 0);
}
static void check_limits(const char *what, int need_motion, int legs)
{
    float lp = (float)g_cal.pitch_pos_cdeg / 100.0f, ln = (float)g_cal.pitch_neg_cdeg / 100.0f, lr = (float)g_cal.roll_pos_cdeg / 100.0f, lrn = (float)g_cal.roll_neg_cdeg / 100.0f;
    NOTE("%s: pitch %.2f..%.2f (limits +%.1f/-%.1f), roll %.2f..%.2f (limits +%.1f/-%.1f), leg difference %ld (limit %ld), %d samples",
         what, s_pmin, s_pmax, lp, ln, s_rmin, s_rmax, lr, lrn, s_dmax, (long)g_cal.max_diff, s_n);
    CHECK(s_n > 100, "%s: too few samples (%d)", what, s_n);
    CHECK(s_pmax <= lp + PTOL && s_pmin >= -ln - PTOL, "%s: pitch %.2f..%.2f broke the limits +%.1f / -%.1f", what, s_pmin, s_pmax, lp, ln);
    CHECK(s_rmax <= lr + PTOL && s_rmin >= -lrn - PTOL, "%s: roll %.2f..%.2f broke the limits +%.1f / -%.1f", what, s_rmin, s_rmax, lr, lrn);
    CHECK(s_dmax <= (long)g_cal.max_diff + (long)SPREAD_TOL_STEPS + 15, "%s: legs differed by %ld steps, limit %ld", what, s_dmax, (long)g_cal.max_diff);
    if (need_motion) CHECK((s_pmax - s_pmin) + (s_rmax - s_rmin) > 1.5f, "%s: the platform hardly tilted (pitch %.2f roll %.2f range)", what, s_pmax - s_pmin, s_rmax - s_rmin);
    if (legs) within_limits(what, 0);
}
static void chk3_circles(void) { check_limits("circles under the limits", 1, 1); sample_stop(); }
static void chk3_waves(void)   { check_limits("sea state 4 under the limits", 1, 1); sample_stop(); }
static void chk3_max(void)     { check_limits("MAX waves under the limits", 1, 1); sample_stop(); }

static long lost_from[3];
static void chk3_manual_zero(void) { for (int i = 0; i < 3; i++) lost_from[i] = sim_real[i]; }
static void chk3_lose_input(void) { sim_rx_lost = 1; }
static void chk3_lost(void)
{
    CHECK(strstr(sim_out, "Input lost") != 0, "no 'Input lost' message");
    CHECK(sim_real[0] == lost_from[0] && sim_real[1] == lost_from[1] && sim_real[2] == lost_from[2],
          "a number typed with a lost character moved the legs: %ld %ld %ld -> %ld %ld %ld", lost_from[0], lost_from[1], lost_from[2], sim_real[0], sim_real[1], sim_real[2]);
}
static void chk3_high(void) { sample_stop(); NOTE("raised for re-homing: legs %ld %ld %ld", sim_real[0], sim_real[1], sim_real[2]); CHECK(sim_real[0] > 2500, "platform was not raised"); }
static void chk3_lowered_first(void)
{
    NOTE("re-home from a raised platform: lowered together, largest leg difference %ld steps (it started at %ld)", sim_spread_max, spread0);
    CHECK(strstr(sim_out, "Lowering all three legs together before homing") != 0, "no 'Lowering all three legs' message");
    CHECK(sim_spread_max <= spread0 + 60, "the legs came down one after the other: difference reached %ld (started at %ld)", sim_spread_max, spread0);
    CHECK(!g_homed, "homed flag still set while re-homing");
}
static void chk3_rehomed(void)
{
    near_steps("after re-homing", 672, 700, 798, 0, 0, 2);
    CHECK(g_homed, "not homed after re-homing");
    CHECK(sim_seen_min[0] >= -2 && sim_seen_min[1] >= -2 && sim_seen_min[2] >= -2, "went past the switch: %ld %ld %ld", sim_seen_min[0], sim_seen_min[1], sim_seen_min[2]);
}

static const SimStep SCENARIO3[] = {
    { "Select: ",                      200, 0,              "1" },
    { "Press any key to start homing", 200, track_reset3,   "g" },
    { "Homing X...",                     0, chk3_power_on_homing, 0 },
    { "=== SETUP ===",                 500, chk3_after_home, "1" },

    /* a flood of jog keys (key repeat, a laggy link) must not run away */
    { "--- Set MINIMUMS",              800, chk3_burst_start, "+++++++uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu" },
    { 0,                              5000, chk3_burst_done, "g" },
    { 0,                              7000, chk3_at_min,     "q" },

    /* find the tilt limits by hand: +8 / -8 pitch, +8 / -6 roll */
    { "=== SETUP ===",                 300, slow_keys,      "4" },
    { "--- Find the TILT",            9000, 0,              "++wwwwp" },
    { "Pitch limit marked.",           500, chk3_ppos,      "zssssp" },
    { "Pitch limit marked.",           500, chk3_pneg,      "zddddr" },
    { "Roll limit marked.",            500, chk3_rpos,      "zaaar" },
    { "Roll limit marked.",            500, chk3_rneg,      "zq" },
    { "=== SETUP ===",                 300, chk3_tilt_done, "5" },

    /* the largest difference between two legs: raise leg X alone by one step */
    { "--- Set the largest DIFFERENCE", 9000, 0,            "xd" },           /* one big step of leg X: about the same twist as 8 degrees of pitch */
    { 0,                              3000, 0,              "m" },
    { "Leg difference limit marked",   500, chk3_diff,      "q" },
    { "=== SETUP ===",                 300, 0,              "s" },
    { "Saved to flash",                300, chk3_saved,     "q" },
    { "SEA PLATFORM",                  500, mid_keys,       "6" },

    /* manual mode cannot go past the limits, in either direction */
    { "=== MANUAL",                   3000, sample_start,   "p80\r" },
    { 0,                              5000, chk3_m_p,       "p-80\r" },
    { 0,                              5000, chk3_m_pn,      "z" },
    { 0,                              4000, 0,              "r80\r" },
    { 0,                              5000, chk3_m_rp,      "r-80\r" },
    { 0,                              5000, chk3_m_rn,      "h2000\rp80\rr80\r" },
    { 0,                              6000, chk3_m_all,     "z" },

    /* a character lost on the way in: the half-typed number is thrown away */
    { 0,                              5000, chk3_manual_zero, "h" },
    { "heave (mm) = ",                 100, chk3_lose_input, "12.5\r" },
    { "Input lost",                   3000, chk3_lost,      "z" },
    { 0,                              3000, 0,              "q" },

    /* every mode keeps inside the limits */
    { "SEA PLATFORM",                  500, 0,              "3" },
    { "=== PITCH / ROLL CIRCLES",     6000, sample_start,   "+++++" },
    { "tilt circle",                  40000, chk3_circles,  "q" },
    { "SEA PLATFORM",                  500, 0,              "4" },
    { "=== WAVES",                     6000, sample_start,  "4" },
    { "Sea state 4",                  60000, chk3_waves,    "q" },
    { "SEA PLATFORM",                  500, 0,              "5" },
    { "=== MAX PLATFORM",              6000, sample_start, 0 },
    { 0,                              60000, chk3_max,      "q" },

    /* raised platform, then re-home: the legs come down together first */
    { "SEA PLATFORM",                  500, 0,              "6" },
    { "=== MANUAL",                   3000, 0,              "h150\r" },
    { 0,                              6000, chk3_high,      "q" },
    { "SEA PLATFORM",                  500, track_reset3,   "h" },
    { "Press any key to start homing", 300, chk3_lowered_first, "g" },
    { "Ready.",                        300, chk3_rehomed,   0 },
};

/* ---- scenario 4: presentation lock ----------------------------------------------------------- */
static void chk4_locked(void) { CHECK(g_locked, "lock not on"); }
static void chk4_refused(void) { CHECK(strstr(sim_out, "LOCKED: not available") != 0, "Setup was not refused"); CHECK(!strstr(sim_out, "=== SETUP"), "Setup opened while locked"); CHECK(g_locked, "lock lost"); }
static void chk4_still(void) { CHECK(g_locked && strstr(sim_out, "Still locked") != 0, "a wrong word unlocked it"); }
static void chk4_off(void) { CHECK(!g_locked && strstr(sim_out, "Presentation lock OFF") != 0, "UNLOCK did not unlock"); }
static const SimStep SCENARIO4[] = {
    { "Select: ",                      200, 0,          "l" },
    { "Presentation lock ON",          200, chk4_locked, "1" },
    { "LOCKED: not available",         200, 0,          "6" },
    { "LOCKED: not available",         200, 0,          "h" },
    { "LOCKED: not available",         200, chk4_refused, "l" },
    { "Type UNLOCK",                   200, 0,          "nope\r" },
    { "Still locked",                  200, chk4_still, "l" },
    { "Type UNLOCK",                   200, 0,          "UnLock\r" },
    { "Presentation lock OFF",         200, chk4_off,   0 },
};

/* ---- scenario 2: power cycle, the saved calibration must come back ---------------------------- */
static void chk_reboot(void)
{
    CHECK(g_cal.from_flash, "calibration not loaded from flash");
    CHECK(g_cal.max[0] == 8456 && g_cal.max[1] == 8484 && g_cal.max[2] == 8582 && g_cal.min[0] == 672 && g_cal.min[2] == 798, "wrong values after reboot");
    CHECK(strstr(sim_out, "saved in flash") != 0, "status line does not say 'saved in flash'");
    CHECK(g_cal.pitch_pos_cdeg >= 780 && g_cal.pitch_neg_cdeg >= 780 && g_cal.roll_pos_cdeg >= 780 && g_cal.roll_neg_cdeg >= 580 && g_cal.max_diff >= 200,
          "tilt / difference limits not restored: %ld %ld %ld %ld %ld", (long)g_cal.pitch_pos_cdeg, (long)g_cal.pitch_neg_cdeg, (long)g_cal.roll_pos_cdeg, (long)g_cal.roll_neg_cdeg, (long)g_cal.max_diff);
    CHECK(strstr(sim_out, "Tilt limits (set by hand)") != 0, "status line does not list the tilt limits");
    CHECK(strstr(sim_out, "Leg difference limit: ") != 0 && !strstr(sim_out, "Leg difference limit: NOT SET"), "status line does not show the leg difference limit");
    CHECK(g_kin_ok, "geometry invalid after reboot");
}
static const SimStep SCENARIO2[] = {
    { "Select: ", 200, chk_reboot, 0 },
};

static int run(const char *name, const SimStep *steps, int n, long s0, long s1, long s2, double end_ms)
{
    printf("[%s]\n", name);
    sim_reset(s0, s1, s2);
    sim_script(steps, n);
    sim_end_ms = end_ms;
    g_homed = 0; g_tele = 0;
    sim_exit_jmp_set = 1;
    if (setjmp(sim_exit_jmp) == 0) { app_init(); app_run(); }
    sim_exit_jmp_set = 0;
    return 0;
}

int main(int argc, char **argv)
{
    sim_echo = (argc > 1 && strcmp(argv[1], "-v") == 0);
    int only3 = (argc > 2 && strcmp(argv[2], "s3") == 0);          /* debugging: run just the safety scenario */
    int only1 = (argc > 2 && strcmp(argv[2], "s1") == 0);          /* debugging: run just scenario 1 */
    if (!only3) {
    run("scenario 1: setup, save, every mode", SCENARIO1, (int)(sizeof SCENARIO1 / sizeof SCENARIO1[0]), 3000, 3500, 4000, 2000000.0);
    if (sim_now_ms() > 1999000.0) { fails++; printf("  FAIL: scenario 1 did not finish (stuck?)\n"); }
    printf("  virtual time used: %.0f s\n", sim_now_ms() / 1000.0);
    if (!sim_echo) { /* show the console text around any failure */ }

    }
    if (only1) { printf("\n%s (%d failures)\n", fails ? "SOME MENU TESTS FAILED" : "ALL MENU TESTS PASSED", fails); return fails ? 1 : 0; }
    sim_poll_hook = sampler;
    run("scenario 3: safety limits, jog flood, lost input, re-homing", SCENARIO3, (int)(sizeof SCENARIO3 / sizeof SCENARIO3[0]), 3000, 3500, 4000, only3 ? 400000.0 : 2000000.0);
    if (sim_now_ms() > 1999000.0) { fails++; printf("  FAIL: scenario 3 did not finish (stuck?)\n"); }
    printf("  virtual time used: %.0f s\n", sim_now_ms() / 1000.0);
    sim_poll_hook = 0; samp_on = 0;

    if (!only3 && !only1) { g_locked = 0; run("scenario 4: presentation lock", SCENARIO4, (int)(sizeof SCENARIO4 / sizeof SCENARIO4[0]), 3000, 3500, 4000, 120000.0); }
    if (!only3) run("scenario 2: power cycle", SCENARIO2, 1, 3000, 3500, 4000, 60000.0);
    printf("\n%s (%d failures)\n", fails ? "SOME MENU TESTS FAILED" : "ALL MENU TESTS PASSED", fails);
    return fails ? 1 : 0;
}
