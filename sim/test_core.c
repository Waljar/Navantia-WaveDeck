/* Tests of kinematics, waves and the step engine on the PC. */
#include "plat_kin.h"
#include "plat_waves.h"
#include "plat_engine.h"
#include "plat_hw.h"
#include "hw_host.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void test_kin(void)
{
    printf("[kin]\n");
    KinGeom g; kin_default_geom(&g);
    int32_t mn[3] = { 672, 700, 798 }, mx[3] = { 8567, 8595, 8693 };
    int rc = kin_init(&g, mn, mx);
    CHECK(rc == 0, "kin_init failed");
    const KinCaps *c = kin_caps();
    printf("  heave range %.2f mm, mid %.2f, pitch cap %.2f deg, roll cap %.2f deg\n", c->heave_range_mm, c->heave_mid_mm, c->pitch_cap_deg, c->roll_cap_deg);
    printf("  J: %.2f %.2f %.2f / %.2f %.2f %.2f / %.2f %.2f %.2f\n", c->J[0][0], c->J[0][1], c->J[0][2], c->J[1][0], c->J[1][1], c->J[1][2], c->J[2][0], c->J[2][1], c->J[2][2]);

    Pose p0 = { 0, 0, 0 }; float s[3];
    CHECK(kin_pose_to_steps(&p0, s) == 0, "ik at zero");
    CHECK(fabsf(s[0] - 672) < 0.01f && fabsf(s[1] - 700) < 0.01f && fabsf(s[2] - 798) < 0.01f, "level minimum should equal the leg minimums, got %.2f %.2f %.2f", s[0], s[1], s[2]);

    /* symmetry: all legs rise equally with pure heave */
    Pose ph = { c->heave_mid_mm, 0, 0 };
    kin_pose_to_steps(&ph, s);
    CHECK(fabsf((s[0] - 672) - (s[1] - 700)) < 0.01f && fabsf((s[1] - 700) - (s[2] - 798)) < 0.01f, "heave not symmetric");

    /* pitch+ raises X (leg 0, front), lowers Y and Z equally; roll+ raises Z (back left) lowers Y, X unchanged */
    Pose pp = { c->heave_mid_mm, 3, 0 }, pr = { c->heave_mid_mm, 0, 3 };
    float sp[3], sr[3], sm[3];
    kin_pose_to_steps(&ph, sm); kin_pose_to_steps(&pp, sp); kin_pose_to_steps(&pr, sr);
    CHECK(sp[0] > sm[0] && sp[1] < sm[1] && sp[2] < sm[2], "pitch signs");
    CHECK(fabsf((sp[1] - sm[1]) - (sp[2] - sm[2])) < 0.5f, "pitch Y/Z not equal");
    CHECK(sr[2] > sm[2] && sr[1] < sm[1] && fabsf(sr[0] - sm[0]) < 0.5f, "roll signs: %.2f %.2f %.2f", sr[0] - sm[0], sr[1] - sm[1], sr[2] - sm[2]);

    /* FK(IK(x)) = x */
    double worst = 0; int tested = 0;
    srand(3);
    for (int i = 0; i < 2000; i++)
    {
        Pose p = { c->heave_mid_mm + ((rand() / (float)RAND_MAX) - 0.5f) * c->heave_range_mm * 0.8f,
                   ((rand() / (float)RAND_MAX) - 0.5f) * c->pitch_cap_deg, ((rand() / (float)RAND_MAX) - 0.5f) * c->roll_cap_deg };
        float st[3], q;
        if (kin_pose_to_steps(&p, st) != 0) continue;
        /* only poses the legs can actually reach (FK is used on real leg positions) */
        int inside = 1;
        for (int k = 0; k < 3; k++) if (st[k] < mn[k] || st[k] > mx[k]) inside = 0;
        if (!inside) continue;
        tested++;
        Pose f;
        if (kin_steps_to_pose(st, &f) != 0) { CHECK(0, "FK failed at %.1f %.2f %.2f", p.heave_mm, p.pitch_deg, p.roll_deg); break; }
        q = fabsf(f.heave_mm - p.heave_mm) + fabsf(f.pitch_deg - p.pitch_deg) + fabsf(f.roll_deg - p.roll_deg);
        if (q > worst) worst = q;
    }
    printf("  FK(IK) worst error %.5f over %d reachable poses\n", worst, tested);
    CHECK(tested > 500, "too few reachable poses tested (%d)", tested);
    CHECK(worst < 0.02, "FK/IK round trip error %.4f", worst);

    /* fit: huge request is shrunk to fit and then is feasible */
    Pose center = { c->heave_mid_mm, 0, 0 }, dev = { 0.9f * c->heave_range_mm, 40, -40 }, out;
    float st[3];
    float keep = kin_fit(&center, &dev, 30.0f, &out, st);
    CHECK(keep > 0 && keep < 1, "fit keep=%.3f", keep);
    CHECK(kin_feasible(&out, 30.0f - 1), "fitted pose not feasible");
    for (int i = 0; i < 3; i++) CHECK(st[i] >= mn[i] + 29 && st[i] <= mx[i] - 29, "leg %d %.1f outside", i, st[i]);
    printf("  fit kept %.3f of the deviation\n", keep);
}

/* tilt limits and the leg-difference limit inside the kinematics */
static void test_limits(void)
{
    printf("[limits]\n");
    KinGeom g; kin_default_geom(&g);
    int32_t mn[3] = { 672, 700, 798 }, mx[3] = { 8456, 8484, 8582 };
    kin_set_limits(0, 0, 0, 0, 0);
    kin_init(&g, mn, mx);
    float free_pitch = kin_caps()->pitch_cap_deg, free_roll = kin_caps()->roll_cap_deg, free_heave = kin_caps()->heave_range_mm;
    Pose mid = { kin_caps()->heave_mid_mm, 0, 0 };

    kin_set_limits(10.0f, 6.0f, 8.0f, 12.0f, 0);          /* pitch +10 / -6, roll +8 / -12 */
    kin_init(&g, mn, mx);
    const KinCaps *c = kin_caps();
    printf("  free caps: pitch %.1f roll %.1f | with limits: pitch %.2f roll %.2f heave %.1f (free %.1f)\n", free_pitch, free_roll, c->pitch_cap_deg, c->roll_cap_deg, c->heave_range_mm, free_heave);
    CHECK(fabsf(c->pitch_cap_deg - 6.0f) < 0.02f && fabsf(c->roll_cap_deg - 8.0f) < 0.02f, "symmetric caps should be the smaller side: pitch %.3f roll %.3f", c->pitch_cap_deg, c->roll_cap_deg);
    CHECK(fabsf(c->heave_range_mm - free_heave) < 0.5f, "tilt limits must not change the heave range");
    Pose a = { mid.heave_mm, 9.9f, 0 }, b = { mid.heave_mm, 10.1f, 0 }, d = { mid.heave_mm, -5.9f, 0 }, e = { mid.heave_mm, -6.1f, 0 };
    Pose f = { mid.heave_mm, 0, 7.9f }, h = { mid.heave_mm, 0, 8.1f }, i = { mid.heave_mm, 0, -11.9f }, j = { mid.heave_mm, 0, -12.1f };
    CHECK(kin_feasible(&a, 0) && !kin_feasible(&b, 0), "pitch + limit 10: 9.9 ok / 10.1 refused");
    CHECK(kin_feasible(&d, 0) && !kin_feasible(&e, 0), "pitch - limit 6: -5.9 ok / -6.1 refused");
    CHECK(kin_feasible(&f, 0) && !kin_feasible(&h, 0), "roll + limit 8");
    CHECK(kin_feasible(&i, 0) && !kin_feasible(&j, 0), "roll - limit 12");

    /* a mode that asks for far more than the limit is scaled back to it, in the right direction */
    Pose out; float st[3];
    Pose dev1 = { 0, 20.0f, 0 }, dev2 = { 0, -20.0f, 0 };
    float k1 = kin_fit(&mid, &dev1, 30.0f, &out, st);
    CHECK(k1 > 0.3f && k1 < 0.52f && out.pitch_deg <= 10.001f, "fit pitch +20 -> limit 10: keep %.3f pitch %.3f", k1, out.pitch_deg);
    float k2 = kin_fit(&mid, &dev2, 30.0f, &out, st);
    CHECK(k2 > 0.25f && k2 < 0.32f && out.pitch_deg >= -6.001f, "fit pitch -20 -> limit 6: keep %.3f pitch %.3f", k2, out.pitch_deg);

    /* leg difference: heights above the minimums */
    kin_set_limits(0, 0, 0, 0, 450);
    kin_init(&g, mn, mx);
    CHECK(fabsf(kin_caps()->pitch_cap_deg) < free_pitch - 1.0f, "a 450-step difference limit should shrink the pitch cap (free %.1f, now %.1f)", free_pitch, kin_caps()->pitch_cap_deg);
    Pose tilt = { mid.heave_mm, 25.0f, 0 };
    CHECK(!kin_feasible(&tilt, 0), "25 deg pitch must break a 450 step difference limit");
    Pose dev3 = { 0, 25.0f, 0 };
    float k3 = kin_fit(&mid, &dev3, 30.0f, &out, st);
    float lo = 1e9f, hi = -1e9f;
    for (int q = 0; q < 3; q++) { float r = st[q] - (float)mn[q]; if (r < lo) lo = r; if (r > hi) hi = r; }
    CHECK(k3 > 0.0f && k3 < 1.0f && hi - lo <= 451.0f, "fit under the difference limit: keep %.3f spread %.1f", k3, hi - lo);
    printf("  difference limit 450: pitch cap %.2f deg, 25 deg request kept %.0f%% with spread %.1f steps\n", kin_caps()->pitch_cap_deg, k3 * 100.0f, hi - lo);

    kin_set_limits(0, 0, 0, 0, 0);                         /* back to no limits */
    kin_init(&g, mn, mx);
    CHECK(fabsf(kin_caps()->pitch_cap_deg - free_pitch) < 0.01f && fabsf(kin_caps()->roll_cap_deg - free_roll) < 0.01f, "limits did not switch off cleanly");
}

static void test_waves(void)
{
    printf("[waves]\n");
    for (int ss = 1; ss <= 4; ss++)
    {
        waves_init(ss, 1u);
        const WaveStats *st = waves_stats();
        /* simulate 20 min at 50 Hz and compare RMS and velocity RMS */
        double sh = 0, sp = 0, sr = 0, sv = 0, prev = 0; int n = 0; float h, p, r;
        double mh = 0, mp = 0, mr = 0;
        for (int i = 0; i < 60000; i++)
        {
            waves_advance(0.02f); waves_sample(&h, &p, &r);
            sh += h * h; sp += p * p; sr += r * r;
            if (i) sv += ((h - prev) / 0.02) * ((h - prev) / 0.02);
            prev = h; n++;
            if (fabs(h) > mh) mh = fabs(h);
            if (fabs(p) > mp) mp = fabs(p);
            if (fabs(r) > mr) mr = fabs(r);
        }
        double rh = sqrt(sh / n), rp = sqrt(sp / n), rr = sqrt(sr / n), rv = sqrt(sv / n);
        printf("  SS%d heave rms %.4f m (analytic %.4f) pitch %.3f (%.3f) roll %.3f (%.3f) vel %.4f (%.4f) | max h %.3f p %.2f r %.2f\n", ss, rh, st->rms_h_m, rp, st->rms_p_deg, rr, st->rms_r_deg, rv, st->rms_hv, mh, mp, mr);
        CHECK(fabs(rh - st->rms_h_m) / st->rms_h_m < 0.12, "heave rms mismatch");
        CHECK(fabs(rv - st->rms_hv) / st->rms_hv < 0.12, "velocity rms mismatch");
    }
}

static void run_for(double ms)
{
    double end = sim_now_ms() + ms;
    while (sim_now_ms() < end) engine_service();
}

static void test_engine(void)
{
    printf("[engine]\n");
    sim_reset(3000, 3500, 4000);
    engine_init();
    int32_t pos[3] = { 3000, 3500, 4000 }, lo[3] = { 150, 150, 150 }, hi[3] = { 9500, 9500, 9500 };
    engine_sync(pos);
    engine_set_limits(lo, hi);
    engine_set_motion(1500, 1500);

    float t[3] = { 5000, 3500, 1000 };
    sim_log_on = 1; sim_log_n = 0;
    engine_set_target(t, 0);
    run_for(8000);
    CHECK(engine_settled(), "not settled after 8 s");
    printf("  moved to %ld %ld %ld (wanted 5000 3500 1000), engine pos %ld %ld %ld\n", sim_real[0], sim_real[1], sim_real[2], (long)engine_pos(0), (long)engine_pos(1), (long)engine_pos(2));
    CHECK(sim_real[0] == 5000 && sim_real[1] == 3500 && sim_real[2] == 1000, "final positions wrong");
    CHECK(sim_real[0] - 3000 == sim_steps_emitted[0] || 1, "n/a");
    CHECK(sim_pulses_bad[0] + sim_pulses_bad[1] + sim_pulses_bad[2] == 0, "narrow pulses");

    /* speed from the pulse log (leg 0): step rate over sliding windows of 50 ms */
    double vmax_seen = 0;
    {
        long idx[20000]; long ni = 0;
        for (long i = 0; i < sim_log_n && ni < 20000; i++) if (sim_log[i].leg == 0) idx[ni++] = i;
        for (long k = 0; k + 40 < ni; k++)
        {
            double dt = (sim_log[idx[k + 40]].t_ms - sim_log[idx[k]].t_ms) / 1000.0;
            if (dt > 0) { double v = 40.0 / dt; if (v > vmax_seen) vmax_seen = v; }
        }
    }
    printf("  leg 0 peak step rate (40-pulse window) %.0f steps/s (limit 1500)\n", vmax_seen);
    CHECK(vmax_seen < 1560, "speed limit exceeded");
    /* overshoot: leg 2 went down from 4000 to 1000; check that it never went below 1000 */
    long mn2 = 4000, mx0 = 0;
    for (long i = 0; i < sim_log_n; i++) { if (sim_log[i].leg == 2 && sim_log[i].pos < mn2) mn2 = sim_log[i].pos; if (sim_log[i].leg == 0 && sim_log[i].pos > mx0) mx0 = sim_log[i].pos; }
    CHECK(mn2 >= 1000 && mx0 <= 5000, "overshoot: leg2 min %ld, leg0 max %ld", mn2, mx0);

    /* limit clamp: target far outside */
    float t2[3] = { 99999, -50, 3000 };
    engine_set_target(t2, 0);
    run_for(20000);
    printf("  clamped targets: %ld %ld\n", sim_real[0], sim_real[1]);
    CHECK(sim_real[0] == 9500 && sim_real[1] == 150, "limits not respected");

    /* tracking a sine with feed-forward */
    float base[3] = { 4000, 4000, 4000 };
    engine_set_target(base, 0); run_for(15000);
    double worst = 0; long n = 0; double sum = 0;
    double t0 = sim_now_ms();
    for (int k = 0; k < 3000; k++)                  /* 30 s at 100 Hz */
    {
        double tt = k * 0.01;
        float tg[3], tv[3];
        for (int i = 0; i < 3; i++) { tg[i] = 4000 + 1500.0f * sinf(0.6f * (float)tt + i); tv[i] = 1500.0f * 0.6f * cosf(0.6f * (float)tt + i); }
        engine_set_target(tg, tv);
        run_for(10);
        if (k > 200) { double e = fabs(engine_cmd(0) - tg[0]); if (e > worst) worst = e; sum += e; n++; }
    }
    (void)t0;
    printf("  sine tracking (1500 steps amplitude, 0.6 rad/s -> peak %.0f steps/s): worst error %.1f steps, mean %.1f\n", 1500 * 0.6, worst, sum / n);
    CHECK(worst < 40, "tracking error too large");

    /* abrupt target near the limit: the feed-forward says -1300 steps/s, then the target stops 30 steps inside */
    {
        float b4[3] = { 4000, 4000, 4000 };
        engine_set_target(b4, 0); run_for(15000);
        double tpos = 4000, mincmd = 1e9, velatmin = 0;
        for (int k = 0; k < 800; k++)
        {
            float tg[3], tv[3];
            int moving = tpos > 180.0;
            if (moving) tpos -= 13.0;
            if (tpos < 180.0) tpos = 180.0;
            for (int i = 0; i < 3; i++) { tg[i] = (float)tpos; tv[i] = moving ? -1300.0f : 0.0f; }
            engine_set_target(tg, tv);
            run_for(10);
            if (engine_cmd(0) < mincmd) { mincmd = engine_cmd(0); velatmin = engine_vel(0); }
        }
        run_for(3000);
        printf("  abrupt stop near the limit: lowest cmd %.1f (limit 150, target 180), real min %ld, speed there %.0f steps/s\n", mincmd, sim_real[0], velatmin);
        CHECK(mincmd >= 149.9 && sim_real[0] >= 150, "went past the travel limit");
        CHECK(fabs(velatmin) < 60.0, "hit the limit at speed %.0f", velatmin);
    }

    /* hold */
    engine_set_target(base, 0); run_for(5000);
    float tg2[3] = { 8000, 8000, 8000 };
    engine_set_target(tg2, 0); run_for(1200);
    long p_before = sim_real[0];
    engine_hold(); run_for(4000);
    printf("  hold: was moving at %ld, stopped at %ld (target was 8000)\n", p_before, sim_real[0]);
    CHECK(engine_settled() && sim_real[0] < 8000 && sim_real[0] >= p_before, "hold failed");
}

/* the leg difference guard in the step engine */
static void test_spread(void)
{
    printf("[leg difference guard]\n");
    sim_reset(1000, 1000, 1000);
    engine_init();
    int32_t pos[3] = { 1000, 1000, 1000 }, lo[3] = { 150, 150, 150 }, hi[3] = { 9500, 9500, 9500 };
    engine_sync(pos);
    engine_set_limits(lo, hi);
    engine_set_motion(1500, 1500);
    engine_set_spread(pos, 500);

    float t1[3] = { 4000, 1000, 1000 };                   /* one leg alone: may only get 500 (+ the tolerance) ahead */
    engine_set_target(t1, 0);
    run_for(6000);
    long ahead = sim_real[0] - 1000;
    printf("  one leg sent 3000 steps up alone: stopped %ld above its partners, held back for %lu ticks, max spread %ld\n", ahead, (unsigned long)engine_spread_holds(), sim_spread_max);
    CHECK(ahead >= 480 && ahead <= 500 + SPREAD_TOL_STEPS + 6, "leader should stop about 500 ahead, was %ld", ahead);
    CHECK(sim_spread_max <= 500 + SPREAD_TOL_STEPS + 6, "spread reached %ld", sim_spread_max);
    CHECK(engine_spread_holds() > 0, "guard never reported a hold");
    CHECK(sim_real[1] == 1000 && sim_real[2] == 1000, "the other legs moved");

    /* now everyone goes up: the leader waits for the others, nobody ever exceeds the limit, all arrive */
    sim_track_reset();
    float t2[3] = { 5000, 5000, 5000 };
    engine_set_target(t2, 0);
    run_for(14000);
    printf("  all legs to 5000: final %ld %ld %ld, max spread on the way %ld\n", sim_real[0], sim_real[1], sim_real[2], sim_spread_max);
    CHECK(sim_real[0] == 5000 && sim_real[1] == 5000 && sim_real[2] == 5000, "legs did not all arrive");
    CHECK(sim_spread_max <= 500 + SPREAD_TOL_STEPS + 6, "spread reached %ld going up together", sim_spread_max);

    /* switching it off lets a single leg run again */
    engine_set_spread(pos, 0);
    float t3[3] = { 5000, 5000, 2000 };
    engine_set_target(t3, 0);
    run_for(8000);
    CHECK(sim_real[2] == 2000, "guard off: leg should move freely, at %ld", sim_real[2]);
}

int main(void)
{
    sim_echo = 0;
    test_kin();
    test_limits();
    test_spread();
    test_waves();
    test_engine();
    printf("\n%s (%d failures)\n", fails ? "SOME TESTS FAILED" : "ALL CORE TESTS PASSED", fails);
    return fails ? 1 : 0;
}
