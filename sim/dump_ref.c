/* Prints reference numbers from the C code so that tools/verify-js.js can check the
 * JavaScript port against them. */
#include "plat_kin.h"
#include "plat_waves.h"
#include <stdio.h>

int main(void)
{
    for (int ss = 1; ss <= 4; ss++)
    {
        waves_init(ss, 1u);
        const WaveStats *s = waves_stats();
        printf("stats %d %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", ss, s->rms_h_m, s->rms_p_deg, s->rms_r_deg, s->rms_hv, s->rms_pv, s->rms_rv, s->rms_ha, s->rms_pa, s->rms_ra);
        for (int i = 1; i <= 3000; i++)
        {
            waves_advance(0.02f);
            if (i % 100 == 0) { float h, p, r; waves_sample(&h, &p, &r); printf("wave %d %d %.6f %.5f %.5f\n", ss, i, h, p, r); }
        }
    }

    KinGeom g; kin_default_geom(&g);
    int32_t mn[3] = { 672, 700, 798 }, mx[3] = { 8456, 8484, 8582 };
    kin_init(&g, mn, mx);
    const KinCaps *c = kin_caps();
    printf("caps %.5f %.5f %.5f %.5f", c->heave_range_mm, c->heave_mid_mm, c->pitch_cap_deg, c->roll_cap_deg);
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) printf(" %.4f", c->J[i][j]);
    printf("\n");
    const float poses[][3] = { {0,0,0}, {20,0,0}, {56.87f,3,-2}, {30,-6,4}, {80,5,5}, {10,-10,8}, {100,0,0}, {56,12,-12} };
    for (int k = 0; k < 8; k++)
    {
        Pose p = { poses[k][0], poses[k][1], poses[k][2] }; float s[3];
        int rc = kin_pose_to_steps(&p, s);
        printf("ik %.3f %.3f %.3f %d %.3f %.3f %.3f\n", p.heave_mm, p.pitch_deg, p.roll_deg, rc, s[0], s[1], s[2]);
    }
    const float devs[][3] = { {40,5,5}, {60,20,-20}, {-60,20,20}, {5,1,1} };
    for (int k = 0; k < 4; k++)
    {
        Pose ctr = { c->heave_mid_mm, 0, 0 }, dv = { devs[k][0], devs[k][1], devs[k][2] }, out; float s[3];
        float keep = kin_fit(&ctr, &dv, 30.0f, &out, s);
        printf("fit %.1f %.1f %.1f %.5f %.3f %.3f %.3f\n", dv.heave_mm, dv.pitch_deg, dv.roll_deg, keep, s[0], s[1], s[2]);
    }
    /* the same workspace with tilt and leg-difference limits switched on */
    kin_set_limits(20.0f, 15.0f, 12.0f, 25.0f, 3000);
    kin_init(&g, mn, mx);
    c = kin_caps();
    printf("lcaps %.5f %.5f %.5f %.5f", c->heave_range_mm, c->heave_mid_mm, c->pitch_cap_deg, c->roll_cap_deg);
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) printf(" %.4f", c->J[i][j]);
    printf("\n");
    const float lp[][3] = { {0,0,0}, {56.87f,19,0}, {56.87f,21,0}, {56.87f,-14,0}, {56.87f,-16,0}, {56.87f,0,11}, {56.87f,0,13}, {56.87f,0,-24}, {56.87f,0,-26}, {10,15,-10}, {100,-10,8}, {56.87f,8,8} };
    for (int k = 0; k < 12; k++)
    {
        Pose p = { lp[k][0], lp[k][1], lp[k][2] };
        printf("lfeas %.3f %.3f %.3f %d\n", p.heave_mm, p.pitch_deg, p.roll_deg, kin_feasible(&p, 0.0f));
    }
    for (int k = 0; k < 4; k++)
    {
        Pose ctr = { c->heave_mid_mm, 0, 0 }, dv = { devs[k][0], devs[k][1], devs[k][2] }, out; float s[3];
        float keep = kin_fit(&ctr, &dv, 30.0f, &out, s);
        printf("lfit %.1f %.1f %.1f %.5f %.3f %.3f %.3f\n", dv.heave_mm, dv.pitch_deg, dv.roll_deg, keep, s[0], s[1], s[2]);
    }
    return 0;
}
