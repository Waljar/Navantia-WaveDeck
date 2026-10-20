/**
 * plat_kin.c - kinematics of the 3-crank platform.
 *
 * Model: three identical legs, each a crank (radius r) turning in a vertical
 * plane, driving a rigid link (length L) whose top end is a ball joint on the
 * platform. The platform's centre is held on the vertical axis (no sideways
 * slip, no yaw), so it has exactly three degrees of freedom: heave, pitch, roll.
 *
 * Inverse kinematics is closed form: for each leg the link constraint
 *     |P - (B + r(cos(t) u + sin(t) k))| = L
 * reduces to  A cos(t) + Bz sin(t) = C,  solved for the crank angle t.
 */
#include "plat_kin.h"
#include <math.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f
#define PI_F    3.14159265358979f

void (*kin_yield_hook)(void) = 0;
static void yield(void) { if (kin_yield_hook) kin_yield_hook(); }

static KinGeom G;
static KinCaps C;
static int     ready = 0;
static int32_t MINS[NUM_LEGS], MAXS[NUM_LEGS];
static float   LIM_PP = 0, LIM_PN = 0, LIM_RP = 0, LIM_RN = 0;   /* tilt limits, deg; 0 = none */
static int32_t LIM_DIFF = 0;                                     /* leg difference limit, steps; 0 = none */

void kin_set_limits(float pp, float pn, float rp, float rn, int32_t md)
{
    LIM_PP = pp; LIM_PN = pn; LIM_RP = rp; LIM_RN = rn; LIM_DIFF = md;
}

/* per-leg constants */
static float Bx[NUM_LEGS], By[NUM_LEGS];       /* crank pivot on the base            */
static float Ux[NUM_LEGS], Uy[NUM_LEGS];       /* horizontal direction of crank at 0 */
static float Jx[NUM_LEGS], Jy[NUM_LEGS];       /* platform joint, level, platform frame */
static int   branch[NUM_LEGS];
static float theta_m;                          /* rad: crank angle at the level minimum */
static float z0;                               /* joint height at the level minimum  */

void kin_default_geom(KinGeom *g)
{
    g->r = GEOM_CRANK_R;
    g->L = GEOM_LINK_L;
    g->Rb = GEOM_BASE_R;
    g->Rp = GEOM_PLATFORM_R;
    g->theta_min_deg = GEOM_THETA_MIN_DEG;
    g->crank_az_deg = GEOM_CRANK_AZ_DEG;
    g->sense = GEOM_SENSE;
    g->leg_az_deg[0] = GEOM_AZ_LEG0;
    g->leg_az_deg[1] = GEOM_AZ_LEG1;
    g->leg_az_deg[2] = GEOM_AZ_LEG2;
    g->configured = GEOMETRY_CONFIGURED;
}

const KinGeom *kin_geom(void) { return &G; }
const KinCaps *kin_caps(void) { return &C; }

static float wrap_pi(float a)
{
    while (a > PI_F)  a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

/* Crank angle (rad) of leg i for a platform joint at (Px,Py,Pz). 0 = ok. */
static int leg_solve(int i, float Px, float Py, float Pz, float *theta)
{
    float Dx = Px - Bx[i], Dy = Py - By[i], Dz = Pz;
    float A  = Dx * Ux[i] + Dy * Uy[i];
    float D2 = Dx * Dx + Dy * Dy + Dz * Dz;
    float Cc = (D2 + G.r * G.r - G.L * G.L) / (2.0f * G.r);
    float R  = sqrtf(A * A + Dz * Dz);
    if (R < 1e-6f || fabsf(Cc) > R) return -1;
    *theta = atan2f(Dz, A) + (float)branch[i] * acosf(Cc / R);
    return 0;
}

int kin_pose_to_steps(const Pose *p, float steps[NUM_LEGS])
{
    if (!ready) return -1;
    float sp = sinf(p->pitch_deg * DEG2RAD), cp = cosf(p->pitch_deg * DEG2RAD);
    float sr = sinf(p->roll_deg  * DEG2RAD), cr = cosf(p->roll_deg  * DEG2RAD);
    float z  = z0 + p->heave_mm;

    for (int i = 0; i < NUM_LEGS; i++)
    {
        float x = Jx[i], y = Jy[i];
        float Px = x * cp;
        float Py = y * cr - x * sp * sr;
        float Pz = y * sr + x * sp * cr + z;
        float th;
        if (leg_solve(i, Px, Py, Pz, &th) != 0) return -1;
        steps[i] = (float)MINS[i] + (float)G.sense * wrap_pi(th - theta_m) * RAD2DEG * STEPS_PER_DEG;
    }
    return 0;
}

int kin_feasible(const Pose *p, float margin)
{
    if (LIM_PP > 0.0f && p->pitch_deg >  LIM_PP + 1e-4f) return 0;
    if (LIM_PN > 0.0f && p->pitch_deg < -LIM_PN - 1e-4f) return 0;
    if (LIM_RP > 0.0f && p->roll_deg  >  LIM_RP + 1e-4f) return 0;
    if (LIM_RN > 0.0f && p->roll_deg  < -LIM_RN - 1e-4f) return 0;
    float s[NUM_LEGS];
    if (kin_pose_to_steps(p, s) != 0) return 0;
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < NUM_LEGS; i++)
    {
        if (s[i] < (float)MINS[i] + margin - 0.5f || s[i] > (float)MAXS[i] - margin + 0.5f) return 0;
        float rel = s[i] - (float)MINS[i];
        if (rel < lo) lo = rel;
        if (rel > hi) hi = rel;
    }
    if (LIM_DIFF > 0 && hi - lo > (float)LIM_DIFF + 0.5f) return 0;
    return 1;
}

float kin_fit(const Pose *center, const Pose *dev, float margin, Pose *out, float steps[NUM_LEGS])
{
    Pose c;
    c.heave_mm  = center->heave_mm  + dev->heave_mm;
    c.pitch_deg = center->pitch_deg + dev->pitch_deg;
    c.roll_deg  = center->roll_deg  + dev->roll_deg;

    if (kin_feasible(&c, margin))
    {
        *out = c;
        return (kin_pose_to_steps(out, steps) == 0) ? 1.0f : -1.0f;
    }
    if (!kin_feasible(center, margin))
    {
        *out = *center;
        kin_pose_to_steps(out, steps);
        return -1.0f;
    }

    float lo = 0.0f, hi = 1.0f;
    for (int it = 0; it < 12; it++)
    {
        float m = 0.5f * (lo + hi);
        Pose t;
        t.heave_mm  = center->heave_mm  + m * dev->heave_mm;
        t.pitch_deg = center->pitch_deg + m * dev->pitch_deg;
        t.roll_deg  = center->roll_deg  + m * dev->roll_deg;
        if (kin_feasible(&t, margin)) lo = m; else hi = m;
        yield();
    }
    out->heave_mm  = center->heave_mm  + lo * dev->heave_mm;
    out->pitch_deg = center->pitch_deg + lo * dev->pitch_deg;
    out->roll_deg  = center->roll_deg  + lo * dev->roll_deg;
    if (kin_pose_to_steps(out, steps) != 0) return -1.0f;
    return lo;
}

/* ---- forward kinematics (Newton) --------------------------------------------- */
static Pose fk_last;

static int fk_try(Pose x, const float target[NUM_LEGS], Pose *out)
{
    const float dh = 0.05f, da = 0.02f;
    float f0[NUM_LEGS];

    if (kin_pose_to_steps(&x, f0) != 0) return -1;

    for (int it = 0; it < 30; it++)
    {
        yield();                                    /* keep the step engine ticking */
        float r[NUM_LEGS], err = 0.0f;
        for (int i = 0; i < NUM_LEGS; i++) { r[i] = target[i] - f0[i]; err += fabsf(r[i]); }
        if (err < 0.15f) { *out = x; return 0; }

        /* numerical Jacobian, column per unknown (backward difference if forward is out of reach) */
        float Jm[NUM_LEGS][3], f1[NUM_LEGS];
        Pose t;
        for (int c = 0; c < 3; c++)
        {
            float step = (c == 0) ? dh : da, sgn = 1.0f;
            t = x;
            if (c == 0) t.heave_mm += step; else if (c == 1) t.pitch_deg += step; else t.roll_deg += step;
            if (kin_pose_to_steps(&t, f1) != 0)
            {
                sgn = -1.0f;
                t = x;
                if (c == 0) t.heave_mm -= step; else if (c == 1) t.pitch_deg -= step; else t.roll_deg -= step;
                if (kin_pose_to_steps(&t, f1) != 0) return -1;
            }
            for (int i = 0; i < NUM_LEGS; i++) Jm[i][c] = (f1[i] - f0[i]) / (sgn * step);
        }

        /* solve Jm * d = r by Cramer's rule */
        float det = Jm[0][0] * (Jm[1][1] * Jm[2][2] - Jm[1][2] * Jm[2][1])
                  - Jm[0][1] * (Jm[1][0] * Jm[2][2] - Jm[1][2] * Jm[2][0])
                  + Jm[0][2] * (Jm[1][0] * Jm[2][1] - Jm[1][1] * Jm[2][0]);
        if (fabsf(det) < 1e-9f) return -1;
        float d[3];
        d[0] = (r[0] * (Jm[1][1] * Jm[2][2] - Jm[1][2] * Jm[2][1])
              - Jm[0][1] * (r[1] * Jm[2][2] - Jm[1][2] * r[2])
              + Jm[0][2] * (r[1] * Jm[2][1] - Jm[1][1] * r[2])) / det;
        d[1] = (Jm[0][0] * (r[1] * Jm[2][2] - Jm[1][2] * r[2])
              - r[0] * (Jm[1][0] * Jm[2][2] - Jm[1][2] * Jm[2][0])
              + Jm[0][2] * (Jm[1][0] * r[2] - r[1] * Jm[2][0])) / det;
        d[2] = (Jm[0][0] * (Jm[1][1] * r[2] - r[1] * Jm[2][1])
              - Jm[0][1] * (Jm[1][0] * r[2] - r[1] * Jm[2][0])
              + r[0] * (Jm[1][0] * Jm[2][1] - Jm[1][1] * Jm[2][0])) / det;

        /* damp: no single step larger than 15 mm / 4 deg */
        float sc = 1.0f;
        if (fabsf(d[0]) > 15.0f) sc = fminf(sc, 15.0f / fabsf(d[0]));
        if (fabsf(d[1]) > 4.0f)  sc = fminf(sc, 4.0f / fabsf(d[1]));
        if (fabsf(d[2]) > 4.0f)  sc = fminf(sc, 4.0f / fabsf(d[2]));

        /* backtrack until the new pose is reachable and does not make the error worse */
        int ok = 0;
        for (int bt = 0; bt < 8; bt++, sc *= 0.5f)
        {
            Pose n = { x.heave_mm + sc * d[0], x.pitch_deg + sc * d[1], x.roll_deg + sc * d[2] };
            float fn[NUM_LEGS];
            if (kin_pose_to_steps(&n, fn) != 0) continue;
            float en = 0.0f;
            for (int i = 0; i < NUM_LEGS; i++) en += fabsf(target[i] - fn[i]);
            if (en < err || bt == 7) { x = n; for (int i = 0; i < NUM_LEGS; i++) f0[i] = fn[i]; ok = 1; break; }
        }
        if (!ok) return -1;
    }
    return -1;
}

int kin_steps_to_pose(const float target[NUM_LEGS], Pose *out)
{
    if (fk_try(fk_last, target, out) == 0) { fk_last = *out; return 0; }
    Pose mid = { C.heave_mid_mm, 0.0f, 0.0f };
    if (fk_try(mid, target, out) == 0) { fk_last = *out; return 0; }
    return -1;
}

/* ---- setup and capability numbers ---------------------------------------------- */
static int heave_ok(float h)
{
    Pose p = { h, 0.0f, 0.0f };
    return kin_feasible(&p, 0.0f);
}

static float max_heave(void)
{
    float hi = 20.0f;
    int guard = 0;
    while (heave_ok(hi) && guard++ < 12) hi *= 2.0f;
    float lo = 0.0f;
    for (int it = 0; it < 30; it++)
    {
        float m = 0.5f * (lo + hi);
        if (heave_ok(m)) lo = m; else hi = m;
    }
    return lo;
}

static float max_tilt(int roll)
{
    float lo = 0.0f, hi = 45.0f;
    for (int it = 0; it < 24; it++)
    {
        float m = 0.5f * (lo + hi);
        Pose a = { C.heave_mid_mm, roll ? 0.0f : m,  roll ? m : 0.0f };
        Pose b = { C.heave_mid_mm, roll ? 0.0f : -m, roll ? -m : 0.0f };
        if (kin_feasible(&a, 0.0f) && kin_feasible(&b, 0.0f)) lo = m; else hi = m;
    }
    return lo;
}

int kin_init(const KinGeom *g, const int32_t mn[NUM_LEGS], const int32_t mx[NUM_LEGS])
{
    G = *g;
    ready = 0;
    for (int i = 0; i < NUM_LEGS; i++) { MINS[i] = mn[i]; MAXS[i] = mx[i]; }
    theta_m = G.theta_min_deg * DEG2RAD;

    for (int i = 0; i < NUM_LEGS; i++)
    {
        float b = G.leg_az_deg[i] * DEG2RAD;
        float c = (G.leg_az_deg[i] + G.crank_az_deg) * DEG2RAD;
        Bx[i] = G.Rb * cosf(b);  By[i] = G.Rb * sinf(b);
        Ux[i] = cosf(c);         Uy[i] = sinf(c);
        Jx[i] = G.Rp * cosf(b);  Jy[i] = G.Rp * sinf(b);
    }

    /* height of the platform joints when the cranks sit at theta_min */
    {
        float Cx = Bx[2] + G.r * cosf(theta_m) * Ux[2];
        float Cy = By[2] + G.r * cosf(theta_m) * Uy[2];
        float Cz = G.r * sinf(theta_m);
        float h2 = (Jx[2] - Cx) * (Jx[2] - Cx) + (Jy[2] - Cy) * (Jy[2] - Cy);
        if (G.L * G.L <= h2) return -1;
        z0 = Cz + sqrtf(G.L * G.L - h2);
    }

    /* choose, per leg, the crank solution that is continuous with theta_min */
    for (int i = 0; i < NUM_LEGS; i++)
    {
        float Dx = Jx[i] - Bx[i], Dy = Jy[i] - By[i], Dz = z0;
        float A  = Dx * Ux[i] + Dy * Uy[i];
        float D2 = Dx * Dx + Dy * Dy + Dz * Dz;
        float Cc = (D2 + G.r * G.r - G.L * G.L) / (2.0f * G.r);
        float R  = sqrtf(A * A + Dz * Dz);
        if (R < 1e-6f || fabsf(Cc) > R) return -1;
        float base = atan2f(Dz, A), d = acosf(Cc / R);
        branch[i] = (fabsf(wrap_pi(base + d - theta_m)) <= fabsf(wrap_pi(base - d - theta_m))) ? 1 : -1;
    }

    ready = 1;
    fk_last.heave_mm = 0.0f; fk_last.pitch_deg = 0.0f; fk_last.roll_deg = 0.0f;

    C.heave_range_mm = max_heave();
    C.heave_mid_mm   = 0.5f * C.heave_range_mm;
    C.pitch_cap_deg  = max_tilt(0);
    C.roll_cap_deg   = max_tilt(1);

    /* Jacobian at mid-heave */
    {
        Pose m = { C.heave_mid_mm, 0.0f, 0.0f };
        Pose a, b;
        float sa[NUM_LEGS], sb[NUM_LEGS];
        a = m; b = m; a.heave_mm += 1.0f; b.heave_mm -= 1.0f;
        if (kin_pose_to_steps(&a, sa) || kin_pose_to_steps(&b, sb)) return -1;
        for (int i = 0; i < NUM_LEGS; i++) C.J[i][0] = (sa[i] - sb[i]) / 2.0f;
        a = m; b = m; a.pitch_deg += 0.25f; b.pitch_deg -= 0.25f;
        if (kin_pose_to_steps(&a, sa) || kin_pose_to_steps(&b, sb)) return -1;
        for (int i = 0; i < NUM_LEGS; i++) C.J[i][1] = (sa[i] - sb[i]) / 0.5f;
        a = m; b = m; a.roll_deg += 0.25f; b.roll_deg -= 0.25f;
        if (kin_pose_to_steps(&a, sa) || kin_pose_to_steps(&b, sb)) return -1;
        for (int i = 0; i < NUM_LEGS; i++) C.J[i][2] = (sa[i] - sb[i]) / 0.5f;
    }
    fk_last = (Pose){ C.heave_mid_mm, 0.0f, 0.0f };
    return 0;
}
