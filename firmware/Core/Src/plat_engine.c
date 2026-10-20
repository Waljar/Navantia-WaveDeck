/**
 * plat_engine.c - see plat_engine.h.
 *
 * Per leg, every tick (dt = 1 / ENGINE_TICK_HZ):
 *   err   = target - cmd
 *   v_des = target_velocity + clamp( min(KP*|err|, sqrt(2*BRAKE*amax*|err|)) )
 *   v     = v + clamp(v_des - v, +-amax*dt)         (acceleration limit)
 *   cmd  += v * dt
 *   a STEP is emitted whenever cmd is more than half a step away from the actual count.
 * The sqrt term is the braking curve, so the leg always stops on the target
 * without overshoot however far away the new target is. A second braking curve
 * in front of the travel limits keeps even an abrupt target (or velocity
 * feed-forward) from driving a leg into a limit at speed.
 */
#include "plat_engine.h"
#include "plat_hw.h"
#include <math.h>

#define KP      30.0f       /* 1/s: pull-in rate near the target */
#define BRAKE   0.7f        /* fraction of amax used for braking onto a target */
#define BRAKE_LIM 0.9f      /* fraction of amax assumed when braking in front of a travel limit */

typedef struct {
    float   cmd, vel, tgt, tv;
    int32_t pos;
    int8_t  dir;            /* +1 away from the switch, -1 toward, 0 unknown */
    uint8_t pulse;
    uint8_t sw_cnt;
    uint32_t pulse_t0;
} Leg;

static Leg      L[NUM_LEGS];
static float    mn[NUM_LEGS], mx[NUM_LEGS];
static float    vmax = VMAX_DEFAULT, amax = AMAX_DEFAULT;
static const float DT = 1.0f / (float)ENGINE_TICK_HZ;
static uint32_t tick_cyc, pulse_cyc, next_tick;
static int      enabled = 0, guard = 0, fault = 0;
static uint32_t late_count = 0, spread_holds = 0;
static int32_t  sp_ref[NUM_LEGS], sp_max = 0;
static const uint8_t HOME_LEVEL[NUM_LEGS] = { HOME_DIR_LEG0, HOME_DIR_LEG1, HOME_DIR_LEG2 };

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void engine_init(void)
{
    tick_cyc  = hw_cycles_per_us() * 1000000u / ENGINE_TICK_HZ;
    pulse_cyc = hw_cycles_per_us() * STEP_PULSE_US;
    enabled = 0;
    fault = 0;
    for (int a = 0; a < NUM_LEGS; a++)
    {
        L[a] = (Leg){0};
        mn[a] = (float)TRAVEL_FLOOR_STEPS;
        mx[a] = (float)TRAVEL_CEIL_STEPS;
        hw_step_pin(a, 0);
    }
}

void engine_sync(const int32_t pos[NUM_LEGS])
{
    for (int a = 0; a < NUM_LEGS; a++)
    {
        hw_step_pin(a, 0);
        L[a].pos = pos[a];
        L[a].cmd = L[a].tgt = (float)pos[a];
        L[a].vel = L[a].tv = 0.0f;
        L[a].dir = 0;
        L[a].pulse = 0;
        L[a].sw_cnt = 0;
    }
    fault = 0;
    next_tick = hw_cycles() + tick_cyc;
    enabled = 1;
}

void engine_disable(void)
{
    enabled = 0;
    for (int a = 0; a < NUM_LEGS; a++) { hw_step_pin(a, 0); L[a].pulse = 0; }
}

void engine_set_limits(const int32_t lo[NUM_LEGS], const int32_t hi[NUM_LEGS])
{
    for (int a = 0; a < NUM_LEGS; a++) { mn[a] = (float)lo[a]; mx[a] = (float)hi[a]; }
}

void engine_set_motion(float v, float a)
{
    vmax = clampf(v, 50.0f, (float)VMAX_LIMIT);
    amax = clampf(a, 100.0f, (float)AMAX_LIMIT);
}

void engine_guard(int on) { guard = on; }

void engine_set_spread(const int32_t ref[NUM_LEGS], int32_t maxd)
{
    for (int a = 0; a < NUM_LEGS; a++) sp_ref[a] = ref ? ref[a] : 0;
    sp_max = (maxd > 0) ? maxd + (int32_t)SPREAD_TOL_STEPS : 0;
}

void engine_set_target(const float t[NUM_LEGS], const float tv[NUM_LEGS])
{
    for (int a = 0; a < NUM_LEGS; a++)
    {
        L[a].tgt = clampf(t[a], mn[a], mx[a]);
        L[a].tv  = tv ? tv[a] : 0.0f;
    }
}

void engine_hold(void)
{
    for (int a = 0; a < NUM_LEGS; a++)
    {
        float v = L[a].vel;
        float stop = v * fabsf(v) / (2.0f * BRAKE * amax);
        L[a].tgt = clampf(L[a].cmd + stop, mn[a], mx[a]);
        L[a].tv  = 0.0f;
    }
}

void engine_estop(void)
{
    for (int a = 0; a < NUM_LEGS; a++)
    {
        L[a].vel = 0.0f;
        L[a].tv  = 0.0f;
        L[a].cmd = L[a].tgt = (float)L[a].pos;
    }
}

static void tick(void)
{
    for (int a = 0; a < NUM_LEGS; a++)
    {
        Leg *l = &L[a];

        float err = l->tgt - l->cmd;
        float ae  = fabsf(err);
        float vp  = KP * ae;
        float vb  = sqrtf(2.0f * BRAKE * amax * ae);
        if (vp > vb) vp = vb;
        float vdes = clampf(((err >= 0.0f) ? vp : -vp) + l->tv, -vmax, vmax);

        /* Never approach a travel limit faster than the leg can still stop in front of it,
         * whatever the target (or its velocity feed-forward) asks for. */
        float room_up = mx[a] - l->cmd, room_dn = l->cmd - mn[a];
        float v_up = sqrtf(2.0f * BRAKE_LIM * amax * (room_up > 0.0f ? room_up : 0.0f));
        float v_dn = sqrtf(2.0f * BRAKE_LIM * amax * (room_dn > 0.0f ? room_dn : 0.0f));
        vdes = clampf(vdes, -v_dn, v_up);

        float dv  = clampf(vdes - l->vel, -amax * DT, amax * DT);
        l->vel += dv;
        l->cmd += l->vel * DT;

        if (l->cmd < mn[a]) { l->cmd = mn[a]; if (l->vel < 0.0f) l->vel = 0.0f; }
        if (l->cmd > mx[a]) { l->cmd = mx[a]; if (l->vel > 0.0f) l->vel = 0.0f; }

        if (l->pulse) continue;                     /* previous pulse still high (catch-up tick) */

        int want = 0;
        /* pos follows cmd rounded to the nearest step (half-step threshold, so a
         * leg that has converged on an integer target sits exactly on it) */
        if (l->cmd - (float)l->pos > 0.5f)      want = +1;
        else if ((float)l->pos - l->cmd > 0.5f) want = -1;
        if (!want) { l->sw_cnt = 0; continue; }

        if (sp_max > 0)                             /* leg difference guard */
        {
            int32_t r[NUM_LEGS], lo0, hi0, lo1, hi1;
            for (int b = 0; b < NUM_LEGS; b++) r[b] = L[b].pos - sp_ref[b];
            lo0 = hi0 = r[0];
            for (int b = 1; b < NUM_LEGS; b++) { if (r[b] < lo0) lo0 = r[b]; if (r[b] > hi0) hi0 = r[b]; }
            r[a] += want;
            lo1 = hi1 = r[0];
            for (int b = 1; b < NUM_LEGS; b++) { if (r[b] < lo1) lo1 = r[b]; if (r[b] > hi1) hi1 = r[b]; }
            if ((hi1 - lo1) > sp_max && (hi1 - lo1) > (hi0 - lo0))
            {
                l->cmd = (float)l->pos;             /* wait here for the others, restart gently */
                l->vel = 0.0f;
                spread_holds++;
                continue;
            }
        }

        if (want < 0 && guard)
        {
            if (hw_limit_raw(a))
            {
                if (++l->sw_cnt >= 3)
                {
                    fault = a + 1;
                    for (int b = 0; b < NUM_LEGS; b++)
                    {
                        L[b].vel = 0.0f;
                        L[b].tv = 0.0f;
                        L[b].cmd = L[b].tgt = (float)L[b].pos;
                    }
                    return;
                }
                continue;
            }
            l->sw_cnt = 0;
        }

        if (l->dir != want)                         /* set DIR now, step on the next tick */
        {
            hw_dir_pin(a, (want > 0) ? !HOME_LEVEL[a] : HOME_LEVEL[a]);
            l->dir = (int8_t)want;
            continue;
        }

        hw_step_pin(a, 1);
        l->pulse = 1;
        l->pulse_t0 = hw_cycles();
        l->pos += want;
    }
}

void engine_service(void)
{
    uint32_t now = hw_cycles();

    for (int a = 0; a < NUM_LEGS; a++)
    {
        if (L[a].pulse && (uint32_t)(now - L[a].pulse_t0) >= pulse_cyc)
        {
            hw_step_pin(a, 0);
            L[a].pulse = 0;
        }
    }
    if (!enabled) return;

    int n = 0;
    while ((int32_t)(now - next_tick) >= 0)
    {
        tick();
        next_tick += tick_cyc;
        if (++n >= 8) { next_tick = now + tick_cyc; late_count++; break; }   /* fell far behind: resync */
    }
}

int32_t engine_pos(int a)    { return L[a].pos; }
float   engine_cmd(int a)    { return L[a].cmd; }
float   engine_vel(int a)    { return L[a].vel; }
float   engine_target(int a) { return L[a].tgt; }

int engine_settled(void)
{
    for (int a = 0; a < NUM_LEGS; a++)
        if (fabsf(L[a].tgt - L[a].cmd) >= 0.5f || fabsf(L[a].vel) > 5.0f) return 0;
    return 1;
}

uint32_t engine_late(void)     { return late_count; }
uint32_t engine_spread_holds(void) { return spread_holds; }
int  engine_fault(void)       { return fault; }
void engine_clear_fault(void) { fault = 0; }
