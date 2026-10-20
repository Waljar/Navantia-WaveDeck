/**
 * app_common.c - start-up, the engine/console service loop, homing wrapper,
 * telemetry, and the main menu.
 */
#include "plat_app.h"
#include "plat_hw.h"
#include "plat_engine.h"
#include "plat_cal.h"
#include "plat_homing.h"
#include <stdio.h>
#include <string.h>

int g_homed = 0;
int g_tele  = 0;
int g_locked = 0;         /* presentation lock: Setup, Manual and re-home are refused (RAM only, a power cycle unlocks) */
int g_kin_ok = 0;

static const char LEG_NAME[NUM_LEGS] = { 'X', 'Y', 'Z' };

/* ---- number formatting ---------------------------------------------------------- */
int fmt_f(char *out, int n, float v, int decimals)
{
    long scale = 1;
    for (int i = 0; i < decimals; i++) scale *= 10;
    int neg = (v < 0.0f);
    float a = neg ? -v : v;
    long q = (long)(a * (float)scale + 0.5f);
    if (q == 0) neg = 0;
    if (decimals > 0)
        return snprintf(out, (size_t)n, "%s%ld.%0*ld", neg ? "-" : "", q / scale, decimals, q % scale);
    return snprintf(out, (size_t)n, "%s%ld", neg ? "-" : "", q);
}

/* ---- service loop ------------------------------------------------------------- */
void app_idle(void)
{
    engine_service();
    con_pump();
}

int app_poll(void)
{
    app_idle();
    return con_poll();
}

int app_getc(void)
{
    int c;
    while ((c = app_poll()) < 0) { }
    return c;
}

void ctl_init(CtlTimer *t, uint32_t hz)
{
    t->period = hw_cycles_per_us() * (1000000u / hz);
    t->next   = hw_cycles() + t->period;
}

int ctl_due(CtlTimer *t)
{
    uint32_t now = hw_cycles();
    if ((int32_t)(now - t->next) < 0) return 0;
    t->next += t->period;
    if ((int32_t)(now - t->next) > 0) t->next = now + t->period;     /* fell behind */
    return 1;
}

int app_check_fault(void)
{
    static uint32_t last_holds = 0, last_note = 0;
    uint32_t holds = engine_spread_holds();
    if (holds != last_holds)
    {
        last_holds = holds;
        if ((uint32_t)(hw_ms() - last_note) > 2000u)
        {
            last_note = hw_ms();
            con_queue("Leg difference limit reached: a leg is being held back until the others catch up.\r\n");
        }
    }
    int f = engine_fault();
    if (!f) return 0;
    engine_estop();
    con_queuef("\r\n!!! HOME SWITCH %c FIRED - motion stopped. The platform must be homed again.\r\n",
               LEG_NAME[f - 1]);
    engine_clear_fault();
    g_homed = 0;
    return 1;
}

void app_settle(uint32_t timeout_ms)
{
    engine_hold();
    uint32_t t0 = hw_ms();
    while (!engine_settled() && (hw_ms() - t0) < timeout_ms) app_idle();
}

/* Wait for the legs to ARRIVE at the target already given (app_settle would cancel that move).
 * abortable: x or q stops the legs where they are and returns 0, as does a fault. 1 = arrived. */
int app_arrive(uint32_t timeout_ms, int abortable)
{
    uint32_t t0 = hw_ms();
    while (!engine_settled() && (hw_ms() - t0) < timeout_ms)
    {
        int k = app_poll();
        if (app_check_fault()) return 0;
        if (abortable && (k == 'x' || k == 'X' || k == 'q' || k == 'Q'))
        {
            engine_hold();
            con_queue("Stopped.\r\n");
            return 0;
        }
    }
    return engine_settled();
}

/* ---- calibration into the engine and the kinematics ------------------------------ */
void app_apply_cal_ex(int with_limits)
{
    KinGeom g;
    kin_default_geom(&g);
    engine_set_limits(g_cal.min, g_cal.max);
    engine_set_motion((float)g_cal.vmax, (float)g_cal.amax);
    if (with_limits)
    {
        engine_set_spread(g_cal.min, g_cal.max_diff);
        kin_set_limits((float)g_cal.pitch_pos_cdeg / 100.0f, (float)g_cal.pitch_neg_cdeg / 100.0f,
                       (float)g_cal.roll_pos_cdeg  / 100.0f, (float)g_cal.roll_neg_cdeg  / 100.0f, g_cal.max_diff);
    }
    else
    {
        engine_set_spread(g_cal.min, 0);
        kin_set_limits(0.0f, 0.0f, 0.0f, 0.0f, 0);
    }
    g_kin_ok = (kin_init(&g, g_cal.min, g_cal.max) == 0);
}

void app_apply_cal(void) { app_apply_cal_ex(1); }

void app_init(void)
{
    hw_init();
    con_init();
    engine_init();
    kin_yield_hook = engine_service;
    cal_load();
    app_apply_cal();
}

/* ---- homing ----------------------------------------------------------------------- */
int app_ensure_homed(void)
{
    if (g_homed) return 1;

    con_printf("\r\nSwitches: X %s  Y %s  Z %s\r\n",
               hw_limit_pressed(0) ? "PRESSED" : "released",
               hw_limit_pressed(1) ? "PRESSED" : "released",
               hw_limit_pressed(2) ? "PRESSED" : "released");
    for (int i = 0; i < NUM_LEGS; i++)
    {
        if (!hw_limit_pressed(i)) continue;
        con_printf("WARNING: the %c switch reads PRESSED. If its crank is not on the switch,\r\n"
                   "check the wiring (NC to the right pin). Press y to continue or q to cancel.\r\n",
                   LEG_NAME[i]);
        con_flush_rx();
        while (1)
        {
            int k = con_getc();
            if (k == 'y' || k == 'Y') break;
            if (k == 'q' || k == 'Q') return 0;
        }
    }

    con_print("\r\nClear the area and keep a hand near the driver power switches.\r\n"
              "Homing lowers all three legs together to their switches.\r\n"
              "Press any key to start homing (q = cancel)...\r\n");
    con_flush_rx();
    int k = con_getc();
    if (k == 'q' || k == 'Q') return 0;

    engine_disable();
    engine_guard(0);
    if (!home_all()) return 0;

    int32_t zero[NUM_LEGS] = { 0, 0, 0 };
    int32_t lo[NUM_LEGS], hi[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) { lo[i] = 0; hi[i] = TRAVEL_CEIL_STEPS; }
    engine_sync(zero);
    engine_set_limits(lo, hi);
    engine_set_motion((float)g_cal.vmax, (float)g_cal.amax);

    con_print("\r\nAll legs homed. Moving to the minimum positions...\r\n");
    float t[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) t[i] = (float)g_cal.min[i];
    engine_set_target(t, 0);
    uint32_t t0 = hw_ms();
    while (!engine_settled() && (hw_ms() - t0) < 30000u) app_idle();

    app_apply_cal();
    engine_guard(1);
    g_homed = 1;
    con_print("Ready.\r\n");
    return 1;
}

/* ---- pose driving ------------------------------------------------------------------ */
float app_drive_pose(const Pose *center, const Pose *dev, float prev[NUM_LEGS], int *have_prev)
{
    Pose out;
    float steps[NUM_LEGS], tv[NUM_LEGS];
    float keep = kin_fit(center, dev, FIT_MARGIN_STEPS, &out, steps);
    if (keep < 0.0f) { engine_hold(); return keep; }

    for (int i = 0; i < NUM_LEGS; i++)
    {
        tv[i] = (*have_prev) ? (steps[i] - prev[i]) * (float)CTRL_HZ : 0.0f;
        prev[i] = steps[i];
    }
    *have_prev = 1;
    engine_set_target(steps, tv);
    return keep;
}

/* ---- telemetry for the web visualiser ------------------------------------------------- */
static long rnd(float v, float k) { float x = v * k; return (long)(x >= 0.0f ? x + 0.5f : x - 0.5f); }

void app_tele(int mode)
{
    static uint32_t last = 0;
    if (!g_tele) return;
    uint32_t now = hw_ms();
    if ((now - last) < 50u) return;
    last = now;

    float s[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) s[i] = engine_cmd(i);
    Pose p = { 0.0f, 0.0f, 0.0f };
    int flags = (g_homed ? 1 : 0) | (g_locked ? 8 : 0);
    if (!g_homed || !g_kin_ok || kin_steps_to_pose(s, &p) != 0) { p.heave_mm = p.pitch_deg = p.roll_deg = 0.0f; flags |= 4; }

    con_queuef("T,%lu,%d,%ld,%ld,%ld,%ld,%ld,%ld,%d\r\n", (unsigned long)now, mode,
               rnd(p.heave_mm, 10.0f), rnd(p.pitch_deg, 100.0f), rnd(p.roll_deg, 100.0f),
               (long)engine_pos(0), (long)engine_pos(1), (long)engine_pos(2), flags);
}

/* ---- go to rest: every leg to its minimum (level, lowest), without homing again ------------- */
int app_lower_to_rest(int announce)
{
    if (announce) con_print("Going to rest: all legs to the minimums (x or q = stop)...\r\n");
    float t[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) t[i] = (float)g_cal.min[i];
    engine_set_target(t, 0);
    uint32_t t0 = hw_ms();
    while (!engine_settled() && (hw_ms() - t0) < 60000u)
    {
        int k = app_poll();
        if (app_check_fault()) return 0;
        if (k == 'x' || k == 'X' || k == 'q' || k == 'Q')
        {
            engine_hold();
            con_queue("Stopped.\r\n");
            return 0;
        }
    }
    if (!engine_settled()) { con_print("The legs did not reach rest in time.\r\n"); return 0; }
    if (announce) con_print("At rest.\r\n");
    return 1;
}

static void toggle_lock(void)
{
    if (!g_locked)
    {
        g_locked = 1;
        con_print("Presentation lock ON: Setup, Manual and re-home are disabled. Press l to unlock.\r\n");
        return;
    }
    con_print("Type UNLOCK and press Enter (anything else cancels): ");
    char b[8]; int n = 0;
    while (1)
    {
        int c = app_getc();
        if (c == '\r' || c == '\n') break;
        if ((c == 8 || c == 127) && n > 0) { n--; continue; }
        if (c >= 32 && n < 7) b[n++] = (char)(c | 0x20);       /* lower case */
    }
    b[n] = 0;
    if (n == 6 && b[0]=='u' && b[1]=='n' && b[2]=='l' && b[3]=='o' && b[4]=='c' && b[5]=='k') { g_locked = 0; con_print("\r\nPresentation lock OFF.\r\n"); }
    else con_print("\r\nStill locked.\r\n");
}

static int refuse_if_locked(void)
{
    if (!g_locked) return 0;
    con_print("LOCKED: not available during the presentation. Press l to unlock.\r\n");
    return 1;
}

static void go_rest(void)
{
    if (!g_homed) { con_print("Not homed yet. Press h to home first.\r\n"); return; }
    app_lower_to_rest(1);
}

int app_rehome(void)
{
    if (g_homed)
    {
        con_print("Lowering all three legs together before homing (x or q = stop)...\r\n");
        if (!app_lower_to_rest(0)) return 0;
    }
    g_homed = 0;
    return app_ensure_homed();
}

/* ---- main menu ------------------------------------------------------------------------- */
static void print_status(void)
{
    const KinCaps *c = kin_caps();
    if (g_locked) con_print("\r\n*** PRESENTATION LOCK ON (Setup, Manual and re-home disabled) ***");
    con_printf("\r\nStatus: %s | calibration: %s | geometry: %s | telemetry: %s\r\n",
               g_homed ? "HOMED" : "not homed",
               g_cal.from_flash ? "saved in flash" : "defaults (run Setup, then save)",
               !g_kin_ok ? "INVALID" : (kin_geom()->configured ? "set" : "NOT SET (mm/deg values are estimates)"),
               g_tele ? "on" : "off");
    con_printf("Step clock late: %lu times (should stay 0 or very small)\r\n", (unsigned long)engine_late());
    con_printf("Legs  min: %ld %ld %ld   max: %ld %ld %ld   speed %ld steps/s, accel %ld steps/s^2\r\n",
               (long)g_cal.min[0], (long)g_cal.min[1], (long)g_cal.min[2],
               (long)g_cal.max[0], (long)g_cal.max[1], (long)g_cal.max[2],
               (long)g_cal.vmax, (long)g_cal.amax);
    {
        char a[20], b[20], c[20], d[20];
        fmt_f(a, sizeof a, (float)g_cal.pitch_pos_cdeg / 100.0f, 1); fmt_f(b, sizeof b, (float)g_cal.pitch_neg_cdeg / 100.0f, 1);
        fmt_f(c, sizeof c, (float)g_cal.roll_pos_cdeg  / 100.0f, 1); fmt_f(d, sizeof d, (float)g_cal.roll_neg_cdeg  / 100.0f, 1);
        if (g_cal.pitch_pos_cdeg || g_cal.pitch_neg_cdeg || g_cal.roll_pos_cdeg || g_cal.roll_neg_cdeg)
            con_printf("Tilt limits (set by hand): pitch +%s / -%s deg, roll +%s / -%s deg  (0.0 = from the geometry only)\r\n", a, b, c, d);
        else
            con_print("Tilt limits: NOT SET by hand (only the geometry limits the tilt) - Setup, option 4\r\n");
        if (g_cal.max_diff) con_printf("Leg difference limit: %ld steps (%ld.%ld deg of crank)\r\n", (long)g_cal.max_diff,
                                       (long)(g_cal.max_diff * 18 / 1000), (long)((g_cal.max_diff * 18 / 100) % 10));
        else                con_print("Leg difference limit: NOT SET - Setup, option 5\r\n");
    }
    if (g_kin_ok)
        con_printf("Workspace: heave %ld mm, pitch +-%ld.%ld deg, roll +-%ld.%ld deg\r\n",
                   (long)(c->heave_range_mm + 0.5f),
                   (long)c->pitch_cap_deg, (long)(c->pitch_cap_deg * 10.0f) % 10,
                   (long)c->roll_cap_deg,  (long)(c->roll_cap_deg * 10.0f) % 10);
}

static void print_menu(void)
{
    print_status();
    con_print("\r\n=========== SEA PLATFORM ===========\r\n"
              " 1  Setup: home, set minimums, set maximum, save\r\n"
              " 2  Heave cycle: min <-> max, slow, pausable\r\n"
              " 3  Pitch / roll circles\r\n"
              " 4  Waves: sea states 1-4\r\n"
              " 5  MAX platform: exaggerated waves + full-range demo\r\n"
              " 6  Manual: type heave / pitch / roll\r\n"
              " g  go to rest (all legs to the minimums)\r\n"
              " h  home now      v  telemetry on/off      ?  show this menu\r\n"
              " l  presentation lock on/off (locks Setup, Manual, re-home)\r\n"
              "  (inside any mode: q = back to this menu, x = emergency stop)\r\n"
              "Select: ");
}

void app_run(void)
{
    con_print("\r\n\r\nSea platform firmware ready.\r\n");
    print_menu();

    while (1)
    {
        int k = app_getc();
        con_print("\r\n");

        switch (k)
        {
            case '1': if (refuse_if_locked()) continue; mode_setup(); break;
            case '2': mode_heave();        break;
            case '3': mode_circles();      break;
            case '4': mode_waves(0);       break;
            case '5': mode_waves(1);       break;
            case '6': if (refuse_if_locked()) continue; mode_manual(); break;
            case 'l': case 'L': toggle_lock(); continue;
            case 'g': case 'G': go_rest(); break;
            case 'h': case 'H':
                if (refuse_if_locked()) continue;
                app_rehome();
                break;
            case 'v': case 'V':
                g_tele = !g_tele;
                con_printf("Telemetry %s.\r\n", g_tele ? "on" : "off");
                continue;
            case '?': case '\r': case '\n': break;
            default: continue;
        }
        app_settle(15000u);
        con_flush_rx();
        print_menu();
    }
}
