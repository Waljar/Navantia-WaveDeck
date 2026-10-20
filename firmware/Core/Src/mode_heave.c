/**
 * mode_heave.c - Mode 2: all three legs together, minimum -> maximum -> minimum,
 * slow, with a pause at each end, and pause / resume / speed control while moving.
 */
#include "plat_app.h"
#include "plat_hw.h"
#include "plat_engine.h"
#include "plat_cal.h"

static const int32_t LEVELS[] = { 200, 400, 700, 1000, 1500, 2000, 3000 };
#define NUM_LEVELS ((int)(sizeof(LEVELS) / sizeof(LEVELS[0])))
static int level_idx = 1;                       /* start slow: 400 steps/s */

enum { ST_WAIT_MIN, ST_UP, ST_WAIT_MAX, ST_DOWN };

static int32_t span_steps(void)
{
    int32_t s = 0x7FFFFFFF;
    for (int i = 0; i < NUM_LEGS; i++)
        if (g_cal.max[i] - g_cal.min[i] < s) s = g_cal.max[i] - g_cal.min[i];
    return s;
}

static void go(int up)
{
    float t[NUM_LEGS];
    float s = (float)span_steps();
    for (int i = 0; i < NUM_LEGS; i++) t[i] = (float)g_cal.min[i] + (up ? s : 0.0f);
    engine_set_target(t, 0);
}

static void apply_speed(void)
{
    int32_t v = LEVELS[level_idx];
    if (v > g_cal.vmax) v = g_cal.vmax;
    engine_set_motion((float)v, (float)g_cal.amax);
}

static void show_speed(void)
{
    int32_t v = LEVELS[level_idx];
    if (v > g_cal.vmax) v = g_cal.vmax;
    int32_t deg100 = v * 18 / 10;                           /* deg/s x 100 */
    int32_t secs = span_steps() / v + v / (g_cal.amax ? g_cal.amax : 1);
    con_queuef("speed %d/%d = %ld steps/s (%ld.%02ld deg/s of crank), about %ld s per stroke\r\n",
               level_idx + 1, NUM_LEVELS, (long)v, (long)(deg100 / 100), (long)(deg100 % 100), (long)secs);
}

void mode_heave(void)
{
    if (!app_ensure_homed()) return;

    uint32_t pause_ms = CYCLE_PAUSE_MS;
    int state = ST_WAIT_MIN, paused = 0, saved_state = 0;
    uint32_t wait_t0 = 0, cycles = 0;

    con_print("\r\n=== HEAVE CYCLE ===\r\n"
              "All three legs, minimum -> maximum -> minimum, pausing at each end.\r\n"
              "p or space = pause / resume (slows to a stop)    + / - = faster / slower\r\n"
              "[ / ] = shorter / longer pause at the ends       q = back to the menu\r\n"
              "x = emergency stop\r\n");
    show_speed();
    engine_set_motion((float)g_cal.vmax, (float)g_cal.amax);   /* getting down to the minimums is not part of the cycle: full speed limit */
    go(0);
    if (!app_arrive(30000u, 1)) return;                   /* start from the minimums (x / q = give up) */
    apply_speed();
    wait_t0 = hw_ms();
    con_flush_rx();

    while (1)
    {
        int k = app_poll();
        if (app_check_fault()) return;
        app_tele(MODE_HEAVE);

        if (k >= 0)
        {
            switch (k)
            {
                case 'q': case 'Q': return;
                case 'x': case 'X': engine_estop(); con_queue("EMERGENCY STOP\r\n"); return;
                case 'p': case 'P': case ' ':
                    if (!paused)
                    {
                        paused = 1; saved_state = state;
                        engine_hold();
                        con_queue("Paused. p / space = resume, q = menu\r\n");
                    }
                    else
                    {
                        paused = 0; state = saved_state;
                        if (state == ST_UP) go(1);
                        if (state == ST_DOWN) go(0);
                        wait_t0 = hw_ms();
                        con_queue("Resumed.\r\n");
                    }
                    break;
                case '+': case '=': if (level_idx < NUM_LEVELS - 1) level_idx++; apply_speed(); show_speed(); break;
                case '-': case '_': if (level_idx > 0) level_idx--;               apply_speed(); show_speed(); break;
                case ']': pause_ms += 500u; con_queuef("pause at the ends: %lu ms\r\n", (unsigned long)pause_ms); break;
                case '[': if (pause_ms >= 500u) pause_ms -= 500u; con_queuef("pause at the ends: %lu ms\r\n", (unsigned long)pause_ms); break;
                default: break;
            }
        }
        if (paused) continue;

        switch (state)
        {
            case ST_WAIT_MIN:
                if (hw_ms() - wait_t0 >= pause_ms) { con_queue("-> max\r\n"); go(1); state = ST_UP; }
                break;
            case ST_UP:
                if (engine_settled()) { state = ST_WAIT_MAX; wait_t0 = hw_ms(); }
                break;
            case ST_WAIT_MAX:
                if (hw_ms() - wait_t0 >= pause_ms) { con_queue("-> min\r\n"); go(0); state = ST_DOWN; }
                break;
            case ST_DOWN:
                if (engine_settled())
                {
                    cycles++;
                    con_queuef("cycle %lu done | X=%ld Y=%ld Z=%ld\r\n", (unsigned long)cycles,
                               (long)engine_pos(0), (long)engine_pos(1), (long)engine_pos(2));
                    state = ST_WAIT_MIN; wait_t0 = hw_ms();
                }
                break;
        }
    }
}
