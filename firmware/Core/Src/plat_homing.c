#include "plat_homing.h"
#include "plat_hw.h"
#include "plat_config.h"

static const char    LEG_NAME[NUM_LEGS] = { 'X', 'Y', 'Z' };
static const uint8_t HOME_LEVEL[NUM_LEGS] = { HOME_DIR_LEG0, HOME_DIR_LEG1, HOME_DIR_LEG2 };

static void step_once(int leg, uint32_t hz)
{
    uint32_t period = 1000000u / hz;
    uint32_t high   = period / 2u;
    hw_step_pin(leg, 1);
    hw_delay_us(high);
    hw_step_pin(leg, 0);
    hw_delay_us(period - high);
}

static void set_dir(int leg, int level)
{
    hw_dir_pin(leg, level);
    hw_delay_us(DIR_SETUP_US);
}

static void move_steps(int leg, int level, uint32_t hz, uint32_t steps)
{
    set_dir(leg, level);
    for (uint32_t i = 0; i < steps; i++) step_once(leg, hz);
}

static int move_until_pressed(int leg, int level, uint32_t hz, uint32_t max_steps)
{
    set_dir(leg, level);
    for (uint32_t i = 0; i < max_steps; i++)
    {
        if (hw_limit_pressed(leg)) return 1;
        step_once(leg, hz);
    }
    return hw_limit_pressed(leg);
}

static int move_until_released(int leg, int level, uint32_t hz, uint32_t max_steps)
{
    set_dir(leg, level);
    for (uint32_t i = 0; i < max_steps; i++)
    {
        if (!hw_limit_pressed(leg)) return 1;
        step_once(leg, hz);
    }
    return !hw_limit_pressed(leg);
}

/* 1 = crank sits on its switch trigger point (position 0). */
static int home_axis(int leg)
{
    int toward = HOME_LEVEL[leg];
    int away   = !toward;

    if (hw_limit_pressed(leg))                      /* already on the switch: back off first */
    {
        if (!move_until_released(leg, away, SEEK_HZ, MAX_SEEK_STEPS)) return 0;
        move_steps(leg, away, SEEK_HZ, BACKOFF_STEPS);
    }
    if (!move_until_pressed(leg, toward, SEEK_HZ, MAX_SEEK_STEPS)) return 0;
    hw_delay_ms(500);

    move_steps(leg, away, BACKOFF_HZ, BACKOFF_STEPS);
    if (hw_limit_pressed(leg)) return 0;            /* the switch must release */
    hw_delay_ms(500);

    return move_until_pressed(leg, toward, LATCH_HZ, BACKOFF_STEPS * 2u);
}

/* Ask what to do about a leg that failed to home. 1 = retry, 0 = give up. */
static int ask_retry(int leg)
{
    con_printf("\r\nHOMING FAILED on %c. Check the switch wiring (NC to the right pin),\r\n"
               "that nothing blocks the crank, and the DIR wiring.\r\n"
               "  r = retry   q = give up\r\n", LEG_NAME[leg]);
    con_flush_rx();
    while (1)
    {
        int c = con_getc();
        if (c == 'r' || c == 'R') return 1;
        if (c == 'q' || c == 'Q') return 0;
    }
}

/* The second half of homing for a leg that is already sitting on its switch. */
static int finish_axis(int leg)
{
    int toward = HOME_LEVEL[leg];
    int away   = !toward;

    hw_delay_ms(500);
    move_steps(leg, away, BACKOFF_HZ, BACKOFF_STEPS);
    if (hw_limit_pressed(leg)) return 0;            /* the switch must release */
    hw_delay_ms(500);
    return move_until_pressed(leg, toward, LATCH_HZ, BACKOFF_STEPS * 2u);
}

/* Lower all three legs toward their switches TOGETHER, in short bursts in turn, so that no leg
 * is ever more than HOME_CHUNK_STEPS ahead of another. (Homing one leg at a time left that leg
 * at the bottom while the others stayed high.) A leg that reaches its switch waits there.
 * Returns 1 when all three sit on their switches, 0 if the user gave up. */
static int lower_together(void)
{
    uint32_t moved[NUM_LEGS] = { 0, 0, 0 };
    int on_switch[NUM_LEGS] = { 0, 0, 0 };

    /* a leg that starts on its switch first backs off so it can be approached properly */
    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        if (!hw_limit_pressed(leg)) continue;
        while (1)
        {
            if (move_until_released(leg, !HOME_LEVEL[leg], SEEK_HZ, MAX_SEEK_STEPS))
            {
                move_steps(leg, !HOME_LEVEL[leg], SEEK_HZ, BACKOFF_STEPS);
                break;
            }
            if (!ask_retry(leg)) return 0;
        }
    }

    con_print("Lowering all three legs together to their switches...\r\n");
    while (!(on_switch[0] && on_switch[1] && on_switch[2]))
    {
        for (int leg = 0; leg < NUM_LEGS; leg++)
        {
            if (on_switch[leg]) continue;
            if (move_until_pressed(leg, HOME_LEVEL[leg], SEEK_HZ, HOME_CHUNK_STEPS))
            {
                on_switch[leg] = 1;
                con_printf("%c is on its switch.\r\n", LEG_NAME[leg]);
                continue;
            }
            moved[leg] += HOME_CHUNK_STEPS;
            if (moved[leg] >= MAX_SEEK_STEPS)
            {
                if (!ask_retry(leg)) return 0;
                moved[leg] = 0;
            }
        }
    }
    return 1;
}

int home_all(void)
{
    if (!lower_together()) return 0;

    /* every leg is now at the bottom, so the slow part can be done one leg at a time */
    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        con_printf("Homing %c...\r\n", LEG_NAME[leg]);
        while (!finish_axis(leg))
        {
            if (!ask_retry(leg)) return 0;
            if (home_axis(leg)) break;              /* the complete original sequence for this one leg */
        }
        con_printf("%c homed OK.\r\n", LEG_NAME[leg]);
    }
    return 1;
}
