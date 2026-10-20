/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Home each axis in turn (X, then Y, then Z) against its
  *                   limit switch, then jog it from a serial terminal to find
  *                   its software min and max travel.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* ---------------------------------------------------------------------------
 * WIRING (push-pull, active-high). PUL-, DIR- and the switch COMs all go to a
 * Nucleo GND (shared ground bus). ENA+ / ENA- left unconnected.
 *
 *   Axis   PUL+          DIR+          Limit switch (NC contact)
 *   X      PA0 (A0)      PA1 (A1)      PB9 (D14)
 *   Y      PB4 (D5)      PB5 (D4)      PB0 (A3)
 *   Z      PA8 (D7)      PA9 (D8)      PA4 (A2)
 *
 * Each driver: 5V/24V selector on 5V. DIP: SW1 off, SW2 on, SW3 on, SW4 off
 * (2000 pulses/rev), SW6 off (closed loop), SW7 off (PUL/DIR).
 * ------------------------------------------------------------------------- */
#define SIGNAL_COMMON_ANODE   0   // 0 = PUL+/DIR+ on GPIO, PUL-/DIR- on GND (push-pull)

#if SIGNAL_COMMON_ANODE
  #define SIG_ON_LEVEL    GPIO_PIN_RESET
  #define SIG_OFF_LEVEL   GPIO_PIN_SET
  #define SIG_GPIO_MODE   GPIO_MODE_OUTPUT_OD
#else
  #define SIG_ON_LEVEL    GPIO_PIN_SET
  #define SIG_OFF_LEVEL   GPIO_PIN_RESET
  #define SIG_GPIO_MODE   GPIO_MODE_OUTPUT_PP
#endif

#define NUM_AXES             3
#define STEPS_PER_MOTOR_REV  2000u    // must match driver DIP switches
#define GEAR_RATIO           10u
#define STEPS_PER_OUTPUT_REV (STEPS_PER_MOTOR_REV * GEAR_RATIO)   // 20000 = 1 crank rev
#define DIR_SETUP_US         20u      // DIR must settle before first STEP edge
#define DIR_FORWARD          1
#define DIR_REVERSE          0

/* ---- Homing ------------------------------------------------------------
 * Sequence: slow seek toward switch -> stop -> pull back -> even slower
 * approach to switch -> that trigger point is position 0.
 * Speeds are step pulses per second. crank deg/s = Hz * 360 / 20000
 *   400 Hz -> 7.2 deg/s       100 Hz -> 1.8 deg/s
 * ---------------------------------------------------------------------- */

/* Direction that moves each crank TOWARD its switch.
 * X is verified. Y and Z are NOT yet verified: if one heads away from its
 * switch, the homing fails after 1.25 revs and the terminal lets you press
 * 'f' to flip the direction and retry. Then set the value here for good. */
#define X_HOME_DIR       DIR_FORWARD
#define Y_HOME_DIR       DIR_FORWARD
#define Z_HOME_DIR       DIR_FORWARD

#define SEEK_HZ          400u          // first (fast) approach
#define BACKOFF_HZ       400u          // speed of the pull-back
#define BACKOFF_STEPS    1000u         // pull-back distance: 1000 steps = 18 deg of crank
#define LATCH_HZ         100u          // second (slow, precise) approach
#define MAX_SEEK_STEPS   (STEPS_PER_OUTPUT_REV * 5u / 4u)   // give up after 1.25 crank revs

/* NC switches: COM to GND, NC to the limit pin.
 * At rest the closed contact holds the pin LOW; pressing the switch (or a broken
 * wire) opens the circuit and the internal pull-up pulls it HIGH = "pressed". */
#define LIMIT_PRESSED_LEVEL  GPIO_PIN_SET

/* ---- Jog / limit-finding ------------------------------------------------
 * Position is counted in motor steps from home (home switch trigger = 0).
 * Positive = AWAY from the home switch. crank deg = steps * 360 / 20000.
 * ---------------------------------------------------------------------- */
#define JOG_HZ          1000u     // jog speed (steps/s) -> crank 3 rpm
#define JOG_START_HZ    200u      // jog ramp starts here
#define JOG_RAMP_STEPS  100u      // ramp length for each jog move

/* ---- Cycle test (all 3 axes together, min <-> max) ------------------------
 * Speeds are step pulses per second: crank rpm = CYCLE_HZ * 60 / 20000.
 *   1500 Hz -> 4.5 rpm  (a 7924-step stroke takes about 5.5 s)
 * ---------------------------------------------------------------------- */
#define CYCLE_HZ            1500u    // cruise speed of each stroke
#define CYCLE_RAMP_STEPS    1500u    // steps used to speed up / slow down each stroke
#define CYCLE_PAUSE_MS      1500u    // pause at each end before reversing

/* ---- Pitch / roll finder ---------------------------------------------------
 * Each move is "weights x step", in per-mille. A positive weight moves that
 * crank AWAY from its home switch (up). The three weights of a pitch or roll
 * move sum to zero, so tilting never changes the average height.
 *
 * GEOMETRY (confirmed: Z is the lone actuator for pitch):
 *   pitch: Z is the single actuator at one end, X and Y share the other end.
 *   roll : X and Y are on opposite sides, Z sits on the centreline.
 * X and Y move 866 per mille (= sin 60) of the roll step, which suits three
 * actuators spaced 120 degrees apart. Swapping the signs only swaps the
 * meaning of pitch + / - or roll + / -, it does not change the results.
 * ---------------------------------------------------------------------- */
#define HEAVE_W_X    1000
#define HEAVE_W_Y    1000
#define HEAVE_W_Z    1000
#define PITCH_W_X    (-500)
#define PITCH_W_Y    (-500)
#define PITCH_W_Z    1000
#define ROLL_W_X     866
#define ROLL_W_Y     (-866)
#define ROLL_W_Z     0

#define TILT_HZ          1000u    // speed of tilt / heave jog moves
#define TILT_RAMP_STEPS  100u     // ramp length of each jog move
#define TILT_START_HEAVE 3960     // steps all axes are raised above the minimum after
                                  // homing, so tilt has room both ways (about mid stroke)

/* Software clamps enforced while jogging, per axis. While searching, leave the
 * defaults (0 .. 1 crank rev). Once you have the real limits, paste the values
 * printed at the end here. */
#define X_SOFT_MIN_STEPS  672     /* measured: 12.09 deg */
#define X_SOFT_MAX_STEPS  8567     /* measured max heave */
#define Y_SOFT_MIN_STEPS  700     /* measured: 12.60 deg */
#define Y_SOFT_MAX_STEPS  8595     /* measured max heave */
#define Z_SOFT_MIN_STEPS  798     /* level minimum, measured outside */
#define Z_SOFT_MAX_STEPS  8693     /* measured max heave */

/* Direction that moves a crank AWAY from its switch */
#define AWAY_DIR(ax)  (((ax)->home_dir == DIR_FORWARD) ? DIR_REVERSE : DIR_FORWARD)

/* USER CODE END PD */

/* USER CODE BEGIN PTD */
typedef struct {
    const char   *name;
    GPIO_TypeDef *step_port;
    uint16_t      step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
    GPIO_TypeDef *limit_port;
    uint16_t      limit_pin;
    int           home_dir;       // DIR level that moves toward the switch
    int32_t       soft_min;       // jog clamps (steps from home)
    int32_t       soft_max;
    int32_t       pos;            // current position, steps from home
    int32_t       min_mark;       // recorded min
    int32_t       max_mark;       // recorded max
    int           max_marked;     // 1 once the max has been marked
} Axis_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PV */
/* Kept only because stm32g4xx_it.c references htim6 in TIM6_DAC_IRQHandler.
 * TIM6 itself is not started in this test. */
TIM_HandleTypeDef htim6;

COM_InitTypeDef BspCOMInit;

static Axis_t axes[NUM_AXES] = {
    { "X", X_STEP_GPIO_Port, X_STEP_Pin, X_DIR_GPIO_Port, X_DIR_Pin,
           X_LIMIT_GPIO_Port, X_LIMIT_Pin, X_HOME_DIR,
           X_SOFT_MIN_STEPS, X_SOFT_MAX_STEPS, 0, 0, 0, 0 },
    { "Y", Y_STEP_GPIO_Port, Y_STEP_Pin, Y_DIR_GPIO_Port, Y_DIR_Pin,
           Y_LIMIT_GPIO_Port, Y_LIMIT_Pin, Y_HOME_DIR,
           Y_SOFT_MIN_STEPS, Y_SOFT_MAX_STEPS, 0, 0, 0, 0 },
    { "Z", Z_STEP_GPIO_Port, Z_STEP_Pin, Z_DIR_GPIO_Port, Z_DIR_Pin,
           Z_LIMIT_GPIO_Port, Z_LIMIT_Pin, Z_HOME_DIR,
           Z_SOFT_MIN_STEPS, Z_SOFT_MAX_STEPS, 0, 0, 0, 0 },
};

/* Jog increment choices in tenths of a degree of crank: 0.5, 1, 2, 5, 10, 20 deg */
static const uint32_t inc_tenths[] = { 5, 10, 20, 50, 100, 200 };
static int inc_idx = 1;                  // start at 1 deg
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void DWT_Init(void);
static void delay_us(uint32_t us);
static void sig_write(GPIO_TypeDef *port, uint16_t pin, int on);
static int  limit_pressed(const Axis_t *ax);
static void step_once(const Axis_t *ax, uint32_t hz);
static void set_dir(const Axis_t *ax, int dir);
static void move_steps(const Axis_t *ax, int dir, uint32_t hz, uint32_t steps);
static int  move_until_pressed(const Axis_t *ax, int dir, uint32_t hz, uint32_t max_steps);
static int  move_until_released(const Axis_t *ax, int dir, uint32_t hz, uint32_t max_steps);
static int  home_axis(const Axis_t *ax);
static uint32_t move_ramped(const Axis_t *ax, int dir, uint32_t steps, uint32_t max_hz,
                            uint32_t ramp, int stop_on_limit);
static void uart_print(const char *s);
static uint8_t uart_getc(void);
static void uart_flush(void);
static void deg_str(char *out, size_t n, int32_t steps);
static const char *dir_name(int dir);
static void print_status(const Axis_t *ax);
static void print_axis_summary(const Axis_t *ax);
static void print_final(void);
static void print_switches(void);
static void jog_by(Axis_t *ax, int32_t delta_steps);
static void return_home(Axis_t *ax);
static void home_with_retry(Axis_t *ax);
static void jog_ui(Axis_t *ax);
static void move_to(Axis_t *ax, int32_t target_steps);
static void print_pos(const Axis_t *ax);
static void run_limit_finder(void);
static void run_min_test(void);
static uint32_t move_all_ramped(int away, uint32_t steps, uint32_t max_hz, uint32_t ramp);
static void jog_all_by(int32_t delta_steps);
static void home_all_to_min(void);
static void print_heave_status(void);
static void heave_ui(void);
static void run_heave_finder(void);
static int  cycle_pause(uint32_t ms);
static void run_cycle_test(void);
static int32_t abs32(int32_t v);
static int32_t spread_of(const int32_t p[NUM_AXES]);
static void move_axes_sync(const int32_t d[NUM_AXES], uint32_t max_hz, uint32_t ramp,
                           uint32_t taken[NUM_AXES]);
static void apply_deltas(const int32_t d[NUM_AXES]);
static void tilt_jog(const int32_t w[NUM_AXES], int32_t step);
static void tilt_to_level(void);
static void tilt_to_bottom(void);
static void home_to_level_limits(void);
static void print_tilt_status(void);
static void print_tilt_results(void);
static void tilt_ui(void);
static void run_tilt_finder(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) { }
}

static void sig_write(GPIO_TypeDef *port, uint16_t pin, int on)
{
    HAL_GPIO_WritePin(port, pin, on ? SIG_ON_LEVEL : SIG_OFF_LEVEL);
}

/* Debounced read of an axis's home switch: pressed twice, 0.5 ms apart. */
static int limit_pressed(const Axis_t *ax)
{
    if (HAL_GPIO_ReadPin(ax->limit_port, ax->limit_pin) != LIMIT_PRESSED_LEVEL) return 0;
    delay_us(500);
    return HAL_GPIO_ReadPin(ax->limit_port, ax->limit_pin) == LIMIT_PRESSED_LEVEL;
}

static void set_dir(const Axis_t *ax, int dir)
{
    sig_write(ax->dir_port, ax->dir_pin, dir);
    delay_us(DIR_SETUP_US);
}

/* One STEP pulse at 'hz' (50% duty). */
static void step_once(const Axis_t *ax, uint32_t hz)
{
    uint32_t period_us = 1000000u / hz;
    uint32_t high_us   = period_us / 2u;

    sig_write(ax->step_port, ax->step_pin, 1);
    delay_us(high_us);
    sig_write(ax->step_port, ax->step_pin, 0);
    delay_us(period_us - high_us);
}

static void move_steps(const Axis_t *ax, int dir, uint32_t hz, uint32_t steps)
{
    set_dir(ax, dir);
    for (uint32_t i = 0; i < steps; i++) step_once(ax, hz);
}

/* Step until the switch is pressed. Returns 1 if hit, 0 if max_steps ran out. */
static int move_until_pressed(const Axis_t *ax, int dir, uint32_t hz, uint32_t max_steps)
{
    set_dir(ax, dir);
    for (uint32_t i = 0; i < max_steps; i++)
    {
        if (limit_pressed(ax)) return 1;
        step_once(ax, hz);
    }
    return limit_pressed(ax);
}

/* Step until the switch is released. Returns 1 if released, 0 if max_steps ran out. */
static int move_until_released(const Axis_t *ax, int dir, uint32_t hz, uint32_t max_steps)
{
    set_dir(ax, dir);
    for (uint32_t i = 0; i < max_steps; i++)
    {
        if (!limit_pressed(ax)) return 1;
        step_once(ax, hz);
    }
    return !limit_pressed(ax);
}

/* Returns 1 on success (crank sitting at the switch trigger point = zero),
 * 0 on failure (switch never hit, or never released). */
static int home_axis(const Axis_t *ax)
{
    const int away = AWAY_DIR(ax);

    /* 0. Already on the switch? Back off it first. */
    if (limit_pressed(ax))
    {
        if (!move_until_released(ax, away, SEEK_HZ, MAX_SEEK_STEPS)) return 0;
        move_steps(ax, away, SEEK_HZ, BACKOFF_STEPS);
    }

    /* 1. Slow seek toward the switch */
    if (!move_until_pressed(ax, ax->home_dir, SEEK_HZ, MAX_SEEK_STEPS)) return 0;
    HAL_Delay(500);

    /* 2. Pull back a little. The switch must release, otherwise something is wrong. */
    move_steps(ax, away, BACKOFF_HZ, BACKOFF_STEPS);
    if (limit_pressed(ax)) return 0;
    HAL_Delay(500);

    /* 3. Very slow second approach: this trigger point is "home" */
    if (!move_until_pressed(ax, ax->home_dir, LATCH_HZ, BACKOFF_STEPS * 2u)) return 0;
    return 1;
}

/* ---------------------------------------------------------------------------
 * Serial helpers
 * ------------------------------------------------------------------------- */
static void uart_print(const char *str)
{
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)str, (uint16_t)strlen(str), 200);
}

/* Blocking read of one character. */
static uint8_t uart_getc(void)
{
    uint8_t c;
    while (HAL_UART_Receive(&hcom_uart[COM1], &c, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);   // recover from overrun
    }
    return c;
}

/* Throw away any keystrokes already waiting, so a prompt really waits. */
static void uart_flush(void)
{
    __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
    __HAL_UART_SEND_REQ(&hcom_uart[COM1], UART_RXDATA_FLUSH_REQUEST);
}

/* Format steps as crank degrees with 2 decimals, e.g. "-12.34" */
static void deg_str(char *out, size_t n, int32_t steps)
{
    int32_t cd = (steps * 9) / 5;                 // centi-degrees: steps*360*100/20000
    uint32_t a = (uint32_t)abs(cd);
    snprintf(out, n, "%s%lu.%02lu", (cd < 0) ? "-" : "", (unsigned long)(a / 100u),
             (unsigned long)(a % 100u));
}

static const char *dir_name(int dir)
{
    return (dir == DIR_FORWARD) ? "DIR_FORWARD" : "DIR_REVERSE";
}

static void print_status(const Axis_t *ax)
{
    char d[16], buf[112];
    uint32_t t = inc_tenths[inc_idx];
    deg_str(d, sizeof(d), ax->pos);
    snprintf(buf, sizeof(buf), "[%s] pos = %ld steps = %s deg | increment = %lu.%lu deg\r\n",
             ax->name, (long)ax->pos, d, (unsigned long)(t / 10u), (unsigned long)(t % 10u));
    uart_print(buf);
}

static void print_axis_summary(const Axis_t *ax)
{
    char d1[16], d2[16], buf[200];

    deg_str(d1, sizeof(d1), ax->min_mark);
    snprintf(buf, sizeof(buf), "\r\n[%s] MIN = %ld steps (%s deg)\r\n",
             ax->name, (long)ax->min_mark, d1);
    uart_print(buf);

    if (ax->max_marked)
    {
        deg_str(d2, sizeof(d2), ax->max_mark);
        snprintf(buf, sizeof(buf), "[%s] MAX = %ld steps (%s deg)\r\n\r\n",
                 ax->name, (long)ax->max_mark, d2);
        uart_print(buf);
    }
    else
    {
        uart_print("MAX not marked yet (jog away from the switch, then press m)\r\n\r\n");
    }
}

/* Live state of all three home switches. With NC wiring each should read
 * "released" when its crank is away from the switch, and "PRESSED" only while
 * the switch is actually being pressed. */
static void print_switches(void)
{
    char buf[120];
    snprintf(buf, sizeof(buf), "Switches:  X = %s   Y = %s   Z = %s\r\n",
             limit_pressed(&axes[0]) ? "PRESSED " : "released",
             limit_pressed(&axes[1]) ? "PRESSED " : "released",
             limit_pressed(&axes[2]) ? "PRESSED " : "released");
    uart_print(buf);
}

/* Results for all axes, with ready-to-paste defines. */
static void print_final(void)
{
    char d1[16], d2[16], buf[200];

    uart_print("\r\n===== RESULTS (crank degrees from home) =====\r\n");
    for (int i = 0; i < NUM_AXES; i++)
    {
        const Axis_t *ax = &axes[i];
        deg_str(d1, sizeof(d1), ax->min_mark);
        if (ax->max_marked)
        {
            deg_str(d2, sizeof(d2), ax->max_mark);
            snprintf(buf, sizeof(buf), "%s: MIN %ld steps (%s deg)   MAX %ld steps (%s deg)\r\n",
                     ax->name, (long)ax->min_mark, d1, (long)ax->max_mark, d2);
        }
        else
        {
            snprintf(buf, sizeof(buf), "%s: MIN %ld steps (%s deg)   MAX not marked\r\n",
                     ax->name, (long)ax->min_mark, d1);
        }
        uart_print(buf);
    }

    uart_print("\r\nPaste into main.c (these are NOT saved, copy them now):\r\n");
    for (int i = 0; i < NUM_AXES; i++)
    {
        const Axis_t *ax = &axes[i];
        if (!ax->max_marked) continue;
        snprintf(buf, sizeof(buf), "#define %s_SOFT_MIN_STEPS  %ld\r\n#define %s_SOFT_MAX_STEPS  %ld\r\n",
                 ax->name, (long)ax->min_mark, ax->name, (long)ax->max_mark);
        uart_print(buf);
    }
    uart_print("\r\nPress p to print this again.\r\n");
}

/* ---------------------------------------------------------------------------
 * Jogging and limit finding
 * ------------------------------------------------------------------------- */

/* Trapezoid move. Returns the number of steps actually taken (may be fewer than
 * requested if stop_on_limit is set and the home switch triggers). */
static uint32_t move_ramped(const Axis_t *ax, int dir, uint32_t steps, uint32_t max_hz,
                            uint32_t ramp, int stop_on_limit)
{
    uint32_t i;
    if (ramp == 0) ramp = 1;
    if (max_hz < JOG_START_HZ) max_hz = JOG_START_HZ;

    set_dir(ax, dir);
    for (i = 0; i < steps; i++)
    {
        if (stop_on_limit && limit_pressed(ax)) break;

        uint32_t from_end = steps - 1u - i;
        uint32_t r = (i < from_end) ? i : from_end;
        if (r > ramp) r = ramp;

        uint32_t hz = JOG_START_HZ + ((max_hz - JOG_START_HZ) * r) / ramp;
        step_once(ax, hz);
    }
    return i;
}

/* Move by delta_steps (+ = away from switch, - = toward it), respecting the
 * software clamps and the home switch. */
static void jog_by(Axis_t *ax, int32_t delta_steps)
{
    int32_t target = ax->pos + delta_steps;

    if (target > ax->soft_max) target = ax->soft_max;
    if (target < ax->soft_min) target = ax->soft_min;

    int32_t delta = target - ax->pos;
    if (delta == 0)
    {
        uart_print("Blocked by software limit.\r\n");
        return;
    }

    if (delta > 0)
    {
        ax->pos += (int32_t)move_ramped(ax, AWAY_DIR(ax), (uint32_t)delta,
                                        JOG_HZ, JOG_RAMP_STEPS, 0);
    }
    else
    {
        uint32_t done = move_ramped(ax, ax->home_dir, (uint32_t)(-delta),
                                    JOG_HZ, JOG_RAMP_STEPS, 1);
        ax->pos -= (int32_t)done;
        if (done < (uint32_t)(-delta))
            uart_print("Home switch hit, stopped.\r\n");
    }
}

/* Drive back to the home switch so this crank is out of the way of the next axis. */
static void return_home(Axis_t *ax)
{
    if (ax->pos > 0)
        move_ramped(ax, ax->home_dir, (uint32_t)ax->pos, JOG_HZ, JOG_RAMP_STEPS, 1);
    ax->pos = 0;
}

/* Home an axis. If it fails, let the user retry or flip the direction. */
static void home_with_retry(Axis_t *ax)
{
    char buf[200];

    while (1)
    {
        snprintf(buf, sizeof(buf), "Homing %s (toward switch = %s)...\r\n",
                 ax->name, dir_name(ax->home_dir));
        uart_print(buf);
        BSP_LED_On(LED_GREEN);

        if (home_axis(ax))
        {
            ax->pos = 0;
            snprintf(buf, sizeof(buf), "%s homed OK.\r\n", ax->name);
            uart_print(buf);
            return;
        }

        snprintf(buf, sizeof(buf),
                 "\r\nHOMING FAILED on %s. If the crank moved AWAY from its switch, the\r\n"
                 "direction is wrong. Other causes: switch wiring, or BACKOFF_STEPS too small.\r\n"
                 "  r = retry   f = flip home direction and retry\r\n", ax->name);
        uart_print(buf);

        uart_flush();
        while (1)
        {
            uint8_t c = uart_getc();
            if (c == 'f' || c == 'F')
            {
                ax->home_dir = (ax->home_dir == DIR_FORWARD) ? DIR_REVERSE : DIR_FORWARD;
                snprintf(buf, sizeof(buf), "Flipped. If this works, set %s_HOME_DIR to %s in main.c.\r\n",
                         ax->name, dir_name(ax->home_dir));
                uart_print(buf);
                break;
            }
            if (c == 'r' || c == 'R') break;
        }
    }
}

/* Interactive jog loop for one axis. Returns when the user presses 'k'. */
static void jog_ui(Axis_t *ax)
{
    int warned = 0;

    uart_print("\r\n=== Jog mode ===\r\n"
               "d = jog away from switch    a = jog toward switch\r\n"
               "+ / - = bigger / smaller increment\r\n"
               "n = mark MIN here   m = mark MAX here   p = print summary\r\n"
               "h = re-home         s = show switch states\r\n"
               "k = done with this axis (it stays in place), go to the next\r\n");
    print_status(ax);
    uart_flush();

    while (1)
    {
        uint8_t c = uart_getc();
        if (c != 'k' && c != 'K') warned = 0;

        uint32_t inc_steps = (inc_tenths[inc_idx] * STEPS_PER_OUTPUT_REV + 1800u) / 3600u;
        if (inc_steps == 0) inc_steps = 1;

        switch (c)
        {
            case 'd': case 'D':  jog_by(ax, (int32_t)inc_steps);   print_status(ax); break;
            case 'a': case 'A':  jog_by(ax, -(int32_t)inc_steps);  print_status(ax); break;

            case '+': case '=':
                if (inc_idx < (int)(sizeof(inc_tenths) / sizeof(inc_tenths[0])) - 1) inc_idx++;
                print_status(ax);
                break;
            case '-': case '_':
                if (inc_idx > 0) inc_idx--;
                print_status(ax);
                break;

            case 'n': case 'N':  ax->min_mark = ax->pos;                    print_axis_summary(ax); break;
            case 'm': case 'M':  ax->max_mark = ax->pos; ax->max_marked = 1; print_axis_summary(ax); break;
            case 'p': case 'P':  print_axis_summary(ax); break;
            case 's': case 'S':  print_switches(); break;

            case 'h': case 'H':
                uart_print("Re-homing...\r\n");
                if (home_axis(ax)) { ax->pos = 0; uart_print("Homed.\r\n"); }
                else               { uart_print("HOMING FAILED.\r\n"); }
                print_status(ax);
                break;

            case 'k': case 'K':
                if (!ax->max_marked && !warned)
                {
                    uart_print("MAX is not marked for this axis. Press k again to continue anyway.\r\n");
                    warned = 1;
                    break;
                }
                print_axis_summary(ax);
                uart_print("Staying where it is (not returning to home).\r\n");
                return;

            default: break;
        }
    }
}

/* Move to an absolute position (steps from home), clamped to the axis's
 * software limits. Toward home it also stops on the home switch. */
static void move_to(Axis_t *ax, int32_t target)
{
    if (target > ax->soft_max) target = ax->soft_max;
    if (target < ax->soft_min) target = ax->soft_min;

    int32_t delta = target - ax->pos;
    if (delta > 0)
    {
        ax->pos += (int32_t)move_ramped(ax, AWAY_DIR(ax), (uint32_t)delta,
                                        JOG_HZ, JOG_RAMP_STEPS, 0);
    }
    else if (delta < 0)
    {
        ax->pos -= (int32_t)move_ramped(ax, ax->home_dir, (uint32_t)(-delta),
                                        JOG_HZ, JOG_RAMP_STEPS, 1);
    }
}

static void print_pos(const Axis_t *ax)
{
    char d[16], buf[96];
    deg_str(d, sizeof(d), ax->pos);
    snprintf(buf, sizeof(buf), "[%s] at %ld steps = %s deg\r\n", ax->name, (long)ax->pos, d);
    uart_print(buf);
}

/* Original mode: home each axis, then jog it to find min/max. Never returns. */
static void run_limit_finder(void)
{
    uart_print("\r\nAxis limit finder: X, then Y, then Z.\r\n");

    /* Open the clamps fully (0 .. 1 crank rev) so the true limits can be searched.
     * The values in the *_SOFT_MIN/MAX defines only apply to the automatic test. */
    for (int i = 0; i < NUM_AXES; i++)
    {
        axes[i].soft_min = 0;
        axes[i].soft_max = (int32_t)STEPS_PER_OUTPUT_REV;
    }

    for (int i = 0; i < NUM_AXES; i++)
    {
        Axis_t *ax = &axes[i];
        char buf[200];

        snprintf(buf, sizeof(buf),
                 "\r\n--- Axis %s ---\r\n"
                 "Clear the area and keep a hand near the %s driver's power switch.\r\n"
                 "Press any key to start homing %s...\r\n", ax->name, ax->name, ax->name);
        uart_print(buf);
        print_switches();
        uart_flush();
        (void)uart_getc();

        if (limit_pressed(ax))
        {
            snprintf(buf, sizeof(buf),
                     "\r\nWARNING: the %s switch reads PRESSED before homing.\r\n"
                     "If the crank is NOT sitting on the switch, the switch is wired wrong\r\n"
                     "(NO instead of NC), a wire is loose, or it is on the wrong pin.\r\n"
                     "Press y to continue anyway, or reset the board to stop.\r\n", ax->name);
            uart_print(buf);
            uart_flush();
            while (1)
            {
                uint8_t k = uart_getc();
                if (k == 'y' || k == 'Y') break;
            }
        }

        home_with_retry(ax);
        jog_ui(ax);
    }

    BSP_LED_On(LED_GREEN);
    print_final();

    /* Review menu: go back into any axis to change its min/max again.
     * The axes are not re-homed, they stay where you left them. */
    while (1)
    {
        uart_print("\r\nx / y / z = jog that axis again (no re-homing)   p = print results\r\n");
        uart_flush();

        uint8_t c = uart_getc();
        int idx = -1;
        if (c == 'x' || c == 'X') idx = 0;
        else if (c == 'y' || c == 'Y') idx = 1;
        else if (c == 'z' || c == 'Z') idx = 2;
        else if (c == 'p' || c == 'P') { print_final(); continue; }

        if (idx < 0) continue;

        jog_ui(&axes[idx]);   // returns when you press k
        print_final();
    }
}

/* Automatic test: home X, Y, Z, then move each to its software minimum.
 * Press r afterwards to repeat. Never returns. */
static void run_min_test(void)
{
    char buf[200];

    while (1)
    {
        uart_print("\r\n=== Automatic homing + move to software minimum ===\r\n");
        print_switches();

        /* Safety: every switch should read released before any motion starts */
        for (int i = 0; i < NUM_AXES; i++)
        {
            Axis_t *ax = &axes[i];
            if (!limit_pressed(ax)) continue;

            snprintf(buf, sizeof(buf),
                     "\r\nWARNING: the %s switch reads PRESSED before homing.\r\n"
                     "If its crank is NOT sitting on the switch, check the wiring (NC to the\r\n"
                     "right pin). Press y to continue anyway, or reset the board to stop.\r\n",
                     ax->name);
            uart_print(buf);
            uart_flush();
            while (1)
            {
                uint8_t k = uart_getc();
                if (k == 'y' || k == 'Y') break;
            }
        }

        /* 1) Home every axis, one at a time */
        for (int i = 0; i < NUM_AXES; i++) home_with_retry(&axes[i]);

        /* 2) Move each axis to its software minimum */
        uart_print("\r\nAll axes homed. Moving to software minimums...\r\n");
        for (int i = 0; i < NUM_AXES; i++)
        {
            move_to(&axes[i], axes[i].soft_min);
            print_pos(&axes[i]);
        }

        BSP_LED_On(LED_GREEN);
        uart_print("\r\nDone. All axes are at their software minimum.\r\n"
                   "Press r to home and repeat the test (reset the board to stop).\r\n");
        uart_flush();
        while (1)
        {
            uint8_t k = uart_getc();
            if (k == 'r' || k == 'R') break;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Heave finder: all three axes move together by the same angle.
 * away = 1 moves every crank away from its home switch (heave up).
 * ------------------------------------------------------------------------- */

/* Trapezoid move of all three axes at once. Returns steps taken (fewer than
 * requested only if a home switch triggers while moving toward home). */
static uint32_t move_all_ramped(int away, uint32_t steps, uint32_t max_hz, uint32_t ramp)
{
    uint32_t i;
    if (ramp == 0) ramp = 1;
    if (max_hz < JOG_START_HZ) max_hz = JOG_START_HZ;

    for (int a = 0; a < NUM_AXES; a++)
        sig_write(axes[a].dir_port, axes[a].dir_pin,
                  away ? AWAY_DIR(&axes[a]) : axes[a].home_dir);
    delay_us(DIR_SETUP_US);

    for (i = 0; i < steps; i++)
    {
        if (!away)
        {
            if (limit_pressed(&axes[0]) || limit_pressed(&axes[1]) || limit_pressed(&axes[2]))
                break;
        }

        uint32_t from_end = steps - 1u - i;
        uint32_t r = (i < from_end) ? i : from_end;
        if (r > ramp) r = ramp;

        uint32_t hz        = JOG_START_HZ + ((max_hz - JOG_START_HZ) * r) / ramp;
        uint32_t period_us = 1000000u / hz;
        uint32_t high_us   = period_us / 2u;

        for (int a = 0; a < NUM_AXES; a++) sig_write(axes[a].step_port, axes[a].step_pin, 1);
        delay_us(high_us);
        for (int a = 0; a < NUM_AXES; a++) sig_write(axes[a].step_port, axes[a].step_pin, 0);
        delay_us(period_us - high_us);
    }
    return i;
}

/* Move all three axes by delta_steps (+ = heave up, away from the switches).
 * Limited so that no axis passes its own soft limits. */
static void jog_all_by(int32_t delta_steps)
{
    int32_t n = (delta_steps < 0) ? -delta_steps : delta_steps;

    for (int a = 0; a < NUM_AXES; a++)
    {
        int32_t room = (delta_steps > 0) ? (axes[a].soft_max - axes[a].pos)
                                         : (axes[a].pos - axes[a].soft_min);
        if (room < n) n = room;
    }
    if (n <= 0)
    {
        uart_print("Blocked by software limit.\r\n");
        return;
    }

    uint32_t done = move_all_ramped(delta_steps > 0, (uint32_t)n, JOG_HZ, JOG_RAMP_STEPS);
    for (int a = 0; a < NUM_AXES; a++)
        axes[a].pos += (delta_steps > 0) ? (int32_t)done : -(int32_t)done;

    if (done < (uint32_t)n) uart_print("A home switch was hit, stopped.\r\n");
}

/* Home X, Y, Z one at a time, then send each to its minimum position. */
static void home_all_to_min(void)
{
    for (int i = 0; i < NUM_AXES; i++) home_with_retry(&axes[i]);

    uart_print("\r\nAll axes homed. Moving to minimum positions...\r\n");
    for (int i = 0; i < NUM_AXES; i++)
    {
        axes[i].soft_max = (int32_t)STEPS_PER_OUTPUT_REV;   // open the top end for the search
        move_to(&axes[i], axes[i].soft_min);
        axes[i].min_mark   = axes[i].soft_min;
        axes[i].max_marked = 0;
        print_pos(&axes[i]);
    }
}

static void print_heave_status(void)
{
    char d[16], buf[200];
    uint32_t t = inc_tenths[inc_idx];
    int32_t off = axes[0].pos - axes[0].soft_min;     // all axes move together

    deg_str(d, sizeof(d), off);
    snprintf(buf, sizeof(buf),
             "heave = +%ld steps (%s deg of crank) | X=%ld Y=%ld Z=%ld | increment = %lu.%lu deg\r\n",
             (long)off, d, (long)axes[0].pos, (long)axes[1].pos, (long)axes[2].pos,
             (unsigned long)(t / 10u), (unsigned long)(t % 10u));
    uart_print(buf);
}

/* Jog all three axes together. Returns when the user presses k. */
static void heave_ui(void)
{
    uart_print("\r\n=== Heave jog (all 3 axes together) ===\r\n"
               "d = heave up (away from switches)    a = heave down (toward minimums)\r\n"
               "+ / - = bigger / smaller increment\r\n"
               "m = mark MAX here (saved for all 3 axes)   n = mark MIN here\r\n"
               "g = go back to the minimum positions       h = re-home all, go to minimums\r\n"
               "s = show switch states   p = print results   k = done\r\n");
    print_heave_status();
    uart_flush();

    while (1)
    {
        uint8_t c = uart_getc();

        uint32_t inc_steps = (inc_tenths[inc_idx] * STEPS_PER_OUTPUT_REV + 1800u) / 3600u;
        if (inc_steps == 0) inc_steps = 1;

        switch (c)
        {
            case 'd': case 'D':  jog_all_by((int32_t)inc_steps);   print_heave_status(); break;
            case 'a': case 'A':  jog_all_by(-(int32_t)inc_steps);  print_heave_status(); break;

            case '+': case '=':
                if (inc_idx < (int)(sizeof(inc_tenths) / sizeof(inc_tenths[0])) - 1) inc_idx++;
                print_heave_status();
                break;
            case '-': case '_':
                if (inc_idx > 0) inc_idx--;
                print_heave_status();
                break;

            case 'm': case 'M':
                for (int a = 0; a < NUM_AXES; a++) { axes[a].max_mark = axes[a].pos; axes[a].max_marked = 1; }
                print_final();
                break;
            case 'n': case 'N':
                for (int a = 0; a < NUM_AXES; a++) axes[a].min_mark = axes[a].pos;
                print_final();
                break;

            case 'g': case 'G':
            {
                int32_t off = axes[0].pos - axes[0].soft_min;
                if (off > 0) jog_all_by(-off);
                print_heave_status();
                break;
            }

            case 'h': case 'H':
                home_all_to_min();
                print_heave_status();
                break;

            case 's': case 'S':  print_switches(); break;
            case 'p': case 'P':  print_final(); break;
            case 'k': case 'K':  return;
            default: break;
        }
    }
}

/* Home all three axes, go to the minimum positions, then jog all three
 * together to find the maximum heave. Never returns. */
static void run_heave_finder(void)
{
    char buf[200];

    uart_print("\r\n=== Heave finder ===\r\n"
               "Homes X, Y, Z, moves each to its minimum, then lets you jog all 3 together.\r\n");
    print_switches();

    for (int i = 0; i < NUM_AXES; i++)
    {
        if (!limit_pressed(&axes[i])) continue;
        snprintf(buf, sizeof(buf),
                 "\r\nWARNING: the %s switch reads PRESSED before homing.\r\n"
                 "If its crank is NOT sitting on the switch, check the wiring (NC to the\r\n"
                 "right pin). Press y to continue anyway, or reset the board to stop.\r\n",
                 axes[i].name);
        uart_print(buf);
        uart_flush();
        while (1)
        {
            uint8_t k = uart_getc();
            if (k == 'y' || k == 'Y') break;
        }
    }

    uart_print("\r\nClear the area and keep a hand near the driver power switches.\r\n"
               "Press any key to start homing...\r\n");
    uart_flush();
    (void)uart_getc();

    home_all_to_min();
    BSP_LED_On(LED_GREEN);

    while (1)
    {
        heave_ui();
        print_final();
        uart_print("\r\nj = jog heave again (no re-homing)   p = print results\r\n");
        uart_flush();
        while (1)
        {
            uint8_t c = uart_getc();
            if (c == 'j' || c == 'J') break;
            if (c == 'p' || c == 'P') print_final();
        }
    }
}

/* ---------------------------------------------------------------------------
 * Cycle test: home, go to min, then repeat min -> max -> min, all 3 together.
 * ------------------------------------------------------------------------- */

/* Cruise speed choices for the cycle (steps/s). 400 steps/s = 7.2 deg/s of crank.
 * The cycle starts slow; + / - change the speed while it runs. */
static const uint32_t cyc_levels[] = { 200, 400, 700, 1000, 1500 };
#define CYC_NUM_LEVELS  ((int)(sizeof(cyc_levels) / sizeof(cyc_levels[0])))
static int cyc_idx = 1;                   // start at 400 steps/s

#define STROKE_DONE  0
#define STROKE_STOP  1
#define STROKE_HIT   2

static int  wobble_stroke(int away);
static void wobble_prepare(void);
static int  cyc_wobble = 0;               // 1 = add the small tilt circles

static void cycle_print_speed(void)
{
    char buf[140];
    uint32_t hz     = cyc_levels[cyc_idx];
    uint32_t deg100 = (hz * 9u) / 5u;                           // deg/s x 100
    int32_t stroke  = axes[0].soft_max - axes[0].soft_min;
    uint32_t secs   = (stroke > 0) ? ((uint32_t)stroke / hz) : 0u;

    snprintf(buf, sizeof(buf),
             "speed %d/%d = %lu steps/s (%lu.%02lu deg/s of crank), about %lu s per stroke\r\n",
             cyc_idx + 1, CYC_NUM_LEVELS, (unsigned long)hz,
             (unsigned long)(deg100 / 100u), (unsigned long)(deg100 % 100u), (unsigned long)secs);
    uart_print(buf);
}

/* Change the cruise speed by one level. */
static void cycle_speed_key(uint8_t k)
{
    if ((k == '+' || k == '=') && cyc_idx < CYC_NUM_LEVELS - 1) cyc_idx++;
    else if ((k == '-' || k == '_') && cyc_idx > 0)             cyc_idx--;
    cycle_print_speed();
}

/* Non-blocking read of one key from the serial port. Returns 0 if none waiting. */
static uint8_t uart_poll(void)
{
    uint8_t c = 0;
    if (__HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_ORE))
        __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
    if (__HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_RXNE))
    {
        if (HAL_UART_Receive(&hcom_uart[COM1], &c, 1, 5) != HAL_OK) c = 0;
    }
    return c;
}

/* Pause for 'ms' at an end of the stroke, watching the serial port.
 * p or space = hold until p / space again, + / - = speed, q = stop (returns 1). */
static int cycle_pause(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < ms)
    {
        uint8_t c;
        if (HAL_UART_Receive(&hcom_uart[COM1], &c, 1, 10) == HAL_OK)
        {
            if (c == 'q' || c == 'Q') return 1;
            if (c == '+' || c == '=' || c == '-' || c == '_') cycle_speed_key(c);
            if (c == 'p' || c == 'P' || c == ' ')
            {
                uart_print("Paused. p / space = resume, + / - = speed, q = stop\r\n");
                while (1)
                {
                    c = uart_getc();
                    if (c == 'p' || c == 'P' || c == ' ') break;
                    if (c == 'q' || c == 'Q') return 1;
                    if (c == '+' || c == '=' || c == '-' || c == '_') cycle_speed_key(c);
                }
                t0 = HAL_GetTick();
            }
        }
        else
        {
            __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
        }
    }
    return 0;
}

/* One stroke of all three axes from where they are now to the max (away = 1) or
 * the min (away = 0). It ramps up and down along the way and listens for keys:
 *   p or space = decelerate to a stop and wait   (press again to resume)
 *   + / -      = faster / slower
 *   q          = decelerate to a stop and give up the stroke
 * Returns STROKE_DONE, STROKE_STOP (user) or STROKE_HIT (a home switch fired). */
static int heave_stroke(int away)
{
    int32_t n = INT32_MAX;

    for (int a = 0; a < NUM_AXES; a++)
    {
        int32_t room = away ? (axes[a].soft_max - axes[a].pos)
                            : (axes[a].pos - axes[a].soft_min);
        if (room < n) n = room;
    }
    if (n <= 0) return STROKE_DONE;

    const float slope = (float)(CYCLE_HZ - JOG_START_HZ) / (float)CYCLE_RAMP_STEPS;  // Hz per step
    float hz = (float)JOG_START_HZ;
    int paused = 0, stopping = 0, result = STROKE_DONE;
    int32_t done = 0;

    for (int a = 0; a < NUM_AXES; a++)
        sig_write(axes[a].dir_port, axes[a].dir_pin,
                  away ? AWAY_DIR(&axes[a]) : axes[a].home_dir);
    delay_us(DIR_SETUP_US);

    for (int32_t i = 0; i < n; i++)
    {
        if (!away)
        {
            if (limit_pressed(&axes[0]) || limit_pressed(&axes[1]) || limit_pressed(&axes[2]))
            {
                result = STROKE_HIT;
                break;
            }
        }

        uint8_t k = uart_poll();
        if (k == 'p' || k == 'P' || k == ' ')      paused = 1;
        else if (k == 'q' || k == 'Q')             stopping = 1;
        else if (k == '+' || k == '=' || k == '-' || k == '_') cycle_speed_key(k);

        /* Target speed: the chosen cruise speed, but slowing down in time for the end */
        float cruise  = (float)cyc_levels[cyc_idx];
        float end_lim = (float)JOG_START_HZ + slope * (float)(n - 1 - i);
        float target  = (cruise < end_lim) ? cruise : end_lim;
        if (paused || stopping) target = (float)JOG_START_HZ;

        if (hz < target)      { hz += slope; if (hz > target) hz = target; }
        else if (hz > target) { hz -= slope; if (hz < target) hz = target; }

        if ((paused || stopping) && hz <= (float)JOG_START_HZ + 0.5f)
        {
            if (stopping) { result = STROKE_STOP; break; }

            char buf[140];
            snprintf(buf, sizeof(buf),
                     "Paused at X=%ld Y=%ld Z=%ld. p / space = resume, + / - = speed, q = stop\r\n",
                     (long)(axes[0].pos + (away ? done : -done)),
                     (long)(axes[1].pos + (away ? done : -done)),
                     (long)(axes[2].pos + (away ? done : -done)));
            uart_print(buf);
            while (paused)
            {
                uint8_t c = uart_getc();
                if (c == 'p' || c == 'P' || c == ' ') paused = 0;
                else if (c == 'q' || c == 'Q')        { paused = 0; stopping = 1; }
                else if (c == '+' || c == '=' || c == '-' || c == '_') cycle_speed_key(c);
            }
            if (stopping) { result = STROKE_STOP; break; }
        }

        uint32_t period_us = 1000000u / (uint32_t)hz;
        uint32_t high_us   = period_us / 2u;

        for (int a = 0; a < NUM_AXES; a++) sig_write(axes[a].step_port, axes[a].step_pin, 1);
        delay_us(high_us);
        for (int a = 0; a < NUM_AXES; a++) sig_write(axes[a].step_port, axes[a].step_pin, 0);
        delay_us(period_us - high_us);
        done++;
    }

    for (int a = 0; a < NUM_AXES; a++)
        axes[a].pos += away ? done : -done;
    return result;
}

/* Never returns. */
static void run_cycle_test(void)
{
    static const int32_t max_steps[NUM_AXES] = { X_SOFT_MAX_STEPS, Y_SOFT_MAX_STEPS, Z_SOFT_MAX_STEPS };
    char buf[200];
    uint32_t cycles = 0;
    int need_home = 1, away = 1, at_end = 1;

    uart_print(cyc_wobble ? "\r\n=== Heave cycle with small tilt circles ===\r\n"
                          : "\r\n=== Heave cycle ===\r\n");
    uart_print(""
               "Homes X, Y, Z, goes to the minimums, then repeats min -> max -> min\r\n"
               "with all three axes together, pausing at each end.\r\n"
               "Keys while it runs:  p or space = pause / resume (slows to a stop first)\r\n"
               "                     + / - = faster / slower     q = stop\r\n");
    print_switches();

    for (int i = 0; i < NUM_AXES; i++)
    {
        if (!limit_pressed(&axes[i])) continue;
        snprintf(buf, sizeof(buf),
                 "\r\nWARNING: the %s switch reads PRESSED before homing.\r\n"
                 "If its crank is NOT sitting on the switch, check the wiring (NC to the\r\n"
                 "right pin). Press y to continue anyway, or reset the board to stop.\r\n",
                 axes[i].name);
        uart_print(buf);
        uart_flush();
        while (1)
        {
            uint8_t k = uart_getc();
            if (k == 'y' || k == 'Y') break;
        }
    }

    uart_print("\r\nClear the area and keep a hand near the driver power switches.\r\n"
               "Press any key to start homing...\r\n");
    uart_flush();
    (void)uart_getc();

    while (1)
    {
        int stop = 0;

        if (need_home)
        {
            home_all_to_min();
            for (int a = 0; a < NUM_AXES; a++) axes[a].soft_max = max_steps[a];
            if (cyc_wobble) wobble_prepare();
            cycles = 0;
            need_home = 0;
            away = 1;
            at_end = 1;
            BSP_LED_On(LED_GREEN);
            uart_print("\r\nCycling slowly. p / space = pause, + / - = speed, q = stop.\r\n");
            cycle_print_speed();
            uart_flush();
        }

        if (at_end) stop = cycle_pause(CYCLE_PAUSE_MS);           // at an end of the stroke

        if (!stop)
        {
            uart_print(away ? "-> max\r\n" : "-> min\r\n");
            int r = cyc_wobble ? wobble_stroke(away) : heave_stroke(away);
            if (r == STROKE_DONE)
            {
                at_end = 1;
                if (!away)
                {
                    cycles++;
                    snprintf(buf, sizeof(buf), "cycle %lu done | X=%ld Y=%ld Z=%ld\r\n",
                             (unsigned long)cycles, (long)axes[0].pos, (long)axes[1].pos, (long)axes[2].pos);
                    uart_print(buf);
                }
                away = !away;
            }
            else
            {
                at_end = 0;
                stop = 1;
                if (r == STROKE_HIT)
                    uart_print("A home switch was hit unexpectedly, stopped.\r\n");
            }
        }

        if (stop)
        {
            snprintf(buf, sizeof(buf),
                     "\r\nStopped at X=%ld Y=%ld Z=%ld.\r\n"
                     "c = continue cycling   h = re-home and restart\r\n",
                     (long)axes[0].pos, (long)axes[1].pos, (long)axes[2].pos);
            uart_print(buf);
            uart_flush();
            while (1)
            {
                uint8_t k = uart_getc();
                if (k == 'c' || k == 'C') break;
                if (k == 'h' || k == 'H') { need_home = 1; break; }
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * Pitch / roll finder
 * ------------------------------------------------------------------------- */
static const int32_t W_HEAVE[NUM_AXES] = { HEAVE_W_X, HEAVE_W_Y, HEAVE_W_Z };
static const int32_t W_PITCH[NUM_AXES] = { PITCH_W_X, PITCH_W_Y, PITCH_W_Z };
static const int32_t W_ROLL[NUM_AXES]  = { ROLL_W_X,  ROLL_W_Y,  ROLL_W_Z  };
static const int32_t level_max[NUM_AXES] = { X_SOFT_MAX_STEPS, Y_SOFT_MAX_STEPS, Z_SOFT_MAX_STEPS };

/* Recorded tilt limits: 0 = pitch +, 1 = pitch -, 2 = roll +, 3 = roll - */
static struct { int32_t pos[NUM_AXES]; int set; } tilt_marks[4];
static const char *const tilt_names[4] = { "PITCH +", "PITCH -", "ROLL +", "ROLL -" };

static int32_t abs32(int32_t v) { return (v < 0) ? -v : v; }

/* Height difference between the highest and lowest axis. */
static int32_t spread_of(const int32_t p[NUM_AXES])
{
    int32_t lo = p[0], hi = p[0];
    for (int a = 1; a < NUM_AXES; a++)
    {
        if (p[a] < lo) lo = p[a];
        if (p[a] > hi) hi = p[a];
    }
    return hi - lo;
}

/* Move the three axes by different step counts d[a] (+ = away from the switch)
 * so that they all start and finish together (Bresenham interleave). The master
 * is whichever axis has the most steps. taken[a] returns the steps really made;
 * a move toward home stops early if that axis's home switch triggers. */
static void move_axes_sync(const int32_t d[NUM_AXES], uint32_t max_hz, uint32_t ramp,
                           uint32_t taken[NUM_AXES])
{
    uint32_t n[NUM_AXES], acc[NUM_AXES], master = 0;

    for (int a = 0; a < NUM_AXES; a++)
    {
        n[a] = (uint32_t)abs32(d[a]);
        acc[a] = 0;
        taken[a] = 0;
        if (n[a] > master) master = n[a];
    }
    if (master == 0) return;
    if (ramp == 0) ramp = 1;
    if (max_hz < JOG_START_HZ) max_hz = JOG_START_HZ;

    for (int a = 0; a < NUM_AXES; a++)
    {
        if (n[a] == 0) continue;
        sig_write(axes[a].dir_port, axes[a].dir_pin,
                  (d[a] > 0) ? AWAY_DIR(&axes[a]) : axes[a].home_dir);
    }
    delay_us(DIR_SETUP_US);

    for (uint32_t i = 0; i < master; i++)
    {
        int stop = 0;
        for (int a = 0; a < NUM_AXES; a++)
            if (d[a] < 0 && limit_pressed(&axes[a])) stop = 1;
        if (stop) break;

        uint32_t from_end = master - 1u - i;
        uint32_t r = (i < from_end) ? i : from_end;
        if (r > ramp) r = ramp;

        uint32_t hz        = JOG_START_HZ + ((max_hz - JOG_START_HZ) * r) / ramp;
        uint32_t period_us = 1000000u / hz;
        uint32_t high_us   = period_us / 2u;

        int step[NUM_AXES];
        for (int a = 0; a < NUM_AXES; a++)
        {
            step[a] = 0;
            acc[a] += n[a];
            if (acc[a] >= master) { acc[a] -= master; step[a] = 1; taken[a]++; }
        }

        for (int a = 0; a < NUM_AXES; a++) if (step[a]) sig_write(axes[a].step_port, axes[a].step_pin, 1);
        delay_us(high_us);
        for (int a = 0; a < NUM_AXES; a++) if (step[a]) sig_write(axes[a].step_port, axes[a].step_pin, 0);
        delay_us(period_us - high_us);
    }
}

/* Make the move d[] and update every axis position. */
static void apply_deltas(const int32_t d[NUM_AXES])
{
    uint32_t taken[NUM_AXES];
    int cut_short = 0;

    move_axes_sync(d, TILT_HZ, TILT_RAMP_STEPS, taken);

    for (int a = 0; a < NUM_AXES; a++)
    {
        axes[a].pos += (d[a] > 0) ? (int32_t)taken[a] : -(int32_t)taken[a];
        if (taken[a] < (uint32_t)abs32(d[a])) cut_short = 1;
    }
    if (cut_short) uart_print("A home switch was hit, stopped.\r\n");
}

/* Heave, pitch or roll jog: step the axes by (weight x step / 1000), reduced if
 * needed so that no axis goes past its own min or max. */
static void tilt_jog(const int32_t w[NUM_AXES], int32_t step)
{
    int32_t d[NUM_AXES];
    int32_t ratio = 1000;                 // per mille of the requested move that fits
    int any = 0;
    int limiting = -1;

    for (int a = 0; a < NUM_AXES; a++)
    {
        int32_t v = w[a] * step;
        d[a] = (v >= 0) ? (v + 500) / 1000 : -((-v + 500) / 1000);
    }

    for (int a = 0; a < NUM_AXES; a++)
    {
        int32_t room;
        if (d[a] > 0)      room = axes[a].soft_max - axes[a].pos;
        else if (d[a] < 0) room = axes[a].pos - axes[a].soft_min;
        else               continue;
        if (room < 0) room = 0;

        int32_t mag = abs32(d[a]);
        if (room < mag)
        {
            int32_t r = (room * 1000) / mag;
            if (r < ratio) { ratio = r; limiting = a; }
        }
    }

    for (int a = 0; a < NUM_AXES; a++)
    {
        if (ratio < 1000) d[a] = (d[a] * ratio) / 1000;
        if (d[a] != 0) any = 1;
    }
    if (!any)
    {
        char b[200];
        if (limiting >= 0)
        {
            snprintf(b, sizeof(b),
                     "Blocked: %s is at its limit (pos %ld, range %ld..%ld). Raise or lower\r\n"
                     "the platform (u / j) so every axis has room, then tilt.\r\n",
                     axes[limiting].name, (long)axes[limiting].pos,
                     (long)axes[limiting].soft_min, (long)axes[limiting].soft_max);
            uart_print(b);
        }
        else
            uart_print("Blocked by software limit.\r\n");
        return;
    }
    apply_deltas(d);
}

/* Remove any tilt: all three axes to the same height above their minimums. */
static void tilt_to_level(void)
{
    int32_t off = 0, d[NUM_AXES];

    for (int a = 0; a < NUM_AXES; a++) off += axes[a].pos - axes[a].soft_min;
    off /= NUM_AXES;
    for (int a = 0; a < NUM_AXES; a++) d[a] = (axes[a].soft_min + off) - axes[a].pos;
    apply_deltas(d);
}

/* Level, and all the way down to the minimum positions. */
static void tilt_to_bottom(void)
{
    int32_t d[NUM_AXES];
    for (int a = 0; a < NUM_AXES; a++) d[a] = axes[a].soft_min - axes[a].pos;
    apply_deltas(d);
}

/* Home all axes, go to the (level) minimum, then apply the level max limits. */
static void home_to_level_limits(void)
{
    home_all_to_min();
    for (int a = 0; a < NUM_AXES; a++) axes[a].soft_max = level_max[a];
}

static void print_tilt_status(void)
{
    char d[16], buf[200];
    uint32_t t = inc_tenths[inc_idx];
    int32_t p[NUM_AXES] = { axes[0].pos, axes[1].pos, axes[2].pos };

    deg_str(d, sizeof(d), spread_of(p));
    snprintf(buf, sizeof(buf),
             "X=%ld Y=%ld Z=%ld | spread %s deg | increment = %lu.%lu deg\r\n",
             (long)axes[0].pos, (long)axes[1].pos, (long)axes[2].pos, d,
             (unsigned long)(t / 10u), (unsigned long)(t % 10u));
    uart_print(buf);
}

static void print_tilt_results(void)
{
    char d[16], buf[200];
    int32_t worst = 0;
    int any = 0;

    uart_print("\r\n===== TILT LIMITS (steps from home) =====\r\n");
    for (int i = 0; i < 4; i++)
    {
        if (!tilt_marks[i].set)
        {
            snprintf(buf, sizeof(buf), "%s: not marked\r\n", tilt_names[i]);
            uart_print(buf);
            continue;
        }
        int32_t sp = spread_of(tilt_marks[i].pos);
        deg_str(d, sizeof(d), sp);
        snprintf(buf, sizeof(buf), "%s: X=%ld Y=%ld Z=%ld | spread %ld steps (%s deg of crank)\r\n",
                 tilt_names[i], (long)tilt_marks[i].pos[0], (long)tilt_marks[i].pos[1],
                 (long)tilt_marks[i].pos[2], (long)sp, d);
        uart_print(buf);
        if (sp > worst) worst = sp;
        any = 1;
    }

    if (any)
    {
        deg_str(d, sizeof(d), worst);
        snprintf(buf, sizeof(buf),
                 "\r\nLargest height difference between any two cranks:\r\n"
                 "#define MAX_AXIS_SPREAD_STEPS  %ld   /* %s deg of crank */\r\n"
                 "(copy it now, it is not saved)\r\n", (long)worst, d);
        uart_print(buf);
    }
}

/* Jog heave, pitch and roll. Returns when the user presses k. */
static void tilt_ui(void)
{
    uart_print("\r\n=== Pitch / roll finder ===\r\n"
               "u / j = heave up / down (all 3 together)\r\n"
               "w / s = pitch + / -          d / a = roll + / -\r\n"
               "+ / - = bigger / smaller increment\r\n"
               "g = level out (remove tilt)    b = back to the bottom (level, minimum)\r\n"
               "1 = mark PITCH +   2 = mark PITCH -   3 = mark ROLL +   4 = mark ROLL -\r\n"
               "p = print results   v = show switch states   h = re-home   k = done\r\n");
    print_tilt_status();
    uart_flush();

    while (1)
    {
        uint8_t c = uart_getc();

        int32_t inc = (int32_t)((inc_tenths[inc_idx] * STEPS_PER_OUTPUT_REV + 1800u) / 3600u);
        if (inc == 0) inc = 1;

        switch (c)
        {
            case 'u': case 'U':  tilt_jog(W_HEAVE,  inc);  print_tilt_status(); break;
            case 'j': case 'J':  tilt_jog(W_HEAVE, -inc);  print_tilt_status(); break;
            case 'w': case 'W':  tilt_jog(W_PITCH,  inc);  print_tilt_status(); break;
            case 's': case 'S':  tilt_jog(W_PITCH, -inc);  print_tilt_status(); break;
            case 'd': case 'D':  tilt_jog(W_ROLL,   inc);  print_tilt_status(); break;
            case 'a': case 'A':  tilt_jog(W_ROLL,  -inc);  print_tilt_status(); break;

            case '+': case '=':
                if (inc_idx < (int)(sizeof(inc_tenths) / sizeof(inc_tenths[0])) - 1) inc_idx++;
                print_tilt_status();
                break;
            case '-': case '_':
                if (inc_idx > 0) inc_idx--;
                print_tilt_status();
                break;

            case 'g': case 'G':  tilt_to_level();   print_tilt_status(); break;
            case 'b': case 'B':  tilt_to_bottom();  print_tilt_status(); break;

            case '1': case '2': case '3': case '4':
            {
                int m = c - '1';
                for (int a = 0; a < NUM_AXES; a++) tilt_marks[m].pos[a] = axes[a].pos;
                tilt_marks[m].set = 1;
                print_tilt_results();
                break;
            }

            case 'h': case 'H':
                home_to_level_limits();
                {
                    int32_t d[NUM_AXES];
                    for (int a = 0; a < NUM_AXES; a++) d[a] = TILT_START_HEAVE;
                    apply_deltas(d);
                }
                print_tilt_status();
                break;

            case 'v': case 'V':  print_switches();      break;
            case 'p': case 'P':  print_tilt_results();  break;
            case 'k': case 'K':  return;
            default: break;
        }
    }
}

/* Home all three axes, go to the level minimum, then jog heave / pitch / roll.
 * Never returns. */
static void run_tilt_finder(void)
{
    char buf[200];

    uart_print("\r\n=== Pitch / roll finder ===\r\n"
               "Homes X, Y, Z, goes to the level minimum, then lets you tilt the platform.\r\n"
               "It then raises to mid-height (tilt is impossible at the very bottom).\r\n");
    print_switches();

    for (int i = 0; i < NUM_AXES; i++)
    {
        if (!limit_pressed(&axes[i])) continue;
        snprintf(buf, sizeof(buf),
                 "\r\nWARNING: the %s switch reads PRESSED before homing.\r\n"
                 "If its crank is NOT sitting on the switch, check the wiring (NC to the\r\n"
                 "right pin). Press y to continue anyway, or reset the board to stop.\r\n",
                 axes[i].name);
        uart_print(buf);
        uart_flush();
        while (1)
        {
            uint8_t k = uart_getc();
            if (k == 'y' || k == 'Y') break;
        }
    }

    uart_print("\r\nClear the area and keep a hand near the driver power switches.\r\n"
               "Press any key to start homing...\r\n");
    uart_flush();
    (void)uart_getc();

    home_to_level_limits();
    uart_print("Raising to mid-height so tilt has room both ways...\r\n");
    {
        int32_t d[NUM_AXES];
        for (int a = 0; a < NUM_AXES; a++) d[a] = TILT_START_HEAVE;
        apply_deltas(d);
    }
    BSP_LED_On(LED_GREEN);

    while (1)
    {
        tilt_ui();
        print_tilt_results();
        uart_print("\r\nj = jog again (no re-homing)   p = print results\r\n");
        uart_flush();
        while (1)
        {
            uint8_t c = uart_getc();
            if (c == 'j' || c == 'J') break;
            if (c == 'p' || c == 'P') print_tilt_results();
        }
    }
}

/* ---------------------------------------------------------------------------
 * Heave cycle with small tilt circles
 *
 * The platform does the normal slow min <-> max heave, and on top of that every
 * leg moves a little up and down, 120 deg apart in phase, so the platform tilts
 * in a slow circle (pitch and roll together). The heave range is shortened by
 * WOBBLE_AMP_STEPS at each end so no leg ever goes past its own min or max.
 * ------------------------------------------------------------------------- */
#define WOBBLE_AMP_STEPS    100      // each leg moves +/- this many steps (100 = 1.8 deg of crank)
#define WOBBLE_PERIOD_S     6        // seconds for one full tilt circle
#define WOBBLE_TICK_HZ      2000u    // control tick: each leg makes at most 1 step per tick

static float wob_h = 0.0f;           // heave above the lowest position (steps)
static float wob_c = 1.0f;           // cos / sin of the tilt-circle phase
static float wob_s = 0.0f;

/* Heave travel left after leaving room for the wobble at both ends. */
static int32_t wobble_span(void)
{
    int32_t n = INT32_MAX;
    for (int a = 0; a < NUM_AXES; a++)
    {
        int32_t r = axes[a].soft_max - axes[a].soft_min;
        if (r < n) n = r;
    }
    return n - 2 * WOBBLE_AMP_STEPS;
}

/* Where leg a should be for the current heave and tilt-circle phase. */
static int32_t wobble_target(int a)
{
    float off = (float)WOBBLE_AMP_STEPS *
                ((float)W_PITCH[a] * wob_c + (float)W_ROLL[a] * wob_s) / 1000.0f;
    float t   = (float)(axes[a].soft_min + WOBBLE_AMP_STEPS) + wob_h + off;
    int32_t v = (int32_t)(t + 0.5f);

    if (v < axes[a].soft_min) v = axes[a].soft_min;
    if (v > axes[a].soft_max) v = axes[a].soft_max;
    return v;
}

/* Start of the wobble cycle: heave at the bottom, circle phase 0. The legs are
 * moved there together from wherever they are (normally the minimums). */
static void wobble_prepare(void)
{
    int32_t d[NUM_AXES];

    wob_h = 0.0f;
    wob_c = 1.0f;
    wob_s = 0.0f;
    for (int a = 0; a < NUM_AXES; a++) d[a] = wobble_target(a) - axes[a].pos;
    apply_deltas(d);
}

/* Same as heave_stroke(), but with the tilt circles added. Runs on a fixed
 * WOBBLE_TICK_HZ clock; each tick the heave and the circle phase advance and
 * each leg takes one step toward its own target if it is not there yet. Same
 * keys: p / space = pause, + / - = speed, q = stop. */
static int wobble_stroke(int away)
{
    const float n = (float)wobble_span();
    if (n <= 0.0f) return STROKE_DONE;

    const float end_h  = away ? n : 0.0f;
    const float dir_f  = away ? 1.0f : -1.0f;
    const float dt     = 1.0f / (float)WOBBLE_TICK_HZ;
    const float slope  = (float)(CYCLE_HZ - JOG_START_HZ) / (float)CYCLE_RAMP_STEPS;
    const float dphi   = 6.2831853f / ((float)WOBBLE_PERIOD_S * (float)WOBBLE_TICK_HZ);
    const float cd     = 1.0f - dphi * dphi / 2.0f;
    const float sd     = dphi - dphi * dphi * dphi / 6.0f;
    const uint32_t tick_cyc = SystemCoreClock / WOBBLE_TICK_HZ;

    float v = (float)JOG_START_HZ;
    int paused = 0, stopping = 0, result = STROKE_DONE;
    int dirs[NUM_AXES] = { -1, -1, -1 };          // last direction written to each DIR pin

    while (result == STROKE_DONE)
    {
        uint32_t t0 = DWT->CYCCNT;
        float remain = (end_h - wob_h) * dir_f;   // heave steps still to go
        if (remain <= 0.0f) break;

        uint8_t k = uart_poll();
        if (k == 'p' || k == 'P' || k == ' ')      paused = 1;
        else if (k == 'q' || k == 'Q')             stopping = 1;
        else if (k == '+' || k == '=' || k == '-' || k == '_') cycle_speed_key(k);

        /* Heave speed: chosen cruise speed, slowing down in time for the end */
        float cruise  = (float)cyc_levels[cyc_idx];
        float end_lim = (float)JOG_START_HZ + slope * remain;
        float target  = (cruise < end_lim) ? cruise : end_lim;
        if (paused || stopping) target = (float)JOG_START_HZ;

        float dv = slope * v * dt;                // same ramp as the plain heave cycle
        if (v < target)      { v += dv; if (v > target) v = target; }
        else if (v > target) { v -= dv; if (v < target) v = target; }

        if ((paused || stopping) && v <= (float)JOG_START_HZ + 0.5f)
        {
            if (stopping) { result = STROKE_STOP; break; }

            char buf[140];
            snprintf(buf, sizeof(buf),
                     "Paused at X=%ld Y=%ld Z=%ld. p / space = resume, + / - = speed, q = stop\r\n",
                     (long)axes[0].pos, (long)axes[1].pos, (long)axes[2].pos);
            uart_print(buf);
            while (paused)
            {
                uint8_t c = uart_getc();
                if (c == 'p' || c == 'P' || c == ' ') paused = 0;
                else if (c == 'q' || c == 'Q')        { paused = 0; stopping = 1; }
                else if (c == '+' || c == '=' || c == '-' || c == '_') cycle_speed_key(c);
            }
            if (stopping) { result = STROKE_STOP; break; }
            t0 = DWT->CYCCNT;
        }

        /* Advance the heave and rotate the tilt-circle phase by one tick */
        wob_h += dir_f * v * dt;
        if ((end_h - wob_h) * dir_f < 0.0f) wob_h = end_h;

        float nc = wob_c * cd - wob_s * sd;
        float ns = wob_s * cd + wob_c * sd;
        float kk = 1.5f - 0.5f * (nc * nc + ns * ns);   // keep the length at 1
        wob_c = nc * kk;
        wob_s = ns * kk;

        /* One step for each leg that is not at its target */
        int step[NUM_AXES] = { 0, 0, 0 };
        int changed = 0;
        for (int a = 0; a < NUM_AXES; a++)
        {
            int32_t diff = wobble_target(a) - axes[a].pos;
            if (diff == 0) continue;

            int want_away = (diff > 0);
            if (!want_away && limit_pressed(&axes[a]))
            {
                result = STROKE_HIT;
                break;
            }
            if (dirs[a] != want_away)
            {
                sig_write(axes[a].dir_port, axes[a].dir_pin,
                          want_away ? AWAY_DIR(&axes[a]) : axes[a].home_dir);
                dirs[a] = want_away;
                changed = 1;
            }
            step[a] = want_away ? 1 : -1;
        }
        if (result != STROKE_DONE) break;
        if (changed) delay_us(DIR_SETUP_US);

        for (int a = 0; a < NUM_AXES; a++)
            if (step[a]) sig_write(axes[a].step_port, axes[a].step_pin, 1);
        while ((DWT->CYCCNT - t0) < tick_cyc / 2u) { }
        for (int a = 0; a < NUM_AXES; a++)
            if (step[a]) { sig_write(axes[a].step_port, axes[a].step_pin, 0); axes[a].pos += step[a]; }
        while ((DWT->CYCCNT - t0) < tick_cyc) { }
    }

    return result;
}

/* ---------------------------------------------------------------------------
 * Level finder: home all legs, then jog each leg on its own (or all together)
 * until the platform is level, and record the leg positions.
 * ------------------------------------------------------------------------- */

/* How far (steps) the jog clamps are opened below the old minimum and above
 * the old maximum while searching. 300 steps = 5.4 deg of crank. */
#define LEVEL_MARGIN_STEPS  300

static const int32_t level_min[NUM_AXES] = { X_SOFT_MIN_STEPS, Y_SOFT_MIN_STEPS, Z_SOFT_MIN_STEPS };

/* Jog step choices in hundredths of a degree of crank */
static const uint32_t lvl_inc_cdeg[] = { 5, 10, 25, 50, 100, 250, 500 };
static int lvl_idx = 3;                       // start at 0.50 deg

static int32_t level_pos[NUM_AXES];
static int     level_set[NUM_AXES];

static int32_t lvl_inc_steps(void)
{
    int32_t s = (int32_t)((lvl_inc_cdeg[lvl_idx] * STEPS_PER_OUTPUT_REV + 18000u) / 36000u);
    return (s < 1) ? 1 : s;
}

static void level_open_limits(void)
{
    for (int a = 0; a < NUM_AXES; a++)
    {
        int32_t lo = level_min[a] - LEVEL_MARGIN_STEPS;
        axes[a].soft_min = (lo < 0) ? 0 : lo;
        axes[a].soft_max = level_max[a] + LEVEL_MARGIN_STEPS;
    }
}

/* Home X, Y, Z, go to the old minimums, then open the clamps a little. */
static void level_rehome(void)
{
    for (int a = 0; a < NUM_AXES; a++) axes[a].soft_min = level_min[a];
    home_to_level_limits();
    level_open_limits();
}

static void level_status(int sel)
{
    char buf[200];
    uint32_t c = lvl_inc_cdeg[lvl_idx];
    snprintf(buf, sizeof(buf),
             "[%s] selected | X=%ld Y=%ld Z=%ld | step = %lu.%02lu deg (%ld steps)\r\n",
             axes[sel].name, (long)axes[0].pos, (long)axes[1].pos, (long)axes[2].pos,
             (unsigned long)(c / 100u), (unsigned long)(c % 100u), (long)lvl_inc_steps());
    uart_print(buf);
}

static void level_results(void)
{
    char d[16], h[16], buf[220];
    int all = 1;

    uart_print("\r\n===== LEVEL POSITIONS (steps from the home switch) =====\r\n");
    for (int a = 0; a < NUM_AXES; a++)
    {
        if (!level_set[a])
        {
            snprintf(buf, sizeof(buf), "%s: not marked\r\n", axes[a].name);
            uart_print(buf);
            all = 0;
            continue;
        }
        deg_str(d, sizeof(d), level_pos[a]);
        deg_str(h, sizeof(h), level_pos[a] - level_min[a]);
        snprintf(buf, sizeof(buf), "%s: LEVEL at %ld steps (%s deg) | %s deg above the old minimum (%ld)\r\n",
                 axes[a].name, (long)level_pos[a], d, h, (long)level_min[a]);
        uart_print(buf);
    }

    if (all)
    {
        uart_print("\r\nCopy these now (they are not saved):\r\n");
        for (int a = 0; a < NUM_AXES; a++)
        {
            snprintf(buf, sizeof(buf), "#define %s_LEVEL_STEPS  %ld\r\n", axes[a].name, (long)level_pos[a]);
            uart_print(buf);
        }
    }
}

static void level_ui(void)
{
    int sel = 0;

    uart_print("\r\n=== Level finder ===\r\n"
               "x / y / z = select a leg      d / a = selected leg up / down\r\n"
               "u / j = all three legs up / down together\r\n"
               "+ / - = bigger / smaller step\r\n"
               "m = mark selected leg LEVEL here    n = mark all three legs LEVEL here\r\n"
               "g = go to the marked level positions   p = print results\r\n"
               "v = show switch states   h = re-home   (up = away from the switch)\r\n");
    level_status(sel);
    uart_flush();

    while (1)
    {
        uint8_t c = uart_getc();
        int32_t inc = lvl_inc_steps();

        switch (c)
        {
            case 'x': case 'X':  sel = 0; level_status(sel); break;
            case 'y': case 'Y':  sel = 1; level_status(sel); break;
            case 'z': case 'Z':  sel = 2; level_status(sel); break;

            case 'd': case 'D':  jog_by(&axes[sel],  inc);  level_status(sel); break;
            case 'a': case 'A':  jog_by(&axes[sel], -inc);  level_status(sel); break;
            case 'u': case 'U':  jog_all_by( inc);          level_status(sel); break;
            case 'j': case 'J':  jog_all_by(-inc);          level_status(sel); break;

            case '+': case '=':
                if (lvl_idx < (int)(sizeof(lvl_inc_cdeg) / sizeof(lvl_inc_cdeg[0])) - 1) lvl_idx++;
                level_status(sel);
                break;
            case '-': case '_':
                if (lvl_idx > 0) lvl_idx--;
                level_status(sel);
                break;

            case 'm': case 'M':
                level_pos[sel] = axes[sel].pos;
                level_set[sel] = 1;
                level_results();
                break;
            case 'n': case 'N':
                for (int a = 0; a < NUM_AXES; a++) { level_pos[a] = axes[a].pos; level_set[a] = 1; }
                level_results();
                break;

            case 'g': case 'G':
            {
                int32_t d[NUM_AXES];
                int ok = 1;
                for (int a = 0; a < NUM_AXES; a++)
                {
                    if (!level_set[a]) { ok = 0; break; }
                    d[a] = level_pos[a] - axes[a].pos;
                }
                if (!ok) { uart_print("Mark all three legs first (n).\r\n"); break; }
                apply_deltas(d);
                level_status(sel);
                break;
            }

            case 'p': case 'P':  level_results();   break;
            case 'v': case 'V':  print_switches();  break;

            case 'h': case 'H':
                level_rehome();
                level_status(sel);
                break;

            default: break;
        }
    }
}

/* Home all three legs, then jog them to find where the platform is level.
 * Never returns. */
static void run_level_finder(void)
{
    char buf[200];

    uart_print("\r\n=== Level finder ===\r\n"
               "Homes X, Y, Z, goes to the old minimums, then lets you jog each leg\r\n"
               "until the platform is level.\r\n");
    print_switches();

    for (int i = 0; i < NUM_AXES; i++)
    {
        if (!limit_pressed(&axes[i])) continue;
        snprintf(buf, sizeof(buf),
                 "\r\nWARNING: the %s switch reads PRESSED before homing.\r\n"
                 "If its crank is NOT sitting on the switch, check the wiring (NC to the\r\n"
                 "right pin). Press y to continue anyway, or reset the board to stop.\r\n",
                 axes[i].name);
        uart_print(buf);
        uart_flush();
        while (1)
        {
            uint8_t k = uart_getc();
            if (k == 'y' || k == 'Y') break;
        }
    }

    uart_print("\r\nClear the area and keep a hand near the driver power switches.\r\n"
               "Press any key to start homing...\r\n");
    uart_flush();
    (void)uart_getc();

    level_rehome();
    BSP_LED_On(LED_GREEN);
    level_ui();                              // never returns
    while (1) { }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  DWT_Init();
  BSP_LED_Init(LED_GREEN);

  /* Serial port over the ST-Link USB cable (115200, 8N1) */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Let the drivers finish powering up / closed-loop encoder init */
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  {
      int mode = 0;   /* 0 = heave cycle with tilt circles (default), 1 = limit finder, 2 = auto test, 3 = heave finder, 4 = cycle test, 5 = pitch/roll finder, 6 = level finder */
      char buf[400];

      uart_print("\r\nPlatform axis test.\r\n");
      uart_flush();
      for (int sec = 5; sec > 0 && !mode; sec--)
      {
          uint8_t c;
          snprintf(buf, sizeof(buf),
                   "Slow heave cycle with small tilt circles starts in %d s.\r\n"
                   "  c = plain heave cycle (no tilt)           h = heave finder (jog all 3)\r\n"
                   "  e = level finder (jog each leg)           p = pitch / roll finder\r\n"
                   "  l = per-axis limit finder                 t = automatic test (home, go to minimums)\r\n"
                   "  reset button = abort\r\n", sec);
          uart_print(buf);

          if (HAL_UART_Receive(&hcom_uart[COM1], &c, 1, 1000) == HAL_OK)
          {
              if (c == 'l' || c == 'L') mode = 1;
              else if (c == 't' || c == 'T') mode = 2;
              else if (c == 'h' || c == 'H') mode = 3;
              else if (c == 'c' || c == 'C') mode = 4;
              else if (c == 'p' || c == 'P') mode = 5;
              else if (c == 'e' || c == 'E') mode = 6;
          }
          else
          {
              __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
          }
      }

      if (mode == 1)      run_limit_finder();  // never returns
      else if (mode == 2) run_min_test();      // never returns
      else if (mode == 3) run_heave_finder();  // never returns
      else if (mode == 4) { cyc_wobble = 0; run_cycle_test(); }   // never returns
      else if (mode == 5) run_tilt_finder();   // never returns
      else if (mode == 6) run_level_finder();  // never returns
      else                { cyc_wobble = 1; run_cycle_test(); }   // never returns
  }

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (170 MHz from HSI + PLL)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Idle all signal outputs in the "OFF" state before switching to output mode */
  HAL_GPIO_WritePin(GPIOA, X_STEP_Pin|X_DIR_Pin|Z_STEP_Pin|Z_DIR_Pin, SIG_OFF_LEVEL);
  HAL_GPIO_WritePin(GPIOB, Y_STEP_Pin|Y_DIR_Pin, SIG_OFF_LEVEL);
  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, SIG_OFF_LEVEL);

  /* STEP/DIR outputs, GPIOA: X_STEP, X_DIR, Z_STEP, Z_DIR */
  GPIO_InitStruct.Pin   = X_STEP_Pin|X_DIR_Pin|Z_STEP_Pin|Z_DIR_Pin;
  GPIO_InitStruct.Mode  = SIG_GPIO_MODE;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* STEP/DIR outputs, GPIOB: Y_STEP, Y_DIR */
  GPIO_InitStruct.Pin = Y_STEP_Pin|Y_DIR_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Enable line (unused: left in the OFF state, ENA wires not connected) */
  GPIO_InitStruct.Pin = Enable_Pin;
  HAL_GPIO_Init(Enable_GPIO_Port, &GPIO_InitStruct);

  /* Limit switches: pulled-up inputs (NC switch to GND) */
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;

  GPIO_InitStruct.Pin = X_LIMIT_Pin;
  HAL_GPIO_Init(X_LIMIT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Y_LIMIT_Pin;
  HAL_GPIO_Init(Y_LIMIT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Z_LIMIT_Pin;
  HAL_GPIO_Init(Z_LIMIT_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
