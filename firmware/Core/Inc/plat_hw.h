/**
 * plat_hw.h - the only interface between the platform code and the hardware.
 * Implemented by plat_hw_stm32.c on the board and by sim/hw_host.c on the PC,
 * which is how the motion code is tested without a motor.
 */
#ifndef PLAT_HW_H
#define PLAT_HW_H

#include <stdint.h>

/* ---- Time ------------------------------------------------------------------- */
void     hw_init(void);                    /* enable the cycle counter etc.           */
uint32_t hw_cycles(void);                  /* free-running CPU cycle counter          */
uint32_t hw_cycles_per_us(void);
uint32_t hw_ms(void);                      /* milliseconds since start                */
void     hw_delay_us(uint32_t us);         /* blocking                                */
void     hw_delay_ms(uint32_t ms);         /* blocking                                */

/* ---- Leg pins (leg 0 = X, 1 = Y, 2 = Z) --------------------------------------- */
void hw_step_pin(int leg, int on);         /* STEP pulse level: 1 = pulse high        */
void hw_dir_pin(int leg, int level);       /* DIR pin level: 1 = high                 */
int  hw_limit_raw(int leg);                /* 1 = switch reads pressed (no debounce)  */
int  hw_limit_pressed(int leg);            /* debounced: pressed twice, 0.5 ms apart  */

/* ---- Console (serial over the ST-Link USB cable, 115200 8N1) ------------------ */
void con_init(void);
void con_print(const char *s);             /* blocking; use when the motors are idle  */
void con_printf(const char *fmt, ...);     /* blocking                                */
void con_queue(const char *s);             /* NON-blocking: safe while motors move    */
void con_queuef(const char *fmt, ...);
void con_pump(void);                       /* call often: moves queued bytes out      */
int  con_poll(void);                       /* next key, or -1 if none waiting         */
int  con_getc(void);                       /* blocking: waits for a key               */
void con_flush_rx(void);                   /* throw away waiting keystrokes           */
int  con_rx_lost(void);                    /* 1 (and cleared) if characters were lost since the last call:
                                              a half-typed number must then be discarded */

/* ---- Flash storage for calibration -------------------------------------------- */
int  hw_flash_read(void *buf, uint32_t len);          /* 0 ok                         */
int  hw_flash_write(const void *buf, uint32_t len);   /* erase + program, 0 ok        */

#endif /* PLAT_HW_H */
