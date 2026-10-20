/**
 * plat_hw_stm32.c - the STM32G474 / Nucleo implementation of plat_hw.h.
 * Pins come from main.h (the CubeMX pin names).
 */
#include "main.h"
#include "plat_hw.h"
#include "plat_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static GPIO_TypeDef *const STEP_PORT[NUM_LEGS]  = { X_STEP_GPIO_Port,  Y_STEP_GPIO_Port,  Z_STEP_GPIO_Port  };
static const uint16_t      STEP_PIN[NUM_LEGS]   = { X_STEP_Pin,        Y_STEP_Pin,        Z_STEP_Pin        };
static GPIO_TypeDef *const DIR_PORT[NUM_LEGS]   = { X_DIR_GPIO_Port,   Y_DIR_GPIO_Port,   Z_DIR_GPIO_Port   };
static const uint16_t      DIR_PIN[NUM_LEGS]    = { X_DIR_Pin,         Y_DIR_Pin,         Z_DIR_Pin         };
static GPIO_TypeDef *const LIM_PORT[NUM_LEGS]   = { X_LIMIT_GPIO_Port, Y_LIMIT_GPIO_Port, Z_LIMIT_GPIO_Port };
static const uint16_t      LIM_PIN[NUM_LEGS]    = { X_LIMIT_Pin,       Y_LIMIT_Pin,       Z_LIMIT_Pin       };

/* ---- time ---------------------------------------------------------------------- */
void hw_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    BSP_LED_Init(LED_GREEN);
}

uint32_t hw_cycles(void)        { return DWT->CYCCNT; }
uint32_t hw_cycles_per_us(void) { return SystemCoreClock / 1000000u; }
uint32_t hw_ms(void)            { return HAL_GetTick(); }
void     hw_delay_ms(uint32_t ms) { HAL_Delay(ms); }

void hw_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) { }
}

/* ---- pins ------------------------------------------------------------------------ */
void hw_step_pin(int leg, int on)
{
    STEP_PORT[leg]->BSRR = on ? (uint32_t)STEP_PIN[leg] : ((uint32_t)STEP_PIN[leg] << 16);
}

void hw_dir_pin(int leg, int level)
{
    DIR_PORT[leg]->BSRR = level ? (uint32_t)DIR_PIN[leg] : ((uint32_t)DIR_PIN[leg] << 16);
}

int hw_limit_raw(int leg)
{
    return ((LIM_PORT[leg]->IDR & LIM_PIN[leg]) != 0u) == (LIMIT_PRESSED_LEVEL != 0);
}

int hw_limit_pressed(int leg)
{
    if (!hw_limit_raw(leg)) return 0;
    hw_delay_us(500);
    return hw_limit_raw(leg);
}

/* ---- console ------------------------------------------------------------------------ */
#define TXQ_SIZE 4096u
static volatile uint16_t txq_head, txq_tail;
static uint8_t txq[TXQ_SIZE];
static COM_InitTypeDef com_init;

void con_init(void)
{
    com_init.BaudRate   = 115200;
    com_init.WordLength = COM_WORDLENGTH_8B;
    com_init.StopBits   = COM_STOPBITS_1;
    com_init.Parity     = COM_PARITY_NONE;
    com_init.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &com_init) != BSP_ERROR_NONE) Error_Handler();

    /* 8-byte receive FIFO: without it the port holds ONE byte, and a burst such as "h12.5<Enter>"
     * from the control panel loses characters whenever the main loop is busy for more than 87 us. */
    (void)HAL_UARTEx_SetRxFifoThreshold(&hcom_uart[COM1], UART_RXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_EnableFifoMode(&hcom_uart[COM1]);
}

static volatile int rx_lost = 0;

int con_rx_lost(void)
{
    int v = rx_lost;
    rx_lost = 0;
    return v;
}

void con_pump(void)
{
    if (txq_tail == txq_head) return;
    if (__HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_TXE))
    {
        hcom_uart[COM1].Instance->TDR = txq[txq_tail];
        txq_tail = (uint16_t)((txq_tail + 1u) % TXQ_SIZE);
    }
}

void con_queue(const char *s)
{
    while (*s)
    {
        uint16_t next = (uint16_t)((txq_head + 1u) % TXQ_SIZE);
        if (next == txq_tail) return;                       /* full: drop the rest */
        txq[txq_head] = (uint8_t)*s++;
        txq_head = next;
    }
}

void con_queuef(const char *fmt, ...)
{
    char b[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    con_queue(b);
}

void con_print(const char *s)
{
    while (txq_tail != txq_head) con_pump();                /* finish anything queued first */
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)s, (uint16_t)strlen(s), 500);
}

void con_printf(const char *fmt, ...)
{
    char b[260];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    con_print(b);
}

int con_poll(void)
{
    uint8_t c;
    if (__HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
        rx_lost = 1;
    }
    if (__HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FEFLAG(&hcom_uart[COM1]);
        rx_lost = 1;
    }
    if (__HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_RXNE))
    {
        if (HAL_UART_Receive(&hcom_uart[COM1], &c, 1, 5) == HAL_OK) return c;
    }
    return -1;
}

int con_getc(void)
{
    int c;
    while ((c = con_poll()) < 0) con_pump();
    return c;
}

void con_flush_rx(void)
{
    __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
    __HAL_UART_SEND_REQ(&hcom_uart[COM1], UART_RXDATA_FLUSH_REQUEST);
    rx_lost = 0;
}

/* ---- flash: the last page of bank 2 (0x0807F800), 2 KB ---------------------------------- */
#define CAL_FLASH_ADDR  0x0807F800UL

int hw_flash_read(void *buf, uint32_t len)
{
    memcpy(buf, (const void *)CAL_FLASH_ADDR, len);
    return 0;
}

int hw_flash_write(const void *buf, uint32_t len)
{
    FLASH_EraseInitTypeDef e;
    uint32_t page_error = 0;
    int rc = 0;

    HAL_FLASH_Unlock();
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.Banks     = (FLASH->OPTR & FLASH_OPTR_DBANK) ? FLASH_BANK_2 : FLASH_BANK_1;
    e.Page      = 127;                                      /* last page in either bank mode */
    e.NbPages   = 1;
    if (HAL_FLASHEx_Erase(&e, &page_error) != HAL_OK) rc = -1;

    for (uint32_t off = 0; rc == 0 && off < len; off += 8u)
    {
        uint64_t d = 0xFFFFFFFFFFFFFFFFull;
        uint32_t n = (len - off) < 8u ? (len - off) : 8u;
        memcpy(&d, (const uint8_t *)buf + off, n);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + off, d) != HAL_OK) rc = -3;
    }
    HAL_FLASH_Lock();
    return rc;
}
