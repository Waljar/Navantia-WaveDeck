/* Minimal stand-in for the ST HAL, only so that the target-only files can be
 * syntax-checked on a PC. NOT used for building the firmware. */
#ifndef STUB_HAL_H
#define STUB_HAL_H
#include <stdint.h>
typedef struct { volatile uint32_t IDR, ODR, BSRR; } GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef *)0x48000000UL)
#define GPIOB ((GPIO_TypeDef *)0x48000400UL)
#define GPIOC ((GPIO_TypeDef *)0x48000800UL)
#define GPIO_PIN_0 1u
#define GPIO_PIN_1 2u
#define GPIO_PIN_4 16u
#define GPIO_PIN_5 32u
#define GPIO_PIN_8 256u
#define GPIO_PIN_9 512u
#define GPIO_PIN_12 4096u
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_INPUT 0u
#define GPIO_NOPULL 0u
#define GPIO_PULLUP 1u
#define GPIO_SPEED_FREQ_LOW 0u
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i);
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s);
typedef enum { HAL_OK = 0, HAL_ERROR } HAL_StatusTypeDef;
HAL_StatusTypeDef HAL_Init(void);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
extern uint32_t SystemCoreClock;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
typedef struct { volatile uint32_t CYCCNT, CTRL; } DWT_Type;
extern CoreDebug_Type *CoreDebug; extern DWT_Type *DWT;
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
typedef struct { volatile uint32_t TDR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
#define UART_FLAG_TXE 1u
#define UART_FLAG_RXNE 2u
#define UART_FLAG_ORE 4u
#define UART_FLAG_FE 16u
#define UART_RXDATA_FLUSH_REQUEST 8u
#define UART_RXFIFO_THRESHOLD_1_8 0u
void __HAL_UART_CLEAR_FEFLAG(UART_HandleTypeDef *h);
int __HAL_UART_GET_FLAG(UART_HandleTypeDef *h, uint32_t f);
void __HAL_UART_CLEAR_OREFLAG(UART_HandleTypeDef *h);
void __HAL_UART_SEND_REQ(UART_HandleTypeDef *h, uint32_t r);
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *h, uint32_t t);
HAL_StatusTypeDef HAL_UARTEx_EnableFifoMode(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *d, uint16_t n, uint32_t t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *h, uint8_t *d, uint16_t n, uint32_t t);
typedef struct { uint32_t TypeErase, Banks, Page, NbPages; } FLASH_EraseInitTypeDef;
typedef struct { volatile uint32_t OPTR; } FLASH_TypeDef;
extern FLASH_TypeDef *FLASH;
#define FLASH_TYPEERASE_PAGES 0u
#define FLASH_BANK_1 1u
#define FLASH_BANK_2 2u
#define FLASH_OPTR_DBANK 0x400000u
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0u
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *err);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint64_t data);
typedef struct { uint32_t OscillatorType, HSEState, LSEState, HSIState, HSICalibrationValue, HSI48State;
                 struct { uint32_t PLLState, PLLSource, PLLM, PLLN, PLLP, PLLQ, PLLR; } PLL; } RCC_OscInitTypeDef;
typedef struct { uint32_t ClockType, SYSCLKSource, AHBCLKDivider, APB1CLKDivider, APB2CLKDivider; } RCC_ClkInitTypeDef;
#define RCC_OSCILLATORTYPE_HSI 1u
#define RCC_HSI_ON 1u
#define RCC_HSICALIBRATION_DEFAULT 64u
#define RCC_PLL_ON 2u
#define RCC_PLLSOURCE_HSI 2u
#define RCC_PLLM_DIV4 3u
#define RCC_PLLP_DIV2 2u
#define RCC_PLLQ_DIV2 0u
#define RCC_PLLR_DIV2 0u
#define RCC_CLOCKTYPE_HCLK 1u
#define RCC_CLOCKTYPE_SYSCLK 2u
#define RCC_CLOCKTYPE_PCLK1 4u
#define RCC_CLOCKTYPE_PCLK2 8u
#define RCC_SYSCLKSOURCE_PLLCLK 3u
#define RCC_SYSCLK_DIV1 0u
#define RCC_HCLK_DIV1 0u
#define PWR_REGULATOR_VOLTAGE_SCALE1_BOOST 1u
#define FLASH_LATENCY_4 4u
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *o);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *c, uint32_t lat);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t s);
void __HAL_RCC_GPIOA_CLK_ENABLE(void); void __HAL_RCC_GPIOB_CLK_ENABLE(void);
void __HAL_RCC_GPIOC_CLK_ENABLE(void); void __HAL_RCC_GPIOF_CLK_ENABLE(void);
typedef struct { int dummy; } TIM_HandleTypeDef;
void __disable_irq(void);
#endif
