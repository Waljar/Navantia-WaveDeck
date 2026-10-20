#ifndef STUB_NUCLEO_H
#define STUB_NUCLEO_H
#include "stm32g4xx_hal.h"
typedef struct { uint32_t BaudRate, WordLength, StopBits, Parity, HwFlowCtl; } COM_InitTypeDef;
#define COM_WORDLENGTH_8B 0u
#define COM_STOPBITS_1 0u
#define COM_PARITY_NONE 0u
#define COM_HWCONTROL_NONE 0u
#define COM1 0
#define LED_GREEN 0
#define BSP_ERROR_NONE 0
extern UART_HandleTypeDef hcom_uart[1];
int32_t BSP_COM_Init(int com, COM_InitTypeDef *i);
int32_t BSP_LED_Init(int led);
#endif
