/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   Pin definitions for the 3-DOF stepper platform.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_nucleo.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* X axis */
#define X_STEP_Pin          GPIO_PIN_0
#define X_STEP_GPIO_Port    GPIOA
#define X_DIR_Pin           GPIO_PIN_1
#define X_DIR_GPIO_Port     GPIOA
#define X_LIMIT_Pin         GPIO_PIN_9
#define X_LIMIT_GPIO_Port   GPIOB

/* Y axis */
#define Y_STEP_Pin          GPIO_PIN_4
#define Y_STEP_GPIO_Port    GPIOB
#define Y_DIR_Pin           GPIO_PIN_5
#define Y_DIR_GPIO_Port     GPIOB
#define Y_LIMIT_Pin         GPIO_PIN_0
#define Y_LIMIT_GPIO_Port   GPIOB     /* A3 on the Arduino header */

/* Z axis */
#define Z_STEP_Pin          GPIO_PIN_8
#define Z_STEP_GPIO_Port    GPIOA
#define Z_DIR_Pin           GPIO_PIN_9
#define Z_DIR_GPIO_Port     GPIOA
#define Z_LIMIT_Pin         GPIO_PIN_4
#define Z_LIMIT_GPIO_Port   GPIOA     /* A2 on the Arduino header */

/* Shared driver enable */
#define Enable_Pin          GPIO_PIN_12
#define Enable_GPIO_Port    GPIOC

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
