/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Sea platform firmware - start-up only.
  *                   All of the behaviour lives in the plat_* and mode_* files:
  *                     app_common.c   menu, homing wrapper, telemetry
  *                     mode_setup.c   mode 1: set minimums / maximum, save
  *                     mode_heave.c   mode 2: heave cycle
  *                     mode_circles.c mode 3: pitch / roll circles
  *                     mode_waves.c   modes 4 and 5: sea states, MAX platform
  *                     mode_manual.c  mode 6: manual heave / pitch / roll
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "plat_app.h"

/* stm32g4xx_it.c refers to this handle, so it has to exist. TIM6 is not used. */
TIM_HandleTypeDef htim6;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  app_init();            /* cycle counter, serial port, step engine, saved calibration */
  HAL_Delay(1000);       /* let the drivers finish powering up / closed-loop encoder init */
  app_run();             /* menu - never returns */

  while (1)
  {
  }
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
  *        STEP / DIR: push-pull outputs, idle low.
  *        Limit switches: inputs WITH pull-up (NC switch, COM to GND).
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, X_STEP_Pin|X_DIR_Pin|Z_STEP_Pin|Z_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, Y_STEP_Pin|Y_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = X_STEP_Pin|X_DIR_Pin|Z_STEP_Pin|Z_DIR_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Y_STEP_Pin|Y_DIR_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Enable_Pin;                  /* unused, left low */
  HAL_GPIO_Init(Enable_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  GPIO_InitStruct.Pin = X_LIMIT_Pin;
  HAL_GPIO_Init(X_LIMIT_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Y_LIMIT_Pin;
  HAL_GPIO_Init(Y_LIMIT_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Z_LIMIT_Pin;
  HAL_GPIO_Init(Z_LIMIT_GPIO_Port, &GPIO_InitStruct);
}

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
