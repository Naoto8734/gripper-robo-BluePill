/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BUILTIN_Pin GPIO_PIN_13
#define LED_BUILTIN_GPIO_Port GPIOC
#define RDC80_Z_A_Pin GPIO_PIN_0
#define RDC80_Z_A_GPIO_Port GPIOA
#define RDC80_Z_B_Pin GPIO_PIN_1
#define RDC80_Z_B_GPIO_Port GPIOA
#define RDC80_G_A_Pin GPIO_PIN_2
#define RDC80_G_A_GPIO_Port GPIOA
#define RDC80_G_B_Pin GPIO_PIN_3
#define RDC80_G_B_GPIO_Port GPIOA
#define A1324_A_Pin GPIO_PIN_4
#define A1324_A_GPIO_Port GPIOA
#define A1324_B_Pin GPIO_PIN_5
#define A1324_B_GPIO_Port GPIOA
#define A1324_C_Pin GPIO_PIN_6
#define A1324_C_GPIO_Port GPIOA
#define A4988_STEP_Z_Pin GPIO_PIN_0
#define A4988_STEP_Z_GPIO_Port GPIOB
#define A4988_STEP_G_Pin GPIO_PIN_1
#define A4988_STEP_G_GPIO_Port GPIOB
#define A4988_DIR_Z_Pin GPIO_PIN_10
#define A4988_DIR_Z_GPIO_Port GPIOB
#define A4988_DIR_G_Pin GPIO_PIN_11
#define A4988_DIR_G_GPIO_Port GPIOB
#define LIMIT_SW_Z_Pin GPIO_PIN_8
#define LIMIT_SW_Z_GPIO_Port GPIOB
#define LIMIT_SW_G_Pin GPIO_PIN_9
#define LIMIT_SW_G_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
