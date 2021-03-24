/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY4_Pin GPIO_PIN_13
#define KEY4_GPIO_Port GPIOC
#define KEY4_EXTI_IRQn EXTI15_10_IRQn
#define DAC10KHZ_Pin GPIO_PIN_4
#define DAC10KHZ_GPIO_Port GPIOA
#define DAC3X_Pin GPIO_PIN_5
#define DAC3X_GPIO_Port GPIOA
#define DAC1KHZ_Pin GPIO_PIN_6
#define DAC1KHZ_GPIO_Port GPIOA
#define MIXSW1_Pin GPIO_PIN_7
#define MIXSW1_GPIO_Port GPIOA
#define MIXSW2_Pin GPIO_PIN_0
#define MIXSW2_GPIO_Port GPIOB
#define MIXSW3_Pin GPIO_PIN_1
#define MIXSW3_GPIO_Port GPIOB
#define SPKAMP_Pin GPIO_PIN_2
#define SPKAMP_GPIO_Port GPIOB
#define DIP1_Pin GPIO_PIN_12
#define DIP1_GPIO_Port GPIOB
#define DIP2_Pin GPIO_PIN_13
#define DIP2_GPIO_Port GPIOB
#define DIP3_Pin GPIO_PIN_14
#define DIP3_GPIO_Port GPIOB
#define TEMPOLED3_Pin GPIO_PIN_15
#define TEMPOLED3_GPIO_Port GPIOB
#define TEMPOLED1_Pin GPIO_PIN_8
#define TEMPOLED1_GPIO_Port GPIOA
#define MIXMIC_Pin GPIO_PIN_11
#define MIXMIC_GPIO_Port GPIOA
#define TEMPOLED2_Pin GPIO_PIN_12
#define TEMPOLED2_GPIO_Port GPIOA
#define LED1K_Pin GPIO_PIN_15
#define LED1K_GPIO_Port GPIOA
#define LED10K_Pin GPIO_PIN_3
#define LED10K_GPIO_Port GPIOB
#define LED3X_Pin GPIO_PIN_4
#define LED3X_GPIO_Port GPIOB
#define LEDMIC_Pin GPIO_PIN_5
#define LEDMIC_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_7
#define KEY1_GPIO_Port GPIOB
#define KEY1_EXTI_IRQn EXTI9_5_IRQn
#define KEY2_Pin GPIO_PIN_8
#define KEY2_GPIO_Port GPIOB
#define KEY2_EXTI_IRQn EXTI9_5_IRQn
#define KEY3_Pin GPIO_PIN_9
#define KEY3_GPIO_Port GPIOB
#define KEY3_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
