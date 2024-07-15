/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BUTTON_20_Pin GPIO_PIN_14
#define BUTTON_20_GPIO_Port GPIOC
#define BUTTON_21_Pin GPIO_PIN_15
#define BUTTON_21_GPIO_Port GPIOC
#define MOUSE_X_Pin GPIO_PIN_0
#define MOUSE_X_GPIO_Port GPIOA
#define MOUSE_Y_Pin GPIO_PIN_1
#define MOUSE_Y_GPIO_Port GPIOA
#define MOUSE_CLICK_Pin GPIO_PIN_2
#define MOUSE_CLICK_GPIO_Port GPIOA
#define BUTTON_22_Pin GPIO_PIN_3
#define BUTTON_22_GPIO_Port GPIOA
#define BUTTON_23_Pin GPIO_PIN_4
#define BUTTON_23_GPIO_Port GPIOA
#define BUTTON_24_Pin GPIO_PIN_5
#define BUTTON_24_GPIO_Port GPIOA
#define BUTTON_25_Pin GPIO_PIN_6
#define BUTTON_25_GPIO_Port GPIOA
#define BUTTON_26_Pin GPIO_PIN_7
#define BUTTON_26_GPIO_Port GPIOA
#define BUTTON_1_Pin GPIO_PIN_0
#define BUTTON_1_GPIO_Port GPIOB
#define BUTTON_2_Pin GPIO_PIN_1
#define BUTTON_2_GPIO_Port GPIOB
#define BUTTON_3_Pin GPIO_PIN_2
#define BUTTON_3_GPIO_Port GPIOB
#define BUTTON_11_Pin GPIO_PIN_10
#define BUTTON_11_GPIO_Port GPIOB
#define BUTTON_12_Pin GPIO_PIN_12
#define BUTTON_12_GPIO_Port GPIOB
#define BUTTON_13_Pin GPIO_PIN_13
#define BUTTON_13_GPIO_Port GPIOB
#define BUTTON_14_Pin GPIO_PIN_14
#define BUTTON_14_GPIO_Port GPIOB
#define BUTTON_15_Pin GPIO_PIN_15
#define BUTTON_15_GPIO_Port GPIOB
#define BUTTON_16_Pin GPIO_PIN_8
#define BUTTON_16_GPIO_Port GPIOA
#define BUTTON_17_Pin GPIO_PIN_9
#define BUTTON_17_GPIO_Port GPIOA
#define BUTTON_18_Pin GPIO_PIN_10
#define BUTTON_18_GPIO_Port GPIOA
#define DM_Pin GPIO_PIN_11
#define DM_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_12
#define DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define BUTTON_19_Pin GPIO_PIN_15
#define BUTTON_19_GPIO_Port GPIOA
#define BUTTON_4_Pin GPIO_PIN_3
#define BUTTON_4_GPIO_Port GPIOB
#define BUTTON_5_Pin GPIO_PIN_4
#define BUTTON_5_GPIO_Port GPIOB
#define BUTTON_6_Pin GPIO_PIN_5
#define BUTTON_6_GPIO_Port GPIOB
#define BUTTON_7_Pin GPIO_PIN_6
#define BUTTON_7_GPIO_Port GPIOB
#define BUTTON_8_Pin GPIO_PIN_7
#define BUTTON_8_GPIO_Port GPIOB
#define BUTTON_9_Pin GPIO_PIN_8
#define BUTTON_9_GPIO_Port GPIOB
#define BUTTON_10_Pin GPIO_PIN_9
#define BUTTON_10_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define MOUSE_X_CENTER_LOW  123
#define MOUSE_X_CENTER_HIGH 129
#define MOUSE_Y_CENTER_LOW  121
#define MOUSE_Y_CENTER_HIGH 127

#define JOYSTICK_NUMBER_OF_BUTTONS 26

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
