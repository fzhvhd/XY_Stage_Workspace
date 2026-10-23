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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Linear_Right_Pin GPIO_PIN_0
#define Linear_Right_GPIO_Port GPIOC
#define Linear_Left_Pin GPIO_PIN_1
#define Linear_Left_GPIO_Port GPIOC
#define Rotary_Right_Pin GPIO_PIN_2
#define Rotary_Right_GPIO_Port GPIOC
#define Rotary_Left_Pin GPIO_PIN_3
#define Rotary_Left_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED_PIN_Pin GPIO_PIN_5
#define LED_PIN_GPIO_Port GPIOA
#define LEM_Linear_Pin GPIO_PIN_7
#define LEM_Linear_GPIO_Port GPIOA
#define LEM_Rotary_Pin GPIO_PIN_0
#define LEM_Rotary_GPIO_Port GPIOB
#define Linear_A_Pin GPIO_PIN_6
#define Linear_A_GPIO_Port GPIOC
#define Linear_B_Pin GPIO_PIN_7
#define Linear_B_GPIO_Port GPIOC
#define Rotary_A_Pin GPIO_PIN_11
#define Rotary_A_GPIO_Port GPIOA
#define Rotary_B_Pin GPIO_PIN_12
#define Rotary_B_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
