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
#include "stm32h7xx_hal.h"

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
#define LC2_DIN_Pin GPIO_PIN_7
#define LC2_DIN_GPIO_Port GPIOF
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LC4_DIN_Pin GPIO_PIN_11
#define LC4_DIN_GPIO_Port GPIOB
#define LC4_CK_Pin GPIO_PIN_12
#define LC4_CK_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define LC3_CK_Pin GPIO_PIN_15
#define LC3_CK_GPIO_Port GPIOA
#define LC1_CK_Pin GPIO_PIN_10
#define LC1_CK_GPIO_Port GPIOC
#define LC1_DIN_Pin GPIO_PIN_11
#define LC1_DIN_GPIO_Port GPIOC
#define IGN_OUT_Pin GPIO_PIN_0
#define IGN_OUT_GPIO_Port GPIOD
#define LOW_SIDE_SW_3_Pin GPIO_PIN_1
#define LOW_SIDE_SW_3_GPIO_Port GPIOD
#define LC2_CK_Pin GPIO_PIN_2
#define LC2_CK_GPIO_Port GPIOD
#define LC3_DIN_Pin GPIO_PIN_8
#define LC3_DIN_GPIO_Port GPIOB
void   MX_USART3_UART_Init(void);
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
