/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void lcd_send_cmd (char cmd);
// clear 0x01 cursor
void lcd_send_data (char data);

void lcd_init (void);

void lcd_send_string (char *str);

void lcd_clear();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Kout3_Pin GPIO_PIN_0
#define Kout3_GPIO_Port GPIOC
#define Kout2_Pin GPIO_PIN_1
#define Kout2_GPIO_Port GPIOC
#define Kout1_Pin GPIO_PIN_2
#define Kout1_GPIO_Port GPIOC
#define Kout0_Pin GPIO_PIN_3
#define Kout0_GPIO_Port GPIOC
#define Kin0_Pin GPIO_PIN_4
#define Kin0_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_5
#define A3_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_0
#define A2_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOB
#define A0_Pin GPIO_PIN_2
#define A0_GPIO_Port GPIOB
#define Kin1_Pin GPIO_PIN_6
#define Kin1_GPIO_Port GPIOC
#define Kin2_Pin GPIO_PIN_7
#define Kin2_GPIO_Port GPIOC
#define Kin3_Pin GPIO_PIN_8
#define Kin3_GPIO_Port GPIOC
#define E0_Pin GPIO_PIN_8
#define E0_GPIO_Port GPIOA
#define E1_Pin GPIO_PIN_9
#define E1_GPIO_Port GPIOA
#define push1_Pin GPIO_PIN_11
#define push1_GPIO_Port GPIOA
#define push1_EXTI_IRQn EXTI15_10_IRQn
#define push2_Pin GPIO_PIN_12
#define push2_GPIO_Port GPIOA
#define push2_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
