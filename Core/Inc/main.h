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
#include "stm32l4xx_hal.h"

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
#define LED_3v3_Pin GPIO_PIN_14
#define LED_3v3_GPIO_Port GPIOC
#define LED_12v_Pin GPIO_PIN_15
#define LED_12v_GPIO_Port GPIOC
#define DRV2_Pin GPIO_PIN_0
#define DRV2_GPIO_Port GPIOA
#define LED_5v_Pin GPIO_PIN_1
#define LED_5v_GPIO_Port GPIOA
#define LED_24v_Pin GPIO_PIN_2
#define LED_24v_GPIO_Port GPIOA
#define I2_FB_Pin GPIO_PIN_3
#define I2_FB_GPIO_Port GPIOA
#define V2_FB_Pin GPIO_PIN_4
#define V2_FB_GPIO_Port GPIOA
#define V1_FB_Pin GPIO_PIN_5
#define V1_FB_GPIO_Port GPIOA
#define I1_FB_Pin GPIO_PIN_6
#define I1_FB_GPIO_Port GPIOA
#define Vout1_sel_Pin GPIO_PIN_8
#define Vout1_sel_GPIO_Port GPIOA
#define DRV1_Pin GPIO_PIN_11
#define DRV1_GPIO_Port GPIOA
#define Vout2_sel_Pin GPIO_PIN_12
#define Vout2_sel_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Button0_Pin GPIO_PIN_15
#define Button0_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define USB_INT_Pin GPIO_PIN_5
#define USB_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
