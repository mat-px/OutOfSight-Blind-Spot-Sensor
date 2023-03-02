/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
#define MISC_GPIO3_Pin GPIO_PIN_14
#define MISC_GPIO3_GPIO_Port GPIOC
#define MISC_GPIO4_Pin GPIO_PIN_15
#define MISC_GPIO4_GPIO_Port GPIOC
#define ON_OFF_MODE_Pin GPIO_PIN_1
#define ON_OFF_MODE_GPIO_Port GPIOA
#define ACCEL_SPI1_CS_Pin GPIO_PIN_3
#define ACCEL_SPI1_CS_GPIO_Port GPIOA
#define MISC_GPIO2_Pin GPIO_PIN_1
#define MISC_GPIO2_GPIO_Port GPIOB
#define ACCEL_INT2_Pin GPIO_PIN_9
#define ACCEL_INT2_GPIO_Port GPIOA
#define ACCEL_INT1_Pin GPIO_PIN_10
#define ACCEL_INT1_GPIO_Port GPIOA
#define MISC_GPIO1_Pin GPIO_PIN_13
#define MISC_GPIO1_GPIO_Port GPIOA
#define LED_OUT_Pin GPIO_PIN_14
#define LED_OUT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
