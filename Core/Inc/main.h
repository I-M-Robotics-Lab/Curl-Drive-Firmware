/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define NRST_Pin GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
#define INHC_Pin GPIO_PIN_0
#define INHC_GPIO_Port GPIOC
#define INHB_Pin GPIO_PIN_1
#define INHB_GPIO_Port GPIOC
#define INHA_Pin GPIO_PIN_2
#define INHA_GPIO_Port GPIOC
#define CAL_Pin GPIO_PIN_0
#define CAL_GPIO_Port GPIOA
#define DR_EN_Pin GPIO_PIN_1
#define DR_EN_GPIO_Port GPIOA
#define DR_CS_Pin GPIO_PIN_2
#define DR_CS_GPIO_Port GPIOA
#define nFAULT_Pin GPIO_PIN_3
#define nFAULT_GPIO_Port GPIOA
#define nFAULT_EXTI_IRQn EXTI3_IRQn
#define SOA_Pin GPIO_PIN_4
#define SOA_GPIO_Port GPIOA
#define SOB_Pin GPIO_PIN_5
#define SOB_GPIO_Port GPIOA
#define SOC_Pin GPIO_PIN_6
#define SOC_GPIO_Port GPIOA
#define TEMP_SENSE_Pin GPIO_PIN_0
#define TEMP_SENSE_GPIO_Port GPIOB
#define VBAT_SENSE_Pin GPIO_PIN_1
#define VBAT_SENSE_GPIO_Port GPIOB
#define AUX_CS_SPI2_Pin GPIO_PIN_11
#define AUX_CS_SPI2_GPIO_Port GPIOB
#define LED_IND_Pin GPIO_PIN_6
#define LED_IND_GPIO_Port GPIOC
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
