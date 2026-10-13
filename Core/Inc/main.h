/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
#define SYS_WKUP2_Pin GPIO_PIN_13
#define SYS_WKUP2_GPIO_Port GPIOC
#define SYS_WKUP2_EXTI_IRQn EXTI15_10_IRQn
#define ADC_MREF_Pin GPIO_PIN_0
#define ADC_MREF_GPIO_Port GPIOC
#define ADC_MIC_Pin GPIO_PIN_1
#define ADC_MIC_GPIO_Port GPIOC
#define LED_BATT_Pin GPIO_PIN_4
#define LED_BATT_GPIO_Port GPIOA
#define LED_MEAS_Pin GPIO_PIN_8
#define LED_MEAS_GPIO_Port GPIOA
#define SENSOR_PWR_EN_Pin GPIO_PIN_9
#define SENSOR_PWR_EN_GPIO_Port GPIOA
#define INT_M_Pin GPIO_PIN_4
#define INT_M_GPIO_Port GPIOC
#define INT_M_EXTI_IRQn EXTI4_IRQn
#define I2C3_SCL_Pin GPIO_PIN_10
#define I2C3_SCL_GPIO_Port GPIOB
#define I2C3_SDA_Pin GPIO_PIN_11
#define I2C3_SDA_GPIO_Port GPIOB
#define LED_NET_BLUE_Pin GPIO_PIN_0
#define LED_NET_BLUE_GPIO_Port GPIOB
#define LED_NET_RED_Pin GPIO_PIN_1
#define LED_NET_RED_GPIO_Port GPIOB
#define SW4_UNUSED_Pin GPIO_PIN_15
#define SW4_UNUSED_GPIO_Port GPIOA
/* Compatibility aliases for the existing BSP; PA15 is not configured as EXTI. */
#define KEY_MEAS_Pin SW4_UNUSED_Pin
#define KEY_MEAS_GPIO_Port SW4_UNUSED_GPIO_Port
#define KEY_MEAS_EXTI_IRQn EXTI15_10_IRQn
#define STTS_EVENT_Pin GPIO_PIN_10
#define STTS_EVENT_GPIO_Port GPIOC
#define STTS_EVENT_EXTI_IRQn EXTI15_10_IRQn
#define STTS_THERM_Pin GPIO_PIN_11
#define STTS_THERM_GPIO_Port GPIOC
#define STTS_THERM_EXTI_IRQn EXTI15_10_IRQn
#define INT1_ADWB_Pin GPIO_PIN_12
#define INT1_ADWB_GPIO_Port GPIOC
#define INT1_ADWB_EXTI_IRQn EXTI15_10_IRQn
#define CS_ADWB_Pin GPIO_PIN_0
#define CS_ADWB_GPIO_Port GPIOD
#define INT2_ADWB_Pin GPIO_PIN_1
#define INT2_ADWB_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
