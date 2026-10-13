/**
  ******************************************************************************
  * @file    stm32wb55rgv6_bsp.h
  * @brief   Board support package for the custom STM32WB55RGV6 board.
  ******************************************************************************
  */

#ifndef STM32WB55RGV6_BSP_H
#define STM32WB55RGV6_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  LED1 = 0,      /* Network blue LED, PB1, active low */
  LED2,          /* Network red LED, PB0, active low */
  LED3,          /* Battery LED, PA4, active low */
  LED4,          /* Measurement LED, PA8, active low */
  LED_BLUE = LED1,
  LED_RED = LED2,
  LED_GREEN = LED_RED, /* Compatibility alias: this board has red, not green. */
  LED_BATT = LED3,
  LED_MEAS = LED4
} Led_TypeDef;

#define LEDn 4U

typedef enum
{
  BUTTON1 = 0,   /* SYS_WKUP2, PC13, active low */
  BUTTON2,       /* KEY_MEAS, PA15, active low */
  BUTTON_SYS_WKUP = BUTTON1,
  BUTTON_MEAS = BUTTON2
} Button_TypeDef;

#define BUTTONn 2U

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

int32_t BSP_LED_Init(Led_TypeDef Led);
int32_t BSP_LED_DeInit(Led_TypeDef Led);
int32_t BSP_LED_On(Led_TypeDef Led);
int32_t BSP_LED_Off(Led_TypeDef Led);
int32_t BSP_LED_Toggle(Led_TypeDef Led);
int32_t BSP_LED_GetState(Led_TypeDef Led);

int32_t BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode);
int32_t BSP_PB_DeInit(Button_TypeDef Button);
int32_t BSP_PB_GetState(Button_TypeDef Button);
void BSP_PB_IRQHandler(Button_TypeDef Button);
void BSP_PB_Callback(Button_TypeDef Button);

int32_t BSP_SENSOR_PWR_Init(void);
void BSP_SENSOR_PWR_On(void);
void BSP_SENSOR_PWR_Off(void);
int32_t BSP_SENSOR_PWR_GetState(void);

int32_t BSP_UART1_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif /* STM32WB55RGV6_BSP_H */
