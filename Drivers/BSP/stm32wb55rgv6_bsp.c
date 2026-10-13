/**
  ******************************************************************************
  * @file    stm32wb55rgv6_bsp.c
  * @brief   Board support package for the custom STM32WB55RGV6 board.
  ******************************************************************************
  */

#include "stm32wb55rgv6_bsp.h"

extern UART_HandleTypeDef huart1;

static const uint16_t LED_PIN[LEDn] =
{
  LED_NET_BLUE_Pin,
  LED_NET_RED_Pin,
  LED_BATT_Pin,
  LED_MEAS_Pin
};

static GPIO_TypeDef *const LED_PORT[LEDn] =
{
  LED_NET_BLUE_GPIO_Port,
  LED_NET_RED_GPIO_Port,
  LED_BATT_GPIO_Port,
  LED_MEAS_GPIO_Port
};

static const uint16_t BUTTON_PIN[BUTTONn] =
{
  SYS_WKUP2_Pin,
  KEY_MEAS_Pin
};

static GPIO_TypeDef *const BUTTON_PORT[BUTTONn] =
{
  SYS_WKUP2_GPIO_Port,
  KEY_MEAS_GPIO_Port
};

static const IRQn_Type BUTTON_IRQn[BUTTONn] =
{
  SYS_WKUP2_EXTI_IRQn,
  KEY_MEAS_EXTI_IRQn
};

static void BSP_GPIO_EnableClock(GPIO_TypeDef *GPIOx)
{
  if (GPIOx == GPIOA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if (GPIOx == GPIOB)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (GPIOx == GPIOC)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
}

static int32_t BSP_LED_IsValid(Led_TypeDef Led)
{
  return ((uint32_t)Led < LEDn) ? 1 : 0;
}

static int32_t BSP_Button_IsValid(Button_TypeDef Button)
{
  return ((uint32_t)Button < BUTTONn) ? 1 : 0;
}

int32_t BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (BSP_LED_IsValid(Led) == 0)
  {
    return -1;
  }

  BSP_GPIO_EnableClock(LED_PORT[Led]);

  GPIO_InitStruct.Pin = LED_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* PA4/PA8 are active low; pull-ups prevent an idle/floating pin from
     producing a faint LED glow. */
  GPIO_InitStruct.Pull = ((Led == LED_MEAS) || (Led == LED_BATT)) ?
                         GPIO_PULLUP : GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT[Led], &GPIO_InitStruct);

  BSP_LED_Off(Led);
  return 0;
}

int32_t BSP_LED_DeInit(Led_TypeDef Led)
{
  if (BSP_LED_IsValid(Led) == 0)
  {
    return -1;
  }

  BSP_LED_Off(Led);
  HAL_GPIO_DeInit(LED_PORT[Led], LED_PIN[Led]);
  return 0;
}

int32_t BSP_LED_On(Led_TypeDef Led)
{
  if (BSP_LED_IsValid(Led) == 0)
  {
    return -1;
  }

  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
  return 0;
}

int32_t BSP_LED_Off(Led_TypeDef Led)
{
  if (BSP_LED_IsValid(Led) == 0)
  {
    return -1;
  }

  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
  return 0;
}

int32_t BSP_LED_Toggle(Led_TypeDef Led)
{
  if (BSP_LED_IsValid(Led) == 0)
  {
    return -1;
  }

  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
  return 0;
}

int32_t BSP_LED_GetState(Led_TypeDef Led)
{
  if (BSP_LED_IsValid(Led) == 0)
  {
    return 0;
  }

  return (HAL_GPIO_ReadPin(LED_PORT[Led], LED_PIN[Led]) == GPIO_PIN_RESET) ? 1 : 0;
}

int32_t BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (BSP_Button_IsValid(Button) == 0)
  {
    return -1;
  }

  BSP_GPIO_EnableClock(BUTTON_PORT[Button]);

  GPIO_InitStruct.Pin = BUTTON_PIN[Button];
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Mode = (Mode == BUTTON_MODE_EXTI) ? GPIO_MODE_IT_FALLING : GPIO_MODE_INPUT;
  HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);

  if (Mode == BUTTON_MODE_EXTI)
  {
    HAL_NVIC_SetPriority(BUTTON_IRQn[Button], 2, 0);
    HAL_NVIC_EnableIRQ(BUTTON_IRQn[Button]);
  }

  return 0;
}

int32_t BSP_PB_DeInit(Button_TypeDef Button)
{
  if (BSP_Button_IsValid(Button) == 0)
  {
    return -1;
  }

  HAL_GPIO_DeInit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
  return 0;
}

int32_t BSP_PB_GetState(Button_TypeDef Button)
{
  if (BSP_Button_IsValid(Button) == 0)
  {
    return 0;
  }

  return (HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]) == GPIO_PIN_RESET) ? 1 : 0;
}

void BSP_PB_IRQHandler(Button_TypeDef Button)
{
  if (BSP_Button_IsValid(Button) == 0)
  {
    return;
  }

  if (__HAL_GPIO_EXTI_GET_IT(BUTTON_PIN[Button]) != 0U)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_PIN[Button]);
    BSP_PB_Callback(Button);
  }
}

__weak void BSP_PB_Callback(Button_TypeDef Button)
{
  (void)Button;
}

int32_t BSP_SENSOR_PWR_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  BSP_GPIO_EnableClock(SENSOR_PWR_EN_GPIO_Port);

  GPIO_InitStruct.Pin = SENSOR_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  BSP_SENSOR_PWR_Off();
  return 0;
}

void BSP_SENSOR_PWR_On(void)
{
  HAL_GPIO_WritePin(SENSOR_PWR_EN_GPIO_Port, SENSOR_PWR_EN_Pin, GPIO_PIN_SET);
}

void BSP_SENSOR_PWR_Off(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* TPS22919 ON is active high.  Re-assert the PA9 output configuration
     before driving it low so that an earlier peripheral/debug GPIO setup
     cannot leave the analog rail enabled or the pin floating.  Program the
     output latch first to avoid a positive pulse when output mode is set. */
  BSP_GPIO_EnableClock(SENSOR_PWR_EN_GPIO_Port);
  HAL_GPIO_WritePin(SENSOR_PWR_EN_GPIO_Port, SENSOR_PWR_EN_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = SENSOR_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(SENSOR_PWR_EN_GPIO_Port, SENSOR_PWR_EN_Pin, GPIO_PIN_RESET);
}

int32_t BSP_SENSOR_PWR_GetState(void)
{
  return (HAL_GPIO_ReadPin(SENSOR_PWR_EN_GPIO_Port, SENSOR_PWR_EN_Pin) == GPIO_PIN_SET) ? 1 : 0;
}

int32_t BSP_UART1_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  if ((pData == 0) || (Size == 0U))
  {
    return -1;
  }

  return (HAL_UART_Transmit(&huart1, (uint8_t *)pData, Size, Timeout) == HAL_OK) ? 0 : -1;
}
