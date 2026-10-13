/* USER CODE BEGIN Header */
/**
  ***************************************************************************************
  * @file    stm32_lpm_if.c
  * @author  MCD Application Team
  * @brief   Low layer function to enter/exit low power modes (stop, sleep).
  ***************************************************************************************
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

/* Includes ------------------------------------------------------------------*/
#include "stm32_lpm_if.h"
#include "stm32_lpm.h"
#include "app_conf.h"
/* USER CODE BEGIN include */
#include "main.h"

/* USER CODE END include */

/* Exported variables --------------------------------------------------------*/
const struct UTIL_LPM_Driver_s UTIL_PowerDriver =
{
  PWR_EnterSleepMode,
  PWR_ExitSleepMode,

  PWR_EnterStopMode,
  PWR_ExitStopMode,

  PWR_EnterOffMode,
  PWR_ExitOffMode,
};

/* Private function prototypes -----------------------------------------------*/
static void Switch_On_HSI(void);
static void EnterLowPower(void);
static void ExitLowPower(void);
/* USER CODE BEGIN Private_Function_Prototypes */
extern void PeriphCommonClock_Config(void);
extern UART_HandleTypeDef huart1;
static void RestoreUsart1Clocking(void);
void PWR_SetRunModeLock(uint8_t lock);

/* USER CODE END Private_Function_Prototypes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN Private_Typedef */

/* USER CODE END Private_Typedef */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Private_Define */

/* USER CODE END Private_Define */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Private_Macro */

/* USER CODE END Private_Macro */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Private_Variables */
static volatile uint8_t RunModeLock;
static volatile uint8_t PeriphClocksRestorePending;
static volatile uint8_t Stop2WakeTraceEligible;
static uint8_t FirstStop2WakeTracePending = 1U;

/* USER CODE END Private_Variables */

/* Functions Definition ------------------------------------------------------*/
/**
  * @brief Enters Low Power Off Mode
  * @param none
  * @retval none
  */
void PWR_EnterOffMode(void)
{
/* USER CODE BEGIN PWR_EnterOffMode_1 */
  /* Turn off the power LED before entering standby mode. */
  HAL_GPIO_WritePin(LED_BATT_GPIO_Port, LED_BATT_Pin, GPIO_PIN_SET);

/* USER CODE END PWR_EnterOffMode_1 */
  /**
   * The systick should be disabled for the same reason than when the device enters stop mode because
   * at this time, the device may enter either OffMode or StopMode.
   */
  HAL_SuspendTick();

  EnterLowPower();

  /************************************************************************************
   * ENTER OFF MODE
   ***********************************************************************************/
  /*
   * There is no risk to clear all the WUF here because in the current implementation, this API is called
   * in critical section. If an interrupt occurs while in that critical section before that point,
   * the flag is set and will be cleared here but the system will not enter Off Mode
   * because an interrupt is pending in the NVIC. The ISR will be executed when moving out
   * of this critical section
   */
  LL_PWR_ClearFlag_WU();

  LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);

  LL_LPM_EnableDeepSleep(); /**< Set SLEEPDEEP bit of Cortex System Control Register */

  /**
   * This option is used to ensure that store operations are completed
   */
#if defined (__CC_ARM) || defined (__ARMCC_VERSION)
  __force_stores();
#endif

  __WFI();

/* USER CODE BEGIN PWR_EnterOffMode_2 */

/* USER CODE END PWR_EnterOffMode_2 */
  return;
}

/**
  * @brief Exits Low Power Off Mode
  * @param none
  * @retval none
  */
void PWR_ExitOffMode(void)
{
/* USER CODE BEGIN PWR_ExitOffMode_1 */
  /*
   * A pending CPU2/IPCC interrupt can abort standby entry.  EnterLowPower()
   * may already have switched CPU1 to HSI in that case, so restore the same
   * 64 MHz PLL clock used after a STOP2 wake-up before returning to the
   * sequencer.  If standby succeeds this function is never reached.
   */
  ExitLowPower();
/* USER CODE END PWR_ExitOffMode_1 */
  HAL_ResumeTick();
/* USER CODE BEGIN PWR_ExitOffMode_2 */
  /* HAL clock configuration uses SysTick-based timeouts. The LPM exit
     callback still runs inside the sequencer's interrupt-masked idle section,
     so defer it until PostIdle. */
  PeriphClocksRestorePending = 1U;
  Stop2WakeTraceEligible = 0U;
  /* Restore the power LED if standby entry was aborted by an interrupt. */
  HAL_GPIO_WritePin(LED_BATT_GPIO_Port, LED_BATT_Pin, GPIO_PIN_RESET);
/* USER CODE END PWR_ExitOffMode_2 */
  return;
}

/**
  * @brief Enters Low Power Stop Mode
  * @note ARM exists the function when waking up
  * @param none
  * @retval none
  */
void PWR_EnterStopMode(void)
{
/* USER CODE BEGIN PWR_EnterStopMode_1 */
  /* Last-chance hardware cutoff for the analog microphone/op-amp rail.
     TPS22919 ON is active high, so PA9 must remain low throughout STOP2. */
  HAL_GPIO_WritePin(SENSOR_PWR_EN_GPIO_Port, SENSOR_PWR_EN_Pin, GPIO_PIN_RESET);

  /* Turn off the power LED before entering STOP2. */
  HAL_GPIO_WritePin(LED_BATT_GPIO_Port, LED_BATT_Pin, GPIO_PIN_SET);

/* USER CODE END PWR_EnterStopMode_1 */
  /**
   * When HAL_DBGMCU_EnableDBGStopMode() is called to keep the debugger active in Stop Mode,
   * the systick shall be disabled otherwise the cpu may crash when moving out from stop mode
   *
   * When in production, the HAL_DBGMCU_EnableDBGStopMode() is not called so that the device can reach best power consumption
   * However, the systick should be disabled anyway to avoid the case when it is about to expire at the same time the device enters
   * stop mode (this will abort the Stop Mode entry).
   */
  HAL_SuspendTick();

  /**
   * This function is called from CRITICAL SECTION
   */
  EnterLowPower();

  /************************************************************************************
   * ENTER STOP MODE
   ***********************************************************************************/
  LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);

  LL_LPM_EnableDeepSleep(); /**< Set SLEEPDEEP bit of Cortex System Control Register */

  /**
   * This option is used to ensure that store operations are completed
   */
#if defined (__CC_ARM) || defined (__ARMCC_VERSION)
  __force_stores();
#endif

  __WFI();

/* USER CODE BEGIN PWR_EnterStopMode_2 */

/* USER CODE END PWR_EnterStopMode_2 */
  return;
}

/**
  * @brief Exits Low Power Stop Mode
  * @note Restore the 64 MHz PLL system clock
  * @param none
  * @retval none
  */
void PWR_ExitStopMode(void)
{
/* USER CODE BEGIN PWR_ExitStopMode_1 */

/* USER CODE END PWR_ExitStopMode_1 */
  /**
   * This function is called from CRITICAL SECTION
   */
  ExitLowPower();

  HAL_ResumeTick();
/* USER CODE BEGIN PWR_ExitStopMode_2 */
  /* STOP2 disables PLLSAI1. Reconfiguring it here may wait on HAL_GetTick(),
     but SysTick cannot run until the sequencer unmasks interrupts. */
  PeriphClocksRestorePending = 1U;
  Stop2WakeTraceEligible = 1U;
  /* The main system PLL is restored above; turn the run LED back on now. */
  HAL_GPIO_WritePin(LED_BATT_GPIO_Port, LED_BATT_Pin, GPIO_PIN_RESET);

/* USER CODE END PWR_ExitStopMode_2 */
  return;
}

/**
  * @brief Enters Low Power Sleep Mode
  * @note ARM exits the function when waking up
  * @param none
  * @retval none
  */
void PWR_EnterSleepMode(void)
{
/* USER CODE BEGIN PWR_EnterSleepMode_1 */
  /* Keep SysTick running while a high-rate sensor stream is active. */
  if (RunModeLock != 0U)
  {
    return;
  }

/* USER CODE END PWR_EnterSleepMode_1 */

  HAL_SuspendTick();

  /************************************************************************************
   * ENTER SLEEP MODE
   ***********************************************************************************/
  LL_LPM_EnableSleep(); /**< Clear SLEEPDEEP bit of Cortex System Control Register */

  /**
   * This option is used to ensure that store operations are completed
   */
#if defined (__CC_ARM) || defined (__ARMCC_VERSION)
  __force_stores();
#endif

  __WFI();
/* USER CODE BEGIN PWR_EnterSleepMode_2 */

/* USER CODE END PWR_EnterSleepMode_2 */
  return;
}

/**
  * @brief Exits Low Power Sleep Mode
  * @note ARM exits the function when waking up
  * @param none
  * @retval none
  */
void PWR_ExitSleepMode(void)
{
/* USER CODE BEGIN PWR_ExitSleepMode_1 */

/* USER CODE END PWR_ExitSleepMode_1 */
  HAL_ResumeTick();
/* USER CODE BEGIN PWR_ExitSleepMode_2 */

/* USER CODE END PWR_ExitSleepMode_2 */
  return;
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/**
  * @brief Setup the system to enter either stop or off mode
  * @param none
  * @retval none
  */
static void EnterLowPower(void)
{
  /**
   * This function is called from CRITICAL SECTION
   */

  while(LL_HSEM_1StepLock(HSEM, CFG_HW_RCC_SEMID));

  if (! LL_HSEM_1StepLock(HSEM, CFG_HW_ENTRY_STOP_MODE_SEMID))
  {
    if(LL_PWR_IsActiveFlag_C2DS() || LL_PWR_IsActiveFlag_C2SB())
    {
      /* Release ENTRY_STOP_MODE semaphore */
      LL_HSEM_ReleaseLock(HSEM, CFG_HW_ENTRY_STOP_MODE_SEMID, 0);

      Switch_On_HSI();
      __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
    }
  }
  else
  {
    Switch_On_HSI();
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
  }

  /* Release RCC semaphore */
  LL_HSEM_ReleaseLock(HSEM, CFG_HW_RCC_SEMID, 0);

  return;
}

/**
  * @brief Restore the system to exit stop mode
  * @param none
  * @retval none
  */
static void ExitLowPower(void)
{
  /* Release ENTRY_STOP_MODE semaphore */
  LL_HSEM_ReleaseLock(HSEM, CFG_HW_ENTRY_STOP_MODE_SEMID, 0);

  while(LL_HSEM_1StepLock(HSEM, CFG_HW_RCC_SEMID));

  if(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
/* Restore the clock configuration of the application in this user section */
/* USER CODE BEGIN ExitLowPower_1 */
    /*
     * STOP2 wakes CPU1 on the 16 MHz HSI and disables the main PLL.  The
     * application and USART1 baud-rate divider are configured for a 64 MHz
     * system clock, so restore HSE + PLL before any sequencer task resumes.
     * PLL dividers are retained in STOP2 (HSE / 8 * 32 / 2 = 64 MHz).
     */
    LL_RCC_HSE_Enable();
    while (LL_RCC_HSE_IsReady() == 0U) { }

    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_3);
    while (__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_3) { }

    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() == 0U) { }

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) { }
/* USER CODE END ExitLowPower_1 */
  }
  else
  {
/* If the application is not running on HSE restore the clock configuration in this user section */
/* USER CODE BEGIN ExitLowPower_2 */

/* USER CODE END ExitLowPower_2 */
  }

  /* Keep HAL clock queries consistent with the restored hardware clock. */
  SystemCoreClockUpdate();

  /* Release RCC semaphore */
  LL_HSEM_ReleaseLock(HSEM, CFG_HW_RCC_SEMID, 0);

  return;
}

/**
  * @brief Switch the system clock on HSI
  * @param none
  * @retval none
  */
static void Switch_On_HSI(void)
{
  LL_RCC_HSI_Enable();
  while(!LL_RCC_HSI_IsReady());
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  LL_RCC_SetSMPSClockSource(LL_RCC_SMPS_CLKSOURCE_HSI);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI);
  return;
}

/* USER CODE BEGIN Private_Functions */
void PWR_SetRunModeLock(uint8_t lock)
{
  RunModeLock = (lock != 0U) ? 1U : 0U;
  __DMB();
}

void PWR_CompleteStopModeWakeup(void)
{
  static const uint8_t wake_begin[] =
      "LPM: STOP2 wake, restoring peripheral clocks\r\n";
  static const uint8_t wake_complete[] =
      "LPM: STOP2 wake complete\r\n";

  if (PeriphClocksRestorePending == 0U)
  {
    return;
  }
  PeriphClocksRestorePending = 0U;
  /* Called from UTIL_SEQ_PostIdle with interrupts enabled, before any BLE or
     Zigbee task can use ADC/SAI or transmit on USART1. */
  RestoreUsart1Clocking();
  if ((FirstStop2WakeTracePending != 0U) &&
      (Stop2WakeTraceEligible != 0U))
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)wake_begin,
                            sizeof(wake_begin) - 1U, 20U);
  }
  PeriphCommonClock_Config();
  if ((FirstStop2WakeTracePending != 0U) &&
      (Stop2WakeTraceEligible != 0U))
  {
    FirstStop2WakeTracePending = 0U;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)wake_complete,
                            sizeof(wake_complete) - 1U, 20U);
  }
}

static void RestoreUsart1Clocking(void)
{
  uint32_t usartdiv;
  uint32_t peripheral_clock = HAL_RCC_GetPCLK2Freq();

  /*
   * USART1 uses PCLK2. Its BRR register is retained in STOP2, but explicitly
   * recomputing it closes the wake-up race where an IPCC event is serviced
   * after CPU1 has fallen back to HSI and before all HAL clock state is in
   * sync again.
   */
  if (huart1.Init.OverSampling == UART_OVERSAMPLING_8)
  {
    uint16_t brr;

    usartdiv = UART_DIV_SAMPLING8(peripheral_clock,
                                 huart1.Init.BaudRate,
                                 huart1.Init.ClockPrescaler);
    brr = (uint16_t)(usartdiv & 0xFFF0U);
    brr |= (uint16_t)((usartdiv & 0x000FU) >> 1U);
    WRITE_REG(huart1.Instance->BRR, brr);
  }
  else
  {
    usartdiv = UART_DIV_SAMPLING16(peripheral_clock,
                                  huart1.Init.BaudRate,
                                  huart1.Init.ClockPrescaler);
    WRITE_REG(huart1.Instance->BRR, (uint16_t)usartdiv);
  }
}

/* USER CODE END Private_Functions */
