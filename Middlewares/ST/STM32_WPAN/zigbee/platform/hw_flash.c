/*****************************************************************************
 * @file    hw_flash.c
 * @author  MCD Application Team
 * @brief   This file contains the Flash driver needed by EE module
 *****************************************************************************
 * @attention
 *
 * Copyright (c) 2018-2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *****************************************************************************
 */

#include "hw_flash.h"
#include "stm32wbxx_hal.h"
#include "stm32wbxx_ll_hsem.h"
#include "app_conf.h"
#include "hw_conf.h"
#include "shci.h"
#include "stm32_lpm.h"

/* This driver is used only by the Zigbee EEPROM emulator. Keep its writes
 * inside the linker-reserved NVM area, without changing the EEPROM format. */
#define NVM_FIRST_ADDRESS (FLASH_BASE + CFG_NVM_BASE_ADDRESS)
#define NVM_END_ADDRESS   (NVM_FIRST_ADDRESS + CFG_EE_BANK0_SIZE)
#define NVM_FIRST_PAGE    (CFG_NVM_BASE_ADDRESS / FLASH_PAGE_SIZE)
#define NVM_END_PAGE      (NVM_FIRST_PAGE + CFG_EE_BANK0_SIZE / FLASH_PAGE_SIZE)
#define FLASH_ARBITRATION_TIMEOUT_MS 1000U

static volatile uint8_t flash_active;

static int HW_FLASH_Begin(void);
static int HW_FLASH_End(int result);
static int HW_FLASH_SingleOperation(uint32_t target, uint64_t data, int erase);

/*****************************************************************************/

int HW_FLASH_Write(uint32_t address, uint64_t data)
{
  int result;

  if ((address < NVM_FIRST_ADDRESS) || (address > NVM_END_ADDRESS - 8U) ||
      ((address & 7U) != 0U))
  {
    return HW_ERROR;
  }
  result = HW_FLASH_Begin();
  if (result != HW_OK) return result;

  result = HW_FLASH_SingleOperation(address, data, 0);
  if (result == HW_OK)
  {
    /* Invalidate the CPU1 data cache before checking the physical contents. */
    if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != 0U)
    {
      __HAL_FLASH_DATA_CACHE_DISABLE();
      __HAL_FLASH_DATA_CACHE_RESET();
      __HAL_FLASH_DATA_CACHE_ENABLE();
    }
    if (*(volatile uint64_t *)(uintptr_t)address != data) result = HW_ERROR;
  }
  return HW_FLASH_End(result);
}

/*****************************************************************************/

int HW_FLASH_Erase(uint32_t page, uint16_t n, int interrupt)
{
  UNUSED(interrupt);
  int result;

  if ((page < NVM_FIRST_PAGE) || (page >= NVM_END_PAGE) ||
      (n == 0U) || ((uint32_t)n > NVM_END_PAGE - page))
  {
    return HW_ERROR;
  }
  result = HW_FLASH_Begin();
  if (result != HW_OK) return result;

  /* SHCI commands must run with interrupts enabled and outside SEM7. Even
   * a rejected ON is paired with OFF in case CPU2 changed its state. */
  result = HW_ERROR;
  if (SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_ON) == SHCI_Success)
  {
    result = HW_OK;
    for (uint32_t offset = 0U; offset < n && result == HW_OK; offset++)
    {
      result = HW_FLASH_SingleOperation(page + offset, 0U, 1);
      if (result == HW_OK)
      {
        uintptr_t address = FLASH_BASE + (page + offset) * FLASH_PAGE_SIZE;
        for (uint32_t word = 0U; word < FLASH_PAGE_SIZE / 8U; word++)
        {
          if (((volatile uint64_t *)address)[word] != UINT64_MAX)
          {
            result = HW_ERROR;
            break;
          }
        }
      }
    }
  }
  if (SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_OFF) != SHCI_Success)
  {
    result = HW_ERROR;
  }
  return HW_FLASH_End(result);
}

static int HW_FLASH_Begin(void)
{
  uint32_t started;

  /* EEPROM calls are task-context only. Do not wait for SysTick or SHCI from
   * an ISR/masked context. The guard also rejects nested sequencer calls:
   * HSEM one-step locking alone accepts a lock already owned by CPU1. */
  if ((__get_IPSR() != 0U) || (__get_PRIMASK() != 0U)) return HW_BUSY;
  __disable_irq();
  if (flash_active != 0U)
  {
    __enable_irq();
    return HW_BUSY;
  }
  flash_active = 1U;
  __enable_irq();
  UTIL_LPM_SetStopMode(1U << CFG_LPM_NVM_FLASH, UTIL_LPM_DISABLE);

  started = HAL_GetTick();
  while (LL_HSEM_1StepLock(HSEM, CFG_HW_FLASH_SEMID) != 0U)
  {
    if ((HAL_GetTick() - started) >= FLASH_ARBITRATION_TIMEOUT_MS)
    {
      flash_active = 0U;
      UTIL_LPM_SetStopMode(1U << CFG_LPM_NVM_FLASH, UTIL_LPM_ENABLE);
      return HW_BUSY;
    }
  }
  if (HAL_FLASH_Unlock() != HAL_OK) return HW_FLASH_End(HW_ERROR);
  /* OPTVERR may be set at power-on on STM32WB. Other errors are handled
   * by the HAL instead of being silently discarded. */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  return HW_OK;
}

static int HW_FLASH_End(int result)
{
  if (HAL_FLASH_Lock() != HAL_OK) result = HW_ERROR;
  LL_HSEM_ReleaseLock(HSEM, CFG_HW_FLASH_SEMID, 0U);
  flash_active = 0U;
  UTIL_LPM_SetStopMode(1U << CFG_LPM_NVM_FLASH, UTIL_LPM_ENABLE);
  return result;
}

static int HW_FLASH_SingleOperation(uint32_t target, uint64_t data, int erase)
{
  uint32_t started = HAL_GetTick();

  for (;;)
  {
    /* Leave >=1 us between SEM7 acquisitions at CPU1's maximum 64 MHz.
     * Apply this to writes as well as erases, including successive EE calls. */
    for (volatile uint32_t gap = 0U; gap < 64U; gap++) __NOP();

    /* Wait outside the critical section so radio/IPCC interrupts and SysTick
     * keep running. Recheck after masking interrupts to close the CPU1 race. */
    if (READ_BIT(FLASH->SR, FLASH_FLAG_PESD | FLASH_FLAG_BSY | FLASH_FLAG_CFGBSY) == 0U)
    {
      __disable_irq();
      if ((LL_HSEM_GetStatus(HSEM, CFG_HW_BLOCK_FLASH_REQ_BY_CPU1_SEMID) == 0U) &&
          (READ_BIT(FLASH->SR, FLASH_FLAG_PESD | FLASH_FLAG_BSY | FLASH_FLAG_CFGBSY) == 0U) &&
          (LL_HSEM_1StepLock(HSEM, CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID) == 0U))
      {
        HAL_StatusTypeDef status;
        uint32_t page_error = UINT32_MAX;

        /* AN5289: exactly one double-word or one page per SEM7 grant.
         * Use the polling HAL, including its completion/error handling;
         * never enable EOP interrupts (this project has no Flash ISR). */
        if (erase != 0)
        {
          FLASH_EraseInitTypeDef request = {0};
          request.TypeErase = FLASH_TYPEERASE_PAGES;
          request.Page = target;
          request.NbPages = 1U;
          status = HAL_FLASHEx_Erase(&request, &page_error);
        }
        else
        {
          status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, target, data);
        }
        LL_HSEM_ReleaseLock(HSEM, CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID, 0U);
        __enable_irq();
        if (status == HAL_TIMEOUT) return HW_TIMEOUT;
        return ((status == HAL_OK) && (page_error == UINT32_MAX)) ? HW_OK : HW_ERROR;
      }
      __enable_irq();
    }
    if ((HAL_GetTick() - started) >= FLASH_ARBITRATION_TIMEOUT_MS) return HW_BUSY;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE***/
