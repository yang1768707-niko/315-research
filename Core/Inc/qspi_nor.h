#ifndef QSPI_NOR_H
#define QSPI_NOR_H

#include "stm32wbxx_hal.h"
#include <stdint.h>

#define QSPI_NOR_SECTOR_SIZE 4096U
#define QSPI_NOR_PAGE_SIZE   256U

/* Enable only for cycle profiling; ordinary low-power runs leave DWT disabled. */
#ifndef QSPI_NOR_PROGRAM_TIMING
#define QSPI_NOR_PROGRAM_TIMING 0
#endif

typedef struct
{
  uint32_t clock_hz;
  uint32_t pages;
  uint32_t setup_cycles;
  uint32_t tx_cycles;
  uint32_t poll_start_delay_cycles;
  uint32_t poll_cycles;
  uint32_t completion_delay_cycles;
  uint32_t max_gap_cycles;
} QspiNor_ProgramTiming_t;

HAL_StatusTypeDef QspiNor_Init(uint32_t *jedec_id, uint32_t *capacity_bytes);
HAL_StatusTypeDef QspiNor_PrepareStop2(void);
HAL_StatusTypeDef QspiNor_EraseRange(uint32_t address, uint32_t length);
HAL_StatusTypeDef QspiNor_Write(uint32_t address, const uint8_t *data, uint32_t length);
HAL_StatusTypeDef QspiNor_Read(uint32_t address, uint8_t *data, uint32_t length);
uint32_t QspiNor_GetCapacity(void);
HAL_StatusTypeDef QspiNor_ProgramPageAsync(uint32_t address, const uint8_t *data, uint16_t length);
HAL_StatusTypeDef QspiNor_ProgramService(void);
void QspiNor_ProgramAbort(void);
/* Called from QSPI/DMA IRQ context when the async writer can advance. */
void QspiNor_ProgramEvent(void);
void QspiNor_ResetProgramTiming(void);
QspiNor_ProgramTiming_t QspiNor_GetProgramTiming(void);
uint8_t QspiNor_IsHighPerformance(void);
uint8_t QspiNor_IsQuadProgram(void);
const char *QspiNor_GetLastErrorStage(void);

#endif
