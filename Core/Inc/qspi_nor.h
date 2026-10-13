#ifndef QSPI_NOR_H
#define QSPI_NOR_H

#include "stm32wbxx_hal.h"
#include <stdint.h>

#define QSPI_NOR_SECTOR_SIZE 4096U
#define QSPI_NOR_PAGE_SIZE   256U

HAL_StatusTypeDef QspiNor_Init(uint32_t *jedec_id, uint32_t *capacity_bytes);
HAL_StatusTypeDef QspiNor_EraseRange(uint32_t address, uint32_t length);
HAL_StatusTypeDef QspiNor_Write(uint32_t address, const uint8_t *data, uint32_t length);
HAL_StatusTypeDef QspiNor_Read(uint32_t address, uint8_t *data, uint32_t length);
uint32_t QspiNor_GetCapacity(void);
HAL_StatusTypeDef QspiNor_ProgramPageAsync(uint32_t address, const uint8_t *data, uint16_t length);
HAL_StatusTypeDef QspiNor_ProgramService(void);
void QspiNor_ProgramAbort(void);
uint8_t QspiNor_IsHighPerformance(void);
uint8_t QspiNor_IsQuadProgram(void);
const char *QspiNor_GetLastErrorStage(void);

#endif
