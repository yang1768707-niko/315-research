#include "qspi_nor.h"

#include "main.h"

#include <string.h>
#define NOR_CMD_READ_ID       0x9FU
#define NOR_CMD_READ_STATUS   0x05U
#define NOR_CMD_READ_CONFIG   0x15U
#define NOR_CMD_WRITE_STATUS  0x01U
#define NOR_CMD_WRITE_ENABLE  0x06U
#define NOR_CMD_READ          0x03U
#define NOR_CMD_QUAD_READ     0x6BU
#define NOR_QUAD_READ_DUMMY   8U
#define NOR_CMD_PAGE_PROGRAM  0x02U
#define NOR_CMD_QUAD_PAGE_PROGRAM 0x38U
#define NOR_CMD_SECTOR_ERASE  0x20U
#define NOR_STATUS_BUSY       0x01U
#define NOR_STATUS_WEL        0x02U
#define NOR_STATUS_QE         0x40U
#define NOR_CONFIG2_HIGH_PERFORMANCE 0x02U
#define NOR_JEDEC_MX25R6435F  0x00C22817UL
#define NOR_TIMEOUT_MS        5000U
#define NOR_ERASE_TIMEOUT_MS  3000U
#define NOR_STARTUP_PRESCALER 3U
#define NOR_STARTUP_FIFO      4U
#define NOR_RUN_PRESCALER     2U
#define NOR_RUN_FIFO          4U

static uint32_t nor_capacity_bytes;
static uint8_t nor_high_performance;
static uint8_t nor_quad_program;
static const char *nor_last_error_stage = "none";
extern QSPI_HandleTypeDef hqspi;
/* ISR callbacks only advance flags. Busy polling is started in task context. */
static volatile uint8_t nor_async_state; /* 0 idle, 1 TX, 2 TX done, 3 WIP, 4 done, 5 error */
static uint32_t nor_async_started;
static QspiNor_ProgramTiming_t nor_timing;
static uint32_t nor_setup_at, nor_tx_at, nor_poll_at, nor_prev_match_at;
static volatile uint32_t nor_tx_done_at, nor_match_at;

void QspiNor_ResetProgramTiming(void)
{
  memset(&nor_timing, 0, sizeof(nor_timing));
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U)
  {
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U)
      nor_timing.clock_hz = SystemCoreClock;
  }
}

QspiNor_ProgramTiming_t QspiNor_GetProgramTiming(void)
{
  return nor_timing;
}

static HAL_StatusTypeDef QspiNor_ConfigureController(uint32_t prescaler,
                                                      uint32_t fifo_threshold)
{
  hqspi.Init.ClockPrescaler = prescaler;
  hqspi.Init.FifoThreshold = fifo_threshold;
  return HAL_QSPI_Init(&hqspi);
}

static void QspiNor_CommandDefaults(QSPI_CommandTypeDef *command)
{
  memset(command, 0, sizeof(*command));
  command->InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command->AddressMode = QSPI_ADDRESS_NONE;
  command->AddressSize = QSPI_ADDRESS_24_BITS;
  command->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command->DataMode = QSPI_DATA_NONE;
  command->DummyCycles = 0U;
  command->DdrMode = QSPI_DDR_MODE_DISABLE;
  command->SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
}

static HAL_StatusTypeDef QspiNor_WaitReady(uint32_t timeout_ms)
{
  QSPI_CommandTypeDef command;
  QSPI_AutoPollingTypeDef polling;
  QspiNor_CommandDefaults(&command);
  command.Instruction = NOR_CMD_READ_STATUS;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = 1U;
  memset(&polling, 0, sizeof(polling));
  polling.Match = 0U;
  polling.Mask = NOR_STATUS_BUSY;
  polling.MatchMode = QSPI_MATCH_MODE_AND;
  polling.StatusBytesSize = 1U;
  polling.Interval = 0x10U;
  polling.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
  return HAL_QSPI_AutoPolling(&hqspi, &command, &polling, timeout_ms);
}

static HAL_StatusTypeDef QspiNor_WriteEnable(void)
{
  QSPI_CommandTypeDef command;
  QSPI_AutoPollingTypeDef polling;
  QspiNor_CommandDefaults(&command);
  command.Instruction = NOR_CMD_WRITE_ENABLE;
  if (HAL_QSPI_Command(&hqspi, &command, NOR_TIMEOUT_MS) != HAL_OK) return HAL_ERROR;

  QspiNor_CommandDefaults(&command);
  command.Instruction = NOR_CMD_READ_STATUS;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = 1U;
  memset(&polling, 0, sizeof(polling));
  polling.Match = NOR_STATUS_WEL;
  polling.Mask = NOR_STATUS_WEL;
  polling.MatchMode = QSPI_MATCH_MODE_AND;
  polling.StatusBytesSize = 1U;
  polling.Interval = 0x10U;
  polling.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
  return HAL_QSPI_AutoPolling(&hqspi, &command, &polling, NOR_TIMEOUT_MS);
}

static HAL_StatusTypeDef QspiNor_ReadStatusAndConfig(uint8_t *status,
                                                     uint8_t config[2])
{
  QSPI_CommandTypeDef command;

  if ((status == NULL) || (config == NULL))
  {
    return HAL_ERROR;
  }

  QspiNor_CommandDefaults(&command);
  command.Instruction = NOR_CMD_READ_STATUS;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = 1U;
  if ((HAL_QSPI_Command(&hqspi, &command, NOR_TIMEOUT_MS) != HAL_OK) ||
      (HAL_QSPI_Receive(&hqspi, status, NOR_TIMEOUT_MS) != HAL_OK))
  {
    return HAL_ERROR;
  }

  QspiNor_CommandDefaults(&command);
  command.Instruction = NOR_CMD_READ_CONFIG;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = 2U;
  if ((HAL_QSPI_Command(&hqspi, &command, NOR_TIMEOUT_MS) != HAL_OK) ||
      (HAL_QSPI_Receive(&hqspi, config, NOR_TIMEOUT_MS) != HAL_OK))
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

static HAL_StatusTypeDef QspiNor_EnableMx25rWriteMode(void)
{
  QSPI_CommandTypeDef command;
  uint8_t status;
  uint8_t config[2];
  uint8_t registers[3];

  if (QspiNor_ReadStatusAndConfig(&status, config) != HAL_OK)
  {
    nor_last_error_stage = "read-status-config";
    return HAL_ERROR;
  }
  if (((status & NOR_STATUS_QE) != 0U) &&
      ((config[1] & NOR_CONFIG2_HIGH_PERFORMANCE) != 0U))
  {
    nor_high_performance = 1U;
    nor_quad_program = 1U;
    return HAL_OK;
  }

  /* MX25R6435F changes QE and its volatile L/H switch through WRSR. Preserve
     all protection bits, enable QE for 1-4-4 Quad Page Program, and select
     high-performance mode. WIP and WEL are read-only and are written as 0. */
  registers[0] = (status & (uint8_t)~(NOR_STATUS_BUSY | NOR_STATUS_WEL)) |
                 NOR_STATUS_QE;
  registers[1] = config[0];
  registers[2] = config[1] | NOR_CONFIG2_HIGH_PERFORMANCE;
  if (QspiNor_WriteEnable() != HAL_OK)
  {
    nor_last_error_stage = "mode-write-enable";
    return HAL_ERROR;
  }
  QspiNor_CommandDefaults(&command);
  command.Instruction = NOR_CMD_WRITE_STATUS;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = sizeof(registers);
  if (HAL_QSPI_Command(&hqspi, &command, NOR_TIMEOUT_MS) != HAL_OK)
  {
    nor_last_error_stage = "mode-wrsr-command";
    return HAL_ERROR;
  }
  if (HAL_QSPI_Transmit(&hqspi, registers, NOR_TIMEOUT_MS) != HAL_OK)
  {
    nor_last_error_stage = "mode-wrsr-data";
    return HAL_ERROR;
  }
  if (QspiNor_WaitReady(NOR_TIMEOUT_MS) != HAL_OK)
  {
    nor_last_error_stage = "mode-wait-ready";
    return HAL_ERROR;
  }
  if (QspiNor_ReadStatusAndConfig(&status, config) != HAL_OK)
  {
    nor_last_error_stage = "mode-readback";
    return HAL_ERROR;
  }
  if (((status & NOR_STATUS_QE) == 0U) ||
      ((config[1] & NOR_CONFIG2_HIGH_PERFORMANCE) == 0U))
  {
    nor_last_error_stage = "mode-verify";
    return HAL_ERROR;
  }

  nor_high_performance = 1U;
  nor_quad_program = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef QspiNor_Init(uint32_t *jedec_id, uint32_t *capacity_bytes)
{
  QSPI_CommandTypeDef command;
  uint8_t id[3] = {0U};
  uint32_t id_value;

  nor_capacity_bytes = 0U;
  nor_high_performance = 0U;
  nor_quad_program = 0U;
  nor_last_error_stage = "startup-controller";
  if (QspiNor_ConfigureController(NOR_STARTUP_PRESCALER,
                                  NOR_STARTUP_FIFO) != HAL_OK)
  {
    return HAL_ERROR;
  }
  QspiNor_CommandDefaults(&command);
  command.Instruction = NOR_CMD_READ_ID;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = sizeof(id);
  if (HAL_QSPI_Command(&hqspi, &command, NOR_TIMEOUT_MS) != HAL_OK)
  {
    nor_last_error_stage = "read-id-command";
    return HAL_ERROR;
  }
  if (HAL_QSPI_Receive(&hqspi, id, NOR_TIMEOUT_MS) != HAL_OK)
  {
    nor_last_error_stage = "read-id-data";
    return HAL_ERROR;
  }
  if ((id[0] == 0U) || (id[0] == 0xFFU) || (id[2] < 0x10U) || (id[2] > 0x18U))
  {
    nor_last_error_stage = "read-id-invalid";
    return HAL_ERROR;
  }

  id_value = ((uint32_t)id[0] << 16U) | ((uint32_t)id[1] << 8U) | id[2];
  nor_capacity_bytes = 1UL << id[2];
  if (jedec_id != NULL)
    *jedec_id = id_value;
  if (capacity_bytes != NULL) *capacity_bytes = nor_capacity_bytes;
  if (QspiNor_WaitReady(NOR_TIMEOUT_MS) != HAL_OK)
  {
    nor_last_error_stage = "startup-wait-ready";
    return HAL_ERROR;
  }
  if ((id_value == NOR_JEDEC_MX25R6435F) &&
      (QspiNor_EnableMx25rWriteMode() != HAL_OK))
  {
    return HAL_ERROR;
  }
  nor_last_error_stage = "run-controller";
  if (QspiNor_ConfigureController(NOR_RUN_PRESCALER,
                                  NOR_RUN_FIFO) != HAL_OK)
  {
    return HAL_ERROR;
  }
  nor_last_error_stage = "none";
  return HAL_OK;
}

HAL_StatusTypeDef QspiNor_EraseRange(uint32_t address, uint32_t length)
{
  if ((length == 0U) || (nor_capacity_bytes == 0U) ||
      (address >= nor_capacity_bytes) || (length > nor_capacity_bytes - address)) return HAL_ERROR;
  uint32_t first = address & ~(QSPI_NOR_SECTOR_SIZE - 1U);
  uint32_t end = (address + length + QSPI_NOR_SECTOR_SIZE - 1U) &
                 ~(QSPI_NOR_SECTOR_SIZE - 1U);
  for (uint32_t current = first; current < end; current += QSPI_NOR_SECTOR_SIZE)
  {
    QSPI_CommandTypeDef command;
    if (QspiNor_WriteEnable() != HAL_OK) return HAL_ERROR;
    QspiNor_CommandDefaults(&command);
    command.Instruction = NOR_CMD_SECTOR_ERASE;
    command.AddressMode = QSPI_ADDRESS_1_LINE;
    command.Address = current;
    if ((HAL_QSPI_Command(&hqspi, &command, NOR_TIMEOUT_MS) != HAL_OK) ||
        (QspiNor_WaitReady(NOR_ERASE_TIMEOUT_MS) != HAL_OK)) return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef QspiNor_Write(uint32_t address, const uint8_t *data, uint32_t length)
{
  if (nor_async_state != 0U) return HAL_BUSY;
  if ((data == NULL) || (length == 0U) || (nor_capacity_bytes == 0U) ||
      (address >= nor_capacity_bytes) || (length > nor_capacity_bytes - address)) return HAL_ERROR;
  while (length > 0U)
  {
    QSPI_CommandTypeDef command;
    uint32_t page_space = QSPI_NOR_PAGE_SIZE - (address % QSPI_NOR_PAGE_SIZE);
    uint32_t chunk = (length < page_space) ? length : page_space;
    if (QspiNor_WriteEnable() != HAL_OK) return HAL_ERROR;
    QspiNor_CommandDefaults(&command);
    command.Instruction = (nor_quad_program != 0U) ?
                          NOR_CMD_QUAD_PAGE_PROGRAM : NOR_CMD_PAGE_PROGRAM;
    command.AddressMode = (nor_quad_program != 0U) ?
                          QSPI_ADDRESS_4_LINES : QSPI_ADDRESS_1_LINE;
    command.Address = address;
    command.DataMode = (nor_quad_program != 0U) ?
                       QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
    command.NbData = chunk;
    if ((HAL_QSPI_Command(&hqspi, &command, NOR_TIMEOUT_MS) != HAL_OK) ||
        (HAL_QSPI_Transmit(&hqspi, (uint8_t *)data, NOR_TIMEOUT_MS) != HAL_OK) ||
        (QspiNor_WaitReady(NOR_TIMEOUT_MS) != HAL_OK)) return HAL_ERROR;
    address += chunk;
    data += chunk;
    length -= chunk;
  }
  return HAL_OK;
}

HAL_StatusTypeDef QspiNor_Read(uint32_t address, uint8_t *data, uint32_t length)
{
  if (nor_async_state != 0U) return HAL_BUSY;
  QSPI_CommandTypeDef command;
  if ((data == NULL) || (length == 0U) || (nor_capacity_bytes == 0U) ||
      (address >= nor_capacity_bytes) || (length > nor_capacity_bytes - address))
  {
    nor_last_error_stage = "read-arguments";
    return HAL_ERROR;
  }
  QspiNor_CommandDefaults(&command);
  /* QE was verified during MX25R6435F initialization. Keep short metadata
     reads and other parts on the original single-line path. */
  uint8_t quad = (nor_quad_program != 0U) && (length >= 128U);
  command.Instruction = quad ? NOR_CMD_QUAD_READ : NOR_CMD_READ;
  command.AddressMode = QSPI_ADDRESS_1_LINE;
  command.Address = address;
  command.DataMode = quad ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  command.DummyCycles = quad ? NOR_QUAD_READ_DUMMY : 0U;
  command.NbData = length;
  if (HAL_QSPI_Command(&hqspi, &command, NOR_TIMEOUT_MS) != HAL_OK)
  {
    nor_last_error_stage = "read-command";
    return HAL_ERROR;
  }
  if (HAL_QSPI_Receive(&hqspi, data, NOR_TIMEOUT_MS) != HAL_OK)
  {
    nor_last_error_stage = "read-data";
    return HAL_ERROR;
  }
  nor_last_error_stage = "none";
  return HAL_OK;
}

uint32_t QspiNor_GetCapacity(void)
{
  return nor_capacity_bytes;
}

uint8_t QspiNor_IsHighPerformance(void)
{
  return nor_high_performance;
}

uint8_t QspiNor_IsQuadProgram(void)
{
  return nor_quad_program;
}

const char *QspiNor_GetLastErrorStage(void)
{
  return nor_last_error_stage;
}

__weak void QspiNor_ProgramEvent(void)
{
}
 
 void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *handle)
 {
   if (handle == &hqspi && nor_async_state == 1U) {
     nor_tx_done_at = DWT->CYCCNT;
     nor_async_state = 2U;
     QspiNor_ProgramEvent();
   }
 }
 void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *handle)
 {
   if (handle == &hqspi && nor_async_state == 3U) {
     nor_match_at = DWT->CYCCNT;
     nor_async_state = 4U;
     QspiNor_ProgramEvent();
   }
 }
 void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *handle)
 {
   if (handle == &hqspi && nor_async_state != 0U) {
     nor_async_state = 5U;
     QspiNor_ProgramEvent();
   }
 }
 void QspiNor_ProgramAbort(void)
 {
   if (nor_async_state != 0U) {
     (void)HAL_QSPI_Abort(&hqspi);
     /* Abort stops the controller, not the NOR's internal programming. */
     (void)QspiNor_WaitReady(NOR_TIMEOUT_MS);
     nor_async_state = 0U;
   }
 }
 HAL_StatusTypeDef QspiNor_ProgramPageAsync(uint32_t address, const uint8_t *data, uint16_t length)
 {
   QSPI_CommandTypeDef command;
   if (nor_async_state != 0U) return HAL_BUSY;
   if (data == NULL || length == 0U || length > QSPI_NOR_PAGE_SIZE ||
       address >= nor_capacity_bytes || length > nor_capacity_bytes - address ||
       length > QSPI_NOR_PAGE_SIZE - address % QSPI_NOR_PAGE_SIZE || hqspi.hdma == NULL)
     return HAL_ERROR;
   nor_setup_at = DWT->CYCCNT;
   if (QspiNor_WriteEnable() != HAL_OK) return HAL_ERROR;
   QspiNor_CommandDefaults(&command);
   command.Instruction = nor_quad_program ? NOR_CMD_QUAD_PAGE_PROGRAM : NOR_CMD_PAGE_PROGRAM;
   command.AddressMode = nor_quad_program ? QSPI_ADDRESS_4_LINES : QSPI_ADDRESS_1_LINE;
   command.Address = address;
   command.DataMode = nor_quad_program ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
   command.NbData = length;
   if (HAL_QSPI_Command(&hqspi, &command, 10U) != HAL_OK) return HAL_ERROR;
   nor_async_started = HAL_GetTick();
   nor_tx_at = DWT->CYCCNT;
   nor_async_state = 1U;
   if (HAL_QSPI_Transmit_DMA(&hqspi, (uint8_t *)data) != HAL_OK) {
     nor_async_state = 5U;
     QspiNor_ProgramAbort();
     return HAL_ERROR;
   }
   return HAL_OK;
 }
 HAL_StatusTypeDef QspiNor_ProgramService(void)
 {
   if (nor_async_state == 2U) {
     QSPI_CommandTypeDef command;
     QSPI_AutoPollingTypeDef polling;
     QspiNor_CommandDefaults(&command);
     command.Instruction = NOR_CMD_READ_STATUS;
     command.DataMode = QSPI_DATA_1_LINE;
     command.NbData = 1U;
     memset(&polling, 0, sizeof(polling));
     polling.Mask = NOR_STATUS_BUSY;
     polling.MatchMode = QSPI_MATCH_MODE_AND;
     polling.StatusBytesSize = 1U;
     polling.Interval = 0x10U;
     polling.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
     nor_poll_at = DWT->CYCCNT;
     nor_async_state = 3U;
     if (HAL_QSPI_AutoPolling_IT(&hqspi, &command, &polling) != HAL_OK)
       nor_async_state = 5U;
   }
   if (nor_async_state == 4U) {
     if (nor_timing.clock_hz != 0U) {
       uint32_t gap = nor_setup_at - nor_prev_match_at;
       if (nor_timing.pages != 0U && gap > nor_timing.max_gap_cycles)
         nor_timing.max_gap_cycles = gap;
       nor_timing.pages++;
       nor_timing.setup_cycles += nor_tx_at - nor_setup_at;
       nor_timing.tx_cycles += nor_tx_done_at - nor_tx_at;
       nor_timing.poll_start_delay_cycles += nor_poll_at - nor_tx_done_at;
       nor_timing.poll_cycles += nor_match_at - nor_poll_at;
       nor_timing.completion_delay_cycles += DWT->CYCCNT - nor_match_at;
       nor_prev_match_at = nor_match_at;
     }
     nor_async_state = 0U;
     return HAL_OK;
   }
   if (nor_async_state == 5U ||
       (nor_async_state != 0U && HAL_GetTick() - nor_async_started > 50U)) {
     nor_last_error_stage = "async-page-timeout/error";
     QspiNor_ProgramAbort();
     return HAL_ERROR;
   }
   return nor_async_state == 0U ? HAL_OK : HAL_BUSY;
 }
