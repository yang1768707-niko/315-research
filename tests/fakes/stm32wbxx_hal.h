#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t ClockPrescaler, FifoThreshold; } QSPI_InitTypeDef;
typedef struct { QSPI_InitTypeDef Init; void *hdma; } QSPI_HandleTypeDef;
typedef struct {
  uint32_t InstructionMode, AddressMode, AddressSize, AlternateByteMode;
  uint32_t DataMode, DummyCycles, DdrMode, SIOOMode, Instruction, NbData, Address;
} QSPI_CommandTypeDef;
typedef struct { uint32_t Match, Mask, MatchMode, StatusBytesSize, Interval, AutomaticStop; } QSPI_AutoPollingTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef unsigned GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef *)1)
#define GPIOB ((GPIO_TypeDef *)2)
#define GPIO_PIN_2 4U
#define GPIO_PIN_3 8U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_8 256U
#define GPIO_PIN_9 512U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_RESET 0U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_HIGH 3U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_AF10_QUADSPI 10U
#define GPIO_MODE_AF_PP 2U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_ANALOG 3U
#define QSPI_INSTRUCTION_1_LINE 1U
#define QSPI_ADDRESS_NONE 0U
#define QSPI_ADDRESS_1_LINE 1U
#define QSPI_ADDRESS_4_LINES 4U
#define QSPI_ADDRESS_24_BITS 24U
#define QSPI_ALTERNATE_BYTES_NONE 0U
#define QSPI_DATA_NONE 0U
#define QSPI_DATA_1_LINE 1U
#define QSPI_DATA_4_LINES 4U
#define QSPI_DDR_MODE_DISABLE 0U
#define QSPI_SIOO_INST_EVERY_CMD 0U
#define QSPI_MATCH_MODE_AND 0U
#define QSPI_AUTOMATIC_STOP_ENABLE 1U
#define HAL_QSPI_STATE_READY 0U
#define HAL_QSPI_STATE_ERROR 1U
#define QSPI_FLAG_BUSY 1U
#define RCC_AHB3RSTR_QUADSPIRST 1U
#define __HAL_QSPI_GET_FLAG(h,f) (0U)
#define SET_BIT(r,b) ((r)|=(b))
#define CLEAR_BIT(r,b) ((r)&=~(b))
#define READ_BIT(r,b) ((r)&(b))
#define __DSB() ((void)0)
#define __NOP() ((void)0)
#define __weak __attribute__((weak))
typedef struct { uint32_t AHB3RSTR; } TestRcc;
extern TestRcc test_rcc;
#define RCC (&test_rcc)
typedef struct { uint32_t CTRL, CYCCNT; } TestDwt;
typedef struct { uint32_t DEMCR; } TestCoreDebug;
extern TestDwt test_dwt;
extern TestCoreDebug test_debug;
extern uint32_t SystemCoreClock;
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define DWT_CTRL_NOCYCCNT_Msk (1U<<25)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define CoreDebug_DEMCR_TRCENA_Msk (1U<<24)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *,uint32_t,uint32_t);
void HAL_GPIO_Init(GPIO_TypeDef *,const GPIO_InitTypeDef *);
HAL_StatusTypeDef HAL_QSPI_Init(QSPI_HandleTypeDef *);
unsigned HAL_QSPI_GetState(QSPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_QSPI_Command(QSPI_HandleTypeDef *,QSPI_CommandTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_QSPI_Receive(QSPI_HandleTypeDef *,uint8_t *,uint32_t);
HAL_StatusTypeDef HAL_QSPI_Transmit(QSPI_HandleTypeDef *,uint8_t *,uint32_t);
HAL_StatusTypeDef HAL_QSPI_AutoPolling(QSPI_HandleTypeDef *,QSPI_CommandTypeDef *,QSPI_AutoPollingTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_QSPI_Transmit_DMA(QSPI_HandleTypeDef *,uint8_t *);
HAL_StatusTypeDef HAL_QSPI_AutoPolling_IT(QSPI_HandleTypeDef *,QSPI_CommandTypeDef *,QSPI_AutoPollingTypeDef *);
HAL_StatusTypeDef HAL_QSPI_Abort(QSPI_HandleTypeDef *);
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *);
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *);
void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *);
#endif
