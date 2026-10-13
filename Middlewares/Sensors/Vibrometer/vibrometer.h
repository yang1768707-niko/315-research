#ifndef VIBROMETER_H
#define VIBROMETER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VIBROMETER_WHO_AM_I_VALUE          0x7BU
#define VIBROMETER_FIFO_OVERRUN_FLAG       0x40U
/* Drain up to 160 seven-byte FIFO words per SPI transaction. The previous
 * 40-word limit repeated HAL/SPI setup four times per watermark and could
 * not consistently keep up with 26.667 kHz while CPU2 was active. */
#define VIBROMETER_FIFO_DMA_MAX_BYTES      1120U

HAL_StatusTypeDef Vibrometer_Init(SPI_HandleTypeDef *hspi, uint8_t *who_am_i);
uint8_t Vibrometer_IsReady(void);
HAL_StatusTypeDef Vibrometer_StartContinuousFifo(void);
HAL_StatusTypeDef Vibrometer_StopSamplingKeepFifo(void);
HAL_StatusTypeDef Vibrometer_Stop(void);
HAL_StatusTypeDef Vibrometer_ReadFifoStatus(uint16_t *sample_count,
                                            uint8_t *status_flags);
HAL_StatusTypeDef Vibrometer_ReadFifoData(uint8_t *data, uint16_t length);
HAL_StatusTypeDef Vibrometer_ReadFifoDataAsync(uint8_t *data, uint16_t length);
void Vibrometer_AbortFifoRead(void);
void Vibrometer_SpiTxRxCompleteCallback(SPI_HandleTypeDef *hspi);
void Vibrometer_SpiErrorCallback(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* VIBROMETER_H */
