#ifndef ZIGBEE_VIBRATION_STORE_H
#define ZIGBEE_VIBRATION_STORE_H

#include "stm32wbxx_hal.h"
#include "vibration_wire.h"
#include <stdint.h>

#define ZIGBEE_VIBRATION_MAX_SAMPLES 40001U

typedef struct
{
  uint32_t bytes;
  uint32_t pages;
  uint32_t partial_pages;
  uint32_t write_ms;
} ZigbeeVibrationWriteStats_t;

HAL_StatusTypeDef ZigbeeVibrationStore_Init(uint32_t *jedec_id,
                                            uint32_t *capacity_bytes);
HAL_StatusTypeDef ZigbeeVibrationStore_Begin(uint16_t sample_rate_hz,
                                             uint16_t window_ms,
                                             uint32_t batch_id);
HAL_StatusTypeDef ZigbeeVibrationStore_Append(const int16_t samples[][3],
                                              uint16_t sample_count);
HAL_StatusTypeDef ZigbeeVibrationStore_Finish(
    const int32_t features_milli[3][5]);
void ZigbeeVibrationStore_Abort(void);
HAL_StatusTypeDef ZigbeeVibrationStore_Service(uint8_t final_flush);
uint16_t ZigbeeVibrationStore_AsyncCapacity(void);
HAL_StatusTypeDef ZigbeeVibrationStore_AppendAsync(const int16_t samples[][3], uint16_t count);
uint32_t ZigbeeVibrationStore_SampleCount(void);
void ZigbeeVibrationStore_GetWriteStats(ZigbeeVibrationWriteStats_t *stats);
uint8_t ZigbeeVibrationStore_IsReady(void);

int ZigbeeVibrationStore_Pending(uint8_t descriptor[V2_BATCH_LEN]);
int ZigbeeVibrationStore_DescribeBatch(uint32_t batch, uint8_t descriptor[V2_BATCH_LEN]);
int ZigbeeVibrationStore_SetRawPolicy(uint32_t batch, uint8_t upload);
int ZigbeeVibrationStore_RawPolicy(uint32_t batch);
int ZigbeeVibrationStore_SaveFeatures(uint32_t batch, const int32_t values[3][5]);
int ZigbeeVibrationStore_ReadBatch(uint32_t batch_id, uint32_t first_sample,
                                   unsigned sample_count, uint8_t *raw);
int ZigbeeVibrationStore_Acknowledge(uint32_t batch_id);
int ZigbeeVibrationStore_DiscardPending(void);
/* Persistently invalidate every slot without erasing waveform data. */
HAL_StatusTypeDef ZigbeeVibrationStore_ResetRecords(void);
unsigned ZigbeeVibrationStore_FreeSlots(void);
#define ZIGBEE_BATCH_METADATA_MAX 192U
int ZigbeeVibrationStore_SaveMetadata(uint32_t batch, const void *data, uint16_t length);
/* 1 valid record, 0 old batch without metadata, -1 read/CRC/format error. */
int ZigbeeVibrationStore_LoadMetadata(uint32_t batch, void *data, uint16_t length);

#endif /* ZIGBEE_VIBRATION_STORE_H */
