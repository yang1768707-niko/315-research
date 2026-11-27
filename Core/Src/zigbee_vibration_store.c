/* Optimize page assembly and inline CRC even in the Debug configuration.
   Storage format, CRC polynomial and commit ordering are unchanged. */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O2")
#endif
#include "zigbee_vibration_store.h"

#include "qspi_nor.h"

#include <stddef.h>
#include <string.h>

#define ZB_VIB_FLASH_MAGIC          0x56494232UL
#define ZB_VIB_SLOT_BYTES           (60U * QSPI_NOR_SECTOR_SIZE)
#define ZB_VIB_MAX_SLOTS            16U
#define ZB_VIB_HEADER_OFFSET        128U
#define ZB_VIB_DATA_OFFSET          QSPI_NOR_SECTOR_SIZE
#define ZB_VIB_FEATURE_OFFSET       512U
#define ZB_VIB_POLICY_OFFSET        1024U
#define ZB_VIB_FEATURE_MAGIC        0x46545232UL
#define ZB_VIB_METADATA_OFFSET      1280U
#define ZB_VIB_METADATA_MAGIC       0x534e5031UL
typedef struct {
  uint32_t magic, length, crc;
  uint8_t payload[ZIGBEE_BATCH_METADATA_MAX];
} ZigbeeBatchMetadata;

typedef struct {
  uint32_t magic;
  int32_t values[3][5];
  uint32_t crc;
} ZigbeeVibrationFeatureRecord_t;

typedef struct
{
  uint32_t magic;
  uint32_t collection_id;
  uint32_t sample_rate_hz;
  uint32_t sample_count;
  uint32_t crc32;
  uint32_t sequence;
  int32_t features[3][5];
  uint32_t header_crc;
  uint32_t acknowledged;
} ZigbeeVibrationFlashHeader_t;

static ZigbeeVibrationFlashHeader_t s_headers[ZB_VIB_MAX_SLOTS];
/* 0=recyclable, 1=pending, 2=acknowledged, 3=quarantined, 4=capturing. */
static uint8_t s_states[ZB_VIB_MAX_SLOTS];
static uint32_t s_upload_batch;
static ZigbeeVibrationOverwrite_t s_overwrite;
static unsigned s_slot_count;
static unsigned s_active_slot;
static unsigned s_next_slot;
static uint32_t s_next_sequence;
static uint32_t s_sample_target;
static uint32_t s_sample_count;
static uint32_t s_write_address;
static uint32_t s_running_crc;
static uint32_t s_batch_id;
static uint16_t s_sample_rate_hz;
static uint8_t s_ready;
static uint8_t s_capturing;
static uint8_t s_pages[2][QSPI_NOR_PAGE_SIZE];
static uint8_t s_fill_page;
static uint16_t s_page_used;
static ZigbeeVibrationWriteStats_t s_write_stats;
static uint16_t s_dma_length;
static uint32_t s_dma_started;

static HAL_StatusTypeDef Store_StartPage(void)
{
  if (s_dma_length != 0U) return HAL_BUSY;
  if (s_page_used == 0U) return HAL_OK;
  if (QspiNor_ProgramPageAsync(s_write_address, s_pages[s_fill_page], s_page_used) != HAL_OK)
    return HAL_ERROR;
  s_dma_length = s_page_used;
  s_dma_started = HAL_GetTick();
  /* Keep the page handed to DMA immutable until ProgramService confirms it. */
  s_fill_page ^= 1U;
  s_page_used = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef ZigbeeVibrationStore_Service(uint8_t final_flush)
{
  if (s_dma_length != 0U) {
    HAL_StatusTypeDef status = QspiNor_ProgramService();
    if (status == HAL_ERROR) return HAL_ERROR;
    if (status == HAL_BUSY) return HAL_BUSY;
    s_write_stats.bytes += s_dma_length;
    s_write_stats.pages++;
    s_write_stats.partial_pages += (s_dma_length != QSPI_NOR_PAGE_SIZE);
    s_write_stats.write_ms += HAL_GetTick() - s_dma_started;
    s_write_address += s_dma_length;
    s_dma_length = 0U;
  }
  if (s_page_used == QSPI_NOR_PAGE_SIZE || (final_flush && s_page_used != 0U)) {
    if (Store_StartPage() != HAL_OK) return HAL_ERROR;
    return HAL_BUSY;
  }
  return HAL_OK;
}

uint16_t ZigbeeVibrationStore_AsyncCapacity(void)
{
  /* At most one page can be submitted per append. Preflight prevents partial
   * consumption on BUSY, including samples straddling a 256-byte boundary. */
  uint32_t bytes = QSPI_NOR_PAGE_SIZE - s_page_used;
  if (s_dma_length == 0U) bytes += QSPI_NOR_PAGE_SIZE;
  uint16_t points = (uint16_t)(bytes / 6U);
  return points > 42U ? 42U : points;
}

HAL_StatusTypeDef ZigbeeVibrationStore_AppendAsync(const int16_t samples[][3], uint16_t count)
{
  if (!s_capturing || samples == NULL || count == 0U || count > s_sample_target - s_sample_count)
    return HAL_ERROR;
  if (count > ZigbeeVibrationStore_AsyncCapacity()) return HAL_BUSY;
  const uint8_t *raw = (const uint8_t *)samples;
  uint32_t bytes = (uint32_t)count * 6U;
  for (uint32_t offset = 0U; offset < bytes;) {
    uint32_t take = QSPI_NOR_PAGE_SIZE - s_page_used;
    if (take > bytes - offset) take = bytes - offset;
    memcpy(s_pages[s_fill_page] + s_page_used, raw + offset, take);
    s_page_used += take;
    offset += take;
    if (s_page_used == QSPI_NOR_PAGE_SIZE && s_dma_length == 0U && Store_StartPage() != HAL_OK)
      return HAL_ERROR;
  }
  s_running_crc = v2_crc_update(s_running_crc, raw, bytes);
  s_sample_count += count;
  return HAL_OK;
}
static HAL_StatusTypeDef ZigbeeVibrationStore_FlushPage(void)
{
  uint32_t started;

  if (s_page_used == 0U)
  {
    return HAL_OK;
  }
  started = HAL_GetTick();
  if (QspiNor_Write(s_write_address, s_pages[s_fill_page], s_page_used) != HAL_OK)
  {
    s_capturing = 0U;
    s_states[s_active_slot] = 0U;
    return HAL_ERROR;
  }
  s_write_stats.write_ms += HAL_GetTick() - started;
  s_write_stats.bytes += s_page_used;
  s_write_stats.pages++;
  if (s_page_used != QSPI_NOR_PAGE_SIZE)
  {
    s_write_stats.partial_pages++;
  }
  s_write_address += s_page_used;
  s_page_used = 0U;
  return HAL_OK;
}

static uint32_t ZigbeeVibrationStore_HeaderCrc(
    const ZigbeeVibrationFlashHeader_t *header)
{
  return v2_crc((const uint8_t *)&header->collection_id,
                offsetof(ZigbeeVibrationFlashHeader_t, header_crc) -
                sizeof(header->magic));
}

static int ZigbeeVibrationStore_Commit(unsigned slot,
                                       ZigbeeVibrationFlashHeader_t *header)
{
  uint32_t address = slot * ZB_VIB_SLOT_BYTES + ZB_VIB_HEADER_OFFSET;

  header->magic = UINT32_MAX;
  header->header_crc = ZigbeeVibrationStore_HeaderCrc(header);
  header->acknowledged = UINT32_MAX;
  if (QspiNor_Write(address, (const uint8_t *)header,
                    sizeof(*header)) != HAL_OK)
  {
    return 0;
  }
  header->magic = ZB_VIB_FLASH_MAGIC;
  if (QspiNor_Write(address, (const uint8_t *)&header->magic,
                    sizeof(header->magic)) != HAL_OK)
  {
    return 0;
  }
  s_headers[slot] = *header;
  s_states[slot] = 1U;
  return 1;
}

HAL_StatusTypeDef ZigbeeVibrationStore_Init(uint32_t *jedec_id,
                                            uint32_t *capacity_bytes)
{
  uint32_t capacity = 0U;

  s_ready = 0U;
  s_capturing = 0U;
  s_upload_batch = 0U;
  memset(&s_overwrite, 0, sizeof(s_overwrite));
  if (QspiNor_Init(jedec_id, &capacity) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (capacity_bytes != NULL)
  {
    *capacity_bytes = capacity;
  }
  s_slot_count = capacity / ZB_VIB_SLOT_BYTES;
  if (s_slot_count > ZB_VIB_MAX_SLOTS)
  {
    s_slot_count = ZB_VIB_MAX_SLOTS;
  }
  if (s_slot_count == 0U)
  {
    return HAL_ERROR;
  }

  s_next_slot = 0U;
  s_next_sequence = 1U;
  for (unsigned slot = 0U; slot < s_slot_count; slot++)
  {
    ZigbeeVibrationFlashHeader_t *header = &s_headers[slot];
    uint32_t address = slot * ZB_VIB_SLOT_BYTES + ZB_VIB_HEADER_OFFSET;

    if (QspiNor_Read(address, (uint8_t *)header, sizeof(*header)) != HAL_OK)
    {
      return HAL_ERROR;
    }
    s_states[slot] = 0U;
    if (header->magic != ZB_VIB_FLASH_MAGIC)
    {
      continue;
    }
    if ((header->header_crc != ZigbeeVibrationStore_HeaderCrc(header)) ||
        (header->sample_count == 0U) ||
        (header->sample_count > ZIGBEE_VIBRATION_MAX_SAMPLES))
    {
      s_states[slot] = 3U;
      continue;
    }
    s_states[slot] = (header->acknowledged == 0U) ? 2U : 1U;
    if ((int32_t)(header->sequence - s_next_sequence) >= 0)
    {
      s_next_sequence = header->sequence + 1U;
    }
  }
  s_ready = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef ZigbeeVibrationStore_Begin(uint16_t sample_rate_hz,
                                             uint16_t window_ms,
                                             uint32_t batch_id)
{
  unsigned selected = s_slot_count;
  unsigned acknowledged = s_slot_count;
  unsigned oldest = s_slot_count;
  uint32_t expected_samples;
  uint32_t expected_bytes;

  if ((s_ready == 0U) || (s_capturing != 0U) ||
      (sample_rate_hz != 26667U) ||
      ((window_ms != 500U) && (window_ms != 1000U) &&
       (window_ms != 1500U)) || (batch_id == 0U))
  {
    return HAL_ERROR;
  }
  expected_samples = ((uint32_t)sample_rate_hz * window_ms + 500U) / 1000U;
  expected_bytes = expected_samples * 6U;
  if ((expected_samples == 0U) ||
      (expected_samples > ZIGBEE_VIBRATION_MAX_SAMPLES) ||
      (expected_bytes > ZB_VIB_SLOT_BYTES - ZB_VIB_DATA_OFFSET))
  {
    return HAL_ERROR;
  }
  for (unsigned index = 0U; index < s_slot_count; index++)
  {
    unsigned slot = (s_next_slot + index) % s_slot_count;
    if ((s_states[slot] == 1U) &&
        (s_headers[slot].collection_id == batch_id))
    {
      return HAL_ERROR;
    }
    if ((selected == s_slot_count) && (s_states[slot] == 0U)) selected = slot;
    if ((acknowledged == s_slot_count) && (s_states[slot] == 2U)) acknowledged = slot;
    if ((s_states[slot] == 1U) &&
        (s_headers[slot].collection_id != s_upload_batch) &&
        ((oldest == s_slot_count) ||
         ((int32_t)(s_headers[slot].sequence - s_headers[oldest].sequence) < 0)))
      oldest = slot;
  }
  if (selected == s_slot_count) selected = acknowledged;
  if (selected == s_slot_count) selected = oldest;
  if (selected == s_slot_count)
  {
    return HAL_BUSY;
  }

  if (s_states[selected] == 1U)
  {
    /* Commit the loss before any erase can partly alter waveform bytes. On
       power loss Init must never resurrect a header for a partly erased batch. */
    uint32_t invalid = 0U, check = UINT32_MAX;
    uint32_t header_address = selected * ZB_VIB_SLOT_BYTES + ZB_VIB_HEADER_OFFSET;
    if (QspiNor_Write(header_address, (const uint8_t *)&invalid, sizeof(invalid)) != HAL_OK ||
        QspiNor_Read(header_address, (uint8_t *)&check, sizeof(check)) != HAL_OK || check != invalid)
      return HAL_ERROR;
    s_overwrite.discarded_batch = s_headers[selected].collection_id;
    s_overwrite.replacement_batch = batch_id;
    s_overwrite.slot = selected;
    s_states[selected] = 0U;
  }

  s_active_slot = selected;
  s_next_slot = (selected + 1U) % s_slot_count;
  s_states[selected] = 4U;
  s_sample_target = expected_samples;
  s_sample_count = 0U;
  s_batch_id = batch_id;
  s_sample_rate_hz = sample_rate_hz;
  s_running_crc = UINT32_MAX;
  s_write_address = selected * ZB_VIB_SLOT_BYTES + ZB_VIB_DATA_OFFSET;
  s_fill_page = 0U;
  s_page_used = 0U;
  memset(&s_write_stats, 0, sizeof(s_write_stats));
  uint32_t erase_started = HAL_GetTick();
  if (QspiNor_EraseRange(selected * ZB_VIB_SLOT_BYTES,
                         ZB_VIB_DATA_OFFSET + expected_bytes) != HAL_OK)
  {
    s_states[selected] = 0U;
    return HAL_ERROR;
  }
  s_write_stats.erase_ms = HAL_GetTick() - erase_started;
  QspiNor_ResetProgramTiming();
  s_capturing = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef ZigbeeVibrationStore_Append(const int16_t samples[][3],
                                              uint16_t sample_count)
{
  if (s_dma_length != 0U) return HAL_BUSY;
  if ((s_capturing == 0U) || (samples == NULL))
  {
    return HAL_ERROR;
  }
  while ((sample_count != 0U) && (s_sample_count < s_sample_target))
  {
    const uint8_t *raw = (const uint8_t *)samples;
    uint16_t chunk = sample_count;
    uint32_t remaining = s_sample_target - s_sample_count;
    if (chunk > remaining)
    {
      chunk = (uint16_t)remaining;
    }
    s_running_crc = v2_crc_update(s_running_crc, raw, (size_t)chunk * 6U);
    for (uint32_t offset = 0U; offset < (uint32_t)chunk * 6U;)
    {
      uint32_t take = QSPI_NOR_PAGE_SIZE - s_page_used;
      uint32_t left = (uint32_t)chunk * 6U - offset;
      if (take > left)
      {
        take = left;
      }
      memcpy(&s_pages[s_fill_page][s_page_used], &raw[offset], take);
      s_page_used += (uint16_t)take;
      offset += take;
      if ((s_page_used == QSPI_NOR_PAGE_SIZE) &&
          (ZigbeeVibrationStore_FlushPage() != HAL_OK))
      {
        return HAL_ERROR;
      }
    }
    s_sample_count += chunk;
    samples += chunk;
    sample_count -= chunk;
  }
  return HAL_OK;
}

HAL_StatusTypeDef ZigbeeVibrationStore_Finish(
    const int32_t features_milli[3][5])
{
  if (s_dma_length != 0U) return HAL_BUSY;
  ZigbeeVibrationFlashHeader_t header;

  if ((s_capturing == 0U) || (s_sample_count == 0U) ||
      (s_sample_count != s_sample_target))
  {
    return HAL_ERROR;
  }
  if (ZigbeeVibrationStore_FlushPage() != HAL_OK)
  {
    return HAL_ERROR;
  }
  memset(&header, 0, sizeof(header));
  header.collection_id = s_batch_id;
  header.sample_rate_hz = s_sample_rate_hz;
  header.sample_count = s_sample_count;
  header.crc32 = s_running_crc ^ UINT32_MAX;
  header.sequence = s_next_sequence++;
  if (features_milli != NULL)
    memcpy(header.features, features_milli, sizeof(header.features));
  else
    for (unsigned a=0U;a<3U;a++) for(unsigned f=0U;f<5U;f++)
      header.features[a][f]=INT32_MIN;
  s_capturing = 0U;
  if (!ZigbeeVibrationStore_Commit(s_active_slot, &header))
  {
    s_states[s_active_slot] = 0U;
    return HAL_ERROR;
  }
  return HAL_OK;
}

void ZigbeeVibrationStore_Abort(void)
{
  QspiNor_ProgramAbort();
  s_dma_length = 0U;
  if ((s_capturing != 0U) && (s_states[s_active_slot] == 4U))
  {
    s_states[s_active_slot] = 0U;
  }
  s_capturing = 0U;
  s_sample_count = 0U;
  s_page_used = 0U;
}

uint32_t ZigbeeVibrationStore_SampleCount(void)
{
  return s_sample_count;
}

void ZigbeeVibrationStore_GetWriteStats(ZigbeeVibrationWriteStats_t *stats)
{
  if (stats != NULL)
  {
    *stats = s_write_stats;
  }
}

uint8_t ZigbeeVibrationStore_IsReady(void)
{
  return s_ready;
}

int ZigbeeVibrationStore_DescribeBatch(uint32_t batch,
                                       uint8_t descriptor[V2_BATCH_LEN])
{
  unsigned oldest = s_slot_count;

  if (descriptor == NULL)
  {
    return 0;
  }
  for (unsigned slot = 0U; slot < s_slot_count; slot++)
  {
    if ((s_states[slot] == 1U) &&
        ((batch == 0U) || (s_headers[slot].collection_id == batch)) &&
        ((oldest == s_slot_count) ||
         ((int32_t)(s_headers[slot].sequence - s_headers[oldest].sequence) < 0)))
    {
      oldest = slot;
    }
  }
  if (oldest == s_slot_count)
  {
    return 0;
  }
  const ZigbeeVibrationFlashHeader_t *header = &s_headers[oldest];
  descriptor[0] = V2_VERSION;
  v2_w32(descriptor + 1, header->collection_id);
  v2_w16(descriptor + 5, (uint16_t)header->sample_rate_hz);
  v2_w32(descriptor + 7, header->sample_count);
  v2_w16(descriptor + 11, V2_POINTS);
  v2_w32(descriptor + 13, header->crc32);
  for (unsigned index = 0U; index < 15U; index++)
  {
    v2_w32(descriptor + 17U + index * 4U,
           (uint32_t)header->features[index / 5U][index % 5U]);
  }
  return 1;
}

int ZigbeeVibrationStore_Pending(uint8_t descriptor[V2_BATCH_LEN])
{
  return ZigbeeVibrationStore_DescribeBatch(0U, descriptor);
}

int ZigbeeVibrationStore_ProtectBatch(uint32_t batch)
{
  if (batch == 0U) { s_upload_batch = 0U; return 1; }
  if (!s_ready || (s_upload_batch && s_upload_batch != batch)) return 0;
  for (unsigned slot = 0U; slot < s_slot_count; ++slot)
    if (s_states[slot] == 1U && s_headers[slot].collection_id == batch) {
      s_upload_batch = batch;
      return 1;
    }
  return 0;
}

int ZigbeeVibrationStore_TakeOverwrite(ZigbeeVibrationOverwrite_t *event)
{
  if (!event || !s_overwrite.discarded_batch) return 0;
  *event = s_overwrite;
  memset(&s_overwrite, 0, sizeof(s_overwrite));
  return 1;
}

int ZigbeeVibrationStore_LoadMetadata(uint32_t batch,void *data,uint16_t length)
{
  if(!data || !length || length>ZIGBEE_BATCH_METADATA_MAX)return -1;
  for(unsigned slot=0;slot<s_slot_count;slot++) {
    if(s_states[slot]!=1U || s_headers[slot].collection_id!=batch)continue;
    ZigbeeBatchMetadata record;
    if(QspiNor_Read(slot*ZB_VIB_SLOT_BYTES+ZB_VIB_METADATA_OFFSET,(uint8_t *)&record,sizeof(record))!=HAL_OK)return -1;
    if(record.magic==UINT32_MAX)return 0;
    if(record.magic!=ZB_VIB_METADATA_MAGIC || record.length!=length ||
       record.crc!=v2_crc(record.payload,length))return -1;
    memcpy(data,record.payload,length);return 1;
  }
  return 0;
}

int ZigbeeVibrationStore_SaveMetadata(uint32_t batch,const void *data,uint16_t length)
{
  if(!data || !length || length>ZIGBEE_BATCH_METADATA_MAX || s_capturing)return 0;
  for(unsigned slot=0;slot<s_slot_count;slot++) {
    if(s_states[slot]!=1U || s_headers[slot].collection_id!=batch)continue;
    uint32_t address=slot*ZB_VIB_SLOT_BYTES+ZB_VIB_METADATA_OFFSET;
    ZigbeeBatchMetadata record;
    if(QspiNor_Read(address,(uint8_t *)&record,sizeof(record))!=HAL_OK)return 0;
    if(record.magic==ZB_VIB_METADATA_MAGIC)
      return record.length==length && record.crc==v2_crc(record.payload,length) &&
             !memcmp(record.payload,data,length);
    if(record.magic!=UINT32_MAX)return 0;
    record.magic=ZB_VIB_METADATA_MAGIC;record.length=length;
    memcpy(record.payload,data,length);record.crc=v2_crc(data,length);
    /* Write into previously unused header-sector bytes. Commit magic last;
     * existing waveform headers, policy and feature records stay unchanged. */
    if(QspiNor_Write(address+4,(uint8_t *)&record+4,8U+length)!=HAL_OK ||
       QspiNor_Write(address,(uint8_t *)&record.magic,4)!=HAL_OK)return 0;
    return 1;
  }
  return 0;
}

int ZigbeeVibrationStore_SetRawPolicy(uint32_t batch, uint8_t upload)
{
  for(unsigned slot=0U;slot<s_slot_count;slot++) {
    if(s_states[slot]!=1U || s_headers[slot].collection_id!=batch) continue;
    if(upload) return 1;
    uint32_t zero=0U;
    return QspiNor_Write(slot*ZB_VIB_SLOT_BYTES+ZB_VIB_POLICY_OFFSET,
                        (const uint8_t *)&zero,sizeof(zero))==HAL_OK;
  }
  return 0;
}

int ZigbeeVibrationStore_RawPolicy(uint32_t batch)
{
  for(unsigned slot=0U;slot<s_slot_count;slot++) {
    if(s_states[slot]!=1U || s_headers[slot].collection_id!=batch) continue;
    uint32_t policy;
    if(QspiNor_Read(slot*ZB_VIB_SLOT_BYTES+ZB_VIB_POLICY_OFFSET,
                   (uint8_t *)&policy,sizeof(policy))!=HAL_OK) return -1;
    return policy==0U ? 0 : 1;
  }
  return -1;
}

int ZigbeeVibrationStore_SaveFeatures(uint32_t batch, const int32_t values[3][5])
{
  for(unsigned slot=0U;slot<s_slot_count;slot++) {
    if(s_states[slot]!=1U || s_headers[slot].collection_id!=batch) continue;
    ZigbeeVibrationFeatureRecord_t record;
    uint32_t address=slot*ZB_VIB_SLOT_BYTES+ZB_VIB_FEATURE_OFFSET;
    if(QspiNor_Read(address,(uint8_t *)&record,sizeof(record))!=HAL_OK) return 0;
    if(record.magic==ZB_VIB_FEATURE_MAGIC &&
       record.crc==v2_crc((const uint8_t *)record.values,sizeof(record.values)))
      return memcmp(record.values,values,sizeof(record.values))==0;
    record.magic=UINT32_MAX;
    memcpy(record.values,values,sizeof(record.values));
    record.crc=v2_crc((const uint8_t *)record.values,sizeof(record.values));
    if(QspiNor_Write(address,(const uint8_t *)&record,sizeof(record))!=HAL_OK) return 0;
    record.magic=ZB_VIB_FEATURE_MAGIC;
    return QspiNor_Write(address,(const uint8_t *)&record.magic,sizeof(record.magic))==HAL_OK;
  }
  return 0;
}

int ZigbeeVibrationStore_ReadBatch(uint32_t batch_id, uint32_t first_sample,
                                   unsigned sample_count, uint8_t *raw)
{
  for (unsigned slot = 0U; slot < s_slot_count; slot++)
  {
    if ((s_states[slot] == 1U) &&
        (s_headers[slot].collection_id == batch_id))
    {
      uint32_t count = s_headers[slot].sample_count;
      if ((raw == NULL) || (sample_count == 0U) ||
          (first_sample >= count) || (sample_count > count - first_sample))
      {
        return 0;
      }
      return QspiNor_Read(slot * ZB_VIB_SLOT_BYTES + ZB_VIB_DATA_OFFSET +
                         first_sample * 6U, raw, sample_count * 6U) == HAL_OK;
    }
  }
  return 0;
}

int ZigbeeVibrationStore_Acknowledge(uint32_t batch_id)
{
  for (unsigned slot = 0U; slot < s_slot_count; slot++)
  {
    if ((s_states[slot] == 1U) &&
        (s_headers[slot].collection_id == batch_id))
    {
      uint32_t zero = 0U;
      uint32_t address = slot * ZB_VIB_SLOT_BYTES + ZB_VIB_HEADER_OFFSET +
                         offsetof(ZigbeeVibrationFlashHeader_t, acknowledged);
      if (QspiNor_Write(address, (const uint8_t *)&zero, sizeof(zero)) != HAL_OK)
      {
        return 0;
      }
      s_states[slot] = 2U;
      s_headers[slot].acknowledged = 0U;
      if (s_upload_batch == batch_id) s_upload_batch = 0U;
      return 1;
    }
  }
  return 0;
}

int ZigbeeVibrationStore_DiscardPending(void)
{
  for (unsigned slot = 0U; slot < s_slot_count; slot++)
  {
    if ((s_states[slot] == 1U) &&
        !ZigbeeVibrationStore_Acknowledge(s_headers[slot].collection_id))
    {
      return 0;
    }
  }
  return 1;
}

unsigned ZigbeeVibrationStore_FreeSlots(void)
{
  unsigned count = 0U;
  for (unsigned slot = 0U; slot < s_slot_count; slot++)
  {
    if ((s_states[slot] == 0U) || (s_states[slot] == 2U))
    {
      count++;
    }
  }
  return count;
}

HAL_StatusTypeDef ZigbeeVibrationStore_ResetRecords(void)
{
  const uint32_t invalid = 0U;
  uint32_t check;
  ZigbeeVibrationStore_Abort();
  /* Init discovers all physical slots, not only currently pending records.
   * Zeroing magic uses NOR's 1->0 programming; payload is erased lazily by
   * Begin. Repeatable after interrupted cleanup and also clears quarantine. */
  if (ZigbeeVibrationStore_Init(NULL, NULL) != HAL_OK) return HAL_ERROR;
  for (unsigned slot = 0U; slot < s_slot_count; ++slot)
  {
    uint32_t address = slot * ZB_VIB_SLOT_BYTES + ZB_VIB_HEADER_OFFSET;
    if (QspiNor_Write(address, (const uint8_t *)&invalid, sizeof(invalid)) != HAL_OK ||
        QspiNor_Read(address, (uint8_t *)&check, sizeof(check)) != HAL_OK || check != 0U)
    {
      s_ready = 0U;
      return HAL_ERROR;
    }
  }
  memset(s_headers, 0, sizeof(s_headers));
  memset(s_states, 0, sizeof(s_states));
  memset(s_pages, 0, sizeof(s_pages));
  memset(&s_write_stats, 0, sizeof(s_write_stats));
  s_active_slot = s_next_slot = 0U;
  s_next_sequence = 1U;
  s_batch_id = s_sample_target = s_write_address = s_running_crc = 0U;
  return ZigbeeVibrationStore_Init(NULL, NULL);
}
