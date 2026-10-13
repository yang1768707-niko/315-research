#include "digital_microphone.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DIGITAL_MIC_SAMPLE_RATE_HZ       46875U
#define DIGITAL_MIC_DEFAULT_WINDOW_MS       500U
#define DIGITAL_MIC_DMA_STEREO_FRAMES    2048U
#define DIGITAL_MIC_DMA_WORDS            (DIGITAL_MIC_DMA_STEREO_FRAMES * 2U)
#define DIGITAL_MIC_DMA_HALF_WORDS       (DIGITAL_MIC_DMA_WORDS / 2U)
#define DIGITAL_MIC_SLOT_INDEX           0U
#define DIGITAL_MIC_WARMUP_MS            25U
#define DIGITAL_MIC_DMA_HALF_FRAMES      (DIGITAL_MIC_DMA_STEREO_FRAMES / 2U)
#define DIGITAL_MIC_WARMUP_BLOCKS        ((DIGITAL_MIC_SAMPLE_RATE_HZ * DIGITAL_MIC_WARMUP_MS / 1000U + DIGITAL_MIC_DMA_HALF_FRAMES - 1U) / DIGITAL_MIC_DMA_HALF_FRAMES)
#define DIGITAL_MIC_FULL_SCALE_SINE_RMS  5931641.0f
#define DIGITAL_MIC_SENSITIVITY_DBFS     (-26.0f)
#define DIGITAL_MIC_REFERENCE_SPL_DB     94.0f
#define DIGITAL_MIC_MIN_RMS_COUNTS       1.0f
#define DIGITAL_MIC_BLOCK_HALF_READY     0x01U
#define DIGITAL_MIC_BLOCK_FULL_READY     0x02U

static SAI_HandleTypeDef *s_sai;
static uint32_t s_dma_buffer[DIGITAL_MIC_DMA_WORDS];
static volatile uint8_t s_block_flags;
static volatile uint8_t s_running;
static uint16_t s_warmup_blocks;
static uint64_t s_sum_squares;
static uint32_t s_sample_count;
static uint32_t s_dropped_blocks;
static uint32_t s_window_samples;
static uint8_t s_last_db_spl;
static uint8_t s_value_valid;

static int32_t DigitalMicrophone_ExtractI2s24(uint32_t word)
{
  return ((int32_t)(word << 8U)) >> 8U;
}

static uint8_t DigitalMicrophone_ClampDb(float value)
{
  int32_t rounded = (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));

  if (rounded < 0)
  {
    rounded = 0;
  }
  else if (rounded > 255)
  {
    rounded = 255;
  }
  return (uint8_t)rounded;
}

static void DigitalMicrophone_ResetWindow(void)
{
  s_sum_squares = 0U;
  s_sample_count = 0U;
}

static void DigitalMicrophone_FinishWindow(void)
{
  float mean_square;
  float rms_counts;
  float dbfs;
  float db_spl;

  if (s_sample_count == 0U)
  {
    return;
  }

  mean_square = (float)((double)s_sum_squares / (double)s_sample_count);
  rms_counts = sqrtf(mean_square);
  if (rms_counts < DIGITAL_MIC_MIN_RMS_COUNTS)
  {
    rms_counts = DIGITAL_MIC_MIN_RMS_COUNTS;
  }

  dbfs = 20.0f * log10f(rms_counts / DIGITAL_MIC_FULL_SCALE_SINE_RMS);
  db_spl = DIGITAL_MIC_REFERENCE_SPL_DB + dbfs - DIGITAL_MIC_SENSITIVITY_DBFS;
  s_last_db_spl = DigitalMicrophone_ClampDb(db_spl);
  s_value_valid = 1U;
  DigitalMicrophone_ResetWindow();
}

static void DigitalMicrophone_ProcessBlock(uint32_t offset, uint32_t word_count)
{
  uint32_t end = offset + word_count;

  if (s_warmup_blocks > 0U)
  {
    s_warmup_blocks--;
    DigitalMicrophone_ResetWindow();
    return;
  }

  for (uint32_t i = offset + DIGITAL_MIC_SLOT_INDEX; i < end; i += 2U)
  {
    int64_t sample = (int64_t)DigitalMicrophone_ExtractI2s24(s_dma_buffer[i]);

    s_sum_squares += (uint64_t)(sample * sample);
    s_sample_count++;
    if (s_sample_count >= s_window_samples)
    {
      DigitalMicrophone_FinishWindow();
    }
  }
}

HAL_StatusTypeDef DigitalMicrophone_Init(SAI_HandleTypeDef *hsai)
{
  s_sai = hsai;
  s_block_flags = 0U;
  s_running = 0U;
  s_dropped_blocks = 0U;
  s_value_valid = 0U;
  s_window_samples = (DIGITAL_MIC_SAMPLE_RATE_HZ * DIGITAL_MIC_DEFAULT_WINDOW_MS) / 1000U;
  DigitalMicrophone_ResetWindow();
  return (hsai != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef DigitalMicrophone_ConfigureWindow(uint16_t window_ms)
{
  if ((window_ms != 500U) && (window_ms != 1000U) &&
      (window_ms != 1500U) && (window_ms != 2000U))
  {
    return HAL_ERROR;
  }
  if (s_running != 0U)
  {
    return HAL_BUSY;
  }
  s_window_samples = ((uint32_t)DIGITAL_MIC_SAMPLE_RATE_HZ * window_ms) / 1000U;
  return HAL_OK;
}

HAL_StatusTypeDef DigitalMicrophone_Start(void)
{
  HAL_StatusTypeDef status;

  if (s_sai == NULL)
  {
    return HAL_ERROR;
  }
  if (s_running != 0U)
  {
    return HAL_OK;
  }

  memset(s_dma_buffer, 0, sizeof(s_dma_buffer));
  s_block_flags = 0U;
  s_warmup_blocks = DIGITAL_MIC_WARMUP_BLOCKS;
  s_value_valid = 0U;
  DigitalMicrophone_ResetWindow();
  s_running = 1U;
  status = HAL_SAI_Receive_DMA(s_sai, (uint8_t *)s_dma_buffer,
                               (uint16_t)DIGITAL_MIC_DMA_WORDS);
  if (status != HAL_OK)
  {
    s_running = 0U;
  }
  return status;
}

void DigitalMicrophone_Process(void)
{
  uint32_t primask;
  uint8_t flags;

  primask = __get_PRIMASK();
  __disable_irq();
  flags = s_block_flags;
  s_block_flags = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if ((flags & DIGITAL_MIC_BLOCK_HALF_READY) != 0U)
  {
    DigitalMicrophone_ProcessBlock(0U, DIGITAL_MIC_DMA_HALF_WORDS);
  }
  if ((flags & DIGITAL_MIC_BLOCK_FULL_READY) != 0U)
  {
    DigitalMicrophone_ProcessBlock(DIGITAL_MIC_DMA_HALF_WORDS,
                                   DIGITAL_MIC_DMA_HALF_WORDS);
  }
}

void DigitalMicrophone_Stop(void)
{
  s_running = 0U;
  if (s_sai != NULL)
  {
    (void)HAL_SAI_DMAStop(s_sai);
  }
  s_block_flags = 0U;
}

uint16_t DigitalMicrophone_AppendPayload(uint8_t *buffer, uint16_t buffer_size)
{
  if ((buffer == NULL) || (buffer_size < DIGITAL_MICROPHONE_PAYLOAD_SIZE) ||
      (s_value_valid == 0U))
  {
    return 0U;
  }

  buffer[0] = s_last_db_spl;
  return DIGITAL_MICROPHONE_PAYLOAD_SIZE;
}

uint32_t DigitalMicrophone_GetDroppedBlocks(void)
{
  return s_dropped_blocks;
}

uint8_t DigitalMicrophone_IsRunning(void)
{
  return s_running;
}

uint8_t DigitalMicrophone_HasValue(void)
{
  return s_value_valid;
}

uint8_t DigitalMicrophone_GetLastValue(void)
{
  return s_last_db_spl;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if ((s_sai != NULL) && (hsai->Instance == s_sai->Instance) && (s_running != 0U))
  {
    if ((s_block_flags & DIGITAL_MIC_BLOCK_HALF_READY) != 0U)
    {
      s_dropped_blocks++;
    }
    s_block_flags |= DIGITAL_MIC_BLOCK_HALF_READY;
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if ((s_sai != NULL) && (hsai->Instance == s_sai->Instance) && (s_running != 0U))
  {
    if ((s_block_flags & DIGITAL_MIC_BLOCK_FULL_READY) != 0U)
    {
      s_dropped_blocks++;
    }
    s_block_flags |= DIGITAL_MIC_BLOCK_FULL_READY;
  }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  if ((s_sai != NULL) && (hsai->Instance == s_sai->Instance))
  {
    s_running = 0U;
    s_dropped_blocks++;
  }
}
