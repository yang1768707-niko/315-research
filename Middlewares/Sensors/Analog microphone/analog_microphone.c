#include "analog_microphone.h"
#include "main.h"
#include "stm32wb55rgv6_bsp.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ANALOG_MIC_SAMPLE_RATE_HZ          256000U
#define ANALOG_MIC_CHANNELS_PER_FRAME      2U
#define ANALOG_MIC_MREF_INDEX              0U
#define ANALOG_MIC_SIGNAL_INDEX            1U
#define ANALOG_MIC_DMA_FRAMES              8192U
#define ANALOG_MIC_DMA_WORDS               (ANALOG_MIC_DMA_FRAMES * ANALOG_MIC_CHANNELS_PER_FRAME)
#define ANALOG_MIC_DMA_HALF_WORDS          (ANALOG_MIC_DMA_WORDS / 2U)
#define ANALOG_MIC_DMA_HALF_FRAMES         (ANALOG_MIC_DMA_FRAMES / 2U)
#define ANALOG_MIC_DEFAULT_WINDOW_MS       500U
#define ANALOG_MIC_WARMUP_MS               50U
#define ANALOG_MIC_WARMUP_BLOCKS           ((ANALOG_MIC_SAMPLE_RATE_HZ * ANALOG_MIC_WARMUP_MS / 1000U + ANALOG_MIC_DMA_HALF_FRAMES - 1U) / ANALOG_MIC_DMA_HALF_FRAMES)
#define ANALOG_MIC_REFERENCE_DB_SPL        94.0f
#define ANALOG_MIC_REFERENCE_RMS_COUNTS    348.0f
#define ANALOG_MIC_MIN_RMS_COUNTS          0.000001f
#define ANALOG_MIC_POWER_STABILIZE_MS      50U
#define ANALOG_MIC_BLOCK_HALF_READY        0x01U
#define ANALOG_MIC_BLOCK_FULL_READY        0x02U

static ADC_HandleTypeDef *s_adc;
static TIM_HandleTypeDef *s_timer;
static uint16_t s_dma_buffer[ANALOG_MIC_DMA_WORDS];
static volatile uint8_t s_block_flags;
static volatile uint8_t s_running;
static uint16_t s_warmup_blocks;
static uint32_t s_dropped_blocks;
static int64_t s_sample_sum;
static uint64_t s_square_sum;
static uint32_t s_energy_samples;
static uint32_t s_window_samples;
static uint8_t s_last_db_spl;
static uint8_t s_value_valid;

static uint8_t AnalogMicrophone_ClampDb(float value)
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

static void AnalogMicrophone_ResetSignalProcessing(void)
{
  s_sample_sum = 0;
  s_square_sum = 0U;
  s_energy_samples = 0U;
}

static void AnalogMicrophone_FinishWindow(void)
{
  double mean;
  double mean_square;
  double variance;
  float rms_counts;
  float safe_rms_counts;
  float db_spl;

  if (s_energy_samples == 0U)
  {
    return;
  }

  /*
   * The microphone signal and MREF channels share the same bias.  Accumulate
   * integer first and remove the remaining DC component once per window.
   * This replaces two floating-point biquads per 256 kHz ADC frame, which
   * previously monopolised CPU1 and starved BLE/IPCC and the vibration pipe.
   */
  mean = (double)s_sample_sum / (double)s_energy_samples;
  mean_square = (double)s_square_sum / (double)s_energy_samples;
  variance = mean_square - (mean * mean);
  if (variance < 0.0)
  {
    variance = 0.0;
  }
  rms_counts = sqrtf((float)variance);
  safe_rms_counts = (rms_counts > ANALOG_MIC_MIN_RMS_COUNTS) ?
                    rms_counts : ANALOG_MIC_MIN_RMS_COUNTS;
  db_spl = ANALOG_MIC_REFERENCE_DB_SPL +
           20.0f * log10f(safe_rms_counts / ANALOG_MIC_REFERENCE_RMS_COUNTS);
  s_last_db_spl = AnalogMicrophone_ClampDb(db_spl);
  s_value_valid = 1U;
  AnalogMicrophone_ResetSignalProcessing();
}

static void AnalogMicrophone_ProcessBlock(uint32_t offset, uint32_t word_count)
{
  uint32_t end = offset + word_count;

  if (s_warmup_blocks > 0U)
  {
    s_warmup_blocks--;
    if (s_warmup_blocks == 0U)
    {
      AnalogMicrophone_ResetSignalProcessing();
    }
    return;
  }

  for (uint32_t i = offset; (i + ANALOG_MIC_SIGNAL_INDEX) < end;
       i += ANALOG_MIC_CHANNELS_PER_FRAME)
  {
    int32_t differential;
    differential = (int32_t)s_dma_buffer[i + ANALOG_MIC_SIGNAL_INDEX] -
                   (int32_t)s_dma_buffer[i + ANALOG_MIC_MREF_INDEX];
    s_sample_sum += (int64_t)differential;
    s_square_sum += (uint64_t)((int64_t)differential * (int64_t)differential);
    s_energy_samples++;
    if (s_energy_samples >= s_window_samples)
    {
      AnalogMicrophone_FinishWindow();
    }
  }
}

HAL_StatusTypeDef AnalogMicrophone_Init(ADC_HandleTypeDef *hadc,
                                        TIM_HandleTypeDef *htim)
{
  HAL_StatusTypeDef status;

  s_adc = hadc;
  s_timer = htim;
  s_block_flags = 0U;
  s_running = 0U;
  s_dropped_blocks = 0U;
  s_value_valid = 0U;
  s_window_samples = (ANALOG_MIC_SAMPLE_RATE_HZ * ANALOG_MIC_DEFAULT_WINDOW_MS) / 1000U;
  AnalogMicrophone_ResetSignalProcessing();

  if ((hadc == NULL) || (htim == NULL))
  {
    return HAL_ERROR;
  }

  /* PA9 only supplies the analog microphone and its op-amp. Keep that
     front-end off until a measurement is actually requested. */
  BSP_SENSOR_PWR_Off();
  status = HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
  return status;
}

HAL_StatusTypeDef AnalogMicrophone_ConfigureWindow(uint16_t window_ms)
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
  s_window_samples = ((uint32_t)ANALOG_MIC_SAMPLE_RATE_HZ * window_ms) / 1000U;
  return HAL_OK;
}

HAL_StatusTypeDef AnalogMicrophone_Start(void)
{
  HAL_StatusTypeDef status;

  if ((s_adc == NULL) || (s_timer == NULL))
  {
    return HAL_ERROR;
  }
  if (s_running != 0U)
  {
    return HAL_OK;
  }

  BSP_SENSOR_PWR_On();
  HAL_Delay(ANALOG_MIC_POWER_STABILIZE_MS);

  memset(s_dma_buffer, 0, sizeof(s_dma_buffer));
  s_block_flags = 0U;
  s_warmup_blocks = ANALOG_MIC_WARMUP_BLOCKS;
  s_value_valid = 0U;
  AnalogMicrophone_ResetSignalProcessing();

  status = HAL_ADC_Start_DMA(s_adc, (uint32_t *)s_dma_buffer,
                             ANALOG_MIC_DMA_WORDS);
  if (status != HAL_OK)
  {
    BSP_SENSOR_PWR_Off();
    return status;
  }

  s_running = 1U;
  status = HAL_TIM_Base_Start(s_timer);
  if (status != HAL_OK)
  {
    s_running = 0U;
    (void)HAL_ADC_Stop_DMA(s_adc);
    BSP_SENSOR_PWR_Off();
  }
  return status;
}

void AnalogMicrophone_Process(void)
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

  if ((flags & ANALOG_MIC_BLOCK_HALF_READY) != 0U)
  {
    AnalogMicrophone_ProcessBlock(0U, ANALOG_MIC_DMA_HALF_WORDS);
  }
  if ((flags & ANALOG_MIC_BLOCK_FULL_READY) != 0U)
  {
    AnalogMicrophone_ProcessBlock(ANALOG_MIC_DMA_HALF_WORDS,
                                  ANALOG_MIC_DMA_HALF_WORDS);
  }
}

void AnalogMicrophone_Stop(void)
{
  s_running = 0U;
  if (s_timer != NULL)
  {
    (void)HAL_TIM_Base_Stop(s_timer);
  }
  if (s_adc != NULL)
  {
    (void)HAL_ADC_Stop_DMA(s_adc);
  }
  s_block_flags = 0U;
  /* Disconnect the analog microphone/op-amp supply whenever acquisition is
     idle, including the interval before STOP2. */
  BSP_SENSOR_PWR_Off();
}

uint16_t AnalogMicrophone_AppendPayload(uint8_t *buffer, uint16_t buffer_size)
{
  if ((buffer == NULL) || (buffer_size < ANALOG_MICROPHONE_PAYLOAD_SIZE) ||
      (s_value_valid == 0U))
  {
    return 0U;
  }

  buffer[0] = s_last_db_spl;
  return ANALOG_MICROPHONE_PAYLOAD_SIZE;
}

uint32_t AnalogMicrophone_GetDroppedBlocks(void)
{
  return s_dropped_blocks;
}

uint8_t AnalogMicrophone_IsRunning(void)
{
  return s_running;
}

uint8_t AnalogMicrophone_HasValue(void)
{
  return s_value_valid;
}

uint8_t AnalogMicrophone_GetLastValue(void)
{
  return s_last_db_spl;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((s_adc != NULL) && (hadc->Instance == s_adc->Instance) && (s_running != 0U))
  {
    if ((s_block_flags & ANALOG_MIC_BLOCK_HALF_READY) != 0U)
    {
      s_dropped_blocks++;
    }
    s_block_flags |= ANALOG_MIC_BLOCK_HALF_READY;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((s_adc != NULL) && (hadc->Instance == s_adc->Instance) && (s_running != 0U))
  {
    if ((s_block_flags & ANALOG_MIC_BLOCK_FULL_READY) != 0U)
    {
      s_dropped_blocks++;
    }
    s_block_flags |= ANALOG_MIC_BLOCK_FULL_READY;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if ((s_adc != NULL) && (hadc->Instance == s_adc->Instance))
  {
    s_running = 0U;
    s_dropped_blocks++;
    BSP_SENSOR_PWR_Off();
  }
}
