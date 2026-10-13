/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @brief   Custom BLE application for sensor and vibrometer data transfer.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "qspi_nor.h"
#include "zigbee_vibration_store.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_conf.h"
#include "app_entry.h"
#include "app_zigbee.h"
#include "hw_if.h"
#include "stm32_lpm.h"
#include "../../Middlewares/Sensors/Analog microphone/analog_microphone.h"
#include "../../Middlewares/Sensors/Digital microphone/digital_microphone.h"
#include "../../Middlewares/Sensors/Magnetometer/magnetometer.h"
#include "../../Middlewares/Sensors/Thermometer/thermometer.h"
#include "../../Middlewares/Sensors/Vibrometer/vibrometer.h"
#include "stm32wb55rgv6_bsp.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* SENSOR_SERVICE */
  uint8_t               Sensor_status_Notification_Status;
  /* VIB_SERVICE */
  uint8_t               Vib_wave_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  uint8_t               IsConnected;
  uint8_t               SensorStreaming;
  uint8_t               SensorEnableMask;
  uint8_t               VibStreaming;
  uint8_t               TxFlowOn;
  uint8_t               SensorSeq;
  uint8_t               VibSeq;
  uint8_t               SensorSendPending;
  uint8_t               SensorAudioReportedMask;
  uint32_t              SensorNextTick;
  uint32_t              AudioRestartNextTick;
  uint32_t              AudioHealthNextTick;
  uint32_t              VibPollNextTick;
  uint32_t              VibTxBytes;
  uint32_t              VibDroppedPackets;
  uint32_t              VibStatNextTick;
  uint32_t              VibStatPackets;
  uint32_t              VibStatSamples;
  uint32_t              VibStatBytes;
  uint32_t              VibStatBusy;
  uint32_t              VibStatPoolAvailable;
  uint32_t              VibStatFlowRecoveries;
  uint32_t              VibStatSpiErrors;
  uint32_t              VibStatFifoOverruns;
  uint16_t              VibStatMaxFifo;
  uint16_t              VibPendingLength;
  uint8_t               VibPacketPending;
  uint8_t               VibDecimationPhase;
  uint32_t              TxRetryTick;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t thermometer;
} DT_SourceState_t;

typedef struct
{
  uint32_t sample_count;
  uint32_t target_samples;
  uint16_t peak_fifo_words;
  uint16_t read_errors;
  uint16_t max_service_gap_ms;
  uint16_t max_service_ms;
  uint8_t complete;
  uint8_t overrun_seen;
  uint8_t sampling_stopped;
  uint8_t timed_out;
  uint16_t flash_errors;
  uint16_t buffer_overruns;
  /* Coarse wall-clock diagnostics; collected silently, printed after capture. */
  uint32_t diag_start_ms;
  uint32_t diag_service_ms;
  uint32_t diag_fifo_ms;
  uint32_t diag_pack_ms;
  uint32_t diag_store_ms;
  uint32_t diag_calls;
  uint32_t diag_write_calls;
  uint32_t diag_write_deferred;
  uint32_t diag_drained_bytes;
  uint16_t diag_ring_peak;
} DT_ZigbeeVibrationAccumulator_t;
/* USER CODE END PTD */

#define DT_ZB_STORE_RING_POINTS         13334U
#define DT_ZB_STORE_WRITE_POINTS        160U
#define DT_ZB_STORE_ACTIVE_FIFO_MAX     64U
#define DT_ZB_CAPTURE_WATCHDOG_SLACK_MS 1500U

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_VIB_PACKET_SIZE              244U
#define DT_SENSOR_PACKET_SIZE           32U
#define DT_SENSOR_MAIN_PERIOD_MS        1000U
#define DT_CMD_STOP                     0x00U
#define DT_CMD_START                    0x01U
#define DT_CMD_TOGGLE                   0x02U
#define DT_SENSOR_FRAME_VERSION         0x03U
#define DT_VIB_FRAME_VERSION            0x02U
#define DT_SENSOR_TLV_FLAGS_SNAPSHOT    0x01U
#define DT_SENSOR_TLV_FLAGS_TEMP_EVENT  0x02U
#define DT_SENSOR_TLV_FLAGS_TEMP_THERM  0x04U
#define DT_SENSOR_TLV_TYPE_MAGNETOMETER 0x01U
#define DT_SENSOR_TLV_TYPE_DIGITAL_MIC  0x02U
#define DT_SENSOR_TLV_TYPE_ANALOG_MIC   0x03U
#define DT_SENSOR_TLV_TYPE_TEMPERATURE  0x04U
#define DT_SENSOR_TLV_TYPE_ENABLE_MASK  0xF0U
#define DT_SENSOR_ID_ALL                0x00U
#define DT_SENSOR_ID_MAGNETOMETER       0x01U
#define DT_SENSOR_ID_DIGITAL_MIC        0x02U
#define DT_SENSOR_ID_ANALOG_MIC         0x03U
#define DT_SENSOR_ID_TEMPERATURE        0x04U
#define DT_SENSOR_STATE_OFF             0x00U
#define DT_SENSOR_STATE_ON              0x01U
#define DT_SENSOR_ENABLE_MAGNETOMETER   0x01U
#define DT_SENSOR_ENABLE_DIGITAL_MIC    0x02U
#define DT_SENSOR_ENABLE_ANALOG_MIC     0x04U
#define DT_SENSOR_ENABLE_TEMPERATURE    0x08U
#define DT_SENSOR_ENABLE_ALL            0x0FU
#define DT_MAGNETOMETER_PAYLOAD_SIZE    6U
#define DT_DIGITAL_MIC_PAYLOAD_SIZE     1U
#define DT_ANALOG_MIC_PAYLOAD_SIZE      1U
#define DT_THERMOMETER_PAYLOAD_SIZE     2U
#define DT_SENSOR_ENABLE_MASK_SIZE      1U
#define DT_VIB_PACKET_HEADER_SIZE       4U
#define DT_VIB_SAMPLE_PAYLOAD_SIZE      6U
#define DT_VIB_MAX_SAMPLES_PER_PACKET   ((DT_VIB_PACKET_SIZE - DT_VIB_PACKET_HEADER_SIZE) / DT_VIB_SAMPLE_PAYLOAD_SIZE)
#define DT_VIB_POLL_PERIOD_MS           5U
#define DT_VIB_STAT_PERIOD_MS           1000U
#define DT_BLE_BUSY_RETRY_PERIOD_MS     5U
#define DT_BLE_POOL_RETRY_PERIOD_MS     30U
#define DT_VIB_BLE_DECIMATION           4U
#define DT_VIB_RAW_SAMPLES_PER_PACKET   (DT_VIB_MAX_SAMPLES_PER_PACKET * DT_VIB_BLE_DECIMATION)
#define DT_VIB_FLAGS_FIFO_WATERMARK     0x01U
#define DT_VIB_FLAGS_POLLED_READ        0x02U
#define DT_VIB_FLAGS_FIFO_MAY_HAVE_MORE 0x04U
#define DT_VIB_FLAGS_DECIMATION_DIV4    0x20U
#define DT_VIB_FIFO_WORD_SIZE           7U
#define DT_ZIGBEE_AUDIO_MAX_WINDOW_MS   1500U
#define DT_ZIGBEE_VIBRATION_RATE_HZ     26667U
#define DT_ZIGBEE_VIB_SENS_G_PER_LSB    0.000488
#define DT_ZIGBEE_VIB_MAX_BURSTS        4U
#define DT_ZIGBEE_VIB_STAT_SHIFT        4U
#define DT_ZIGBEE_VIB_FREQ_SETTLE       256U
#define DT_AUDIO_RESTART_PERIOD_MS       1000U
#define DT_AUDIO_HEALTH_PERIOD_MS        2000U
#define DT_PERIODIC_STREAM_LOGS          0U
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
static uint8_t VibTxBuffer[DT_VIB_PACKET_SIZE];
static uint8_t SensorTxBuffer[DT_SENSOR_PACKET_SIZE];
static uint8_t VibFifoRawBuffer[VIBROMETER_FIFO_DMA_MAX_BYTES];
static uint8_t SensorTimerId;
static volatile uint8_t VibFifoIrqPending;
static uint8_t ZigbeeAudioWindowActive;
static uint32_t ZigbeeAudioWindowDeadline;
static uint32_t ZigbeeAudioWindowStartTick;
static uint8_t ZigbeeDigitalMicStarted;
static uint8_t ZigbeeAnalogMicStarted;
static volatile uint8_t ZigbeeVibrationWindowActive;
static uint32_t ZigbeeVibrationWindowDeadline;
static uint32_t ZigbeeVibrationLastServiceTick;
static uint16_t ZigbeeVibrationWindowMs;
static uint16_t ZigbeeDigitalMicWindowMs;
static uint16_t ZigbeeAnalogMicWindowMs;
static uint8_t ZigbeeSensorEnableMask = CUSTOM_APP_ZB_SENSOR_ALL;
static uint32_t ZigbeeSnapshotBatchId;
static uint8_t ZigbeeVibrationStoreActive;
static int16_t ZigbeeStoreSamples[160U][3];
static DT_ZigbeeVibrationAccumulator_t ZigbeeVibration;
static int16_t ZigbeeStoreRing[DT_ZB_STORE_RING_POINTS][3];
static uint16_t ZigbeeStoreHead, ZigbeeStoreTail, ZigbeeStoreCount;
static uint16_t ZigbeeFifoReadPending;
static uint8_t ZigbeeVibrationServicing;

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c3;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern SAI_HandleTypeDef hsai_BlockA1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* SENSOR_SERVICE */
static void Custom_Sensor_status_Update_Char(void);
static void Custom_Sensor_status_Send_Notification(void);
/* VIB_SERVICE */
static void Custom_Vib_wave_Update_Char(void);
static void Custom_Vib_wave_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void DT_SendTask(void);
static void DT_VibDrainTask(void);
static void DT_StartSensorStream(void);
static void DT_StopSensorStream(void);
static void DT_ApplySensorEnableMask(uint8_t requested_mask);
static uint8_t DT_SensorIdToMask(uint8_t sensor_id);
static void DT_StartVibStream(void);
static void DT_StopVibStream(void);
static void DT_ToggleVibStream(void);
static void DT_HandleSensorCtrl(const uint8_t *payload, uint8_t length);
static void DT_HandleVibCtrl(const uint8_t *payload, uint8_t length);
static void DT_SensorTimerCallback(void);
static void DT_UpdateStatusLeds(void);
static void DT_UpdateLowPowerLock(void);
static void DT_Log(const char *message);
static void DT_ProcessBleAudio(uint32_t now);
#if (DT_PERIODIC_STREAM_LOGS != 0U)
static void DT_LogAudioHealth(uint32_t now);
static void DT_VibLogStatistics(uint32_t now);
#endif
static HAL_StatusTypeDef DT_VibInitHardware(void);
static HAL_StatusTypeDef DT_VibBuildPacket(uint16_t *packet_len, uint16_t *sample_count);
static HAL_StatusTypeDef DT_DrainZigbeeStore(uint16_t max_points);
static void DT_ProcessZigbeeVibration(uint32_t now)
    __attribute__((optimize("O3")));
static HAL_StatusTypeDef DT_StartZigbeeVibration(uint16_t window_ms);
static void DT_AbortZigbeeSnapshot(void);
extern void PWR_SetRunModeLock(uint8_t lock);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */
  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */
    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* SENSOR_SERVICE */
    case CUSTOM_STM_SENSOR_STATUS_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SENSOR_STATUS_NOTIFY_ENABLED_EVT */
      Custom_App_Context.Sensor_status_Notification_Status = 1U;
      DT_StartSensorStream();
      /* USER CODE END CUSTOM_STM_SENSOR_STATUS_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SENSOR_STATUS_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SENSOR_STATUS_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Sensor_status_Notification_Status = 0U;
      Custom_App_Context.SensorEnableMask = 0U;
      DT_StopSensorStream();
      /* USER CODE END CUSTOM_STM_SENSOR_STATUS_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SENSOR_CTRL_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SENSOR_CTRL_READ_EVT */
      /* USER CODE END CUSTOM_STM_SENSOR_CTRL_READ_EVT */
      break;

    case CUSTOM_STM_SENSOR_CTRL_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SENSOR_CTRL_WRITE_NO_RESP_EVT */
      DT_HandleSensorCtrl(pNotification->DataTransfered.pPayload, pNotification->DataTransfered.Length);
      /* USER CODE END CUSTOM_STM_SENSOR_CTRL_WRITE_NO_RESP_EVT */
      break;

    /* VIB_SERVICE */
    case CUSTOM_STM_VIB_WAVE_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VIB_WAVE_NOTIFY_ENABLED_EVT */
      Custom_App_Context.Vib_wave_Notification_Status = 1U;
      /* Notify only arms the BLE pipe. FE92 command 0x01 starts vib streaming. */
      /* USER CODE END CUSTOM_STM_VIB_WAVE_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_VIB_WAVE_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VIB_WAVE_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Vib_wave_Notification_Status = 0U;
      DT_StopVibStream();
      /* USER CODE END CUSTOM_STM_VIB_WAVE_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_VIB_CTRL_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VIB_CTRL_READ_EVT */
      /* USER CODE END CUSTOM_STM_VIB_CTRL_READ_EVT */
      break;

    case CUSTOM_STM_VIB_CTRL_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VIB_CTRL_WRITE_NO_RESP_EVT */
      DT_HandleVibCtrl(pNotification->DataTransfered.pPayload, pNotification->DataTransfered.Length);
      /* USER CODE END CUSTOM_STM_VIB_CTRL_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      /* TX pool recovery is handled by ACI_GATT_TX_POOL_AVAILABLE. */
      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */
      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */
  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */
  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
      Custom_App_Context.ConnectionHandle = pNotification->ConnectionHandle;
      Custom_App_Context.IsConnected = 1U;
      /* A new phone connection starts with no sensor selected. Do not reuse
         the enable mask from a previous connection or an interrupted command. */
      Custom_App_Context.SensorEnableMask = 0U;
      DT_AbortZigbeeSnapshot();
      APP_ZIGBEE_SetBleConnected(1U);
      DT_UpdateStatusLeds();
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
      Custom_App_Context.IsConnected = 0U;
      Custom_App_Context.ConnectionHandle = 0xFFFFU;
      Custom_App_Context.Sensor_status_Notification_Status = 0U;
      Custom_App_Context.Vib_wave_Notification_Status = 0U;
      Custom_App_Context.SensorEnableMask = 0U;
      /* Queue Zigbee MAC resume before synchronous peripheral shutdown. The
         radio task runs once this HCI callback returns. */
      APP_ZIGBEE_SetBleConnected(0U);
      DT_UpdateStatusLeds();
      DT_StopSensorStream();
      DT_StopVibStream();
      DT_Log("BLE: all BLE acquisition stopped after disconnect\r\n");
      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */
      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */
  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
  HAL_StatusTypeDef analog_mic_status;
  HAL_StatusTypeDef digital_mic_status;
  HAL_StatusTypeDef magnetometer_status;
  HAL_StatusTypeDef temperature_status;
  char sensor_log[96];
  char temperature_log[64];

  BSP_LED_Init(LED_MEAS);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);
  DT_UpdateStatusLeds();
  BSP_PB_Init(BUTTON_MEAS, BUTTON_MODE_EXTI);

  analog_mic_status = AnalogMicrophone_Init(&hadc1, &htim2);
  digital_mic_status = DigitalMicrophone_Init(&hsai_BlockA1);
  magnetometer_status = Magnetometer_Init(&hi2c3);
  (void)snprintf(sensor_log,
                 sizeof(sensor_log),
                 "SENS: mag=%d who=0x%02X amic=%d dmic=%d\r\n",
                 (int)magnetometer_status,
                 Magnetometer_GetWhoAmI(),
                 (int)analog_mic_status,
                 (int)digital_mic_status);
  DT_Log(sensor_log);
  /* Run SPI1 at about 8 MHz to keep up with the 26.667 kHz FIFO stream. */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    DT_Log("VIB: SPI1 high-speed init failed\r\n");
  }
  (void)DT_VibInitHardware();
  uint32_t flash_jedec_id = 0U, flash_capacity = 0U;
  if (ZigbeeVibrationStore_Init(&flash_jedec_id, &flash_capacity) == HAL_OK)
  {
    (void)snprintf(sensor_log, sizeof(sensor_log),
                   "VIB-FLASH: JEDEC=0x%06lX capacity=%lu bytes mode=%s program=%s\r\n",
                   (unsigned long)flash_jedec_id,
                   (unsigned long)flash_capacity,
                   (QspiNor_IsHighPerformance() != 0U) ? "high-performance" : "standard",
                   (QspiNor_IsQuadProgram() != 0U) ? "1-4-4" : "1-1-1");
    DT_Log(sensor_log);
  }
  else
  {
    (void)snprintf(sensor_log, sizeof(sensor_log),
                   "VIB-FLASH: initialization failed stage=%s; raw Zigbee upload unavailable\r\n",
                   QspiNor_GetLastErrorStage());
    DT_Log(sensor_log);
  }


  temperature_status = Thermometer_Init(&hi2c3);
  (void)snprintf(temperature_log,
                 sizeof(temperature_log),
                 "TEMP: status=%d pid=0x%02X mid=0x%02X\r\n",
                 (int)temperature_status,
                 Thermometer_GetProductId(),
                 Thermometer_GetManufacturerId());
  DT_Log(temperature_log);

  /* The identity checks above run with the sensors awake once. From this
     point they remain in their device standby modes until an acquisition
     explicitly requests them. */
  (void)Magnetometer_EnterLowPower();
  (void)Thermometer_EnterLowPower();

  Custom_App_Context.SensorEnableMask = 0U;
  Custom_App_Context.TxFlowOn = 1U;
  (void)HW_TS_Create(CFG_TIM_PROC_ID_ISR, &SensorTimerId, hw_ts_Repeated, DT_SensorTimerCallback);
  UTIL_SEQ_RegTask(1U << CFG_TASK_DT_SEND_ID, UTIL_SEQ_RFU, DT_SendTask);
  UTIL_SEQ_RegTask(1U << CFG_TASK_DT_VIB_DRAIN_ID, UTIL_SEQ_RFU,
                   DT_VibDrainTask);
  UTIL_LPM_SetOffMode(1U << CFG_LPM_APP_SENSORS, UTIL_LPM_DISABLE);
  DT_UpdateLowPowerLock();
  DT_Log("LPM: STOP2 enabled while sensor pipelines are idle\r\n");
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void Custom_APP_TxPoolAvailable(void)
{
  if (Custom_App_Context.VibStreaming != 0U)
  {
    Custom_App_Context.VibStatPoolAvailable++;
  }
  Custom_App_Context.TxFlowOn = 1U;
  Custom_App_Context.TxRetryTick = 0U;
  if (((Custom_App_Context.Sensor_status_Notification_Status != 0U) &&
       (Custom_App_Context.SensorSendPending != 0U)) ||
      ((Custom_App_Context.VibStreaming != 0U) &&
       (Custom_App_Context.Vib_wave_Notification_Status != 0U)))
  {
    UTIL_SEQ_SetTask(1U << CFG_TASK_DT_SEND_ID, CFG_SCH_PRIO_1);
  }
}

void Custom_APP_Process(void)
{
  uint32_t now;

  now = HAL_GetTick();

  /* Reassert active-low LED outputs after every wake and keep them tied to
     the authoritative BLE/Zigbee connection and acquisition state. */
  DT_UpdateStatusLeds();

  if (ZigbeeAudioWindowActive != 0U)
  {
    char audio_log[112];
    uint8_t audio_ready;

    /* Finish the two 500 ms audio integration windows before starting the
       CPU-heavy vibration FIFO drain. Otherwise DMA halves are overwritten
       while vibration features are being accumulated. */
    DigitalMicrophone_Process();
    AnalogMicrophone_Process();
    audio_ready = (((ZigbeeDigitalMicStarted == 0U) ||
                    (DigitalMicrophone_HasValue() != 0U)) &&
                   ((ZigbeeAnalogMicStarted == 0U) ||
                    (AnalogMicrophone_HasValue() != 0U))) ? 1U : 0U;
    if ((audio_ready != 0U) ||
        ((int32_t)(now - ZigbeeAudioWindowDeadline) >= 0))
    {
      ZigbeeAudioWindowActive = 0U;
      (void)snprintf(audio_log, sizeof(audio_log),
                     "AUDIO: Zigbee window %lu ms dmic=%u amic=%u drop_d=%lu drop_a=%lu\r\n",
                     (unsigned long)(now - ZigbeeAudioWindowStartTick),
                     (unsigned int)DigitalMicrophone_HasValue(),
                     (unsigned int)AnalogMicrophone_HasValue(),
                     (unsigned long)DigitalMicrophone_GetDroppedBlocks(),
                     (unsigned long)AnalogMicrophone_GetDroppedBlocks());
      DT_Log(audio_log);
      DigitalMicrophone_Stop();
      AnalogMicrophone_Stop();
      if (((ZigbeeSensorEnableMask & CUSTOM_APP_ZB_SENSOR_VIBRATION) != 0U) &&
          (DT_StartZigbeeVibration(ZigbeeVibrationWindowMs) != HAL_OK))
      {
        DT_Log("VIB: Zigbee capture start failed after audio\r\n");
      }
      DT_UpdateLowPowerLock();
    }
  }
  if (ZigbeeVibrationWindowActive != 0U)
  {
    DT_ProcessZigbeeVibration(now);
  }

  if (Custom_App_Context.IsConnected == 0U)
  {
    return;
  }

  DT_ProcessBleAudio(now);
#if (DT_PERIODIC_STREAM_LOGS != 0U)
  DT_LogAudioHealth(now);
  DT_VibLogStatistics(now);
#endif
  if ((Custom_App_Context.TxFlowOn == 0U) &&
      ((int32_t)(now - Custom_App_Context.TxRetryTick) >= 0))
  {
    /* Retry with backoff if the controller pool event is delayed or lost. */
    Custom_App_Context.TxFlowOn = 1U;
    Custom_App_Context.TxRetryTick = now + DT_BLE_BUSY_RETRY_PERIOD_MS;
    if (Custom_App_Context.VibStreaming != 0U)
    {
      Custom_App_Context.VibStatFlowRecoveries++;
    }
    UTIL_SEQ_SetTask(1U << CFG_TASK_DT_SEND_ID, CFG_SCH_PRIO_1);
  }
  if ((Custom_App_Context.VibStreaming != 0U) &&
      (Custom_App_Context.Vib_wave_Notification_Status != 0U) &&
      (Vibrometer_IsReady() != 0U) &&
      ((int32_t)(now - Custom_App_Context.VibPollNextTick) >= 0))
  {
    Custom_App_Context.VibPollNextTick = now + DT_VIB_POLL_PERIOD_MS;
    UTIL_SEQ_SetTask(1U << CFG_TASK_DT_SEND_ID, CFG_SCH_PRIO_1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == STTS_EVENT_Pin)
  {
    /* Latch the STTS751 EVENT alert and process it in the normal BLE context. */
    Thermometer_OnEventAlert();
  }
  else if (GPIO_Pin == STTS_THERM_Pin)
  {
    /* Latch the STTS751 THERM alert and process it in the normal BLE context. */
    Thermometer_OnThermAlert();
  }
  else if (GPIO_Pin == INT1_ADWB_Pin)
  {
    /* Queue an urgent drain. When Zigbee tasks are already pending, relying
       only on the main-loop poll can leave the 512-word FIFO full. */
    VibFifoIrqPending = 1U;
    if (ZigbeeVibrationWindowActive != 0U)
    {
      UTIL_SEQ_SetTask(1U << CFG_TASK_DT_VIB_DRAIN_ID, CFG_SCH_PRIO_0);
    }
  }
}

void Custom_APP_PrepareStop2(void)
{
  /* STOP2 is only selected when DT_UpdateLowPowerLock() confirms that no
     acquisition pipeline is active. Make every external device state
     explicit before the core clock and peripheral clocks are stopped. */
  DigitalMicrophone_Stop();
  AnalogMicrophone_Stop();
  /* Final board-level assertion: PA9 low disconnects +3.3V_ANA before
     CPU1 enters STOP2. */
  BSP_SENSOR_PWR_Off();
  if (Vibrometer_IsReady() != 0U)
  {
    (void)Vibrometer_Stop();
  }
  (void)Magnetometer_EnterLowPower();
  (void)Thermometer_EnterLowPower();

  /* PB0/PB1 network LEDs are active-low, so SET turns both LEDs off. */
  HAL_GPIO_WritePin(LED_NET_BLUE_GPIO_Port,
                    LED_NET_BLUE_Pin | LED_NET_RED_Pin,
                    GPIO_PIN_SET);
}

void Custom_APP_ResumeFromStop2(void)
{
  /* Low-speed sensors intentionally stay in standby after a wake-up. Their
     acquisition entry points resume them only when data is actually needed. */
  DT_UpdateStatusLeds();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  Vibrometer_SpiTxRxCompleteCallback(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  Vibrometer_SpiErrorCallback(hspi);
}

uint8_t Custom_APP_ZigbeeBeginSnapshot(uint16_t vibration_rate_hz,
                                       uint16_t vibration_window_ms,
                                       uint16_t digital_mic_window_ms,
                                       uint16_t analog_mic_window_ms,
                                       uint8_t sensor_enable_mask,
                                       uint32_t batch_id)
{
  HAL_StatusTypeDef digital_status;
  HAL_StatusTypeDef analog_status;
  char audio_log[72];

  if ((Custom_App_Context.IsConnected != 0U) ||
      (Custom_App_Context.SensorStreaming != 0U) ||
      (Custom_App_Context.VibStreaming != 0U) ||
      (ZigbeeAudioWindowActive != 0U) ||
      (ZigbeeVibrationWindowActive != 0U))
  {
    return 0U;
  }

  /* Keep the low-speed sensors awake for the complete snapshot window. */
  (void)Magnetometer_ExitLowPower();
  (void)Thermometer_ExitLowPower();

  ZigbeeDigitalMicStarted = 0U;
  ZigbeeAnalogMicStarted = 0U;
  ZigbeeVibrationWindowMs = vibration_window_ms;
  ZigbeeDigitalMicWindowMs = digital_mic_window_ms;
  ZigbeeAnalogMicWindowMs = analog_mic_window_ms;
  ZigbeeSensorEnableMask = sensor_enable_mask & CUSTOM_APP_ZB_SENSOR_ALL;
  ZigbeeSnapshotBatchId = batch_id;
  ZigbeeVibrationStoreActive = 0U;
  if ((((ZigbeeSensorEnableMask & CUSTOM_APP_ZB_SENSOR_DIGITAL_MIC) != 0U) &&
       (DigitalMicrophone_ConfigureWindow(digital_mic_window_ms) != HAL_OK)) ||
      (((ZigbeeSensorEnableMask & CUSTOM_APP_ZB_SENSOR_ANALOG_MIC) != 0U) &&
       (AnalogMicrophone_ConfigureWindow(analog_mic_window_ms) != HAL_OK)))
  {
    DT_Log("AUDIO: invalid Zigbee microphone window\r\n");
    return 0U;
  }
  digital_status = ((ZigbeeSensorEnableMask & CUSTOM_APP_ZB_SENSOR_DIGITAL_MIC) != 0U) ?
                   DigitalMicrophone_Start() : HAL_ERROR;
  analog_status = ((ZigbeeSensorEnableMask & CUSTOM_APP_ZB_SENSOR_ANALOG_MIC) != 0U) ?
                  AnalogMicrophone_Start() : HAL_ERROR;
  ZigbeeDigitalMicStarted = (digital_status == HAL_OK) ? 1U : 0U;
  ZigbeeAnalogMicStarted = (analog_status == HAL_OK) ? 1U : 0U;
  (void)snprintf(audio_log, sizeof(audio_log),
                 "AUDIO: Zigbee start mask=0x%02x dmic=%u amic=%u\r\n",
                 (unsigned int)ZigbeeSensorEnableMask,
                 (unsigned int)ZigbeeDigitalMicStarted,
                 (unsigned int)ZigbeeAnalogMicStarted);
  DT_Log(audio_log);
  if ((ZigbeeDigitalMicStarted != 0U) ||
      (ZigbeeAnalogMicStarted != 0U))
  {
    ZigbeeAudioWindowStartTick = HAL_GetTick();
    uint16_t audio_window_ms = (ZigbeeDigitalMicStarted != 0U) ?
                               ZigbeeDigitalMicWindowMs : 0U;
    if ((ZigbeeAnalogMicStarted != 0U) &&
        (ZigbeeAnalogMicWindowMs > audio_window_ms))
    {
      audio_window_ms = ZigbeeAnalogMicWindowMs;
    }
    ZigbeeAudioWindowDeadline = ZigbeeAudioWindowStartTick + audio_window_ms + 150U;
    ZigbeeAudioWindowActive = 1U;
    DT_UpdateLowPowerLock();
    return 1U;
  }

  DigitalMicrophone_Stop();
  AnalogMicrophone_Stop();
  if (((ZigbeeSensorEnableMask & CUSTOM_APP_ZB_SENSOR_VIBRATION) != 0U) &&
      (DT_StartZigbeeVibration(vibration_window_ms) != HAL_OK))
  {
    DT_Log("VIB: Zigbee capture start failed after audio start errors\r\n");
    return 0U;
  }
  return 1U;
}

static HAL_StatusTypeDef DT_StartZigbeeVibration(uint16_t window_ms)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t target_samples =
      ((uint32_t)DT_ZIGBEE_VIBRATION_RATE_HZ * window_ms + 500U) / 1000U;

  if ((target_samples == 0U) ||
      (target_samples > ZIGBEE_VIBRATION_MAX_SAMPLES))
  {
    char log_line[112];
    (void)snprintf(log_line, sizeof(log_line),
                   "VIB: window=%u ms requires unsupported sample count=%lu (max=%lu)\r\n",
                   (unsigned int)window_ms, (unsigned long)target_samples,
                   (unsigned long)ZIGBEE_VIBRATION_MAX_SAMPLES);
    DT_Log(log_line);
    return HAL_ERROR;
  }

  if (Vibrometer_IsReady() == 0U)
  {
    (void)DT_VibInitHardware();
  }
  if (Vibrometer_IsReady() != 0U)
  {
    /* Erasing the QSPI slot can take tens of milliseconds. Complete it before
       enabling the sensor FIFO; doing it in the opposite order overflows the
       512-word FIFO before its first service. */
    if (ZigbeeVibrationStore_Begin(DT_ZIGBEE_VIBRATION_RATE_HZ,
                                   window_ms,
                                   ZigbeeSnapshotBatchId) != HAL_OK)
    {
      if (ZigbeeVibrationStore_IsReady() == 0U)
      {
        DT_Log("VIB-FLASH: store unavailable; Zigbee vibration capture cannot start\r\n");
      }
      else
      {
        DT_Log("VIB-FLASH: no writable slot for Zigbee capture\r\n");
      }
      return HAL_ERROR;
    }
    ZigbeeVibrationStoreActive = 1U;
    ZigbeeStoreHead = ZigbeeStoreTail = ZigbeeStoreCount = 0U;
    ZigbeeFifoReadPending = 0U;
    status = Vibrometer_StartContinuousFifo();
    if (status != HAL_OK)
    {
      ZigbeeVibrationStore_Abort();
      ZigbeeVibrationStoreActive = 0U;
      return status;
    }
    (void)memset(&ZigbeeVibration, 0, sizeof(ZigbeeVibration));
    ZigbeeVibration.target_samples = target_samples;
    ZigbeeVibration.diag_start_ms = HAL_GetTick();
    /* Count-based acquisition fixes the waveform length. The extra time is
       only a watchdog allowance for interleaved QSPI page programs. */
    ZigbeeVibrationWindowDeadline = HAL_GetTick() + window_ms +
                                     DT_ZB_CAPTURE_WATCHDOG_SLACK_MS;
    ZigbeeVibrationLastServiceTick = HAL_GetTick();
    ZigbeeVibrationWindowActive = 1U;
    {
      char log_line[112];
      (void)snprintf(log_line, sizeof(log_line),
                     "VIB: streaming capture window=%u ms target=%lu ring=%u points\r\n",
                     (unsigned int)window_ms,
                     (unsigned long)target_samples,
                     (unsigned int)DT_ZB_STORE_RING_POINTS);
      DT_Log(log_line);
    }
    DT_UpdateLowPowerLock();
  }
  return status;
}

uint8_t Custom_APP_ZigbeeSnapshotReady(void)
{
  return ((ZigbeeAudioWindowActive == 0U) &&
          (ZigbeeVibrationWindowActive == 0U)) ? 1U : 0U;
}

void Custom_APP_ZigbeeFinishSnapshot(Custom_APP_ZigbeeSnapshot_t *snapshot)
{
  uint8_t payload[6];
  char log_line[128];

  if (snapshot == NULL)
  {
    return;
  }

  (void)memset(snapshot, 0, sizeof(*snapshot));
  DigitalMicrophone_Process();
  AnalogMicrophone_Process();

  if (Thermometer_AppendPayload(payload, sizeof(payload)) == 2U)
  {
    snapshot->temperature_centi_c = (int16_t)((uint16_t)payload[0] |
                                               ((uint16_t)payload[1] << 8U));
    snapshot->valid_mask |= CUSTOM_APP_ZB_VALID_TEMPERATURE;
  }
  if (Magnetometer_AppendPayload(payload, sizeof(payload)) == 6U)
  {
    snapshot->magnetic_0p1_ut[0] = (int16_t)((uint16_t)payload[0] |
                                             ((uint16_t)payload[1] << 8U));
    snapshot->magnetic_0p1_ut[1] = (int16_t)((uint16_t)payload[2] |
                                             ((uint16_t)payload[3] << 8U));
    snapshot->magnetic_0p1_ut[2] = (int16_t)((uint16_t)payload[4] |
                                             ((uint16_t)payload[5] << 8U));
    snapshot->valid_mask |= CUSTOM_APP_ZB_VALID_MAGNETIC;
  }
  if ((ZigbeeDigitalMicStarted != 0U) &&
      (DigitalMicrophone_AppendPayload(payload, sizeof(payload)) == 1U))
  {
    snapshot->digital_mic_db_spl = payload[0];
    snapshot->valid_mask |= CUSTOM_APP_ZB_VALID_DIGITAL_MIC;
  }
  if ((ZigbeeAnalogMicStarted != 0U) &&
      (AnalogMicrophone_AppendPayload(payload, sizeof(payload)) == 1U))
  {
    snapshot->analog_mic_db_spl = payload[0];
    snapshot->valid_mask |= CUSTOM_APP_ZB_VALID_ANALOG_MIC;
  }
  uint32_t expected_vibration_samples =
      ((uint32_t)DT_ZIGBEE_VIBRATION_RATE_HZ * ZigbeeVibrationWindowMs + 500U) /
      1000U;

  if (((ZigbeeSensorEnableMask & CUSTOM_APP_ZB_SENSOR_VIBRATION) != 0U) &&
      (ZigbeeVibration.complete != 0U) &&
      (ZigbeeVibration.overrun_seen == 0U) &&
      (ZigbeeVibration.read_errors == 0U) &&
      (ZigbeeVibration.flash_errors == 0U) &&
      (ZigbeeVibration.buffer_overruns == 0U) &&
      (ZigbeeVibration.timed_out == 0U) &&
      (ZigbeeVibration.sample_count == expected_vibration_samples) &&
      (ZigbeeVibration.sample_count >= DT_ZIGBEE_VIB_FREQ_SETTLE))
  {
    snapshot->vibration_rate_hz = DT_ZIGBEE_VIBRATION_RATE_HZ;
    snapshot->vibration_sample_count = ZigbeeVibration.sample_count;
    snapshot->valid_mask |= CUSTOM_APP_ZB_VALID_VIBRATION;
    if ((ZigbeeVibrationStoreActive != 0U) &&
        (ZigbeeVibrationStore_Finish(NULL) == HAL_OK))
    {
      ZigbeeVibrationWriteStats_t write_stats;
      snapshot->vibration_sample_count = ZigbeeVibrationStore_SampleCount();
      ZigbeeVibrationStore_GetWriteStats(&write_stats);
      (void)snprintf(log_line, sizeof(log_line),
                     "VIB-FLASH: wrote=%lu bytes pages=%lu partial=%lu write=%lu ms rate=%lu B/s\r\n",
                     (unsigned long)write_stats.bytes,
                     (unsigned long)write_stats.pages,
                     (unsigned long)write_stats.partial_pages,
                     (unsigned long)write_stats.write_ms,
                     (unsigned long)((write_stats.write_ms != 0U) ?
                         (((uint64_t)write_stats.bytes * 1000U) /
                          write_stats.write_ms) : 0U));
      DT_Log(log_line);
    }
    else
    {
      ZigbeeVibrationStore_Abort();
      snapshot->valid_mask &= (uint8_t)~CUSTOM_APP_ZB_VALID_VIBRATION;
      DT_Log("VIB-FLASH: capture commit failed\r\n");
    }
    ZigbeeVibrationStoreActive = 0U;
  }
  else if (ZigbeeVibrationStoreActive != 0U)
  {
    ZigbeeVibrationWriteStats_t write_stats;
    ZigbeeVibrationStore_GetWriteStats(&write_stats);
    (void)snprintf(log_line, sizeof(log_line),
                   "VIB-FLASH: incomplete wrote=%lu bytes pages=%lu partial=%lu write=%lu ms rate=%lu B/s\r\n",
                   (unsigned long)write_stats.bytes,
                   (unsigned long)write_stats.pages,
                   (unsigned long)write_stats.partial_pages,
                   (unsigned long)write_stats.write_ms,
                   (unsigned long)((write_stats.write_ms != 0U) ?
                       (((uint64_t)write_stats.bytes * 1000U) /
                        write_stats.write_ms) : 0U));
    DT_Log(log_line);
    ZigbeeVibrationStore_Abort();
    ZigbeeVibrationStoreActive = 0U;
    DT_Log("VIB-FLASH: incomplete/overrun capture discarded; truncated waveform will not upload\r\n");
  }

  if (ZigbeeAudioWindowActive != 0U)
  {
    ZigbeeAudioWindowActive = 0U;
    DigitalMicrophone_Stop();
    AnalogMicrophone_Stop();
    DT_UpdateLowPowerLock();
  }
  ZigbeeDigitalMicStarted = 0U;
  ZigbeeAnalogMicStarted = 0U;
}

void Custom_APP_ZigbeeActivityChanged(void)
{
  /* Zigbee calls this whenever acquisition state or its APS transmit queue
     changes. It keeps the LEDs and STOP2 decision synchronized with the
     actual end of the radio transaction, rather than merely queueing data. */
  DT_UpdateStatusLeds();
  DT_UpdateLowPowerLock();
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* SENSOR_SERVICE */
__USED void Custom_Sensor_status_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sensor_status_UC_1*/
  /* USER CODE END Sensor_status_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SENSOR_STATUS, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sensor_status_UC_Last*/
  /* USER CODE END Sensor_status_UC_Last*/
  return;
}

__USED void Custom_Sensor_status_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sensor_status_NS_1*/
  /* USER CODE END Sensor_status_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SENSOR_STATUS, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sensor_status_NS_Last*/
  /* USER CODE END Sensor_status_NS_Last*/

  return;
}

/* VIB_SERVICE */
__USED void Custom_Vib_wave_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Vib_wave_UC_1*/
  /* USER CODE END Vib_wave_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_VIB_WAVE, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Vib_wave_UC_Last*/
  /* USER CODE END Vib_wave_UC_Last*/
  return;
}

__USED void Custom_Vib_wave_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Vib_wave_NS_1*/
  /* USER CODE END Vib_wave_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_VIB_WAVE, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Vib_wave_NS_Last*/
  /* USER CODE END Vib_wave_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
static void DT_StartSensorStream(void)
{
  if ((Custom_App_Context.IsConnected == 0U) ||
      (Custom_App_Context.Sensor_status_Notification_Status == 0U))
  {
    return;
  }

  DT_ApplySensorEnableMask(Custom_App_Context.SensorEnableMask);
}

static void DT_StopSensorStream(void)
{
  Custom_App_Context.SensorStreaming = 0U;
  Custom_App_Context.SensorSendPending = 0U;
  AnalogMicrophone_Stop();
  DigitalMicrophone_Stop();
  (void)Magnetometer_EnterLowPower();
  (void)Thermometer_EnterLowPower();
  HW_TS_Stop(SensorTimerId);
  DT_UpdateLowPowerLock();
}

static void DT_ApplySensorEnableMask(uint8_t requested_mask)
{
  HAL_StatusTypeDef analog_status = HAL_OK;
  HAL_StatusTypeDef digital_status = HAL_OK;
  uint8_t active_mask = (uint8_t)(requested_mask & DT_SENSOR_ENABLE_ALL);
  char sensor_log[80];

  if ((Custom_App_Context.IsConnected == 0U) ||
      (Custom_App_Context.Sensor_status_Notification_Status == 0U))
  {
    AnalogMicrophone_Stop();
    DigitalMicrophone_Stop();
    (void)Magnetometer_EnterLowPower();
    (void)Thermometer_EnterLowPower();
    Custom_App_Context.SensorEnableMask = active_mask;
    Custom_App_Context.SensorStreaming = 0U;
    Custom_App_Context.SensorSendPending = 0U;
    HW_TS_Stop(SensorTimerId);
    DT_UpdateLowPowerLock();
    return;
  }

  if ((active_mask & DT_SENSOR_ENABLE_MAGNETOMETER) != 0U)
  {
    if (Magnetometer_ExitLowPower() != HAL_OK)
    {
      active_mask &= (uint8_t)~DT_SENSOR_ENABLE_MAGNETOMETER;
    }
  }
  else
  {
    (void)Magnetometer_EnterLowPower();
  }

  if ((active_mask & DT_SENSOR_ENABLE_TEMPERATURE) != 0U)
  {
    if (Thermometer_ExitLowPower() != HAL_OK)
    {
      active_mask &= (uint8_t)~DT_SENSOR_ENABLE_TEMPERATURE;
    }
  }
  else
  {
    (void)Thermometer_EnterLowPower();
  }

  if ((active_mask & DT_SENSOR_ENABLE_ANALOG_MIC) != 0U)
  {
    analog_status = AnalogMicrophone_Start();
    if (analog_status != HAL_OK)
    {
      active_mask &= (uint8_t)~DT_SENSOR_ENABLE_ANALOG_MIC;
    }
  }
  else
  {
    AnalogMicrophone_Stop();
  }

  if ((active_mask & DT_SENSOR_ENABLE_DIGITAL_MIC) != 0U)
  {
    digital_status = DigitalMicrophone_Start();
    if (digital_status != HAL_OK)
    {
      active_mask &= (uint8_t)~DT_SENSOR_ENABLE_DIGITAL_MIC;
    }
  }
  else
  {
    DigitalMicrophone_Stop();
  }

  Custom_App_Context.SensorEnableMask = active_mask;
  Custom_App_Context.SensorStreaming = (active_mask != 0U) ? 1U : 0U;
  Custom_App_Context.SensorAudioReportedMask = 0U;
  Custom_App_Context.AudioRestartNextTick = HAL_GetTick() + DT_AUDIO_RESTART_PERIOD_MS;
  Custom_App_Context.AudioHealthNextTick = HAL_GetTick() + DT_AUDIO_HEALTH_PERIOD_MS;
  /* Send an immediate state frame, including when all low-speed sensors are off. */
  Custom_App_Context.SensorSendPending = 1U;
  Custom_App_Context.TxFlowOn = 1U;
  HW_TS_Stop(SensorTimerId);
  if (Custom_App_Context.SensorStreaming != 0U)
  {
    HW_TS_Start(SensorTimerId,
                (DT_SENSOR_MAIN_PERIOD_MS * 1000U) / CFG_TS_TICK_VAL);
  }
  DT_UpdateLowPowerLock();
  (void)snprintf(sensor_log,
                 sizeof(sensor_log),
                 "SENS: mask=0x%02X amic=%d dmic=%d\r\n",
                 active_mask,
                 (int)analog_status,
                 (int)digital_status);
  DT_Log(sensor_log);
  UTIL_SEQ_SetTask(1U << CFG_TASK_DT_SEND_ID, CFG_SCH_PRIO_1);
}

static uint8_t DT_SensorIdToMask(uint8_t sensor_id)
{
  switch (sensor_id)
  {
    case DT_SENSOR_ID_MAGNETOMETER:
      return DT_SENSOR_ENABLE_MAGNETOMETER;

    case DT_SENSOR_ID_DIGITAL_MIC:
      return DT_SENSOR_ENABLE_DIGITAL_MIC;

    case DT_SENSOR_ID_ANALOG_MIC:
      return DT_SENSOR_ENABLE_ANALOG_MIC;

    case DT_SENSOR_ID_TEMPERATURE:
      return DT_SENSOR_ENABLE_TEMPERATURE;

    default:
      return 0U;
  }
}

static void DT_StartVibStream(void)
{
  HAL_StatusTypeDef status;

  if ((Custom_App_Context.IsConnected == 0U) ||
      (Custom_App_Context.Vib_wave_Notification_Status == 0U))
  {
    return;
  }

  if (Vibrometer_IsReady() == 0U)
  {
    (void)DT_VibInitHardware();
  }

  if (Vibrometer_IsReady() == 0U)
  {
    DT_Log("VIB: start rejected, device is not ready\r\n");
    return;
  }

  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_SENSORS, UTIL_LPM_DISABLE);
  status = Vibrometer_StartContinuousFifo();
  if (status != HAL_OK)
  {
    DT_UpdateLowPowerLock();
    DT_Log("VIB: start rejected, FIFO init failed\r\n");
    return;
  }

  Custom_App_Context.VibStreaming = 1U;
  DT_UpdateLowPowerLock();
  Custom_App_Context.TxFlowOn = 1U;
  Custom_App_Context.TxRetryTick = 0U;
  Custom_App_Context.VibPollNextTick = HAL_GetTick();
  Custom_App_Context.VibStatNextTick = HAL_GetTick() + DT_VIB_STAT_PERIOD_MS;
  Custom_App_Context.VibStatPackets = 0U;
  Custom_App_Context.VibStatSamples = 0U;
  Custom_App_Context.VibStatBytes = 0U;
  Custom_App_Context.VibStatBusy = 0U;
  Custom_App_Context.VibStatPoolAvailable = 0U;
  Custom_App_Context.VibStatFlowRecoveries = 0U;
  Custom_App_Context.VibStatSpiErrors = 0U;
  Custom_App_Context.VibStatFifoOverruns = 0U;
  Custom_App_Context.VibStatMaxFifo = 0U;
  Custom_App_Context.VibPendingLength = 0U;
  Custom_App_Context.VibPacketPending = 0U;
  Custom_App_Context.VibDecimationPhase = 0U;
  VibFifoIrqPending = 0U;
  DT_UpdateStatusLeds();
  DT_Log("VIB: stream armed, BLE preview 6667 Hz (4:1)\r\n");
  UTIL_SEQ_SetTask(1U << CFG_TASK_DT_SEND_ID, CFG_SCH_PRIO_1);
}

static void DT_StopVibStream(void)
{
  Custom_App_Context.VibStreaming = 0U;
  Custom_App_Context.VibPacketPending = 0U;
  Custom_App_Context.VibPendingLength = 0U;
  VibFifoIrqPending = 0U;
  if (Vibrometer_IsReady() != 0U)
  {
    if (Vibrometer_Stop() != HAL_OK)
    {
      DT_Log("VIB: stop failed\r\n");
    }
  }
  DT_UpdateStatusLeds();
  DT_UpdateLowPowerLock();
}

static void DT_UpdateStatusLeds(void)
{
  uint8_t network_connected =
      ((Custom_App_Context.IsConnected != 0U) ||
       (APP_ZIGBEE_IsConnected() != 0U)) ? 1U : 0U;
  uint8_t measuring =
      ((Custom_App_Context.SensorStreaming != 0U) ||
       (Custom_App_Context.VibStreaming != 0U) ||
       (APP_ZIGBEE_IsMeasuring() != 0U)) ? 1U : 0U;

  if (network_connected != 0U)
  {
    BSP_LED_Off(LED_RED);
    BSP_LED_On(LED_BLUE);
  }
  else
  {
    BSP_LED_Off(LED_BLUE);
    BSP_LED_On(LED_RED);
  }

  if (measuring != 0U)
  {
    BSP_LED_On(LED_MEAS);
  }
  else
  {
    BSP_LED_Off(LED_MEAS);
  }
}

static void DT_ToggleVibStream(void)
{
  if (Custom_App_Context.VibStreaming != 0U)
  {
    DT_StopVibStream();
  }
  else
  {
    DT_StartVibStream();
  }
}

static void DT_SensorTimerCallback(void)
{
  if ((Custom_App_Context.SensorStreaming != 0U) &&
      (Custom_App_Context.Sensor_status_Notification_Status != 0U))
  {
    Custom_App_Context.SensorSendPending = 1U;
    UTIL_SEQ_SetTask(1U << CFG_TASK_DT_SEND_ID, CFG_SCH_PRIO_1);
  }
}

static void DT_UpdateLowPowerLock(void)
{
  if ((Custom_App_Context.IsConnected != 0U) ||
      (APP_ZIGBEE_IsPendingAuthorization() != 0U) ||
      (Custom_App_Context.VibStreaming != 0U) ||
      (ZigbeeAudioWindowActive != 0U) ||
      (ZigbeeVibrationWindowActive != 0U) ||
      (APP_ZIGBEE_IsTransmitting() != 0U) ||
      (Custom_App_Context.SensorStreaming != 0U))
  {
    /* An active BLE link and all measurement modes keep CPU1 in RUN. PA4
       therefore remains asserted throughout the BLE connection. */
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_SENSORS, UTIL_LPM_DISABLE);
    PWR_SetRunModeLock(1U);
  }
  else
  {
    PWR_SetRunModeLock(0U);
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_SENSORS, UTIL_LPM_ENABLE);
  }
}

static void DT_ProcessBleAudio(uint32_t now)
{
  uint8_t ready_mask = 0U;
  uint8_t restart_needed = 0U;
  HAL_StatusTypeDef analog_status = HAL_OK;
  HAL_StatusTypeDef digital_status = HAL_OK;
  char line[80];

  if ((Custom_App_Context.SensorEnableMask & DT_SENSOR_ENABLE_DIGITAL_MIC) != 0U)
  {
    if (DigitalMicrophone_IsRunning() == 0U)
    {
      restart_needed = 1U;
    }
    else
    {
      DigitalMicrophone_Process();
    }
    if (DigitalMicrophone_HasValue() != 0U)
    {
      ready_mask |= DT_SENSOR_ENABLE_DIGITAL_MIC;
    }
  }

  if ((Custom_App_Context.SensorEnableMask & DT_SENSOR_ENABLE_ANALOG_MIC) != 0U)
  {
    if (AnalogMicrophone_IsRunning() == 0U)
    {
      restart_needed = 1U;
    }
    else
    {
      AnalogMicrophone_Process();
    }
    if (AnalogMicrophone_HasValue() != 0U)
    {
      ready_mask |= DT_SENSOR_ENABLE_ANALOG_MIC;
    }
  }

  /* Publish the first completed RMS window immediately rather than waiting
     for the next one-second status timer. */
  if ((ready_mask & (uint8_t)~Custom_App_Context.SensorAudioReportedMask) != 0U)
  {
    Custom_App_Context.SensorSendPending = 1U;
    if (Custom_App_Context.TxFlowOn != 0U)
    {
      UTIL_SEQ_SetTask(1U << CFG_TASK_DT_SEND_ID, CFG_SCH_PRIO_1);
    }
  }

  if ((restart_needed != 0U) &&
      ((int32_t)(now - Custom_App_Context.AudioRestartNextTick) >= 0))
  {
    Custom_App_Context.AudioRestartNextTick = now + DT_AUDIO_RESTART_PERIOD_MS;
    if (((Custom_App_Context.SensorEnableMask & DT_SENSOR_ENABLE_ANALOG_MIC) != 0U) &&
        (AnalogMicrophone_IsRunning() == 0U))
    {
      AnalogMicrophone_Stop();
      analog_status = AnalogMicrophone_Start();
    }
    if (((Custom_App_Context.SensorEnableMask & DT_SENSOR_ENABLE_DIGITAL_MIC) != 0U) &&
        (DigitalMicrophone_IsRunning() == 0U))
    {
      DigitalMicrophone_Stop();
      digital_status = DigitalMicrophone_Start();
    }
    (void)snprintf(line, sizeof(line),
                   "AUDIO: DMA restart amic=%d dmic=%d\r\n",
                   (int)analog_status, (int)digital_status);
    DT_Log(line);
  }
}

#if (DT_PERIODIC_STREAM_LOGS != 0U)
static void DT_LogAudioHealth(uint32_t now)
{
  char line[144];

  if (((Custom_App_Context.SensorEnableMask &
        (DT_SENSOR_ENABLE_DIGITAL_MIC | DT_SENSOR_ENABLE_ANALOG_MIC)) == 0U) ||
      ((int32_t)(now - Custom_App_Context.AudioHealthNextTick) < 0))
  {
    return;
  }

  Custom_App_Context.AudioHealthNextTick = now + DT_AUDIO_HEALTH_PERIOD_MS;
  (void)snprintf(line, sizeof(line),
                 "AUDIO: amic=%u/%u/%udB drop=%lu dmic=%u/%u/%udB drop=%lu\r\n",
                 (unsigned int)AnalogMicrophone_IsRunning(),
                 (unsigned int)AnalogMicrophone_HasValue(),
                 (unsigned int)AnalogMicrophone_GetLastValue(),
                 (unsigned long)AnalogMicrophone_GetDroppedBlocks(),
                 (unsigned int)DigitalMicrophone_IsRunning(),
                 (unsigned int)DigitalMicrophone_HasValue(),
                 (unsigned int)DigitalMicrophone_GetLastValue(),
                 (unsigned long)DigitalMicrophone_GetDroppedBlocks());
  DT_Log(line);
}
#endif

static void DT_HandleSensorCtrl(const uint8_t *payload, uint8_t length)
{
  uint8_t sensor_mask;
  uint8_t requested_mask;

  if ((payload == 0) || (length == 0U))
  {
    return;
  }

  if (length == 1U)
  {
    /* Keep the original one-byte all-sensor commands backward compatible. */
    switch (payload[0])
    {
      case DT_CMD_STOP:
        DT_ApplySensorEnableMask(0U);
        break;

      case DT_CMD_START:
        DT_ApplySensorEnableMask(DT_SENSOR_ENABLE_ALL);
        break;

      case DT_CMD_TOGGLE:
        DT_ApplySensorEnableMask((Custom_App_Context.SensorEnableMask != 0U) ?
                                 0U : DT_SENSOR_ENABLE_ALL);
        break;

      default:
        break;
    }
    return;
  }

  if ((payload[1] != DT_SENSOR_STATE_OFF) &&
      (payload[1] != DT_SENSOR_STATE_ON))
  {
    return;
  }

  if (payload[0] == DT_SENSOR_ID_ALL)
  {
    requested_mask = (payload[1] == DT_SENSOR_STATE_ON) ? DT_SENSOR_ENABLE_ALL : 0U;
  }
  else
  {
    sensor_mask = DT_SensorIdToMask(payload[0]);
    if (sensor_mask == 0U)
    {
      return;
    }

    requested_mask = Custom_App_Context.SensorEnableMask;
    if (payload[1] == DT_SENSOR_STATE_ON)
    {
      requested_mask |= sensor_mask;
    }
    else
    {
      requested_mask &= (uint8_t)~sensor_mask;
    }
  }

  DT_ApplySensorEnableMask(requested_mask);
}

static void DT_HandleVibCtrl(const uint8_t *payload, uint8_t length)
{
  if ((payload == 0) || (length == 0U))
  {
    return;
  }

  switch (payload[0])
  {
    case DT_CMD_STOP:
      DT_StopVibStream();
      DT_Log("VIB: stream stopped by command\r\n");
      break;

    case DT_CMD_START:
      DT_StartVibStream();
      break;

    case DT_CMD_TOGGLE:
      DT_ToggleVibStream();
      break;

    default:
      break;
  }
}

static void DT_VibDrainTask(void)
{
  if (ZigbeeVibrationWindowActive != 0U)
  {
    DT_ProcessZigbeeVibration(HAL_GetTick());
  }
}

static void DT_SendTask(void)
{
  tBleStatus status = BLE_STATUS_SUCCESS;
  uint16_t payload_offset;
  uint16_t vib_packet_len;
  uint16_t vib_sample_count;
  uint8_t audio_items_included = 0U;
  DT_SourceState_t source_state;

  if ((Custom_App_Context.IsConnected == 0U) ||
      (Custom_App_Context.TxFlowOn == 0U))
  {
    return;
  }

  if ((Custom_App_Context.Sensor_status_Notification_Status != 0U) &&
      (Custom_App_Context.SensorSendPending != 0U))
  {
    /* Low-speed sensor packet v3: seq + version + flags + item_count + TLV items. */
    uint8_t item_count = 0U;
    uint8_t sensor_flags = DT_SENSOR_TLV_FLAGS_SNAPSHOT;
    uint8_t temperature_alert_flags = 0U;
    uint16_t item_count_offset;
    uint16_t tlv_offset;
    uint16_t item_length;
    uint16_t temperature_tlv_offset;
    uint16_t temperature_length;

    source_state.thermometer = Thermometer_SaveState();

    if ((Custom_App_Context.SensorEnableMask & DT_SENSOR_ENABLE_TEMPERATURE) != 0U)
    {
      temperature_alert_flags = Thermometer_GetAlertFlags();
      if ((temperature_alert_flags & THERMOMETER_ALERT_EVENT) != 0U)
      {
        sensor_flags |= DT_SENSOR_TLV_FLAGS_TEMP_EVENT;
      }
      if ((temperature_alert_flags & THERMOMETER_ALERT_THERM) != 0U)
      {
        sensor_flags |= DT_SENSOR_TLV_FLAGS_TEMP_THERM;
      }
    }

    payload_offset = 0U;
    SensorTxBuffer[payload_offset++] = Custom_App_Context.SensorSeq++;
    SensorTxBuffer[payload_offset++] = DT_SENSOR_FRAME_VERSION;
    SensorTxBuffer[payload_offset++] = sensor_flags;
    item_count_offset = payload_offset++;

    /* Report the active sensor set in every frame so the client can resynchronize. */
    SensorTxBuffer[payload_offset++] = DT_SENSOR_TLV_TYPE_ENABLE_MASK;
    SensorTxBuffer[payload_offset++] = DT_SENSOR_ENABLE_MASK_SIZE;
    SensorTxBuffer[payload_offset++] = Custom_App_Context.SensorEnableMask;
    item_count++;

    if ((Custom_App_Context.SensorEnableMask & DT_SENSOR_ENABLE_MAGNETOMETER) != 0U)
    {
      tlv_offset = payload_offset;
      SensorTxBuffer[payload_offset++] = DT_SENSOR_TLV_TYPE_MAGNETOMETER;
      SensorTxBuffer[payload_offset++] = DT_MAGNETOMETER_PAYLOAD_SIZE;
      item_length = Magnetometer_AppendPayload(&SensorTxBuffer[payload_offset],
                                               (uint16_t)(DT_SENSOR_PACKET_SIZE - payload_offset));
      if (item_length == DT_MAGNETOMETER_PAYLOAD_SIZE)
      {
        payload_offset = (uint16_t)(payload_offset + item_length);
        item_count++;
      }
      else
      {
        payload_offset = tlv_offset;
      }
    }

    if ((Custom_App_Context.SensorEnableMask & DT_SENSOR_ENABLE_DIGITAL_MIC) != 0U)
    {
      tlv_offset = payload_offset;
      SensorTxBuffer[payload_offset++] = DT_SENSOR_TLV_TYPE_DIGITAL_MIC;
      SensorTxBuffer[payload_offset++] = DT_DIGITAL_MIC_PAYLOAD_SIZE;
      item_length = DigitalMicrophone_AppendPayload(&SensorTxBuffer[payload_offset],
                                                     (uint16_t)(DT_SENSOR_PACKET_SIZE - payload_offset));
      if (item_length == DT_DIGITAL_MIC_PAYLOAD_SIZE)
      {
        payload_offset = (uint16_t)(payload_offset + item_length);
        item_count++;
        audio_items_included |= DT_SENSOR_ENABLE_DIGITAL_MIC;
      }
      else
      {
        payload_offset = tlv_offset;
      }
    }

    if ((Custom_App_Context.SensorEnableMask & DT_SENSOR_ENABLE_ANALOG_MIC) != 0U)
    {
      tlv_offset = payload_offset;
      SensorTxBuffer[payload_offset++] = DT_SENSOR_TLV_TYPE_ANALOG_MIC;
      SensorTxBuffer[payload_offset++] = DT_ANALOG_MIC_PAYLOAD_SIZE;
      item_length = AnalogMicrophone_AppendPayload(&SensorTxBuffer[payload_offset],
                                                    (uint16_t)(DT_SENSOR_PACKET_SIZE - payload_offset));
      if (item_length == DT_ANALOG_MIC_PAYLOAD_SIZE)
      {
        payload_offset = (uint16_t)(payload_offset + item_length);
        item_count++;
        audio_items_included |= DT_SENSOR_ENABLE_ANALOG_MIC;
      }
      else
      {
        payload_offset = tlv_offset;
      }
    }

    if ((Custom_App_Context.SensorEnableMask & DT_SENSOR_ENABLE_TEMPERATURE) != 0U)
    {
      temperature_tlv_offset = payload_offset;
      SensorTxBuffer[payload_offset++] = DT_SENSOR_TLV_TYPE_TEMPERATURE;
      SensorTxBuffer[payload_offset++] = DT_THERMOMETER_PAYLOAD_SIZE;
      temperature_length = Thermometer_AppendPayload(&SensorTxBuffer[payload_offset],
                                                      (uint16_t)(DT_SENSOR_PACKET_SIZE - payload_offset));
      if (temperature_length == DT_THERMOMETER_PAYLOAD_SIZE)
      {
        payload_offset = (uint16_t)(payload_offset + temperature_length);
        item_count++;
      }
      else
      {
        /* Omit the temperature TLV while the STTS751 is offline. */
        payload_offset = temperature_tlv_offset;
      }
    }

    SensorTxBuffer[item_count_offset] = item_count;

    Custom_App_Context.TxFlowOn = 0U;
    status = Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_SENSOR_STATUS,
                                                         SensorTxBuffer,
                                                         payload_offset);
    if (status == BLE_STATUS_SUCCESS)
    {
      Custom_App_Context.SensorSendPending = 0U;
      Custom_App_Context.SensorAudioReportedMask |= audio_items_included;
    }
    else
    {
      Custom_App_Context.SensorSeq--;
      Thermometer_RestoreState(source_state.thermometer);
    }
  }
  else if ((Custom_App_Context.VibStreaming != 0U) &&
           (Custom_App_Context.Vib_wave_Notification_Status != 0U))
  {
    if (Vibrometer_IsReady() == 0U)
    {
      return;
    }

    if (Custom_App_Context.VibPacketPending != 0U)
    {
      vib_packet_len = Custom_App_Context.VibPendingLength;
      vib_sample_count = VibTxBuffer[3];
    }
    else
    {
      status = DT_VibBuildPacket(&vib_packet_len, &vib_sample_count);
      if (status != HAL_OK)
      {
        Custom_App_Context.VibStatSpiErrors++;
        return;
      }
      if (vib_sample_count == 0U)
      {
        return;
      }

      VibTxBuffer[0] = Custom_App_Context.VibSeq++;
      Custom_App_Context.VibPendingLength = vib_packet_len;
      Custom_App_Context.VibPacketPending = 1U;
    }

    Custom_App_Context.TxFlowOn = 0U;
    status = Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_VIB_WAVE,
                                                         VibTxBuffer,
                                                         vib_packet_len);
    if (status == BLE_STATUS_SUCCESS)
    {
      Custom_App_Context.VibTxBytes += vib_packet_len;
      Custom_App_Context.VibStatPackets++;
      Custom_App_Context.VibStatSamples += vib_sample_count;
      Custom_App_Context.VibStatBytes += vib_packet_len;
      Custom_App_Context.VibPacketPending = 0U;
      Custom_App_Context.VibPendingLength = 0U;
    }
    else if ((status == BLE_STATUS_INSUFFICIENT_RESOURCES) ||
             (status == BLE_STATUS_BUSY))
    {
      Custom_App_Context.VibStatBusy++;
    }
    else
    {
      Custom_App_Context.VibDroppedPackets++;
      Custom_App_Context.VibPacketPending = 0U;
      Custom_App_Context.VibPendingLength = 0U;
    }
  }
  else
  {
    return;
  }

  if (status == BLE_STATUS_SUCCESS)
  {
    Custom_App_Context.TxFlowOn = 1U;
    Custom_App_Context.TxRetryTick = 0U;
    /* The main-loop poll provides bounded pacing for the next vibration packet. */
  }
  else if (status == BLE_STATUS_INSUFFICIENT_RESOURCES)
  {
    Custom_App_Context.TxFlowOn = 0U;
    Custom_App_Context.TxRetryTick = HAL_GetTick() + DT_BLE_POOL_RETRY_PERIOD_MS;
  }
  else if (status == BLE_STATUS_BUSY)
  {
    /* CPU2 is temporarily occupied (commonly by Zigbee). Keep the exact
       pending packet and retry it quickly instead of treating it as lost. */
    Custom_App_Context.TxFlowOn = 0U;
    Custom_App_Context.TxRetryTick = HAL_GetTick() + DT_BLE_BUSY_RETRY_PERIOD_MS;
  }
  else
  {
    Custom_App_Context.TxFlowOn = 1U;
    Custom_App_Context.TxRetryTick = 0U;
  }
}

static void DT_Log(const char *message)
{
  char line[224];
  uint32_t elapsed_ms;
  int length;

  if (message == NULL)
  {
    return;
  }
  elapsed_ms = APP_ElapsedMs();
  length = snprintf(line, sizeof(line), "[+%lu.%03lus] %s",
                    (unsigned long)(elapsed_ms / 1000U),
                    (unsigned long)(elapsed_ms % 1000U), message);
  if (length <= 0)
  {
    return;
  }
  if ((size_t)length >= sizeof(line))
  {
    length = (int)sizeof(line) - 1;
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)line,
                          (uint16_t)length, 100U);
}

#if (DT_PERIODIC_STREAM_LOGS != 0U)
static void DT_VibLogStatistics(uint32_t now)
{
  char line[160];

  if ((Custom_App_Context.VibStreaming == 0U) ||
      ((int32_t)(now - Custom_App_Context.VibStatNextTick) < 0))
  {
    return;
  }

  Custom_App_Context.VibStatNextTick = now + DT_VIB_STAT_PERIOD_MS;
  (void)snprintf(line, sizeof(line),
                 "VIB: pps=%lu sps=%lu Bps=%lu fifo=%u busy=%lu pool=%lu retry=%lu ovr=%lu spi=%lu\r\n",
                 (unsigned long)Custom_App_Context.VibStatPackets,
                 (unsigned long)Custom_App_Context.VibStatSamples,
                 (unsigned long)Custom_App_Context.VibStatBytes,
                 (unsigned int)Custom_App_Context.VibStatMaxFifo,
                 (unsigned long)Custom_App_Context.VibStatBusy,
                 (unsigned long)Custom_App_Context.VibStatPoolAvailable,
                 (unsigned long)Custom_App_Context.VibStatFlowRecoveries,
                 (unsigned long)Custom_App_Context.VibStatFifoOverruns,
                 (unsigned long)Custom_App_Context.VibStatSpiErrors);
  DT_Log(line);

  Custom_App_Context.VibStatPackets = 0U;
  Custom_App_Context.VibStatSamples = 0U;
  Custom_App_Context.VibStatBytes = 0U;
  Custom_App_Context.VibStatBusy = 0U;
  Custom_App_Context.VibStatPoolAvailable = 0U;
  Custom_App_Context.VibStatFlowRecoveries = 0U;
  Custom_App_Context.VibStatSpiErrors = 0U;
  Custom_App_Context.VibStatFifoOverruns = 0U;
  Custom_App_Context.VibStatMaxFifo = 0U;
}
#endif

static HAL_StatusTypeDef DT_VibInitHardware(void)
{
  HAL_StatusTypeDef status;
  uint8_t who_am_i = 0xFFU;
  char log_line[48];

  VibFifoIrqPending = 0U;

  status = Vibrometer_Init(&hspi1, &who_am_i);
  (void)snprintf(log_line, sizeof(log_line), "VIB: WHO=0x%02X status=%d\r\n", who_am_i, (int)status);
  DT_Log(log_line);

  if (status != HAL_OK)
  {
    DT_Log("VIB: device init failed\r\n");
    return status;
  }

  if (Vibrometer_Stop() != HAL_OK)
  {
    DT_Log("VIB: initial stop failed\r\n");
    return HAL_ERROR;
  }
  DT_Log("VIB: device ready, FIFO idle\r\n");
  return HAL_OK;
}

static HAL_StatusTypeDef DT_DrainZigbeeStore(uint16_t max_points)
{
  uint32_t started = HAL_GetTick();
  ZigbeeVibration.diag_write_calls++;
  while ((ZigbeeStoreCount != 0U) && (max_points != 0U))
  {
    uint16_t count = ZigbeeStoreCount;
    uint16_t capacity = ZigbeeVibrationStore_AsyncCapacity();
    if (capacity == 0U) break;
    if (count > capacity) count = capacity;

    if (count > max_points) count = max_points;
    if (count > DT_ZB_STORE_RING_POINTS - ZigbeeStoreTail)
      count = DT_ZB_STORE_RING_POINTS - ZigbeeStoreTail;
    if (ZigbeeVibrationStore_AppendAsync(&ZigbeeStoreRing[ZigbeeStoreTail], count) != HAL_OK)
    {
      ZigbeeVibration.diag_store_ms += HAL_GetTick() - started;
      return HAL_ERROR;
    }

    ZigbeeStoreTail = (ZigbeeStoreTail + count) % DT_ZB_STORE_RING_POINTS;
    ZigbeeStoreCount -= count;
    ZigbeeVibration.diag_drained_bytes += (uint32_t)count * 6U;
    max_points -= count;
  }
  ZigbeeVibration.diag_store_ms += HAL_GetTick() - started;
  return HAL_OK;
}

static void DT_ProcessZigbeeVibration(uint32_t now)
{
  uint16_t fifo_count;
  uint16_t read_count;
  uint16_t accepted_count;
  uint8_t fifo_flags;
  uint32_t service_gap;
  uint32_t service_ms;
  uint32_t stage_started;
  uint32_t capture_elapsed;
  HAL_StatusTypeDef fifo_status;
  uint8_t burst_budget_used = 1U;
  char log_line[240];

  if ((ZigbeeVibrationWindowActive == 0U) || (ZigbeeVibrationServicing != 0U))
    return;
  ZigbeeVibrationServicing = 1U;
  if (ZigbeeVibration.flash_errors == 0U &&
      ZigbeeVibrationStore_Service(0U) == HAL_ERROR) {
    ZigbeeVibration.flash_errors++;
    ZigbeeVibrationStore_Abort();
  }
  if (ZigbeeVibration.flash_errors != 0U) {
    Vibrometer_AbortFifoRead();
    ZigbeeFifoReadPending = 0U;
  }

  /* A sequencer task can service this directly from the watermark event;
     refresh the time instead of using a possibly stale main-loop value. */
  now = HAL_GetTick();
  ZigbeeVibration.diag_calls++;
  service_gap = now - ZigbeeVibrationLastServiceTick;
  ZigbeeVibrationLastServiceTick = now;
  if (service_gap > ZigbeeVibration.max_service_gap_ms)
  {
    ZigbeeVibration.max_service_gap_ms =
        (uint16_t)((service_gap > UINT16_MAX) ? UINT16_MAX : service_gap);
  }

  /* Target samples define the waveform length. A watchdog terminates a
     stalled sensor; a timed-out record is never committed as valid. */
  if (((int32_t)(now - ZigbeeVibrationWindowDeadline) >= 0) &&
      (ZigbeeVibration.sampling_stopped == 0U))
  {
    ZigbeeVibration.timed_out = 1U;
    Vibrometer_AbortFifoRead();
    ZigbeeFifoReadPending = 0U;
    if (Vibrometer_StopSamplingKeepFifo() != HAL_OK)
      ZigbeeVibration.read_errors++;
    ZigbeeVibration.sampling_stopped = 1U;
  }

  for (uint32_t burst = 0U; burst < DT_ZIGBEE_VIB_MAX_BURSTS; burst++)
  {
    if (ZigbeeVibration.sampling_stopped != 0U) break;
    stage_started = HAL_GetTick();
    if (ZigbeeFifoReadPending == 0U) {
      fifo_status = Vibrometer_ReadFifoStatus(&fifo_count, &fifo_flags);
      if (fifo_status != HAL_OK) { ZigbeeVibration.read_errors++; break; }
      if (fifo_count > ZigbeeVibration.peak_fifo_words) ZigbeeVibration.peak_fifo_words = fifo_count;
      if (fifo_flags & VIBROMETER_FIFO_OVERRUN_FLAG) { ZigbeeVibration.overrun_seen = 1U; break; }
      if (fifo_count == 0U) { burst_budget_used = 0U; break; }
      read_count = fifo_count;
      if (read_count > VIBROMETER_FIFO_DMA_MAX_BYTES / DT_VIB_FIFO_WORD_SIZE)
        read_count = VIBROMETER_FIFO_DMA_MAX_BYTES / DT_VIB_FIFO_WORD_SIZE;
      ZigbeeFifoReadPending = read_count;
    }
    read_count = ZigbeeFifoReadPending;
    fifo_status = Vibrometer_ReadFifoDataAsync(VibFifoRawBuffer,
        (uint16_t)(read_count * DT_VIB_FIFO_WORD_SIZE));
    ZigbeeVibration.diag_fifo_ms += HAL_GetTick() - stage_started;
    if (fifo_status == HAL_BUSY) break;
    ZigbeeFifoReadPending = 0U;
    if (fifo_status != HAL_OK) { ZigbeeVibration.read_errors++; break; }

    accepted_count = read_count;
    if (accepted_count >
        (uint16_t)(ZigbeeVibration.target_samples - ZigbeeVibration.sample_count))
    {
      accepted_count = (uint16_t)(ZigbeeVibration.target_samples -
                                  ZigbeeVibration.sample_count);
    }

    /* The FIFO data has already been removed from the sensor. Preserve it in
       the SRAM elasticity ring before scheduling a bounded QSPI write. */
    if (accepted_count > DT_ZB_STORE_RING_POINTS - ZigbeeStoreCount)
    {
      ZigbeeVibration.buffer_overruns++;
      break;
    }

    stage_started = HAL_GetTick();
    for (uint16_t sample = 0U;
         sample < accepted_count;
         sample++)
    {
      const uint8_t *data = &VibFifoRawBuffer[sample * DT_VIB_FIFO_WORD_SIZE];
      int16_t raw[3];

      raw[0] = (int16_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8U));
      raw[1] = (int16_t)((uint16_t)data[3] | ((uint16_t)data[4] << 8U));
      raw[2] = (int16_t)((uint16_t)data[5] | ((uint16_t)data[6] << 8U));
      ZigbeeStoreSamples[sample][0] = raw[0];
      ZigbeeStoreSamples[sample][1] = raw[1];
      ZigbeeStoreSamples[sample][2] = raw[2];
      ZigbeeVibration.sample_count++;
    }
    for (uint16_t i = 0U; i < accepted_count; ++i)
    {
      memcpy(ZigbeeStoreRing[ZigbeeStoreHead], ZigbeeStoreSamples[i], 6U);
      ZigbeeStoreHead = (ZigbeeStoreHead + 1U) % DT_ZB_STORE_RING_POINTS;
    }
    ZigbeeStoreCount += accepted_count;
    ZigbeeVibration.diag_pack_ms += HAL_GetTick() - stage_started;
    if (ZigbeeStoreCount > ZigbeeVibration.diag_ring_peak)
      ZigbeeVibration.diag_ring_peak = ZigbeeStoreCount;

    if (ZigbeeVibration.sample_count == ZigbeeVibration.target_samples)
    {
      if (Vibrometer_StopSamplingKeepFifo() != HAL_OK)
        ZigbeeVibration.read_errors++;
      ZigbeeVibration.sampling_stopped = 1U;
      break;
    }
  }

  if ((ZigbeeVibration.sampling_stopped == 0U) &&
      (ZigbeeVibration.read_errors == 0U) &&
      (ZigbeeVibration.overrun_seen == 0U) &&
      (ZigbeeVibration.buffer_overruns == 0U) &&
      (ZigbeeVibration.timed_out == 0U) &&
      (ZigbeeStoreCount >= DT_ZB_STORE_WRITE_POINTS))
  {
    /* QSPI TX and internal NOR programming proceed independently of SPI.
       Drain as much as the two page buffers can accept without waiting. */
    if (DT_DrainZigbeeStore(DT_ZB_STORE_WRITE_POINTS) != HAL_OK)
      ZigbeeVibration.flash_errors++;

  }

  if ((ZigbeeVibration.sampling_stopped != 0U) &&
      (ZigbeeVibration.read_errors == 0U) &&
      (ZigbeeVibration.overrun_seen == 0U) &&
      (ZigbeeVibration.buffer_overruns == 0U) &&
      (ZigbeeVibration.timed_out == 0U) && (ZigbeeStoreCount != 0U))
  {
    if (DT_DrainZigbeeStore(DT_ZB_STORE_WRITE_POINTS) != HAL_OK)
      ZigbeeVibration.flash_errors++;
  }
  if ((ZigbeeVibration.read_errors != 0U) ||
      (ZigbeeVibration.overrun_seen != 0U) ||
      (ZigbeeVibration.buffer_overruns != 0U) ||
      (ZigbeeVibration.flash_errors != 0U) || (ZigbeeVibration.timed_out != 0U))
  {
    Vibrometer_AbortFifoRead();
    ZigbeeFifoReadPending = 0U;
    (void)Vibrometer_Stop();
    ZigbeeVibration.sampling_stopped = 1U;
    ZigbeeStoreCount = 0U;
  }

  service_ms = HAL_GetTick() - now;
  ZigbeeVibration.diag_service_ms += service_ms;
  if (service_ms > ZigbeeVibration.max_service_ms)
  {
    ZigbeeVibration.max_service_ms =
        (uint16_t)((service_ms > UINT16_MAX) ? UINT16_MAX : service_ms);
  }

  if (ZigbeeVibration.sampling_stopped != 0U)
  {
    HAL_StatusTypeDef flush_status = ZigbeeVibrationStore_Service(ZigbeeStoreCount == 0U);
    if (flush_status == HAL_ERROR) {
      ZigbeeVibration.flash_errors++;
      ZigbeeVibrationStore_Abort();
      flush_status = HAL_OK;
    }
    if (ZigbeeStoreCount == 0U && flush_status == HAL_OK)
    {
      capture_elapsed = HAL_GetTick() - ZigbeeVibration.diag_start_ms;
      (void)Vibrometer_Stop();
      ZigbeeVibrationWindowActive = 0U;
      ZigbeeVibration.complete = (ZigbeeVibration.sample_count != 0U) ? 1U : 0U;
      (void)snprintf(log_line, sizeof(log_line),
                     "VIB: capture samples=%lu target=%lu overrun=%u peak_fifo=%u spi_err=%u flash_err=%u buffer_full=%u timeout=%u gap=%u ms service=%u ms\r\n",
                     (unsigned long)ZigbeeVibration.sample_count,
                     (unsigned long)ZigbeeVibration.target_samples,
                     (unsigned int)ZigbeeVibration.overrun_seen,
                     (unsigned int)ZigbeeVibration.peak_fifo_words,
                     (unsigned int)ZigbeeVibration.read_errors,
                     (unsigned int)ZigbeeVibration.flash_errors,
                     (unsigned int)ZigbeeVibration.buffer_overruns,
                     (unsigned int)ZigbeeVibration.timed_out,
                     (unsigned int)ZigbeeVibration.max_service_gap_ms,
                     (unsigned int)ZigbeeVibration.max_service_ms);
      DT_Log(log_line);
      /* Millisecond buckets are approximate and include interrupt preemption.
         drained_bytes includes bytes staged in the page buffer, not just NOR. */
      (void)snprintf(log_line, sizeof(log_line),
                     "VIB-DIAG: async-v2 elapsed=%lu service=%lu fifo=%lu pack=%lu store=%lu ms calls=%lu writes=%lu deferred=%lu ring_peak=%u drained_bytes=%lu\r\n",
                     (unsigned long)capture_elapsed,
                     (unsigned long)ZigbeeVibration.diag_service_ms,
                     (unsigned long)ZigbeeVibration.diag_fifo_ms,
                     (unsigned long)ZigbeeVibration.diag_pack_ms,
                     (unsigned long)ZigbeeVibration.diag_store_ms,
                     (unsigned long)ZigbeeVibration.diag_calls,
                     (unsigned long)ZigbeeVibration.diag_write_calls,
                     (unsigned long)ZigbeeVibration.diag_write_deferred,
                     (unsigned int)ZigbeeVibration.diag_ring_peak,
                     (unsigned long)ZigbeeVibration.diag_drained_bytes);
      DT_Log(log_line);
      DT_UpdateLowPowerLock();
    }
    else
    {
      /* Sampling has stopped; flush the final partial block before committing
         the QSPI record. */
      UTIL_SEQ_SetTask(1U << CFG_TASK_DT_VIB_DRAIN_ID, CFG_SCH_PRIO_0);
    }
  }
  else if ((burst_budget_used != 0U) || (ZigbeeStoreCount != 0U))
  {
    /* A bounded service keeps BLE/IPCC/Zigbee tasks responsive. If the FIFO
       remained above its watermark, its IRQ line will stay high and produce
       no new edge, so explicitly queue another drain task. */
    if ((ZigbeeFifoReadPending != 0U) ||
        (ZigbeeStoreCount >= DT_ZB_STORE_WRITE_POINTS) ||
        ((Vibrometer_ReadFifoStatus(&fifo_count, &fifo_flags) == HAL_OK) &&
         (fifo_count >= 128U)))
    {
      UTIL_SEQ_SetTask(1U << CFG_TASK_DT_VIB_DRAIN_ID, CFG_SCH_PRIO_0);
    }
  }
  ZigbeeVibrationServicing = 0U;
}

static void DT_AbortZigbeeSnapshot(void)
{
  /* No Zigbee producer owns the sensors during an established BLE stream. */
  if ((ZigbeeAudioWindowActive == 0U) &&
      (ZigbeeVibrationWindowActive == 0U) && (ZigbeeVibrationStoreActive == 0U) &&
      ((Custom_App_Context.SensorStreaming != 0U) || (Custom_App_Context.VibStreaming != 0U)))
    return;
  Vibrometer_AbortFifoRead();
  ZigbeeFifoReadPending = 0U;
  /* Task context only. Stop producers before resetting their storage. */
  ZigbeeAudioWindowActive = 0U;
  ZigbeeVibrationWindowActive = 0U;
  DigitalMicrophone_Stop();
  AnalogMicrophone_Stop();
  if (Vibrometer_IsReady() != 0U) (void)Vibrometer_Stop();
  ZigbeeDigitalMicStarted = ZigbeeAnalogMicStarted = 0U;
  ZigbeeVibrationStoreActive = 0U;
  ZigbeeStoreHead = ZigbeeStoreTail = ZigbeeStoreCount = 0U;
  ZigbeeSnapshotBatchId = 0U;
  memset(&ZigbeeVibration, 0, sizeof(ZigbeeVibration));
  ZigbeeVibrationStore_Abort();
  /* Preserve BLE's original fixed 500 ms microphone integration windows. */
  (void)DigitalMicrophone_ConfigureWindow(500U);
  (void)AnalogMicrophone_ConfigureWindow(500U);
  DT_UpdateLowPowerLock();
}



static HAL_StatusTypeDef DT_VibBuildPacket(uint16_t *packet_len, uint16_t *sample_count)
{
  uint16_t fifo_count = 0U;
  uint16_t raw_remaining;
  uint16_t raw_batch_count;
  uint16_t offset;
  uint16_t i;
  const uint8_t *data;
  uint8_t flags = DT_VIB_FLAGS_DECIMATION_DIV4;
  uint8_t fifo_status_flags = 0U;
  HAL_StatusTypeDef status;

  if ((packet_len == 0) || (sample_count == 0))
  {
    return HAL_ERROR;
  }

  *packet_len = 0U;
  *sample_count = 0U;

  if (VibFifoIrqPending != 0U)
  {
    flags |= DT_VIB_FLAGS_FIFO_WATERMARK;
  }
  else
  {
    flags |= DT_VIB_FLAGS_POLLED_READ;
  }
  VibFifoIrqPending = 0U;

  status = Vibrometer_ReadFifoStatus(&fifo_count, &fifo_status_flags);
  if (status != HAL_OK)
  {
    return status;
  }
  if (fifo_count > Custom_App_Context.VibStatMaxFifo)
  {
    Custom_App_Context.VibStatMaxFifo = fifo_count;
  }
  if ((fifo_status_flags & VIBROMETER_FIFO_OVERRUN_FLAG) != 0U)
  {
    Custom_App_Context.VibStatFifoOverruns++;
  }
  if (fifo_count == 0U)
  {
    return HAL_OK;
  }

  raw_remaining = fifo_count;
  if (raw_remaining > DT_VIB_RAW_SAMPLES_PER_PACKET)
  {
    raw_remaining = DT_VIB_RAW_SAMPLES_PER_PACKET;
    flags |= DT_VIB_FLAGS_FIFO_MAY_HAVE_MORE;
  }

  (void)memset(VibTxBuffer, 0, sizeof(VibTxBuffer));
  VibTxBuffer[1] = DT_VIB_FRAME_VERSION;
  offset = DT_VIB_PACKET_HEADER_SIZE;

  /*
   * IIS3DWB always samples at 26.667 kHz, which is faster than a phone can
   * consume as individual GATT notifications.  Drain every FIFO word here,
   * but retain one uniformly-spaced sample out of four for the BLE live
   * preview.  This avoids FIFO saturation and keeps the preview time axis
   * well-defined instead of dropping arbitrary whole packets.
   */
  while (raw_remaining > 0U)
  {
    raw_batch_count = raw_remaining;
    if (raw_batch_count > DT_VIB_MAX_SAMPLES_PER_PACKET)
    {
      raw_batch_count = DT_VIB_MAX_SAMPLES_PER_PACKET;
    }

    status = Vibrometer_ReadFifoData(VibFifoRawBuffer,
                                      (uint16_t)(raw_batch_count * DT_VIB_FIFO_WORD_SIZE));
    if (status != HAL_OK)
    {
      return status;
    }

    for (i = 0U; i < raw_batch_count; i++)
    {
      if ((Custom_App_Context.VibDecimationPhase == 0U) &&
          (*sample_count < DT_VIB_MAX_SAMPLES_PER_PACKET))
      {
        data = &VibFifoRawBuffer[i * DT_VIB_FIFO_WORD_SIZE];
        VibTxBuffer[offset++] = data[1];
        VibTxBuffer[offset++] = data[2];
        VibTxBuffer[offset++] = data[3];
        VibTxBuffer[offset++] = data[4];
        VibTxBuffer[offset++] = data[5];
        VibTxBuffer[offset++] = data[6];
        (*sample_count)++;
      }

      Custom_App_Context.VibDecimationPhase++;
      if (Custom_App_Context.VibDecimationPhase >= DT_VIB_BLE_DECIMATION)
      {
        Custom_App_Context.VibDecimationPhase = 0U;
      }
    }
    raw_remaining = (uint16_t)(raw_remaining - raw_batch_count);
  }

  VibTxBuffer[2] = flags;
  VibTxBuffer[3] = (uint8_t)(*sample_count);

  *packet_len = (uint16_t)(DT_VIB_PACKET_HEADER_SIZE +
                           ((*sample_count) * DT_VIB_SAMPLE_PAYLOAD_SIZE));
  return HAL_OK;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/

void Custom_APP_ZigbeeAbortSnapshot(void)
{
  DT_AbortZigbeeSnapshot();
}
