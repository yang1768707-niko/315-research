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
#include "scalar_delivery.h"
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
#include "comm_protocol.h"
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
#define DT_ZB_STORE_REFILL_POINTS       ((QSPI_NOR_PAGE_SIZE + DT_VIB_SAMPLE_PAYLOAD_SIZE - 1U) / DT_VIB_SAMPLE_PAYLOAD_SIZE)
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
#define DT_CMD_WORK_MODE_QUERY          0xF0U
#define DT_CMD_WORK_MODE_SET            0xF1U
#define DT_WORK_MODE_RESULT_OK          COMM_OK
#define DT_WORK_MODE_RESULT_BAD_VALUE   COMM_BAD_VALUE
#define DT_WORK_MODE_RESULT_SAVE_FAILED COMM_SAVE_FAILED
#define DT_WORK_MODE_BACKUP_REGISTER    RTC_BKP_DR19
#define DT_WORK_MODE_BACKUP_MAGIC       0x57423500UL
#define DT_WORK_MODE_BACKUP_MASK        0xFFFFFF00UL
#define DT_WORK_MODE_REPLY_RETRY_MS     20U
#define DT_WORK_MODE_REPLY_ATTEMPTS     20U
#define DT_WORK_MODE_RESTART_DELAY_MS   1500U
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
/* Unified BLE preview uses the existing sensor drivers. */
static uint8_t CommStreamMode, CommBatchPending, CommDraining, CommDrainStatus;
static uint32_t CommBleSession, CommBatchId, CommRawIndex;
static uint8_t CommMtuPending;
static uint32_t CommMtuRetry;
static uint16_t CommBleCapacity = 20U, CommFactor = COMM_DEFAULT_DECIMATION;
static uint16_t CommActiveFactor = COMM_DEFAULT_DECIMATION, CommPhase;
static uint32_t CommWindows[5], CommDeadlines[5];
static uint16_t CommTimedMask, CommStatusMask;
static uint16_t CommStatusByCode[5];
static CommScalars CommFinalScalars;
static uint8_t CommFinalPending;
static uint16_t CommFrameLength;
static uint8_t CommFrame[COMM_FRAME_MAX];
static uint8_t CommFrameCode;
static uint32_t CommRetryAt;
static int16_t CommSamples[40][3];
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
static DT_ZigbeeVibrationAccumulator_t ZigbeeVibration;
static int16_t ZigbeeStoreRing[DT_ZB_STORE_RING_POINTS][3];
static uint16_t ZigbeeStoreHead, ZigbeeStoreTail, ZigbeeStoreCount;
static uint16_t ZigbeeFifoReadPending;
static uint8_t ZigbeeVibrationServicing;
static uint32_t ZigbeeCaptureWriteMs, ZigbeeCaptureCommittedBytes;

/* The selected BLE policy is applied after reset, not in an active link.
   RTC BKP19 is reserved for this mode, separate from Zigbee NVM. */
static uint8_t DT_ActiveWorkMode;
static uint8_t DT_ConfiguredWorkMode;
static uint8_t DT_WorkModeResult;
static uint8_t DT_WorkModeSendPending, DT_WorkModeReadPending;
static uint8_t DT_WorkModeSendAttempts, DT_WorkModeReadAttempts;
static uint32_t DT_WorkModeReplyTick;
static uint8_t DT_ModeRestartOnNotify;
static uint8_t DT_WorkModeTimerId, DT_WorkModeTimerReady;
static uint8_t DT_ModeRestartPending, DT_ModeRestartScheduled;
static uint32_t DT_ModeRestartTick;
extern RTC_HandleTypeDef hrtc;

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
static void DT_LoadWorkMode(void);
static void DT_SaveWorkMode(uint8_t mode);
static void DT_QueueWorkModeStatus(void);
static void DT_ProcessWorkMode(uint32_t now);
static void DT_ScheduleWorkMode(void);
static void DT_ServiceWorkModeTask(void);
static void DT_ConfirmModeRestart(uint8_t configured);
static CommResult DT_CommWorkMode(const CommRequest *request, uint8_t *active, uint8_t *configured);
static uint16_t DT_CommActiveMask(void);
static CommResult DT_CommExecute(const CommRequest *request, uint16_t *active);
static CommResult DT_CommBleSend(const CommSource *source, const uint8_t *frame, uint16_t length);
static void DT_CommSendTask(void);
static void DT_CommPoll(uint32_t now);
static uint8_t DT_CommReady(uint8_t vibration);
static void DT_CommStop(uint16_t mask);
static void DT_CommCollectScalars(CommScalars *values);
static void DT_CommQueueStatus(uint8_t status, uint16_t mask);
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
      DT_QueueWorkModeStatus();
      /* USER CODE END CUSTOM_STM_SENSOR_STATUS_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SENSOR_STATUS_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SENSOR_STATUS_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Sensor_status_Notification_Status = 0U;
      DT_WorkModeSendPending = 0U;
      Custom_App_Context.SensorEnableMask = 0U;
      DT_StopSensorStream();
      if (CommStreamMode)
      {
        DT_StopVibStream();
        CommStreamMode = 0U;
        CommDraining = 0U;
        CommTimedMask = 0U;
        CommFrameLength = 0U;
        CommBatchPending = 0U;
        CommFinalPending = 0U;
        CommStatusMask = 0U;
        memset(CommStatusByCode, 0, sizeof(CommStatusByCode));
      }
      /* USER CODE END CUSTOM_STM_SENSOR_STATUS_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SENSOR_CTRL_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SENSOR_CTRL_READ_EVT */
      DT_WorkModeReadPending = 1U;
      DT_ScheduleWorkMode();
      /* USER CODE END CUSTOM_STM_SENSOR_CTRL_READ_EVT */
      break;

    case CUSTOM_STM_SENSOR_CTRL_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SENSOR_CTRL_WRITE_NO_RESP_EVT */
      if (pNotification->DataTransfered.Length >= COMM_HEADER_SIZE) {
        CommSource source={COMM_LINK_BLE,CommBleSession};
        (void)Comm_Receive(&source,pNotification->DataTransfered.pPayload,pNotification->DataTransfered.Length);
      } else DT_HandleSensorCtrl(pNotification->DataTransfered.pPayload, pNotification->DataTransfered.Length);
      /* GATT writes replace FE82's stored bytes; restore its readable status
         from normal main-loop context, never issue nested ACI commands here. */
      DT_WorkModeReadPending = 1U;
      DT_WorkModeReadAttempts = 0U;
      DT_ScheduleWorkMode();
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
      if (CommStreamMode)
      {
        CommDraining = 0U;
        CommTimedMask &= (uint16_t)~COMM_SENSOR_VIBRATION;
        if (CommFrameCode == COMM_DATA_WAVE)
          CommFrameLength = 0U;
        DT_CommQueueStatus(COMM_STATUS_STOPPED, COMM_SENSOR_VIBRATION);
      }
      /* USER CODE END CUSTOM_STM_VIB_WAVE_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_VIB_CTRL_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VIB_CTRL_READ_EVT */
      /* USER CODE END CUSTOM_STM_VIB_CTRL_READ_EVT */
      break;

    case CUSTOM_STM_VIB_CTRL_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VIB_CTRL_WRITE_NO_RESP_EVT */
      if (pNotification->DataTransfered.Length >= COMM_HEADER_SIZE) {
        CommSource source={COMM_LINK_BLE,CommBleSession};
        (void)Comm_Receive(&source,pNotification->DataTransfered.pPayload,pNotification->DataTransfered.Length);
        DT_ScheduleWorkMode();
      } else DT_HandleVibCtrl(pNotification->DataTransfered.pPayload, pNotification->DataTransfered.Length);
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
      if (CommStreamMode) {
        DT_CommStop(COMM_SENSOR_ALL);DT_StopVibStream();DT_StopSensorStream();
      }
      CommStreamMode=0;CommBleCapacity=20U;
      CommTimedMask=0;CommFrameLength=0;CommStatusMask=0;CommBatchPending=0;
      memset(CommStatusByCode,0,sizeof(CommStatusByCode));CommFinalPending=0;CommDraining=0;
      Comm_SetSession(COMM_LINK_BLE,++CommBleSession);
      CommMtuPending=1U;CommMtuRetry=HAL_GetTick();
      Custom_App_Context.IsConnected = 1U;
      DT_ModeRestartOnNotify = 0U;
      DT_WorkModeSendPending = 0U;
      DT_WorkModeReadPending = 1U;
      DT_WorkModeReadAttempts = 0U;
      DT_WorkModeReplyTick = 0U;
      DT_ScheduleWorkMode();
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
      Comm_SetSession(COMM_LINK_BLE,++CommBleSession);
      CommTimedMask=0;CommFrameLength=0;CommStatusMask=0;CommBatchPending=0;
      memset(CommStatusByCode,0,sizeof(CommStatusByCode));CommFinalPending=0;CommDraining=0;
      CommStreamMode=0;
      Custom_App_Context.IsConnected = 0U;
      DT_WorkModeSendPending = DT_WorkModeReadPending = 0U;
      if (DT_WorkModeTimerReady != 0U && DT_ModeRestartScheduled == 0U)
        HW_TS_Stop(DT_WorkModeTimerId);
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

  DT_LoadWorkMode();
  Comm_Init(DT_CommExecute);
  Comm_RegisterWorkModeHandler(DT_CommWorkMode);
  Comm_RegisterTransport(COMM_LINK_BLE,DT_CommBleSend);
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
    DT_Log("VIB-FLASH: quad read=1-1-4 double-buffer=2x256 local-merge=20261127\r\n");
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
  DT_WorkModeTimerReady =
      HW_TS_Create(CFG_TIM_PROC_ID_ISR, &DT_WorkModeTimerId, hw_ts_SingleShot,
                   DT_ScheduleWorkMode) == hw_ts_Successful;
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
  if (DT_WorkModeTimerReady == 0U)
    DT_ProcessWorkMode(now);
  if(CommMtuPending && Custom_App_Context.IsConnected && (int32_t)(now-CommMtuRetry)>=0) {
    /* Request MTU ourselves so ACI_ATT_EXCHANGE_MTU_RESP always provides the
     * peer's receive limit, including phones that initiate their own exchange. */
    tBleStatus mtu_status=aci_gatt_exchange_config(Custom_App_Context.ConnectionHandle);
    if(mtu_status==BLE_STATUS_SUCCESS)CommMtuPending=0U;
    else CommMtuRetry=now+1000U;
  }
  if (Comm_HasPending())
    DT_ScheduleWorkMode();
  DT_CommPoll(now);

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

static uint8_t stop2_i2c_parked;
static uint8_t stop2_spi_parked;

static void DT_ParkSensorBuses(void)
{
  GPIO_InitTypeDef gpio = { .Mode = GPIO_MODE_ANALOG, .Pull = GPIO_NOPULL };

  if (HAL_I2C_GetState(&hi2c3) == HAL_I2C_STATE_READY &&
      __HAL_I2C_GET_FLAG(&hi2c3, I2C_FLAG_BUSY) == RESET)
  {
    gpio.Pin = I2C3_SCL_Pin | I2C3_SDA_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);
    __HAL_RCC_I2C3_CLK_DISABLE();
    stop2_i2c_parked = 1U;
  }

  if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY &&
      __HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) == RESET)
  {
    HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(GPIOB, (uint16_t)gpio.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &gpio);
    __HAL_RCC_SPI1_CLK_DISABLE();
    stop2_spi_parked = 1U;
  }
}

static void DT_RestoreSensorBuses(void)
{
  /* Keep the initialized peripheral registers and DMA bindings across STOP2. */
  GPIO_InitTypeDef gpio = { .Pull = GPIO_NOPULL };
  if (stop2_i2c_parked != 0U)
  {
    __HAL_RCC_I2C3_CLK_ENABLE();
    gpio.Pin = I2C3_SCL_Pin | I2C3_SDA_Pin;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOB, &gpio);
    stop2_i2c_parked = 0U;
  }
  if (stop2_spi_parked != 0U)
  {
    __HAL_RCC_SPI1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &gpio);
    stop2_spi_parked = 0U;
  }
}

void QspiNor_ProgramEvent(void)
{
  if (ZigbeeVibrationWindowActive != 0U)
    UTIL_SEQ_SetTask(1U << CFG_TASK_DT_VIB_DRAIN_ID, CFG_SCH_PRIO_0);
}

uint8_t Custom_APP_PrepareStop2(void)
{
  /* STOP2 is only selected when DT_UpdateLowPowerLock() confirms that no
     acquisition pipeline is active. Make every external device state
     explicit before the core clock and peripheral clocks are stopped. */
  DigitalMicrophone_PrepareStop2();
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
  DT_ParkSensorBuses();

  static uint8_t flash_sleep_error_reported;
  HAL_StatusTypeDef flash_sleep = QspiNor_PrepareStop2();
  if (flash_sleep == HAL_OK)
  {
    flash_sleep_error_reported = 0U;
  }
  else if (flash_sleep != HAL_BUSY && flash_sleep_error_reported == 0U)
  {
    flash_sleep_error_reported = 1U;
    DT_Log("VIB-FLASH: deep power-down preparation failed\r\n");
  }

  if (flash_sleep != HAL_OK) return 0U;

  /* PB0/PB1 network LEDs are active-low, so SET turns both LEDs off. */
  HAL_GPIO_WritePin(LED_NET_BLUE_GPIO_Port,
                    LED_NET_BLUE_Pin | LED_NET_RED_Pin,
                    GPIO_PIN_SET);
  return 1U;
}

void Custom_APP_ResumeFromStop2(void)
{
  DT_RestoreSensorBuses();
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
    status = ZigbeeVibrationStore_Begin(DT_ZIGBEE_VIBRATION_RATE_HZ,
                                         window_ms, ZigbeeSnapshotBatchId);
    ZigbeeVibrationOverwrite_t overwritten;
    if (ZigbeeVibrationStore_TakeOverwrite(&overwritten))
    {
      char overwrite_log[160];
      (void)snprintf(overwrite_log, sizeof(overwrite_log),
                     "VIB-FLASH: cache full; discarded oldest pending batch=%lu for new batch=%lu slot=%u; active upload protected\r\n",
                     (unsigned long)overwritten.discarded_batch,
                     (unsigned long)overwritten.replacement_batch, overwritten.slot);
      DT_Log(overwrite_log);
    }
    if (status != HAL_OK)
    {
      if (ZigbeeVibrationStore_IsReady() == 0U)
      {
        DT_Log("VIB-FLASH: store unavailable; Zigbee vibration capture cannot start\r\n");
      }
      else
      {
        DT_Log(status == HAL_BUSY ?
            "VIB-FLASH: no reclaimable slot (upload protected or quarantined)\r\n" :
            "VIB-FLASH: slot preparation failed; check Flash erase/write/read\r\n");
      }
      return HAL_ERROR;
    }
    ZigbeeVibrationStoreActive = 1U;
    ZigbeeStoreHead = ZigbeeStoreTail = ZigbeeStoreCount = 0U;
    ZigbeeFifoReadPending = 0U;
    ZigbeeCaptureWriteMs = ZigbeeCaptureCommittedBytes = 0U;
    {
      char log_line[112];
      (void)snprintf(log_line, sizeof(log_line),
                     "VIB: prepare capture window=%u ms target=%lu ring=%u points\r\n",
                     (unsigned int)window_ms,
                     (unsigned long)target_samples,
                     (unsigned int)DT_ZB_STORE_RING_POINTS);
      DT_Log(log_line);
    }
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
  snapshot->scalar_required_mask=scalar_expected_mask(ZigbeeSensorEnableMask);
  snapshot->scalar_policy_version=SCALAR_POLICY_VERSION;
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
Custom_APP_WorkMode_t Custom_APP_GetWorkMode(void)
{
  return (Custom_APP_WorkMode_t)DT_ActiveWorkMode;
}

Custom_APP_WorkMode_t Custom_APP_GetConfiguredWorkMode(void)
{
  return (Custom_APP_WorkMode_t)DT_ConfiguredWorkMode;
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
  if (DT_CommReady(0U) == 0U)
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

  if (DT_CommReady(0U) == 0U)
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

  if (DT_CommReady(1U) == 0U)
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
  DT_Log("VIB: stream armed\r\n");
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
  if ((Custom_App_Context.SensorStreaming != 0U) && DT_CommReady(0U))
  {
    Custom_App_Context.SensorSendPending = 1U;
    UTIL_SEQ_SetTask(1U << CFG_TASK_DT_SEND_ID, CFG_SCH_PRIO_1);
  }
}

static void DT_UpdateLowPowerLock(void)
{
  if ((Custom_App_Context.IsConnected != 0U) ||
      (DT_ModeRestartScheduled != 0U) ||
      (Comm_HasPending() != 0U) ||
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

static void DT_LoadWorkMode(void)
{
  uint32_t stored = HAL_RTCEx_BKUPRead(&hrtc, DT_WORK_MODE_BACKUP_REGISTER);
  uint8_t mode = (uint8_t)stored;
  char line[80];
  if (((stored & DT_WORK_MODE_BACKUP_MASK) != DT_WORK_MODE_BACKUP_MAGIC) ||
      (mode > CUSTOM_APP_WORK_MODE_INSPECTION))
  {
    mode = CUSTOM_APP_WORK_MODE_ONLINE;
  }
  DT_ActiveWorkMode = DT_ConfiguredWorkMode = mode;
  DT_ModeRestartPending = DT_ModeRestartScheduled = 0U;
  DT_WorkModeResult = DT_WORK_MODE_RESULT_OK;
  DT_ModeRestartOnNotify = 0U;
  DT_WorkModeSendPending = DT_WorkModeReadPending = 0U;
  DT_WorkModeSendAttempts = DT_WorkModeReadAttempts = 0U;
  DT_WorkModeReplyTick = 0U;
  (void)snprintf(line, sizeof(line), "MODE: boot active=%u saved=%u (BLE policy)\r\n",
                 (unsigned int)mode, (unsigned int)mode);
  DT_Log(line);
}

static void DT_SaveWorkMode(uint8_t mode)
{
  uint32_t stored = DT_WORK_MODE_BACKUP_MAGIC | (uint32_t)mode;
  uint32_t was_writable = PWR->CR1 & PWR_CR1_DBP;
  DT_ModeRestartPending = DT_ModeRestartScheduled = 0U;
  DT_ModeRestartOnNotify = 0U;
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, DT_WORK_MODE_BACKUP_REGISTER, stored);
  if (HAL_RTCEx_BKUPRead(&hrtc, DT_WORK_MODE_BACKUP_REGISTER) == stored)
  {
    DT_ConfiguredWorkMode = mode;
    DT_WorkModeResult = DT_WORK_MODE_RESULT_OK;
    DT_Log((mode == CUSTOM_APP_WORK_MODE_INSPECTION) ?
           "MODE: inspection saved; applies after reset\r\n" :
           "MODE: online saved; applies after reset\r\n");
  }
  else
  {
    DT_WorkModeResult = DT_WORK_MODE_RESULT_SAVE_FAILED;
    DT_Log("MODE: backup save/verify failed\r\n");
  }
  if (was_writable == 0U) HAL_PWR_DisableBkUpAccess();
}

static void DT_QueueWorkModeStatus(void)
{
  DT_WorkModeReadPending = 1U;
  DT_WorkModeReadAttempts = 0U;
  DT_WorkModeSendPending = 1U;
  DT_WorkModeSendAttempts = 0U;
  DT_WorkModeReplyTick = 0U;
  DT_ScheduleWorkMode();
}

/* 模式指令在公共队列中执行，保存后仅改变 configured，复位才应用广播策略。 */
static CommResult DT_CommWorkMode(const CommRequest *request,uint8_t *active,uint8_t *configured)
{
  if(request->code==COMM_CMD_WORK_MODE_SET) DT_SaveWorkMode(request->mode);
  else DT_WorkModeResult=DT_WORK_MODE_RESULT_OK;
  if (request->code == COMM_CMD_WORK_MODE_SET &&
      request->source.link == COMM_LINK_BLE && DT_WorkModeResult == COMM_OK)
    DT_ModeRestartPending = DT_ConfiguredWorkMode != DT_ActiveWorkMode;
  *active=DT_ActiveWorkMode;*configured=DT_ConfiguredWorkMode;
  /* FE82 的可读快照延后更新；指令响应由 Comm_Process 按来源和请求序号返回。 */
  DT_WorkModeReadPending=1U;DT_WorkModeReadAttempts=0U;DT_WorkModeReplyTick=0U;
  if (Custom_App_Context.IsConnected != 0U)
    DT_ScheduleWorkMode();
  return (CommResult)DT_WorkModeResult;
}

/* FE82 可读快照与 FE81 主动通知均使用统一协议，不再发送旧 v3 模式 TLV。 */
static void DT_ProcessWorkMode(uint32_t now)
{
  tBleStatus status;
  CommResult sent;
  char line[104];
  uint8_t payload[3],read_value[COMM_HEADER_SIZE+3U];
  if (DT_ModeRestartScheduled != 0U && (int32_t)(now - DT_ModeRestartTick) >= 0)
  {
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != 0U ||
        __HAL_FLASH_GET_FLAG(FLASH_FLAG_CFGBSY) != 0U) return;
    DT_ModeRestartPending = DT_ModeRestartScheduled = 0U;
    DT_Log("MODE: restarting to apply saved mode\r\n");
    NVIC_SystemReset();
    return;
  }
  if ((Custom_App_Context.IsConnected == 0U) ||
      ((DT_WorkModeSendPending == 0U) && (DT_WorkModeReadPending == 0U)) ||
      ((DT_WorkModeReplyTick != 0U) && ((int32_t)(now - DT_WorkModeReplyTick) < 0)))
    return;
  DT_WorkModeReplyTick = now + DT_WORK_MODE_REPLY_RETRY_MS;
  payload[0]=DT_WorkModeResult;payload[1]=DT_ActiveWorkMode;payload[2]=DT_ConfiguredWorkMode;
  if (DT_WorkModeReadPending != 0U)
  {
    /* 可读属性是状态快照，序号固定为 0；不能作为命令响应使用。 */
    uint16_t length = Comm_EncodeFrame(read_value, sizeof(read_value), COMM_KIND_EVENT,
                                       COMM_DATA_WORK_MODE, 0U, payload, sizeof(payload));
    status = Custom_STM_App_Update_Char_Variable_Length(
        CUSTOM_STM_SENSOR_CTRL, read_value, (uint8_t)length);
    DT_WorkModeReadAttempts++;
    if ((status == BLE_STATUS_SUCCESS) || (DT_WorkModeReadAttempts >= DT_WORK_MODE_REPLY_ATTEMPTS))
      DT_WorkModeReadPending = 0U;
  }
  if ((DT_WorkModeSendPending == 0U) ||
      (Custom_App_Context.Sensor_status_Notification_Status == 0U)) return;
  sent = Comm_SendWorkMode(COMM_LINK_BLE, (CommResult)payload[0], payload[1], payload[2]);
  DT_WorkModeSendAttempts++;
  if ((sent == COMM_OK) || (DT_WorkModeSendAttempts == 1U))
  {
    (void)snprintf(line, sizeof(line), "MODE: unified FE81 result=%u active=%u saved=%u status=%u\r\n",
        (unsigned int)sent,
        (unsigned int)payload[1], (unsigned int)payload[2], (unsigned int)payload[0]);
    DT_Log(line);
  }
  if (sent == COMM_OK)
  {
    DT_WorkModeSendPending = 0U;
    if (DT_ModeRestartOnNotify != 0U && payload[0] == COMM_OK)
      DT_ConfirmModeRestart(payload[2]);
  }
  else if (DT_WorkModeSendAttempts >= DT_WORK_MODE_REPLY_ATTEMPTS)
  {
    DT_WorkModeSendPending = 0U;
    DT_Log("MODE: notification retries exhausted; query again\r\n");
  }
}

static void DT_ScheduleWorkMode(void)
{
  UTIL_SEQ_SetTask(1U << CFG_TASK_DT_SEND_ID, CFG_SCH_PRIO_1);
}

static void DT_ConfirmModeRestart(uint8_t configured)
{
  if (DT_ModeRestartPending == 0U || DT_ModeRestartScheduled != 0U ||
      configured != DT_ConfiguredWorkMode || configured == DT_ActiveWorkMode)
    return;
  DT_ModeRestartTick = HAL_GetTick() + DT_WORK_MODE_RESTART_DELAY_MS;
  DT_ModeRestartScheduled = 1U;
  DT_Log("MODE: saved mode reply queued; restart in 1500 ms\r\n");
  DT_ScheduleWorkMode();
}

static void DT_ServiceWorkModeTask(void)
{
  /* The sequencer can run this while the Zigbee startup task is waiting.
     The RTC callback only schedules the task and never calls ACI. */
  DT_ProcessWorkMode(HAL_GetTick());
  if (DT_WorkModeTimerReady != 0U &&
      (Comm_HasPending() || DT_ModeRestartScheduled != 0U ||
       (Custom_App_Context.IsConnected != 0U &&
        (DT_WorkModeReadPending != 0U ||
         (DT_WorkModeSendPending != 0U && Custom_App_Context.Sensor_status_Notification_Status != 0U)))))
  {
    HW_TS_Start(DT_WorkModeTimerId,
                (DT_WORK_MODE_REPLY_RETRY_MS * 1000U + CFG_TS_TICK_VAL - 1U) / CFG_TS_TICK_VAL);
  }
}

static void DT_HandleSensorCtrl(const uint8_t *payload, uint8_t length)
{
  uint8_t sensor_mask;
  uint8_t requested_mask;

  if ((payload == 0) || (length == 0U))
  {
    return;
  }

  if ((payload[0] == DT_CMD_WORK_MODE_QUERY) || (payload[0] == DT_CMD_WORK_MODE_SET))
  {
    if ((payload[0] == DT_CMD_WORK_MODE_QUERY) && (length == 1U))
    {
      DT_WorkModeResult = DT_WORK_MODE_RESULT_OK;
      DT_Log("MODE: query received\r\n");
    }
    else if ((payload[0] == DT_CMD_WORK_MODE_SET) && (length == 2U) &&
             (payload[1] <= CUSTOM_APP_WORK_MODE_INSPECTION))
    {
      DT_SaveWorkMode(payload[1]);
      if (DT_WorkModeResult == COMM_OK)
      {
        DT_ModeRestartPending = DT_ConfiguredWorkMode != DT_ActiveWorkMode;
        DT_ModeRestartOnNotify = 1U;
      }
    }
    else
    {
      DT_WorkModeResult = DT_WORK_MODE_RESULT_BAD_VALUE;
      DT_Log("MODE: invalid command/value\r\n");
    }
    DT_QueueWorkModeStatus();
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
  /* This HCI-masked task also runs while Zigbee startup waits for events. */
  Comm_Process();
  DT_ServiceWorkModeTask();
  DT_UpdateLowPowerLock();
  if (CommStreamMode) { DT_CommSendTask(); return; }
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
  uint8_t was_sampling = ZigbeeVibration.sampling_stopped == 0U;
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
    for (uint16_t sample = 0U; sample < accepted_count; sample++)
    {
      const uint8_t *data = &VibFifoRawBuffer[sample * DT_VIB_FIFO_WORD_SIZE];
      int16_t *xyz = ZigbeeStoreRing[ZigbeeStoreHead];
      xyz[0] = (int16_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8U));
      xyz[1] = (int16_t)((uint16_t)data[3] | ((uint16_t)data[4] << 8U));
      xyz[2] = (int16_t)((uint16_t)data[5] | ((uint16_t)data[6] << 8U));
      if (++ZigbeeStoreHead == DT_ZB_STORE_RING_POINTS)
        ZigbeeStoreHead = 0U;
    }
    ZigbeeVibration.sample_count += accepted_count;
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
      (ZigbeeStoreCount >= DT_ZB_STORE_REFILL_POINTS))
  {
    /* Start feeding NOR when one page of samples is queued. Keep the
       per-task budget separate so a watermark burst can be drained promptly. */
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
    if (was_sampling != 0U)
    {
      ZigbeeVibrationWriteStats_t stats;
      ZigbeeVibrationStore_GetWriteStats(&stats);
      ZigbeeCaptureCommittedBytes = stats.bytes;
      ZigbeeCaptureWriteMs = HAL_GetTick() - ZigbeeVibration.diag_start_ms;
    }
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
      ZigbeeVibrationWriteStats_t stats;
      ZigbeeVibrationStore_GetWriteStats(&stats);
      (void)snprintf(log_line, sizeof(log_line),
                     "VIB-NOR: erase_ms=%lu capture_ms=%lu capture_bytes=%lu drain_ms=%lu drain_bytes=%lu\r\n",
                     (unsigned long)stats.erase_ms,
                     (unsigned long)ZigbeeCaptureWriteMs,
                     (unsigned long)ZigbeeCaptureCommittedBytes,
                     (unsigned long)(capture_elapsed - ZigbeeCaptureWriteMs),
                     (unsigned long)(stats.bytes - ZigbeeCaptureCommittedBytes));
      DT_Log(log_line);
#if (QSPI_NOR_PROGRAM_TIMING != 0)
      QspiNor_ProgramTiming_t timing = QspiNor_GetProgramTiming();
      (void)snprintf(log_line, sizeof(log_line),
                     "VIB-NOR-CYCLES: hz=%lu pages=%lu setup=%lu tx=%lu poll_delay=%lu poll=%lu done_delay=%lu max_gap=%lu\r\n",
                     (unsigned long)timing.clock_hz,
                     (unsigned long)timing.pages,
                     (unsigned long)timing.setup_cycles,
                     (unsigned long)timing.tx_cycles,
                     (unsigned long)timing.poll_start_delay_cycles,
                     (unsigned long)timing.poll_cycles,
                     (unsigned long)timing.completion_delay_cycles,
                     (unsigned long)timing.max_gap_cycles);
      DT_Log(log_line);
#endif
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
        ((ZigbeeStoreCount >= DT_ZB_STORE_REFILL_POINTS) &&
         (ZigbeeVibrationStore_AsyncCapacity() != 0U)) ||
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

/* Unified acquisition adapter: peripheral start/stop functions above remain the
 * sole owners of the sensor drivers. Commands enter through BLE GATT. */
void Custom_APP_ProtocolMtu(uint16_t handle, uint16_t mtu)
{
  if (Custom_App_Context.IsConnected && handle==Custom_App_Context.ConnectionHandle && mtu>=23U)
    CommBleCapacity=(mtu-3U>COMM_FRAME_MAX)?COMM_FRAME_MAX:(uint16_t)(mtu-3U);
}

static uint16_t DT_CommActiveMask(void)
{
  uint16_t mask=Custom_App_Context.SensorStreaming ?
      Comm_MaskFromBle(Custom_App_Context.SensorEnableMask):0U;
  if(Custom_App_Context.VibStreaming && !CommDraining)mask|=COMM_SENSOR_VIBRATION;
  return mask;
}

static uint8_t DT_CommReady(uint8_t vibration)
{
  return Custom_App_Context.IsConnected &&
      (vibration?Custom_App_Context.Vib_wave_Notification_Status:
                  Custom_App_Context.Sensor_status_Notification_Status);
}

static CommResult DT_CommBleSend(const CommSource *source,const uint8_t *frame,uint16_t length)
{
  if(source->session!=CommBleSession || !Custom_App_Context.IsConnected)return COMM_NOT_READY;
  uint8_t wave=frame[1]==COMM_KIND_DATA && frame[2]==COMM_DATA_WAVE;
  if(!(wave?Custom_App_Context.Vib_wave_Notification_Status:
             Custom_App_Context.Sensor_status_Notification_Status))return COMM_NOT_READY;
  if(length>CommBleCapacity || (!wave && length>80U))return COMM_BAD_LENGTH;
  tBleStatus result=Custom_STM_App_Update_Char_Variable_Length(
      wave?CUSTOM_STM_VIB_WAVE:CUSTOM_STM_SENSOR_STATUS,(uint8_t *)frame,length);
  if(result==BLE_STATUS_SUCCESS) {
    if (length >= COMM_HEADER_SIZE + 3U &&
        frame[1] == COMM_KIND_RESPONSE && frame[2] == COMM_CMD_WORK_MODE_SET &&
        frame[COMM_HEADER_SIZE] == COMM_OK)
      DT_ConfirmModeRestart(frame[COMM_HEADER_SIZE + 2U]);
    return COMM_OK;
  }
  if(result==BLE_STATUS_BUSY || result==BLE_STATUS_INSUFFICIENT_RESOURCES)return COMM_BUSY;
  return COMM_NOT_READY;
}

static void DT_CommQueueStatus(uint8_t status,uint16_t mask)
{
  if(status>COMM_STATUS_OVERFLOW || !mask)return;
  for(unsigned i=0;i<5;i++)CommStatusByCode[i]&=(uint16_t)~mask;
  CommStatusByCode[status]|=mask;CommStatusMask|=mask;
}

static void DT_CommStop(uint16_t mask)
{
  uint16_t active=DT_CommActiveMask();
  if((mask & active & COMM_SENSOR_SCALARS) && CommStreamMode) {
    /* Preserve the last available measurements before putting peripherals to sleep. */
    DT_CommCollectScalars(&CommFinalScalars);
    CommFinalScalars.valid_mask&=mask;
    CommFinalPending=CommFinalScalars.valid_mask!=0;
  }
  if(mask & COMM_SENSOR_SCALARS)
    DT_ApplySensorEnableMask(Comm_MaskToBle(active & (uint16_t)~mask));
  if((mask & COMM_SENSOR_VIBRATION) && Custom_App_Context.VibStreaming && !CommDraining) {
    if(CommStreamMode && Vibrometer_StopSamplingKeepFifo()==HAL_OK) {
      CommDraining=1U;CommDrainStatus=COMM_STATUS_STOPPED;
    } else DT_StopVibStream();
  }
  CommTimedMask&=(uint16_t)~mask;
}

static CommResult DT_CommExecute(const CommRequest *r,uint16_t *active)
{
  *active=DT_CommActiveMask();
  if(r->code==COMM_CMD_STATUS) {
    return COMM_OK;
  }
  if(r->code==COMM_CMD_WINDOW) {
    /* No mid-batch clock changes; timed windows are acquisition durations,
     * independent of the microphones' existing RMS integration windows. */
    if(*active & r->mask)return COMM_BUSY;
    for(unsigned i=0;i<5;i++)if(r->mask&(1U<<i))CommWindows[i]=r->duration_ms;
    return COMM_OK;
  }
  if(r->code==COMM_CMD_DECIMATION) {
    if(*active & COMM_SENSOR_VIBRATION)return COMM_BUSY;
    CommFactor=r->factor;return COMM_OK;
  }
  if(r->code!=COMM_CMD_SENSOR_ENABLE)return COMM_UNSUPPORTED;
  /* Scheduled A-version Flash capture is a separate resource owner. Never
   * interrupt it behind its state machine, or corrupt a persisted batch. */
  if(ZigbeeAudioWindowActive || ZigbeeVibrationWindowActive || APP_ZIGBEE_IsMeasuring())return COMM_BUSY;
  if(r->state==COMM_STOP) {
    uint16_t stopped=*active & r->mask;
    DT_CommStop(r->mask);*active=DT_CommActiveMask();
    if(stopped && CommStreamMode) DT_CommQueueStatus(COMM_STATUS_STOPPED,stopped & (uint16_t)~(CommDraining?COMM_SENSOR_VIBRATION:0U));
    return COMM_OK;
  }
  uint16_t added=r->mask & (uint16_t)~*active;
  if(!added)return COMM_OK; /* retry must not restart a sampling window */
  if(*active && !CommStreamMode)return COMM_BUSY;
  if(CommFrameLength || CommStatusMask || CommDraining || CommFinalPending)return COMM_BUSY;
  if(!Custom_App_Context.IsConnected || !Custom_App_Context.Sensor_status_Notification_Status)
    return COMM_NOT_READY;
  if((added & COMM_SENSOR_SCALARS) && CommBleCapacity<32U)return COMM_NOT_READY;
  if((added & COMM_SENSOR_VIBRATION) &&
     (!Custom_App_Context.Vib_wave_Notification_Status || CommBleCapacity<26U))return COMM_NOT_READY;
  CommStreamMode=1U;
  if(!*active) {++CommBatchId;CommBatchPending=1U;CommActiveFactor=CommFactor;}
  if(added & COMM_SENSOR_VIBRATION) {
    /* Starting a new waveform establishes its own immutable index/rate. */
    if(*active) {++CommBatchId;CommBatchPending=1U;}
    CommRawIndex=0;CommPhase=0;CommActiveFactor=CommFactor;
    DT_StartVibStream();
  }
  if(added & COMM_SENSOR_SCALARS)
    DT_ApplySensorEnableMask(Comm_MaskToBle(*active | added));
  uint16_t actual=DT_CommActiveMask();
  if((actual & added)!=added) {
    DT_CommStop(added);*active=DT_CommActiveMask();return COMM_HARDWARE_ERROR;
  }
  uint32_t now=HAL_GetTick();
  for(unsigned i=0;i<5;i++)if(added&(1U<<i)) {
    if(CommWindows[i]) {CommTimedMask|=(uint16_t)(1U<<i);CommDeadlines[i]=now+CommWindows[i];}
    else CommTimedMask&=(uint16_t)~(1U<<i);
  }
  *active=actual;DT_CommQueueStatus(COMM_STATUS_SAMPLING,added);
  CommRetryAt=now;
  UTIL_SEQ_SetTask(1U<<CFG_TASK_DT_SEND_ID,CFG_SCH_PRIO_1);
  return COMM_OK;
}

static void DT_CommPoll(uint32_t now)
{
  if(!CommStreamMode)return;
  uint16_t expired=0;
  for(unsigned i=0;i<5;i++)if((CommTimedMask&(1U<<i)) && (int32_t)(now-CommDeadlines[i])>=0)
    expired|=(uint16_t)(1U<<i);
  if(expired) {
    DT_CommStop(expired);
    if((expired & COMM_SENSOR_VIBRATION) && CommDraining)CommDrainStatus=COMM_STATUS_COMPLETE;
    DT_CommQueueStatus(COMM_STATUS_COMPLETE,expired & (uint16_t)~(CommDraining?COMM_SENSOR_VIBRATION:0U));
  }
  if(!DT_CommReady(0U) && DT_CommActiveMask()) {
    DT_CommStop(COMM_SENSOR_ALL);DT_StopVibStream();CommDraining=0;CommFrameLength=0;
    DT_CommQueueStatus(COMM_STATUS_FAULT,COMM_SENSOR_ALL);
  }
  if((DT_CommActiveMask() || CommDraining || CommFinalPending || CommFrameLength || CommStatusMask || CommBatchPending ||
      Custom_App_Context.SensorSendPending) && (int32_t)(now-CommRetryAt)>=0)
    UTIL_SEQ_SetTask(1U<<CFG_TASK_DT_SEND_ID,CFG_SCH_PRIO_1);
}

static void DT_CommCollectScalars(CommScalars *values)
{
  uint8_t data[6];uint16_t mask=DT_CommActiveMask();memset(values,0,sizeof(*values));
  if((mask&COMM_SENSOR_TEMPERATURE) && Thermometer_AppendPayload(data,sizeof(data))==2U) {
    values->valid_mask|=COMM_SENSOR_TEMPERATURE;values->temperature_centi_c=(int16_t)Comm_ReadU16(data);
  }
  if((mask&COMM_SENSOR_MAGNETIC) && Magnetometer_AppendPayload(data,sizeof(data))==6U) {
    values->valid_mask|=COMM_SENSOR_MAGNETIC;
    for(unsigned a=0;a<3;a++)values->magnetic_0p1_ut[a]=(int16_t)Comm_ReadU16(data+2*a);
  }
  if((mask&COMM_SENSOR_DIGITAL_MIC) && DigitalMicrophone_AppendPayload(data,sizeof(data))==1U) {
    values->valid_mask|=COMM_SENSOR_DIGITAL_MIC;values->digital_centi_db=(uint16_t)data[0]*100U;
  }
  if((mask&COMM_SENSOR_ANALOG_MIC) && AnalogMicrophone_AppendPayload(data,sizeof(data))==1U) {
    values->valid_mask|=COMM_SENSOR_ANALOG_MIC;values->analog_centi_db=(uint16_t)data[0]*100U;
  }
}

static void DT_CommSendTask(void)
{
  uint32_t now=HAL_GetTick();
  if((int32_t)(now-CommRetryAt)<0)return;
  CommRetryAt=now+1U;
  CommResult result;
  /* A batch description must precede every waveform. Retain it on congestion. */
  if(CommBatchPending) {
    result=Comm_SendBatch(COMM_LINK_BLE,CommBatchId,CommActiveFactor);
    if(result==COMM_OK)CommBatchPending=0;
    return;
  }
  if(CommFinalPending && !CommFrameLength) {
    result=Comm_SendScalars(COMM_LINK_BLE,CommBatchId,&CommFinalScalars);
    if(result==COMM_OK)CommFinalPending=0;
    return;
  }
  if(CommStatusMask && !CommFrameLength) {
    for(unsigned status=0;status<5;status++)if(CommStatusByCode[status]) {
      result=Comm_SendStatus(COMM_LINK_BLE,(uint8_t)status,CommStatusByCode[status]);
      if(result==COMM_OK) {CommStatusMask&=(uint16_t)~CommStatusByCode[status];CommStatusByCode[status]=0;}
      break;
    }
    return;
  }
  uint16_t capacity=CommBleCapacity;
  if(!CommFrameLength && Custom_App_Context.SensorSendPending) {
    CommScalars values;DT_CommCollectScalars(&values);
    uint16_t n=Comm_EncodeScalars(CommFrame+COMM_HEADER_SIZE,sizeof(CommFrame)-COMM_HEADER_SIZE,CommBatchId,&values);
    CommFrameCode=COMM_DATA_SCALARS;
    CommFrameLength=Comm_EncodeFrame(CommFrame,sizeof(CommFrame),COMM_KIND_DATA,CommFrameCode,
        Custom_App_Context.SensorSeq++,CommFrame+COMM_HEADER_SIZE,n);
    Custom_App_Context.SensorSendPending=0;
    Custom_App_Context.SensorAudioReportedMask|=Comm_MaskToBle(values.valid_mask)&6U;
  }
  if(!CommFrameLength && Custom_App_Context.VibStreaming) {
    uint16_t fifo_count=0;uint8_t flags=0;
    if(Vibrometer_ReadFifoStatus(&fifo_count,&flags)!=HAL_OK) {
      DT_StopVibStream();CommDraining=0;CommTimedMask&=(uint16_t)~COMM_SENSOR_VIBRATION;
      DT_CommQueueStatus(COMM_STATUS_FAULT,COMM_SENSOR_VIBRATION);return;
    }
    if(flags & VIBROMETER_FIFO_OVERRUN_FLAG) {
      /* Never invent sample indices across lost hardware samples, and never
       * silently change the decimation factor to hide a slow receiver. */
      DT_StopVibStream();CommDraining=0;CommTimedMask&=(uint16_t)~COMM_SENSOR_VIBRATION;
      DT_CommQueueStatus(COMM_STATUS_OVERFLOW,COMM_SENSOR_VIBRATION);return;
    }
    if(capacity<COMM_HEADER_SIZE+COMM_WAVE_PREFIX_SIZE+6U)return;
    unsigned max_points=(capacity-COMM_HEADER_SIZE-COMM_WAVE_PREFIX_SIZE)/6U;
    uint16_t raw=(uint16_t)(max_points*CommActiveFactor);
    if(raw>fifo_count)raw=fifo_count;
    if(raw>VIBROMETER_FIFO_DMA_MAX_BYTES/7U)raw=VIBROMETER_FIFO_DMA_MAX_BYTES/7U;
    if(!raw) {
      if(CommDraining) {
        DT_StopVibStream();CommDraining=0;
        DT_CommQueueStatus(CommDrainStatus,COMM_SENSOR_VIBRATION);
      }
      return;
    }
    if(CommRawIndex>UINT32_MAX-raw) {
      DT_StopVibStream();CommDraining=0;CommTimedMask&=(uint16_t)~COMM_SENSOR_VIBRATION;
      DT_CommQueueStatus(COMM_STATUS_COMPLETE,COMM_SENSOR_VIBRATION);return;
    }
    if(Vibrometer_ReadFifoData(VibFifoRawBuffer,(uint16_t)(raw*7U))!=HAL_OK) {
      DT_StopVibStream();CommDraining=0;CommTimedMask&=(uint16_t)~COMM_SENSOR_VIBRATION;
      DT_CommQueueStatus(COMM_STATUS_FAULT,COMM_SENSOR_VIBRATION);return;
    }
    uint16_t points=0;uint32_t first=CommRawIndex;
    for(unsigned i=0;i<raw;i++) {
      if(!CommPhase) {
        if(!points)first=CommRawIndex;
        for(unsigned a=0;a<3;a++)CommSamples[points][a]=(int16_t)Comm_ReadU16(VibFifoRawBuffer+i*7U+1U+a*2U);
        points++;
      }
      CommPhase=(uint16_t)((CommPhase+1U)%CommActiveFactor);CommRawIndex++;
    }
    if(!points)return;
    uint16_t n=Comm_EncodeWave(CommFrame+COMM_HEADER_SIZE,sizeof(CommFrame)-COMM_HEADER_SIZE,
                             CommBatchId,first,CommSamples,points);
    CommFrameCode=COMM_DATA_WAVE;
    CommFrameLength=Comm_EncodeFrame(CommFrame,sizeof(CommFrame),COMM_KIND_DATA,CommFrameCode,
        Custom_App_Context.VibSeq++,CommFrame+COMM_HEADER_SIZE,n);
  }
  if(!CommFrameLength)return;
  result=Comm_SendFrame(COMM_LINK_BLE,CommFrame,CommFrameLength);
  if(result==COMM_OK) {
    CommFrameLength=0;
    /* Drain promptly while stack buffers allow it; other sequencer tasks run
     * between packets. BUSY retains the exact bytes and raw point indices. */
    CommRetryAt=now;
    if(Custom_App_Context.VibStreaming)UTIL_SEQ_SetTask(1U<<CFG_TASK_DT_SEND_ID,CFG_SCH_PRIO_1);
  } else if(result==COMM_BUSY)CommRetryAt=now+DT_BLE_BUSY_RETRY_PERIOD_MS;
  else {
    CommFrameLength=0;DT_CommStop(COMM_SENSOR_ALL);DT_StopVibStream();CommDraining=0;
    DT_CommQueueStatus(COMM_STATUS_FAULT,COMM_SENSOR_ALL);
  }
}
