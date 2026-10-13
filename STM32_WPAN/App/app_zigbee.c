/**
  ******************************************************************************
  * @file    app_zigbee.c
  * @brief   Zigbee sleepy sensor node with the existing BLE radio/sensor ownership.
  ******************************************************************************
  */

#include "app_common.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_zigbee.h"
#include "custom_app.h"
#include "main.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "stm32_seq.h"
#include "stm32wbxx_core_interface_def.h"
#include "zigbee_interface.h"
#include "zigbee_types.h"
#include "vibration_tx.h"
#include "sampling_cadence.h"
#include "vibration_features.h"
#include "zigbee_vibration_store.h"

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>

extern RNG_HandleTypeDef hrng;
static uint8_t recovery_token[8];
static uint8_t recovery_token_valid;
static uint8_t recovery_config_pending;
static uint8_t recovery_retry_count;
static SamplingCadence sampling_cadence;
static uint8_t sampling_timer = 0xFFU;
static volatile uint8_t sampling_due;
static void ZbApp_SamplingTimerCallback(void);
static void ZbApp_ArmSamplingTimer(void);
/* Stored in the persistence length word: cannot be a valid blob length. */
#define ZB_CLEANUP_PENDING_MARKER 0x434C454EUL
static uint8_t leave_cleanup_pending;
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ee.h"
#include "hw_flash.h"
#include "zcl/zcl.h"
#include "zcl/general/zcl.basic.h"
#include "zcl/general/zcl.temp.meas.h"
#include "zigbee.nwk.h"
#include "zigbee.bdb.h"
#include "zigbee.zdo.h"

#define ZB_APP_ENDPOINT                    1U
#define ZB_COORDINATOR_ENDPOINT            8U
#define ZB_ALL_CHANNELS_MASK               0x07FFF800UL
#define ZB_JOIN_RETRY_MS                   5000U
#define ZB_COMMISSIONING_JOIN_RETRY_MS     1000U
#define ZB_RTC_JOIN_MAX_ATTEMPTS           3U
#define ZB_SAVED_CHANNEL_REJOIN_ATTEMPTS   2U
/* CPU2 restore can take >5 s even on a reachable network. Give restore and
 * live-parent verification separate budgets instead of racing the callback. */
#define ZB_SAVED_NETWORK_BUDGET_MS         15000U
#define ZB_RESTORED_PARENT_VERIFY_MS       10000U
#define ZB_FRESH_ADMISSION_BUDGET_MS       30000U
#define ZB_UNAPPROVED_WAKE_DELAY_MS        300000U
#define ZB_AUDIO_SETTLE_MS                 800U
#define ZB_CONFIG_QUERY_JOIN               0x01U
#define ZB_CONFIG_QUERY_COLLECTION         0x02U
#define ZB_CONFIG_QUERY_IDLE               0x03U
#define ZB_CONFIG_FLAG_MONITOR_ENABLED     0x02U
#define ZB_CONFIG_FLAG_MONITOR_PRESENT     0x04U
#define ZB_CONFIG_QUERY_RETRY_MS            2000U
#define ZB_CONFIG_QUERY_MAX_ATTEMPTS        3U
#define ZB_CONFIG_FLAG_CHANNEL_MIGRATION    0x01U
#define ZB_COMMISSIONING_FAST_POLL_MS       3000U
/* CPU2 FW 1.23.0 has blocked when persistence was exported directly inside
 * the join callback. Export later from the sequencer after radio traffic is
 * idle; pending nodes use a short delay while authorized nodes retain the
 * persistence notifications only enqueue work; export outside callbacks. */
#define ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED     1U
#define ZB_PERSIST_SAVE_DELAY_MS              100U
#define ZB_PERSIST_RETRY_DELAY_MS            1000U
#define ZB_PERSIST_SLEEPY_POLL_MARKER_V1    0x534C5031UL
#define ZB_PERSIST_SLEEPY_POLL_MARKER       0x534C5032UL
#define ZB_PERSIST_AUTH_PENDING             0x50454E44UL
#define ZB_PERSIST_AUTHORIZED               0x41555448UL
#define ZB_PERSIST_DATA_OFFSET                 12U
#define ZB_PENDING_PERSIST_DELAY_MS           3000U
#define ZB_PERSIST_MAX_DATA_BYTES           \
  (ST_PERSIST_MAX_ALLOC_SZ - ZB_PERSIST_DATA_OFFSET)
#define ZB_TEMP_MIN                        (-27315)
#define ZB_TEMP_MAX                        32767
#define ZB_TEMP_TOLERANCE                  2048

#define ZB_CMD_REPORT_START                0x08U
#define ZB_CMD_REPORT_STOP                 0x09U
#define ZB_CMD_MONITOR_CONTROL             0x0BU
#define ZB_CMD_MONITOR_CONFIG              0x0CU
#define ZB_CMD_TEMP_REPORT                 0x86U
#define ZB_CMD_MAG_REPORT                  0x87U
#define ZB_CMD_DIGITAL_MIC_REPORT          0x88U
#define ZB_CMD_ANALOG_MIC_REPORT           0x89U
#define ZB_CMD_VIBRATION_SUMMARY           0x8CU
#define ZB_CMD_MONITOR_STATE               0x8DU
#define ZB_CMD_CONFIG_QUERY                0x99U

#define ZB_MONITOR_SLEEP                   0x00U
#define ZB_MONITOR_WAITING                 0x01U
#define ZB_MONITOR_CAPTURING               0x02U
#define ZB_MONITOR_UPLOADING               0x03U
#define ZB_MONITOR_COMPLETE                0x04U
#define ZB_MONITOR_ERROR                   0x05U
#define ZB_TX_QUEUE_DEPTH                  16U
#define ZB_TX_MAX_PAYLOAD                  70U
#define ZB_TX_MAX_IN_FLIGHT                 3U
#define ZB_APP_TRAFFIC_TASKS               \
  ((1U << CFG_TASK_ZIGBEE_NETWORK_FORM) |  \
   (1U << CFG_TASK_ZIGBEE_REPORT) |        \
   (1U << CFG_TASK_ZIGBEE_TX) |            \
   (1U << CFG_TASK_ZIGBEE_ANNOUNCE) |      \
   (1U << CFG_TASK_ZIGBEE_PERSIST) |       \
   (1U << CFG_TASK_ZIGBEE_UPLOAD))

typedef struct
{
  uint32_t auto_interval_sec;
  uint16_t poll_interval_sec;
  uint16_t vibration_rate_hz;
  uint16_t digital_mic_rate_hz;
  uint32_t analog_mic_rate_hz;
  uint16_t vibration_window_ms;
  uint16_t digital_mic_window_ms;
  uint16_t analog_mic_window_ms;
  uint16_t raw_upload_every_cycles;
  uint32_t raw_upload_generation;
  uint8_t sensor_enable_mask;
} ZbMonitorConfig_t;

typedef struct
{
  uint8_t busy;
  uint8_t frame[82U];
  uint32_t submitted_ms;
} ZbUploadRadioSlot_t;

typedef union
{
  uint8_t bytes[ST_PERSIST_MAX_ALLOC_SZ];
  uint32_t words[ST_PERSIST_MAX_ALLOC_SZ / sizeof(uint32_t)];
} ZbPersistCache_t;

typedef struct
{
  uint8_t command_id;
  uint8_t length;
  uint8_t await_confirm;
  uint8_t payload[ZB_TX_MAX_PAYLOAD];
} ZbTxFrame_t;

typedef struct
{
  struct ZigBeeT *zb;
  struct ZbZclClusterT *temperature_server;
  struct ZbMsgFilterT *leave_filter;
  struct ZbMsgFilterT *security_diag_filter;
  enum ZbStatusCodeT join_status;
  enum ZclStatusCodeT (*temperature_command)(struct ZbZclClusterT *,
                                              struct ZbZclHeaderT *,
                                              struct ZbApsdeDataIndT *);
  ZbMonitorConfig_t monitor_config;
  Custom_APP_ZigbeeSnapshot_t pending_snapshot;
  uint32_t batch_id;
  uint32_t pending_batch_id;
  uint64_t rejoin_ext_pan_id;
  uint32_t rejoin_channel_mask;
  uint16_t rejoin_pan_id;
  uint16_t rejoin_short_address;
  uint16_t rejoin_manager_address;
  uint8_t join_retry_timer;
  uint8_t report_timer;
  uint8_t persist_timer;
  uint8_t upload_timer;
  uint8_t reporting_active;
  uint8_t monitor_enabled;
  uint8_t config_valid;
  uint8_t monitor_state;
  uint8_t collection_requested;
  uint8_t audio_waiting;
  uint8_t join_blocked_until_reset;
  uint8_t nvm_ready;
  uint8_t restore_pending;
  uint8_t parent_online;
  uint8_t parent_state_known;
  uint8_t parent_sync_pending;
  uint8_t commissioning_window_open;
  uint8_t rtc_join_active;
  uint8_t rtc_join_attempts;
  uint8_t rejoin_identity_valid;
  uint8_t rejoin_via_startup;
  uint8_t rejoin_probe_pending;
  uint8_t rejoin_probe_rtc;
  uint8_t restore_recovery_active;
  uint8_t saved_channel_rejoin_attempts;
  uint8_t offline_capture_requested;
  uint8_t pending_snapshot_valid;
  uint8_t tx_head;
  uint8_t tx_tail;
  uint8_t tx_count;
  uint8_t tx_in_flight;
  uint8_t report_has_summary_confirm;
  uint8_t radio_exclusive;
  uint8_t radio_paused;
  uint8_t radio_pause_pending;
  uint8_t radio_retry_pending;
  uint8_t security_join_cleanup_pending;
  uint8_t startup_wait_active;
  uint8_t radio_control_deferred;
  uint8_t persist_save_pending;
  uint8_t persist_tx_hold;
  uint8_t config_query_pending;
  uint8_t config_query_reason;
  uint8_t config_query_attempts;
  uint8_t channel_migration_wait;
  uint8_t channel_migration_target;
  uint8_t upload_allowed;
  uint8_t upload_sequence;
  uint8_t manual_announce_required;
  uint16_t raw_upload_cycle_counter;
} ZbAppContext_t;

struct ZbStartupWaitInfo
{
  enum ZbStatusCodeT status;
};
static struct ZbStartupWaitInfo startup_wait_info;
static uint32_t startup_wait_generation;
/* Accepted startup/rejoin remains live on CPU2 after an application timeout. */
static uint8_t startup_operation_inflight;
static uint8_t startup_drain_phase;
static uint32_t persist_restore_generation;
/* Actual CPU2 operation lifetime, separate from application callback validity. */
static uint32_t persist_restore_request_generation;
static uint8_t persist_restore_inflight;
static uint32_t command_confirm_generation;
static uint8_t admission_timer_id = 0xFFU;
/* 1: saved recovery, 2: fresh admission, 3: unapproved sleep,
 * 5: draining restore, 6: draining startup/rejoin, 7: late completion quarantine. */
static volatile uint8_t admission_phase;
static uint8_t network_authorized;
#include "monitor_standby.h"
static MonitorStandby monitor_standby;
static uint8_t network_persist_format_v2;
static volatile uint8_t admission_expired;
static struct nwk_fastpoll_entry_t *ZbAdmissionFastPoll;

extern UART_HandleTypeDef huart1;
extern RNG_HandleTypeDef hrng;

static void ZbApp_StackInit(void);
static void ZbApp_ConfigEndpoint(void);
static void ZbApp_NetworkJoin(void);
static void ZbApp_AdmissionTimerCallback(void);
static void ZbApp_AdmissionTimeoutTask(void);
static void ZbApp_AdmissionWakeTask(void);
static void ZbApp_StartAdmissionBudget(uint8_t phase);
static void ZbApp_CheckAuthorizationBudget(void);
static void ZbApp_ScheduleMonitorStandby(void);
static void ZbApp_EnterAdmissionSleep(void);
static void ZbApp_SetMaximumTxPower(void);
static void ZbApp_StartupOperationCompleted(uint32_t generation);
static enum zb_msg_filter_rc ZbApp_SecurityDiagnostic(struct ZigBeeT *zb,
    uint32_t id, void *message, void *arg);
static void ZbApp_SecurityDiagnosticTask(void);
static void ZbApp_ReadSecurityDiagnostic(uint8_t stage);
static void ZbApp_ReportTask(void);
static void ZbApp_TxTask(void);
static void ZbApp_UploadTask(void);
static void ZbApp_RadioControlTask(void);
static void ZbApp_RadioPausedCallback(void *arg);
static void ZbApp_AnnounceTask(void);
static void ZbApp_JoinRetryTimerCallback(void);
static void ZbApp_ReportTimerCallback(void);
static void ZbApp_UploadTimerCallback(void);
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
static void ZbApp_PersistTimerCallback(void);
#endif
static void ZbApp_StartTimer(uint8_t timer_id, uint32_t delay_ms);
static void ZbApp_SetParentOnline(uint8_t online, enum ZbStatusCodeT status);
static void ZbApp_VerifyParent(void);
static uint8_t ZbApp_IsTrustedCoordinator(const struct ZbApsdeDataIndT *data_indication);
static void ZbApp_StartLinkRecovery(void);
static void ZbApp_PrepareFreshJoin(void);
static void ZbApp_StartFreshJoinFallback(void);
static void ZbApp_CompleteJoinFailureCycle(void);
static void ZbApp_StartAutonomousReporting(void);
static void ZbApp_ScheduleNextReport(void);
static void ZbApp_SendSnapshot(const Custom_APP_ZigbeeSnapshot_t *snapshot,
                               uint32_t batch_id);
static void ZbApp_SendPendingSnapshot(void);
static enum ZclStatusCodeT ZbApp_SendMonitorState(uint8_t state);
static enum ZclStatusCodeT ZbApp_SendMonitorStateForBatch(uint8_t state,
                                                          uint32_t batch_id);
static enum ZclStatusCodeT ZbApp_CommandHandler(struct ZbZclClusterT *cluster,
                                                struct ZbZclHeaderT *header,
                                                struct ZbApsdeDataIndT *data_indication);
static enum ZclStatusCodeT ZbApp_SendCommand(uint8_t command_id,
                                             const uint8_t *payload,
                                             uint16_t length);
static void ZbApp_CommandConfirm(struct ZbZclCommandRspT *response, void *arg);
static void ZbApp_ResetTxQueue(void);
static enum zb_msg_filter_rc ZbApp_LeaveFilter(struct ZigBeeT *zb,
                                               uint32_t id,
                                               void *message,
                                               void *arg);
static bool ZbApp_CheckWirelessFirmware(void);
static void ZbApp_Log(const char *format, ...);
static int16_t ZbApp_Magnetic0p1uTToRaw(int16_t value);
static void ZbApp_RequestConfig(uint8_t reason);
static void ZbApp_StartConfigTransaction(void);
static void ZbApp_StopConfigTransaction(void);
static void ZbApp_FinishCollectionCycle(uint8_t success);
static int ZbApp_UploadRead(void *context, uint32_t batch, uint32_t first,
                            unsigned points, uint8_t *raw);
static int ZbApp_UploadSend(void *context, uint8_t command,
                            const uint8_t *data, size_t length);
static void ZbApp_UploadConfirm(struct ZbApsdeDataConfT *confirm, void *arg);
static void ZbApp_StartUploadFastPoll(void);
static void ZbApp_StopUploadFastPoll(void);
static void ZbApp_StartAdmissionFastPoll(void);
static void ZbApp_StopAdmissionFastPoll(void);

static bool ZbApp_NvmInit(void);
static bool ZbApp_NvmRead(void);
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
static bool ZbApp_NvmWrite(void);
static bool ZbApp_NvmWriteAuthorization(void);
#endif
static void ZbApp_NvmErase(void);
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
static bool ZbApp_PersistSave(void);
static void ZbApp_PersistTask(void);
static void ZbApp_PersistNotify(struct ZigBeeT *zb, void *arg);
static void ZbApp_EnablePersistence(void);
static void ZbApp_SchedulePersistSave(uint32_t delay_ms);
#endif
static enum ZbStatusCodeT ZbApp_StartPersistedNetwork(void);
static void ZbApp_PersistRestoreCallback(enum ZbStatusCodeT status, void *arg);

static void Wait_Getting_Ack_From_M0(void);
static void Receive_Ack_From_M0(void);
static void Receive_Notification_From_M0(void);
static void APP_ZIGBEE_ProcessNotifyM0ToM4(void);
static void APP_ZIGBEE_ProcessRequestM0ToM4(void);
enum ZbStatusCodeT ZbStartupWait(struct ZigBeeT *zb, struct ZbStartupT *config);
static enum ZbStatusCodeT ZbStartupRejoinWait(struct ZigBeeT *zb);

static TL_CmdPacket_t *p_ZIGBEE_otcmdbuffer;
static TL_EvtPacket_t *p_ZIGBEE_notif_M0_to_M4;
static TL_EvtPacket_t *p_ZIGBEE_request_M0_to_M4;
static __IO uint32_t CptReceiveNotifyFromM0;
static __IO uint32_t CptReceiveRequestFromM0;

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_ZIGBEE_Config_t ZigbeeConfigBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ZigbeeOtCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ZigbeeNotifRspEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ZigbeeNotifRequestBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];

static ZbAppContext_t ZbApp = {
  .join_status = (enum ZbStatusCodeT)0x01,
  .monitor_config = {300U, 5U, 26667U, 46875U, 256000U,
                     500U, 1000U, 1000U, 1U, 0U,
                     CUSTOM_APP_ZB_SENSOR_ALL},
  .join_retry_timer = 0xFFU,
  .report_timer = 0xFFU,
  .persist_timer = 0xFFU,
  .upload_timer = 0xFFU,
  .monitor_state = ZB_MONITOR_SLEEP
};
static ZbPersistCache_t ZbPersistCache;
static ZbTxFrame_t ZbTxQueue[ZB_TX_QUEUE_DEPTH];
static V2Tx ZbVibrationUpload;
static volatile uint8_t ZbUploadQueueSleeping;
static volatile uint8_t ZbUploadQueueWake;
static struct {
  VibFeatures stats;
  uint32_t batch, count, deadline, expected_crc;
  uint16_t rate;
  int32_t features[3][5];
  uint8_t active, feature_only, raw_done, features_ready, queued, confirmed, tries;
} ZbDeferred;
static ZbUploadRadioSlot_t ZbUploadSlots[3U];
static struct nwk_fastpoll_entry_t *ZbUploadFastPoll;
static struct nwk_fastpoll_entry_t *ZbConfigFastPoll;

/* Metadata only. Copy in the CPU2 callback, print/read attributes in a task.
 * Internal CPU2 transmit requests/confirms are NOT exposed by this observer. */
static struct {
  uint32_t tick, generation;
  uint64_t source;
  uint16_t cluster, short_addr, target, length;
  uint8_t kind, a, b, c;
} security_diag_queue[8];
static uint8_t security_diag_read, security_diag_write, security_diag_state_pending;
static uint32_t security_diag_dropped, security_diag_state_generation;

uint8_t g_ot_notification_allowed;

static void ZbApp_Log(const char *format, ...)
{
  char line[224];
  va_list args;
  uint32_t elapsed_ms = APP_ElapsedMs();
  int prefix_len;
  int length;

  prefix_len = snprintf(line, sizeof(line), "[+%lu.%03lus] ",
                        (unsigned long)(elapsed_ms / 1000U),
                        (unsigned long)(elapsed_ms % 1000U));
  if ((prefix_len < 0) || ((size_t)prefix_len >= sizeof(line)))
  {
    return;
  }
  va_start(args, format);
  length = vsnprintf(line + prefix_len, sizeof(line) - (size_t)prefix_len,
                     format, args);
  va_end(args);
  if (length <= 0)
  {
    return;
  }
  if ((size_t)length >= sizeof(line) - (size_t)prefix_len)
  {
    length = (int)(sizeof(line) - (size_t)prefix_len) - 1;
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)line,
                          (uint16_t)(prefix_len + length), 300U);
}

void APP_ZIGBEE_Init(void)
{
  SHCI_CmdStatus_t status;
  uint32_t random_batch_seed;

  ZbApp_Log("ZIGBEE: phase2 node initialization start\r\n");
  ZbApp_Log("ZIGBEE: build ble-zigbee-single-upload-20261013 | %s %s\r\n", __DATE__, __TIME__);
  if (!ZbApp_CheckWirelessFirmware())
  {
    ZbApp_Log("ZIGBEE: compatible BLE/Zigbee RFD/FFD CPU2 firmware required\r\n");
    return;
  }

  if ((HAL_RNG_GenerateRandomNumber(&hrng, &random_batch_seed) != HAL_OK) ||
      (random_batch_seed == 0U))
  {
    random_batch_seed = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^
                        HAL_GetUIDw2() ^ HAL_GetTick();
    if (random_batch_seed == 0U)
    {
      random_batch_seed = 1U;
    }
  }
  ZbApp.batch_id = random_batch_seed;

  APP_ZIGBEE_RegisterCmdBuffer(&ZigbeeOtCmdBuffer);
  APP_ZIGBEE_TL_INIT();
  UTIL_SEQ_RegTask(1U << CFG_TASK_NOTIFY_FROM_M0_TO_M4,
                   UTIL_SEQ_RFU, APP_ZIGBEE_ProcessNotifyM0ToM4);
  UTIL_SEQ_RegTask(1U << CFG_TASK_REQUEST_FROM_M0_TO_M4,
                   UTIL_SEQ_RFU, APP_ZIGBEE_ProcessRequestM0ToM4);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM,
                   UTIL_SEQ_RFU, ZbApp_NetworkJoin);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_ADMISSION_TIMEOUT,
                   UTIL_SEQ_RFU, ZbApp_AdmissionTimeoutTask);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_ADMISSION_WAKE,
                   UTIL_SEQ_RFU, ZbApp_AdmissionWakeTask);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_NVM_ERASE,
                   UTIL_SEQ_RFU, ZbApp_NvmErase);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_REPORT,
                   UTIL_SEQ_RFU, ZbApp_ReportTask);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_TX,
                   UTIL_SEQ_RFU, ZbApp_TxTask);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_UPLOAD,
                   UTIL_SEQ_RFU, ZbApp_UploadTask);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_RADIO_CONTROL,
                   UTIL_SEQ_RFU, ZbApp_RadioControlTask);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_ANNOUNCE,
                   UTIL_SEQ_RFU, ZbApp_AnnounceTask);
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_SECURITY_DIAG,
                   UTIL_SEQ_RFU, ZbApp_SecurityDiagnosticTask);
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_PERSIST,
                   UTIL_SEQ_RFU, ZbApp_PersistTask);
#endif
  ZbApp_ResetTxQueue();
  ZbApp.radio_exclusive = 0U;
  ZbApp.radio_retry_pending = 0U;
  ZbApp.startup_wait_active = 0U;
  ZbApp.radio_control_deferred = 0U;
  memset(&ZbVibrationUpload, 0, sizeof(ZbVibrationUpload));
  ZbUploadQueueSleeping = 0U;
  memset(ZbUploadSlots, 0, sizeof(ZbUploadSlots));
  ZbApp.restore_pending = 0U;
  ZbApp.commissioning_window_open = 1U;
  ZbApp.rtc_join_active = 0U;
  ZbApp.rtc_join_attempts = 0U;
  ZbApp.rejoin_identity_valid = 0U;
  ZbApp.rejoin_via_startup = 0U;
  ZbApp.rejoin_probe_pending = 0U;
  ZbApp.rejoin_probe_rtc = 0U;
  ZbApp.restore_recovery_active = 0U;
  ZbApp.saved_channel_rejoin_attempts = 0U;
  ZbApp.rejoin_channel_mask = 0U;
  ZbApp.offline_capture_requested = 0U;
  ZbApp.pending_snapshot_valid = 0U;
  UTIL_SEQ_ResumeTask(ZB_APP_TRAFFIC_TASKS);

  if (HW_TS_Create(CFG_TIM_PROC_ID_ZIGBEE, &ZbApp.join_retry_timer,
                   hw_ts_SingleShot, ZbApp_JoinRetryTimerCallback) != hw_ts_Successful)
  {
    ZbApp.join_retry_timer = 0xFFU;
    ZbApp_Log("ZIGBEE: join retry timer unavailable\r\n");
  }
  if (HW_TS_Create(CFG_TIM_PROC_ID_ZIGBEE, &admission_timer_id,
                   hw_ts_SingleShot, ZbApp_AdmissionTimerCallback) != hw_ts_Successful)
  {
    admission_timer_id = 0xFFU;
    ZbApp.join_blocked_until_reset = 1U;
    ZbApp_Log("ZIGBEE: admission timer unavailable; commissioning disabled\r\n");
  }
  if (HW_TS_Create(CFG_TIM_PROC_ID_ZIGBEE, &ZbApp.report_timer,
                   hw_ts_SingleShot, ZbApp_ReportTimerCallback) != hw_ts_Successful)
  {
    ZbApp.report_timer = 0xFFU;
    ZbApp_Log("ZIGBEE: report timer unavailable\r\n");
  }
  if (HW_TS_Create(CFG_TIM_PROC_ID_ZIGBEE, &ZbApp.upload_timer,
                   hw_ts_SingleShot, ZbApp_UploadTimerCallback) != hw_ts_Successful)
  {
    ZbApp.upload_timer = 0xFFU;
    ZbApp_Log("ZIGBEE: vibration upload timer unavailable\r\n");
  }
  if (HW_TS_Create(CFG_TIM_PROC_ID_ZIGBEE, &sampling_timer,
                   hw_ts_SingleShot, ZbApp_SamplingTimerCallback) != hw_ts_Successful)
  {
    sampling_timer = 0xFFU;
    ZbApp_Log("ZIGBEE: sampling RTC timer unavailable\r\n");
  }
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
  if (HW_TS_Create(CFG_TIM_PROC_ID_ZIGBEE, &ZbApp.persist_timer,
                   hw_ts_SingleShot, ZbApp_PersistTimerCallback) != hw_ts_Successful)
  {
    ZbApp.persist_timer = 0xFFU;
    ZbApp_Log("ZIGBEE: persistence timer unavailable\r\n");
  }
#endif

  /* Keep the first RTC sampling deadline referenced to power-up. The
     commissioning-window close at 30 seconds must not postpone it to 90 s. */
  ZbApp.reporting_active = 1U;
  ZbApp.monitor_enabled = 0U;
  ZbApp.monitor_state = ZB_MONITOR_SLEEP;

#if (CFG_ZIGBEE_NVM_ENABLED != 0U)
  ZbApp.nvm_ready = ZbApp_NvmInit() ? 1U : 0U;
#endif

  status = SHCI_C2_ZIGBEE_Init();
  ZbApp_Log("ZIGBEE: CPU2 init status=%u\r\n", (unsigned int)status);
  if (status == SHCI_Success)
  {
    /* CPU2 FW 1.23.0 previously returned 0x11 (unsupported feature) for
       BLE_IP and 0x12 (invalid parameters) for ZIGBEE_IP here. These
       synchronous SHCI commands can also stall before stack creation, so do
       not issue them. CPU1 STOP2 remains managed by the normal LPM path. */
    ZbApp_Log("ZIGBEE: CPU2 radio low-power commands skipped\r\n");
    /* ZbInit() and endpoint setup contain synchronous CPU1/CPU2 command
       exchanges. Do not let a nested sequencer idle path enter STOP2 while
       CPU1 is waiting for those acknowledgements; switching the system clock
       in the middle of stack creation can leave initialization stalled just
       after CPU2 initialization. Normal STOP2 operation is restored once
       every endpoint and persistence-restore request has been submitted. */
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_DISABLE);
    ZbApp_StackInit();
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
  }
}

void APP_ZIGBEE_Stop(void)
{
  if (admission_timer_id != 0xFFU) HW_TS_Stop(admission_timer_id);
  admission_phase = 0U;
  admission_expired = 0U;
  ++startup_wait_generation;
  ++persist_restore_generation;
  if (ZbApp.join_retry_timer != 0xFFU)
  {
    HW_TS_Stop(ZbApp.join_retry_timer);
  }
  if (ZbApp.report_timer != 0xFFU)
  {
    HW_TS_Stop(ZbApp.report_timer);
  }
  if (ZbApp.upload_timer != 0xFFU)
  {
    HW_TS_Stop(ZbApp.upload_timer);
  }
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
  if (ZbApp.persist_timer != 0xFFU)
  {
    HW_TS_Stop(ZbApp.persist_timer);
  }
#endif
  if (ZbApp.zb != NULL)
  {
    ZbDestroy(ZbApp.zb);
    ZbApp.zb = NULL;
  }
  ZbApp.join_status = (enum ZbStatusCodeT)0x01;
  ZbApp.reporting_active = 0U;
  ZbApp.monitor_enabled = 0U;
  ZbApp.monitor_state = ZB_MONITOR_SLEEP;
  ZbApp.collection_requested = 0U;
  ZbApp.audio_waiting = 0U;
  ZbApp.parent_online = 0U;
  ZbApp.parent_state_known = 0U;
  ZbApp.parent_sync_pending = 0U;
  ZbApp.restore_pending = 0U;
  ZbApp.commissioning_window_open = 0U;
  ZbApp.rtc_join_active = 0U;
  ZbApp.rtc_join_attempts = 0U;
  ZbApp.rejoin_identity_valid = 0U;
  ZbApp.rejoin_via_startup = 0U;
  ZbApp.rejoin_probe_pending = 0U;
  ZbApp.rejoin_probe_rtc = 0U;
  ZbApp.restore_recovery_active = 0U;
  ZbApp.saved_channel_rejoin_attempts = 0U;
  ZbApp.rejoin_channel_mask = 0U;
  ZbApp.offline_capture_requested = 0U;
  ZbApp.pending_snapshot_valid = 0U;
  ZbApp.report_has_summary_confirm = 0U;
  ZbApp.radio_exclusive = 0U;
  ZbApp.radio_retry_pending = 0U;
  ZbApp.startup_wait_active = 0U;
  ZbApp.radio_control_deferred = 0U;
  ZbApp.persist_save_pending = 0U;
  ZbApp.persist_tx_hold = 0U;
  UTIL_SEQ_ResumeTask(ZB_APP_TRAFFIC_TASKS);
  ZbApp_ResetTxQueue();
}

uint8_t APP_ZIGBEE_IsConnected(void)
{
  return ((ZbApp.zb != NULL) &&
          (ZbApp.join_status == ZB_STATUS_SUCCESS) &&
          (ZbApp.parent_online != 0U) &&
          (ZbApp.join_blocked_until_reset == 0U)) ? 1U : 0U;
}

uint8_t APP_ZIGBEE_IsPendingAuthorization(void)
{
  return ((ZbApp.zb != NULL) &&
          (ZbApp.join_status == ZB_STATUS_SUCCESS) &&
          (ZbApp.parent_online != 0U) &&
          (network_authorized == 0U) &&
          (ZbApp.join_blocked_until_reset == 0U)) ? 1U : 0U;
}

uint8_t APP_ZIGBEE_IsAdmissionSleeping(void)
{
  /* Application idle interval, not a paused CPU2/MAC. The wireless firmware
   * remains responsible for its own low-power state. */
  return (admission_phase == 3U) ? 1U : 0U;
}

uint8_t APP_ZIGBEE_IsMeasuring(void)
{
  /* Waiting between online-monitoring periods is not active acquisition. */
  return ((ZbApp.monitor_state == ZB_MONITOR_CAPTURING) ||
          (ZbApp.audio_waiting != 0U)) ? 1U : 0U;
}

uint8_t APP_ZIGBEE_IsTransmitting(void)
{
  /* The batch remains active until its queued frames have been submitted and
     the coordinator's vibration-summary acknowledgement (or the fallback
     monitor-state confirmation) has completed. */
  uint8_t upload_busy = (v2_tx_active(&ZbVibrationUpload) ||
      ZbDeferred.active || ZbApp.upload_allowed) ? 1U : 0U;
  if (ZbUploadQueueSleeping != 0U) upload_busy = 0U;
  for (unsigned index = 0U; index < 3U; index++)
  {
    if (ZbUploadSlots[index].busy != 0U)
    {
      upload_busy = 1U;
    }
  }
  return ((ZbApp.tx_count != 0U) ||
          (ZbApp.tx_in_flight != 0U) ||
          (ZbApp.config_query_pending != 0U) ||
          (upload_busy != 0U)) ? 1U : 0U;
}

void APP_ZIGBEE_SetBleConnected(uint8_t connected)
{
  connected = (connected != 0U) ? 1U : 0U;
  if (ZbApp.radio_exclusive == connected)
  {
    return;
  }

  ZbApp.radio_exclusive = connected;
  if (connected != 0U)
  {
    if (ZbApp.report_timer != 0xFFU)
    {
      HW_TS_Stop(ZbApp.report_timer);
    }
    if (ZbApp.join_retry_timer != 0xFFU)
    {
      HW_TS_Stop(ZbApp.join_retry_timer);
    }
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
    if (ZbApp.persist_timer != 0xFFU)
    {
      HW_TS_Stop(ZbApp.persist_timer);
    }
#endif
    if (ZbApp.upload_timer != 0xFFU) HW_TS_Stop(ZbApp.upload_timer);
    if (sampling_timer != 0xFFU) HW_TS_Stop(sampling_timer);
    ZbApp.pending_snapshot_valid = 0U;
    /* The sensor layer aborts an active Zigbee capture before this call. */
    ZbApp.audio_waiting = 0U;
    ZbApp.collection_requested = 0U;
    ZbApp.monitor_state = ZB_MONITOR_WAITING;
    UTIL_SEQ_PauseTask(ZB_APP_TRAFFIC_TASKS);
  }
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_RADIO_CONTROL, CFG_SCH_PRIO_0);
}

void APP_ZIGBEE_SetCommissioningWindow(uint8_t open)
{
  /* BLE's legacy 30-second advertising timer must not override Zigbee's
     independent authorization/recovery budget or enable monitoring. */
  if (open && !network_authorized && ZbApp.zb && !ZbApp.radio_exclusive &&
      !ZbApp.restore_pending) {
    ZbApp.join_blocked_until_reset = 0U;
    ZbApp_StartAdmissionBudget(2U);
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
  }
}

static void ZbApp_RadioPausedCallback(void *arg)
{
  UNUSED(arg);
  ZbApp.radio_pause_pending = 0U;
  ZbApp.radio_paused = 1U;
  ZbApp_Log("ZIGBEE: MAC paused; BLE has exclusive radio access\r\n");
  if (ZbApp.radio_exclusive == 0U)
  {
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_RADIO_CONTROL, CFG_SCH_PRIO_0);
  }
}

/* BLE retains priority and the original pause/resume interface. */
static void ZbApp_RadioControlTask(void)
{
  enum ZbStatusCodeT status;

  /* ZbStartupWait/ZbStartupRejoinWait drive the stack through a nested
     sequencer loop. A BLE connection event can arrive inside that loop.
     Do not issue a MAC pause/resume while startup is still using the MAC;
     run the latest BLE radio request as soon as startup returns. */
  if ((ZbApp.startup_wait_active != 0U) ||
      (startup_operation_inflight != 0U) || (persist_restore_inflight != 0U))
  {
    if (ZbApp.radio_control_deferred == 0U)
    {
      ZbApp_Log("ZIGBEE: radio switch deferred until join/rejoin completes\r\n");
    }
    ZbApp.radio_control_deferred = 1U;
    ZbApp.radio_retry_pending = 1U;
    ZbApp_StartTimer(ZbApp.join_retry_timer, 50U);
    return;
  }
  ZbApp.radio_control_deferred = 0U;

  if (ZbApp.radio_exclusive != 0U)
  {
    if ((ZbApp.zb == NULL) || (ZbApp.radio_paused != 0U) ||
        (ZbApp.radio_pause_pending != 0U))
    {
      return;
    }
    /* A submitted parent APS probe must finish before pausing the MAC.
       Otherwise its confirm can be lost, leaving parent_sync_pending set
       across the BLE session and blocking verification after disconnect.
       A merely queued probe has no in-flight confirm and is safe to pause. */
    uint8_t upload_inflight = 0U;
    for (unsigned i = 0U; i < 3U; i++) upload_inflight |= ZbUploadSlots[i].busy;
    if ((ZbApp.tx_in_flight != 0U) || upload_inflight)
    {
      ZbApp.radio_control_deferred = 1U;
      ZbApp.radio_retry_pending = 1U;
      ZbApp_StartTimer(ZbApp.join_retry_timer, 50U);
      return;
    }
    ZbApp_StopUploadFastPoll();
    ZbApp_StopConfigTransaction();
    ZbApp.config_query_pending = 0U;
    ZbApp_StopAdmissionFastPoll();
    /* No APS callback is outstanding here. Drop stale queued control frames;
       the retained Flash descriptor reconstructs the upload after resume. */
    ZbApp_ResetTxQueue();
    ZbApp.parent_sync_pending = 0U;
    ZbApp.rejoin_probe_pending = 0U;
    if (admission_phase == 2U || admission_phase == 1U)
      UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
    memset(&ZbVibrationUpload, 0, sizeof(ZbVibrationUpload));
    memset(&ZbDeferred, 0, sizeof(ZbDeferred));
    ZbUploadQueueSleeping = 0U;
    ZbApp.upload_allowed = 0U; /* Committed Flash batches are retained. */
    status = ZbStatePause(ZbApp.zb, ZbApp_RadioPausedCallback, NULL);
    if (status == ZB_STATUS_SUCCESS)
    {
      ZbApp.radio_pause_pending = 1U;
      ZbApp.radio_retry_pending = 0U;
      ZbApp_Log("ZIGBEE: MAC pause requested for BLE connection\r\n");
    }
    else
    {
      ZbApp.radio_retry_pending = 1U;
      ZbApp_Log("ZIGBEE: MAC pause busy status=0x%02x; retrying\r\n",
                (unsigned int)status);
      ZbApp_StartTimer(ZbApp.join_retry_timer, 50U);
    }
    return;
  }

  if (ZbApp.radio_pause_pending != 0U)
  {
    return;
  }
  if ((ZbApp.zb != NULL) && (ZbApp.radio_paused != 0U))
  {
    status = ZbStateResume(ZbApp.zb);
    if (status != ZB_STATUS_SUCCESS)
    {
      ZbApp.radio_retry_pending = 1U;
      ZbApp_Log("ZIGBEE: MAC resume failed status=0x%02x; retrying\r\n",
                (unsigned int)status);
      ZbApp_StartTimer(ZbApp.join_retry_timer, 50U);
      return;
    }
    ZbApp.radio_paused = 0U;
    ZbApp_Log("ZIGBEE: MAC resumed after BLE disconnect\r\n");
  }

  ZbApp.radio_retry_pending = 0U;
  UTIL_SEQ_ResumeTask(ZB_APP_TRAFFIC_TASKS);
  if ((ZbApp.zb != NULL) &&
      (ZbApp.join_status == ZB_STATUS_SUCCESS) &&
      (ZbApp.join_blocked_until_reset == 0U))
  {
    if (ZbApp.parent_online == 0U)
    {
      ZbApp_VerifyParent();
    }
    if ((ZbApp.tx_count != 0U) &&
        (ZbApp.tx_in_flight < ZB_TX_MAX_IN_FLIGHT))
    {
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_TX, CFG_SCH_PRIO_1);
    }
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ANNOUNCE, CFG_SCH_PRIO_0);
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
    if ((ZbApp.persist_save_pending != 0U) &&
        (ZbApp.commissioning_window_open == 0U))
    {
      ZbApp_StartTimer(ZbApp.persist_timer, ZB_PERSIST_RETRY_DELAY_MS);
    }
#endif
    ZbApp_ScheduleNextReport();
    uint8_t pending[V2_BATCH_LEN];
    if (ZigbeeVibrationStore_Pending(pending) && network_authorized) {
      ZbApp.upload_allowed = 1U;
      ZbApp_StartTimer(ZbApp.upload_timer, 1U);
    }
    if (!ZbApp.config_valid) ZbApp_RequestConfig(ZB_CONFIG_QUERY_JOIN);
    else if (!ZbApp.monitor_enabled) ZbApp_ScheduleMonitorStandby();
    if (admission_expired)
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ADMISSION_TIMEOUT, CFG_SCH_PRIO_0);
    ZbApp_Log("ZIGBEE: Zigbee resumed after BLE disconnect; Flash and sampling configuration retained\r\n");
  }
  else if ((ZbApp.zb != NULL) &&
           (ZbApp.restore_pending == 0U) &&
           (ZbApp.join_blocked_until_reset == 0U) &&
           ((ZbApp.commissioning_window_open != 0U) ||
            (ZbApp.rtc_join_active != 0U)))
  {
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
  }
  else if ((ZbApp.zb != NULL) &&
           (ZbApp.join_blocked_until_reset == 0U) &&
           (ZbApp.reporting_active != 0U))
  {
    ZbApp_ScheduleNextReport();
  }
}

static void ZbApp_StartAdmissionBudget(uint8_t phase)
{
  if (phase == 2U && admission_phase == 2U) return;
  if (phase == 2U) network_authorized = 0U;
  if (admission_timer_id == 0xFFU) return;
  admission_phase = phase;
  admission_expired = 0U;
  ZbApp.commissioning_window_open = 1U;
  ZbApp_StartTimer(admission_timer_id, phase == 1U ?
      ZB_SAVED_NETWORK_BUDGET_MS : ZB_FRESH_ADMISSION_BUDGET_MS);
  ZbApp_Log("ZIGBEE: %s window started (%lu ms)\r\n",
      phase == 1U ? "saved-network recovery" : "fresh-node authorization",
      (unsigned long)(phase == 1U ? ZB_SAVED_NETWORK_BUDGET_MS : ZB_FRESH_ADMISSION_BUDGET_MS));
}

static void ZbApp_AdmissionTimerCallback(void)
{
  if (admission_phase == 3U)
  {
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ADMISSION_WAKE, CFG_SCH_PRIO_0);
    return;
  }
  admission_expired = 1U;
  /* Wake a nested startup wait without resetting the stack from an ISR. */
  if (ZbApp.startup_wait_active != 0U)
    UTIL_SEQ_SetEvt(EVENT_ZIGBEE_STARTUP_ENDED);
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ADMISSION_TIMEOUT, CFG_SCH_PRIO_0);
}

static void ZbApp_CheckAuthorizationBudget(void)
{
  if (network_authorized == 0U && admission_expired == 0U &&
      ZbApp.join_status == ZB_STATUS_SUCCESS &&
      ZbApp.config_valid != 0U) {
    if (admission_timer_id != 0xFFU) HW_TS_Stop(admission_timer_id);
    network_authorized = 1U;
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
    bool authorization_saved = ZbApp_NvmWriteAuthorization();
#endif
    admission_phase = 0U;
    ZbApp_StopAdmissionFastPoll();
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
    ZbApp_Log("ZIGBEE: authorization complete; normal low-power operation enabled\r\n");
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
    ZbApp_SchedulePersistSave(authorization_saved != false ?
        ZB_PERSIST_SAVE_DELAY_MS : ZB_PENDING_PERSIST_DELAY_MS);
#endif
  }
}

static void ZbApp_StartAdmissionFastPoll(void)
{
  if ((ZbAdmissionFastPoll != NULL) || (ZbApp.zb == NULL) ||
      (network_authorized != 0U) ||
      (ZbApp.join_status != ZB_STATUS_SUCCESS))
  {
    return;
  }
  ZbAdmissionFastPoll = ZbNwkFastPollRequest(
      ZbApp.zb, ZB_NWK_SYNC_DEFAULT_DELAY_MS, 0U);
  if (ZbAdmissionFastPoll != NULL)
  {
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_DISABLE);
    ZbApp_Log("ZIGBEE: pending authorization; fast-poll remains active until authorize/leave\r\n");
  }
  else
  {
    ZbApp_Log("ZIGBEE: pending-authorization fast-poll request failed\r\n");
  }
}

static void ZbApp_StopAdmissionFastPoll(void)
{
  if (ZbAdmissionFastPoll == NULL)
  {
    return;
  }
  if (ZbApp.zb != NULL)
  {
    (void)ZbNwkFastPollRelease(ZbApp.zb, ZbAdmissionFastPoll);
  }
  ZbAdmissionFastPoll = NULL;
}

/* TX power is an interface setting, not an RF-on request. CPU2 still controls
 * radio sleep and reception. Reapply after reset/restore; STOP2 retains it. */
static void ZbApp_SetMaximumTxPower(void)
{
  int8_t actual = 0;
  bool set_ok, read_ok;
  if (ZbApp.zb == NULL) return;
  UTIL_LPM_SetStopMode(1U << CFG_LPM_SECURITY_DIAG, UTIL_LPM_DISABLE);
  read_ok = ZbNwkIfGetTxPower(ZbApp.zb, "wpan0", &actual);
  if (read_ok && actual == 6) {
    UTIL_LPM_SetStopMode(1U << CFG_LPM_SECURITY_DIAG, UTIL_LPM_ENABLE);
    return;
  }
  set_ok = ZbNwkIfSetTxPower(ZbApp.zb, "wpan0", 6);
  read_ok = ZbNwkIfGetTxPower(ZbApp.zb, "wpan0", &actual);
  ZbApp_Log("ZIGBEE: TX power target=+6 dBm set=%u read=%u actual=%d dBm verified=%u\r\n",
      (unsigned)set_ok, (unsigned)read_ok, (int)actual,
      (unsigned)(set_ok && read_ok && actual == 6));
  UTIL_LPM_SetStopMode(1U << CFG_LPM_SECURITY_DIAG, UTIL_LPM_ENABLE);
}

static void ZbApp_EnterAdmissionSleep(void)
{
  admission_phase = 3U;
  admission_expired = 0U;
  ZbApp_StartTimer(admission_timer_id, ZB_UNAPPROVED_WAKE_DELAY_MS);
  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
  Custom_APP_ZigbeeActivityChanged();
  ZbApp_Log("ZIGBEE: unapproved node idle for 5 minutes; STOP2 allowed, CPU2 low power managed by stack\r\n");
}

static void ZbApp_AdmissionWakeTask(void)
{
  if ((admission_phase != 3U) || (network_authorized != 0U) ||
      (ZbApp.zb == NULL))
  {
    return;
  }
  if (ZbApp.radio_exclusive != 0U)
  {
    ZbApp_Log("ZIGBEE: 5-minute wake deferred while radio is reserved\r\n");
    ZbApp_StartTimer(admission_timer_id, 1000U);
    return;
  }
  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_DISABLE);
  ZbApp.join_blocked_until_reset = 0U;
  ZbApp_PrepareFreshJoin();
  ZbApp_StartAdmissionBudget(2U);
  Custom_APP_ZigbeeActivityChanged();
  ZbApp_Log("ZIGBEE: 5-minute wake; fresh 30-second commissioning started\r\n");
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
}

static void ZbApp_StartupOperationCompleted(uint32_t generation)
{
  if (generation != startup_wait_generation) return;
  startup_operation_inflight = 0U;
  if ((admission_phase == 6U && admission_expired == 0U) || admission_phase == 7U)
  {
    HW_TS_Stop(admission_timer_id);
    admission_phase = startup_drain_phase;
    admission_expired = 1U;
    ZbApp_Log("ZIGBEE-DIAG: expired startup/rejoin completed; reset queued\r\n");
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ADMISSION_TIMEOUT, CFG_SCH_PRIO_0);
  }
}

static void ZbApp_AdmissionTimeoutTask(void)
{
  if (ZbApp.radio_exclusive) {
    ZbApp_StartTimer(admission_timer_id, 1000U);
    return;
  }
  uint8_t phase = admission_phase;
  if (!admission_expired || !phase || ZbApp.startup_wait_active) return;
  if (phase == 7U) return; /* Only the matching late callback releases quarantine. */
  if (phase == 6U)
  {
    /* No completion notification: do not issue conflicting commands to CPU2. */
    admission_phase = 7U;
    admission_expired = 1U;
    ZbApp.join_blocked_until_reset = 1U;
    ZbApp.commissioning_window_open = 0U;
    ZbApp_Log("ZIGBEE-DIAG: startup/rejoin drain exceeded 30 s; quarantined until late completion, then cleanup\r\n");
    return;
  }
  if (startup_operation_inflight != 0U)
  {
    startup_drain_phase = phase;
    admission_phase = 6U;
    admission_expired = 0U;
    ZbApp.restore_pending = 1U;
    if (ZbApp.join_retry_timer != 0xFFU) HW_TS_Stop(ZbApp.join_retry_timer);
    ZbApp_StartTimer(admission_timer_id, 30000U);
    ZbApp_Log("ZIGBEE-DIAG: admission expired; draining startup/rejoin gen=%lu (max 30 s)\r\n",
              (unsigned long)startup_wait_generation);
    return;
  }
  if (phase == 5U)
  {
    admission_phase = 0U;
    admission_expired = 0U;
    ZbApp.join_blocked_until_reset = 1U;
    ZbApp.commissioning_window_open = 0U;
    ZbApp_Log("ZIGBEE-DIAG: restore drain timed out after 10 s; join blocked until reset\r\n");
    return;
  }
  if (phase == 1U && persist_restore_inflight != 0U)
  {
    /* Invalidate application handling but let CPU2 finish before resetting.
     * Clearing restore_pending alone cannot cancel ZbStartupPersist. */
    ++persist_restore_generation;
    admission_phase = 5U;
    admission_expired = 0U;
    ZbApp.restore_pending = 1U;
    if (ZbApp.join_retry_timer != 0xFFU) HW_TS_Stop(ZbApp.join_retry_timer);
    ZbApp_StartTimer(admission_timer_id, 10000U);
    ZbApp_Log("ZIGBEE-DIAG: recovery budget expired; waiting for restore callback gen=%lu (max 10 s)\r\n",
              (unsigned long)persist_restore_request_generation);
    return;
  }
  if (phase == 1U && ZbApp.join_status == ZB_STATUS_SUCCESS)
  {
    /* Missing application acknowledgement is not permission to factory-reset
     * an already restored network. Retry with its credentials intact. */
    admission_expired = 0U;
    ZbApp.restore_pending = 0U;
    ZbApp.rtc_join_active = 0U;
    ZbApp.parent_sync_pending = 0U;
    if (recovery_retry_count < 3U) recovery_retry_count++;
    ZbApp_StartTimer(admission_timer_id,
        recovery_retry_count < 3U ? ZB_RESTORED_PARENT_VERIFY_MS : 60000U);
    ZbApp.manual_announce_required = 1U;
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ANNOUNCE, CFG_SCH_PRIO_0);
    ZbApp_Log("ZIGBEE: recovery ACK timeout; saved network retained, handshake retry=%u\r\n",
              (unsigned int)recovery_retry_count);
    return;
  }
  ZbApp_Log("ZIGBEE-DIAG: admission timeout phase=%u restore_pending=%u startup_active=%u join=0x%02x restore_gen=%lu startup_gen=%lu\r\n",
            (unsigned int)phase, (unsigned int)ZbApp.restore_pending,
            (unsigned int)ZbApp.startup_wait_active, (unsigned int)ZbApp.join_status,
            (unsigned long)persist_restore_generation, (unsigned long)startup_wait_generation);
  ++startup_wait_generation;
  ++persist_restore_generation;
  ZbApp.restore_pending = 0U;
  ZbApp.rejoin_probe_pending = 0U;
  ZbApp.rejoin_probe_rtc = 0U;
  ZbApp.rtc_join_active = 0U;
  ZbApp.monitor_enabled = 0U;
  ZbApp.config_valid = 0U;
  ZbApp.collection_requested = 0U;
  if (ZbApp.join_retry_timer != 0xFFU) HW_TS_Stop(ZbApp.join_retry_timer);
  if (ZbApp.report_timer != 0xFFU) HW_TS_Stop(ZbApp.report_timer);
  if (ZbApp.persist_timer != 0xFFU) HW_TS_Stop(ZbApp.persist_timer);
  ZbApp.persist_save_pending = 0U;
  ZbApp.persist_tx_hold = 0U;
  /* Startup/rejoin completion has been drained. Reset remaining APS/NWK
   * state in task context, keeping CPU1 out of STOP2 during the IPCC command. */
  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_DISABLE);
  ZbApp_Log("ZIGBEE-DIAG: admission ZbReset begin restore_gen=%lu startup_gen=%lu\r\n",
            (unsigned long)persist_restore_generation, (unsigned long)startup_wait_generation);
  ZbReset(ZbApp.zb);
  ZbApp_SetMaximumTxPower();
  ZbApp_Log("ZIGBEE-DIAG: admission ZbReset returned\r\n");
  ZbApp.security_join_cleanup_pending = 0U;
  ZbApp.join_status = (enum ZbStatusCodeT)0x01;
  ZbApp_PrepareFreshJoin();
  if (phase == 1U) {
    /* ST Commissioning_Server_Router restart pattern: ZbReset, then queue
     * NETWORK_FORM. Pause/resume is MAC ownership, not startup cancellation. */
    admission_phase = 0U;
    admission_expired = 0U;
    ZbApp.join_blocked_until_reset = 0U;
    ZbApp_StartAdmissionBudget(2U);
    Custom_APP_ZigbeeActivityChanged();
    ZbApp_Log("ZIGBEE-DIAG: recovery operations completed; fresh 30-second join starts\r\n");
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
  } else {
    admission_phase = 0U;
    admission_expired = 0U;
    ZbApp.join_blocked_until_reset = 1U;
    ZbApp.commissioning_window_open = 0U;
    ZbApp.reporting_active = 0U;
    ZbApp.monitor_state = ZB_MONITOR_SLEEP;
    ZbApp_Log("ZIGBEE: 30 s fresh-join window expired; commissioning stopped\r\n");
    ZbApp_EnterAdmissionSleep();
  }
}

static void ZbApp_ReadSecurityDiagnostic(uint8_t stage)
{
  static const enum ZbBdbAttrIdT attrs[] = {
    ZB_BDB_TrustCenterRequiresKeyExchange, ZB_BDB_TCLinkKeyExchangeMethod,
    ZB_BDB_TCLK_ExchangeAttempts, ZB_BDB_TCLK_ExchangeAttemptsMax,
    ZB_BDB_TrustCenterNodeJoinTimeout
  };
  uint8_t seq = 0U, level = 0U;
  uint32_t counter = 0U;
  uint64_t tc = 0U;
  enum ZbStatusCodeT status, seq_status;
  if (ZbApp.zb == NULL) return;
  UTIL_LPM_SetStopMode(1U << CFG_LPM_SECURITY_DIAG, UTIL_LPM_DISABLE);
  ZbApp_Log("ZIGBEE-SEC: state read stage=%u gen=%lu phase=%u (task-time snapshot)\r\n",
      (unsigned)stage, (unsigned long)startup_wait_generation, (unsigned)admission_phase);
  for (unsigned i = 0U; i < sizeof(attrs) / sizeof(attrs[0]); i++) {
    uint32_t value = 0U;
    unsigned size = 1U;
    /* Read only; retry size/parameter rejection for firmware representations.
     * A rejected attribute is NOT evidence of a disabled security setting. */
    for (;;) {
      value = 0U;
      status = ZbBdbGet(ZbApp.zb, attrs[i], &value, size);
      if (status != ZB_APS_STATUS_INVALID_PARAMETER || size == 4U) break;
      size *= 2U;
    }
    ZbApp_Log("ZIGBEE-SEC: BDB attr=0x%04x status=0x%02x size=%u value=%lu (invalid if status!=0)\r\n",
        (unsigned)attrs[i], (unsigned)status, size, (unsigned long)value);
  }
  status = ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_SecurityLevel, &level, sizeof(level));
  ZbApp_Log("ZIGBEE-SEC: NWK security-level status=0x%02x value=%u\r\n", (unsigned)status, (unsigned)level);
  seq_status = ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_ActiveKeySeqNumber, &seq, sizeof(seq));
  status = ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_OutgoingCounter, &counter, sizeof(counter));
  ZbApp_Log("ZIGBEE-SEC: NWK active-seq status=0x%02x value=%u; outgoing status=0x%02x value=%lu\r\n",
      (unsigned)seq_status, (unsigned)seq, (unsigned)status, (unsigned long)counter);
  status = ZbApsGet(ZbApp.zb, ZB_APS_IB_ID_TRUST_CENTER_ADDRESS, &tc, sizeof(tc));
  ZbApp_Log("ZIGBEE-SEC: TC-address status=0x%02x value=%08lx%08lx\r\n",
      (unsigned)status, (unsigned long)(tc >> 32U), (unsigned long)tc);
  if (seq_status == ZB_STATUS_SUCCESS) {
    struct ZbApsmeGetKeyReqT req = {0};
    struct ZbApsmeGetKeyConfT conf = {0};
    req.keyType = ZB_SEC_KEYTYPE_STANDARD_NWK;
    req.keySeqNumber = seq;
    ZbApsmeGetKeyReq(ZbApp.zb, &req, &conf);
    ZbApp_Log("ZIGBEE-SEC: active NWK-key lookup status=0x%02x seq=%u (existence only, no key bytes)\r\n",
        (unsigned)conf.status, (unsigned)seq);
    volatile uint8_t *wipe = (volatile uint8_t *)&conf;
    for (unsigned i = 0U; i < sizeof(conf); i++) wipe[i] = 0U;
  }
  UTIL_LPM_SetStopMode(1U << CFG_LPM_SECURITY_DIAG, UTIL_LPM_ENABLE);
}

static enum zb_msg_filter_rc ZbApp_SecurityDiagnostic(struct ZigBeeT *zb,
    uint32_t id, void *message, void *arg)
{
  uint8_t next = (uint8_t)((security_diag_write + 1U) % 8U);
  (void)zb; (void)arg;
  if (message == NULL) return ZB_MSG_CONTINUE;
  if (id == ZB_MSG_FILTER_APSDE_DATA_IND) {
    const struct ZbApsdeDataIndT *p = message;
    if (p->profileId != ZB_ZDO_PROFILE_ID || p->dst.endpoint != 0U ||
        (p->clusterId != ZB_ZDO_NODE_DESC_REQ &&
         p->clusterId != (ZB_ZDO_NODE_DESC_REQ | 0x8000U) &&
         p->clusterId != ZB_ZDO_DEVICE_ANNCE)) return ZB_MSG_CONTINUE;
  }
  if (id == ZB_MSG_FILTER_TRANSPORT_KEY_IND) {
    security_diag_state_pending = 1U;
    security_diag_state_generation = startup_wait_generation;
  }
  if (next == security_diag_read) {
    security_diag_dropped++;
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_SECURITY_DIAG, CFG_SCH_PRIO_1);
    return ZB_MSG_CONTINUE;
  }
  memset(&security_diag_queue[security_diag_write], 0,
         sizeof(security_diag_queue[security_diag_write]));
  security_diag_queue[security_diag_write].tick = APP_ElapsedMs();
  security_diag_queue[security_diag_write].generation = startup_wait_generation;
  if (id == ZB_MSG_FILTER_TRANSPORT_KEY_IND) {
    const struct ZbApsmeTransKeyIndT *p = message;
    security_diag_queue[security_diag_write].kind = 1U;
    security_diag_queue[security_diag_write].source = p->srcExtAddr;
    security_diag_queue[security_diag_write].a = (uint8_t)p->keyType;
    security_diag_queue[security_diag_write].b = (uint8_t)p->encryptKeyType;
    security_diag_queue[security_diag_write].c = p->keyType == ZB_SEC_KEYTYPE_STANDARD_NWK ?
        p->keyinfo.nwkKey.keySeqNumber : 0xFFU;
  } else if (id == ZB_MSG_FILTER_VERIFY_KEY_IND) {
    const struct ZbApsmeVerifyKeyIndT *p = message;
    security_diag_queue[security_diag_write].kind = 2U;
    security_diag_queue[security_diag_write].a = (uint8_t)p->keyType;
    security_diag_queue[security_diag_write].b = (uint8_t)p->encryptKeyType;
  } else if (id == ZB_MSG_FILTER_CONFIRM_KEY_IND) {
    const struct ZbApsmeConfirmKeyIndT *p = message;
    security_diag_queue[security_diag_write].kind = 3U;
    security_diag_queue[security_diag_write].a = (uint8_t)p->status;
    security_diag_queue[security_diag_write].b = (uint8_t)p->keyType;
    security_diag_queue[security_diag_write].c = (uint8_t)p->encryptKeyType;
  } else if (id == ZB_MSG_FILTER_APSDE_DATA_IND) {
    const struct ZbApsdeDataIndT *p = message;
    const uint8_t *buf = p->asdu;
    security_diag_queue[security_diag_write].kind = 4U;
    security_diag_queue[security_diag_write].cluster = p->clusterId;
    security_diag_queue[security_diag_write].short_addr =
        p->src.mode == ZB_APSDE_ADDRMODE_SHORT ? p->src.nwkAddr : 0xFFFFU;
    security_diag_queue[security_diag_write].source =
        p->src.mode == ZB_APSDE_ADDRMODE_EXT ? p->src.extAddr : 0U;
    security_diag_queue[security_diag_write].length = p->asduLength;
    security_diag_queue[security_diag_write].a = (uint8_t)p->securityStatus;
    security_diag_queue[security_diag_write].b =
        buf != NULL && p->asduLength >= 1U ? buf[0] : 0xFFU;
    security_diag_queue[security_diag_write].c = 0xFFU;
    security_diag_queue[security_diag_write].target = 0xFFFFU;
    if (buf != NULL && p->clusterId == ZB_ZDO_NODE_DESC_REQ && p->asduLength >= 3U)
      security_diag_queue[security_diag_write].target =
          (uint16_t)buf[1] | ((uint16_t)buf[2] << 8U);
    if (buf != NULL && p->clusterId == (ZB_ZDO_NODE_DESC_REQ | 0x8000U) && p->asduLength >= 4U) {
      security_diag_queue[security_diag_write].c = buf[1];
      security_diag_queue[security_diag_write].target =
          (uint16_t)buf[2] | ((uint16_t)buf[3] << 8U);
    }
  } else return ZB_MSG_CONTINUE;
  security_diag_write = next;
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_SECURITY_DIAG, CFG_SCH_PRIO_1);
  return ZB_MSG_CONTINUE;
}

static void ZbApp_SecurityDiagnosticTask(void)
{
  while (security_diag_read != security_diag_write) {
    /* Copy before any UART/IPCC wait can let another callback run. */
    __typeof__(security_diag_queue[0]) event = security_diag_queue[security_diag_read];
    security_diag_read = (uint8_t)((security_diag_read + 1U) % 8U);
    ZbApp_Log("ZIGBEE-SEC: RX kind=%u at=%lu ms gen=%lu src=%08lx%08lx a=0x%02x b=0x%02x c=0x%02x\r\n",
        (unsigned)event.kind, (unsigned long)event.tick, (unsigned long)event.generation,
        (unsigned long)(event.source >> 32U), (unsigned long)event.source,
        (unsigned)event.a, (unsigned)event.b, (unsigned)event.c);
    if (event.kind == 1U)
      ZbApp_Log("ZIGBEE-SEC: TRANSPORT_KEY received type=%u encrypt=0x%02x seq=%u (indication, not completion)\r\n",
          (unsigned)event.a, (unsigned)event.b, (unsigned)event.c);
    else if (event.kind == 2U)
      ZbApp_Log("ZIGBEE-SEC: VERIFY_KEY received type=%u encrypt=0x%02x\r\n", (unsigned)event.a, (unsigned)event.b);
    else if (event.kind == 3U)
      ZbApp_Log("ZIGBEE-SEC: CONFIRM_KEY status=0x%02x type=%u encrypt=0x%02x\r\n", (unsigned)event.a, (unsigned)event.b, (unsigned)event.c);
    else
      ZbApp_Log("ZIGBEE-ZDO: RX cluster=0x%04x src=0x%04x seq=%u security=0x%02x len=%u target=0x%04x rsp_status=0x%02x (0xff if absent)\r\n",
          (unsigned)event.cluster, (unsigned)event.short_addr, (unsigned)event.b,
          (unsigned)event.a, (unsigned)event.length, (unsigned)event.target, (unsigned)event.c);
  }
  if (security_diag_dropped != 0U) {
    uint32_t dropped = security_diag_dropped;
    security_diag_dropped = 0U;
    ZbApp_Log("ZIGBEE-SEC: metadata queue dropped=%lu; missing logs do not prove missing packets\r\n", (unsigned long)dropped);
  }
  if (security_diag_state_pending != 0U) {
    uint32_t generation = security_diag_state_generation;
    security_diag_state_pending = 0U;
    if (generation == startup_wait_generation && admission_expired == 0U)
      ZbApp_ReadSecurityDiagnostic(1U);
    else
      ZbApp_Log("ZIGBEE-SEC: post-key snapshot skipped; generation/phase expired\r\n");
  }
}

static void ZbApp_StackInit(void)
{
  enum ZbStatusCodeT restore_status = ZB_STATUS_ALLOC_FAIL;

  ZbApp.zb = ZbInit(0U, NULL, NULL);
  if (ZbApp.zb == NULL)
  {
    ZbApp_Log("ZIGBEE: ZbInit failed\r\n");
    return;
  }
  ZbApp_SetMaximumTxPower();
  ZbApp_ConfigEndpoint();
  ZbApp.security_diag_filter = ZbMsgFilterRegister(ZbApp.zb,
      ZB_MSG_FILTER_TRANSPORT_KEY_IND | ZB_MSG_FILTER_VERIFY_KEY_IND |
      ZB_MSG_FILTER_CONFIRM_KEY_IND | ZB_MSG_FILTER_APSDE_DATA_IND,
      255U, ZbApp_SecurityDiagnostic, NULL);
  ZbApp_Log("ZIGBEE-SEC: metadata-only observer registered=%u\r\n",
            ZbApp.security_diag_filter != NULL ? 1U : 0U);
  ZbApp.leave_filter = ZbMsgFilterRegister(ZbApp.zb,
                                            ZB_MSG_FILTER_LEAVE_IND,
                                            255U,
                                            ZbApp_LeaveFilter,
                                            NULL);
  if (ZbApp.leave_filter == NULL)
  {
    ZbApp_Log("ZIGBEE: leave filter registration failed\r\n");
  }

#if (CFG_ZIGBEE_NVM_ENABLED != 0U)
  if (ZbApp.nvm_ready != 0U)
  {
    uint32_t marker;
    if (EE_Read(0, ZIGBEE_DB_START_ADDR, &marker) == EE_OK &&
        marker == ZB_CLEANUP_PENDING_MARKER)
    {
      leave_cleanup_pending = 1U;
      ZbApp.join_blocked_until_reset = 1U;
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NVM_ERASE, CFG_SCH_PRIO_0);
      ZbApp_Log("ZIGBEE: interrupted leave cleanup detected; resume before commissioning\r\n");
      return;
    }
    restore_status = ZbApp_StartPersistedNetwork();
  }
#endif
  if (restore_status == ZB_STATUS_SUCCESS)
  {
    ZbApp_Log("ZIGBEE: persisted network restore submitted\r\n");
  }
  else
  {
    ZbApp_StartAdmissionBudget(2U);
    ZbApp_Log("ZIGBEE: no restorable network, scanning channels 11-26\r\n");
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
  }
}

static void ZbApp_ConfigEndpoint(void)
{
  struct ZbApsmeAddEndpointReqT request;
  struct ZbApsmeAddEndpointConfT confirm;
  uint8_t manufacturer[] = {7U, 'H', 'u', 'a', 'Y', 'a', 'n', 'g'};
  uint8_t model[] = {9U, 'H', 'Y', '-', 'S', 'e', 'n', 's', 'o', 'r'};

  memset(&request, 0, sizeof(request));
  memset(&confirm, 0, sizeof(confirm));
  request.profileId = ZCL_PROFILE_HOME_AUTOMATION;
  request.deviceId = ZCL_DEVICE_ONOFF_SWITCH;
  request.endpoint = ZB_APP_ENDPOINT;
  ZbZclAddEndpoint(ZbApp.zb, &request, &confirm);
  if (confirm.status != ZB_STATUS_SUCCESS)
  {
    ZbApp_Log("ZIGBEE: endpoint registration failed status=0x%02x\r\n",
              confirm.status);
    return;
  }

  ZbApp.temperature_server = ZbZclTempMeasServerAlloc(ZbApp.zb,
                                                       ZB_APP_ENDPOINT,
                                                       ZB_TEMP_MIN,
                                                       ZB_TEMP_MAX,
                                                       ZB_TEMP_TOLERANCE);
  if (ZbApp.temperature_server == NULL)
  {
    ZbApp_Log("ZIGBEE: temperature cluster allocation failed\r\n");
    return;
  }
  ZbApp.temperature_command = ZbApp.temperature_server->command;
  ZbApp.temperature_server->command = ZbApp_CommandHandler;
  ZbZclClusterEndpointRegister(ZbApp.temperature_server);
  (void)ZbZclBasicWriteDirect(ZbApp.zb, ZB_APP_ENDPOINT,
                              ZCL_BASIC_ATTR_MFR_NAME,
                              manufacturer, sizeof(manufacturer));
  (void)ZbZclBasicWriteDirect(ZbApp.zb, ZB_APP_ENDPOINT,
                              ZCL_BASIC_ATTR_MODEL_NAME,
                              model, sizeof(model));
  ZbApp_Log("ZIGBEE: endpoint 1 and private sensor commands ready\r\n");
  /* Current board is powered externally through ST-Link, not a battery.
   * This is a hardware configuration, not automatic adapter detection. */
  uint8_t power_source = ZCL_BASIC_POWER_DC;
  (void)ZbZclBasicWriteDirect(ZbApp.zb, ZB_APP_ENDPOINT,
                              ZCL_BASIC_ATTR_POWER_SOURCE,
                              &power_source, sizeof(power_source));
  ZbApp_Log("ZIGBEE: power source=external DC (ST-Link hardware configuration)\r\n");
}

static void ZbApp_NetworkJoin(void)
{
  struct ZbStartupT config;
  enum ZbStatusCodeT status;
  uint8_t rtc_triggered;
  uint8_t rejoin_existing;

  if ((ZbApp.radio_exclusive != 0U) ||
      (ZbApp.startup_wait_active != 0U) ||
      (ZbApp.zb == NULL) ||
      (ZbApp.restore_pending != 0U) ||
      (ZbApp.rejoin_probe_pending != 0U) ||
      ((ZbApp.join_status == ZB_STATUS_SUCCESS) &&
       (ZbApp.parent_online != 0U)) ||
      (ZbApp.join_blocked_until_reset != 0U) ||
      ((ZbApp.commissioning_window_open == 0U) &&
       (ZbApp.rtc_join_active == 0U) &&
       (ZbApp.join_status == ZB_STATUS_SUCCESS)))
  {
    return;
  }
  rtc_triggered = ZbApp.rtc_join_active;
  if (rtc_triggered != 0U)
  {
    ZbApp.rtc_join_attempts++;
    ZbApp_Log("ZIGBEE: RTC join attempt %u/%u\r\n",
              (unsigned int)ZbApp.rtc_join_attempts,
              (unsigned int)ZB_RTC_JOIN_MAX_ATTEMPTS);
  }
  rejoin_existing = (ZbApp.join_status == ZB_STATUS_SUCCESS) ? 1U : 0U;
  if ((rejoin_existing != 0U) &&
      (ZbApp.restore_recovery_active != 0U) &&
      (ZbApp.saved_channel_rejoin_attempts < 0xFFU))
  {
    ZbApp.saved_channel_rejoin_attempts++;
    ZbApp_Log("ZIGBEE: saved-channel recovery attempt %u/%u mask=0x%08lx\r\n",
              (unsigned int)ZbApp.saved_channel_rejoin_attempts,
              (unsigned int)ZB_SAVED_CHANNEL_REJOIN_ATTEMPTS,
              (unsigned long)ZbApp.rejoin_channel_mask);
  }
  /*
   * Both startup helpers run nested sequencer idle loops. Keep CPU1 in normal
   * Sleep during the synchronous transaction so an IPCC wake cannot expose a
   * transient HSI/PCLK2 baud-rate window on USART1.
   */
  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_DISABLE);
  ZbApp.startup_wait_active = 1U;
  if (rejoin_existing != 0U)
  {
    if ((ZbApp.rejoin_via_startup != 0U) &&
        (ZbApp.rejoin_identity_valid != 0U))
    {
      /* ZbStartupRejoin requires the stack still to consider itself joined.
         After NO_NETWORKS it can reject later RTC retries immediately. Use
         ST's disconnected-device rejoin form, retaining the persisted PAN,
         security material and IEEE identity; never reset or factory-join. */
      ZbStartupConfigGetProDefaults(&config);
      config.startupControl = ZbStartTypeRejoin;
      config.shortAddress = ZbApp.rejoin_short_address;
      config.panId = ZbApp.rejoin_pan_id;
      config.extendedPanId = ZbApp.rejoin_ext_pan_id;
      config.networkManagerAddress = ZbApp.rejoin_manager_address;
      config.channelList.count = 1U;
      config.channelList.list[0].page = 0U;
      config.channelList.list[0].channelMask =
          ((ZbApp.restore_recovery_active != 0U) &&
           (ZbApp.rejoin_channel_mask != 0U)) ?
          ZbApp.rejoin_channel_mask : ZB_ALL_CHANNELS_MASK;
      config.capability &= ~(MCP_ASSOC_CAP_DEV_TYPE |
                             MCP_ASSOC_CAP_ALT_COORD |
                             MCP_ASSOC_CAP_RXONIDLE);
      config.fastPollPeriod = 250U;
      config.endDeviceTimeout = 0xFFU;
      if (ZbApp.restore_recovery_active != 0U)
      {
        ZbApp_Log("ZIGBEE: saved-channel startup rejoin using saved PAN and key\r\n");
      }
      else
      {
        ZbApp_Log("ZIGBEE: all-channel startup rejoin using saved PAN and key\r\n");
      }
      status = ZbStartupWait(ZbApp.zb, &config);
      if ((status == ZB_NWK_STATUS_INVALID_REQUEST) ||
          (status == ZB_NWK_STATUS_ALREADY_PRESENT))
      {
        /* This firmware still sees a joined stack: return to the normal
           secure-rejoin API on the next scheduled attempt. */
        ZbApp.rejoin_via_startup = 0U;
      }
    }
    else
    {
      /* Keep the restored PAN, network key and IEEE identity. A secure
         rejoin refreshes the parent without duplicating the gateway entry. */
      ZbApp_Log("ZIGBEE: secure rejoin using persisted network\r\n");
      status = ZbStartupRejoinWait(ZbApp.zb);
      if ((status == ZB_NWK_STATUS_NO_NETWORKS) &&
          (ZbApp.rejoin_identity_valid != 0U))
      {
        ZbApp.rejoin_via_startup = 1U;
        ZbApp_Log("ZIGBEE: normal rejoin found no network; startup rejoin queued\r\n");
      }
    }
  }
  else
  {
    ZbSetLogging(ZbApp.zb, ZB_LOG_MASK_LEVEL_3, NULL);
    ZbStartupConfigGetProDefaults(&config);
    config.startupControl = ZbStartTypeJoin;
    memcpy(config.security.preconfiguredLinkKey, sec_key_ha, ZB_SEC_KEYSIZE);
    config.channelList.count = 1U;
    config.channelList.list[0].page = 0U;
    config.channelList.list[0].channelMask = ZB_ALL_CHANNELS_MASK;
    /* Sleepy end device: the parent buffers downlink data until our RTC poll. */
    config.capability &= ~(MCP_ASSOC_CAP_DEV_TYPE |
                           MCP_ASSOC_CAP_ALT_COORD |
                           MCP_ASSOC_CAP_RXONIDLE);
    config.fastPollPeriod = 250U;
    config.endDeviceTimeout = 0xFFU;

    ZbApp_Log("ZIGBEE: scanning channels 11-26\r\n");
    status = ZbStartupWait(ZbApp.zb, &config);
  }
  ZbApp.startup_wait_active = 0U;
  if (admission_expired != 0U) {
    ZbApp_AdmissionTimeoutTask();
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
    return;
  }
  if (ZbApp.radio_control_deferred != 0U)
  {
    ZbApp.radio_control_deferred = 0U;
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_RADIO_CONTROL, CFG_SCH_PRIO_0);
  }
  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
  /* Keep the persisted blob until a replacement network has really joined.
     Startup recovery is nevertheless bounded: after two attempts on the
     saved channel, reset only the live stack state and begin a normal
     all-channel join. A power loss before that new join succeeds can still
     restore the previous blob on the next boot. */
  if ((status == ZB_STATUS_SUCCESS) || (rejoin_existing == 0U))
  {
    ZbApp.join_status = status;
  }
  if (status == ZB_STATUS_SUCCESS)
  {
    if (rejoin_existing != 0U)
    {
      /* ZbStartup(ZbStartTypeRejoin) can report success as soon as the local
         stack is restarted. Do not treat that as proof of a reachable parent,
         or upload records until an APS confirm from the coordinator succeeds.
         Security persistence is maintained independently of reachability. */
      ZbApp.parent_online = 0U;
      ZbApp.parent_state_known = 0U;
      ZbApp.rejoin_probe_pending = 1U;
      ZbApp.rejoin_probe_rtc = rtc_triggered;
      if (admission_phase == 1U && admission_expired == 0U)
      {
        HW_TS_Stop(admission_timer_id);
        ZbApp_StartTimer(admission_timer_id, ZB_RESTORED_PARENT_VERIFY_MS);
        ZbApp_Log("ZIGBEE: rejoin verification budget renewed (10000 ms)\r\n");
      }
      ZbApp_Log("ZIGBEE: rejoin candidate; waiting for coordinator APS confirmation\r\n");
      ZbApp_VerifyParent();
      if (ZbApp.parent_sync_pending == 0U)
      {
        ZbApp.rejoin_probe_pending = 0U;
        ZbApp.rejoin_probe_rtc = 0U;
        if (ZbApp.commissioning_window_open != 0U)
        {
          ZbApp_StartTimer(ZbApp.join_retry_timer,
                           ZB_COMMISSIONING_JOIN_RETRY_MS);
        }
        else if ((ZbApp.rtc_join_active != 0U) &&
                 (ZbApp.rtc_join_attempts < ZB_RTC_JOIN_MAX_ATTEMPTS))
        {
          ZbApp_StartTimer(ZbApp.join_retry_timer, ZB_JOIN_RETRY_MS);
        }
        else if (ZbApp.rtc_join_active != 0U)
        {
          ZbApp_CompleteJoinFailureCycle();
        }
      }
      return;
    }
    ZbApp.rejoin_via_startup = 0U;
    ZbApp.parent_online = 1U;
    ZbApp.parent_state_known = 1U;
    ZbApp.rtc_join_active = 0U;
    ZbApp.rtc_join_attempts = 0U;
    ZbApp_Log("ZIGBEE: join success, endpoint=1 coordinator_endpoint=8\r\n");
    if ((admission_phase == 2U) && (network_authorized == 0U))
    {
      /* The 30-second budget limits discovery only. Once joined, remain
       * reachable and idle until the gateway either authorizes this node or
       * sends a standard Leave while closing the scan. */
      HW_TS_Stop(admission_timer_id);
      admission_expired = 0U;
      ZbApp.commissioning_window_open = 0U;
      ZbApp_StartAdmissionFastPoll();
      Custom_APP_ZigbeeActivityChanged();
      ZbApp_Log("ZIGBEE: joined as pending node; authorization wait has no timeout\r\n");
    }
    /* Uplink-only sleepy node: the 60-second report is also the parent
       keepalive, so CPU2 is not woken by a separate maintenance poll. */
    /* A successful fresh Join normally makes the stack send Device Announce,
       but that notification can be missed around a rapid Leave/recommission
       cycle. Send one explicit standard announcement as well so the gateway
       can always pair it with the coordinator's current Neighbor Table. The
       backend de-duplicates announcements by IEEE address. */
    ZbApp.manual_announce_required = 1U;
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ANNOUNCE, CFG_SCH_PRIO_0);
    if (rtc_triggered != 0U)
    {
      ZbApp.reporting_active = 1U;
      ZbApp.monitor_state = (ZbApp.monitor_enabled != 0U) ?
                            ZB_MONITOR_WAITING : ZB_MONITOR_SLEEP;
      ZbApp.offline_capture_requested = 0U;
      ZbApp_SendPendingSnapshot();
      if ((ZbApp.upload_allowed != 0U) || v2_tx_active(&ZbVibrationUpload))
      {
        UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_UPLOAD, CFG_SCH_PRIO_0);
      }
      else if (ZbApp.monitor_enabled != 0U)
      {
        UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
      }
      ZbApp_Log("ZIGBEE: RTC rejoin succeeded; pending and current records queued\r\n");
    }
    else
    {
      ZbApp_StartAutonomousReporting();
    }
#if ((CFG_ZIGBEE_NVM_ENABLED != 0U) && \
     (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U))
    if (ZbApp.nvm_ready != 0U)
    {
      ZbApp_EnablePersistence();
      uint32_t persist_delay = network_authorized != 0U ?
          ZB_PERSIST_SAVE_DELAY_MS : ZB_PENDING_PERSIST_DELAY_MS;
      ZbApp_SchedulePersistSave(persist_delay);
      ZbApp_Log("ZIGBEE: %s network persistence scheduled (%lu ms)\r\n",
                network_authorized != 0U ? "authorized" : "pending",
                (unsigned long)persist_delay);
    }
#elif (CFG_ZIGBEE_NVM_ENABLED != 0U)
    /*
     * CPU2 firmware 1.23.0 can leave the synchronous persistence command
     * waiting indefinitely immediately after a sleepy child joins. Zigbee
     * notifications still run inside that wait, so the coordinator sees a
     * successful Default Response while the CPU1 report task never runs.
     * Keep the sequencer free until persistence is reintroduced through a
     * non-blocking/verified path.
     */
    ZbApp_Log("ZIGBEE: sleepy-node persistence autosave deferred; scheduler ready\r\n");
#endif
    return;
  }
  if ((rejoin_existing != 0U) &&
      (ZbApp.restore_recovery_active != 0U) &&
      (ZbApp.saved_channel_rejoin_attempts >=
       ZB_SAVED_CHANNEL_REJOIN_ATTEMPTS))
  {
    ZbApp_StartFreshJoinFallback();
    return;
  }
  if (ZbApp.commissioning_window_open != 0U)
  {
    ZbApp_Log("ZIGBEE: join failed status=0x%02x; retrying all channels in %lu ms\r\n",
              status, (unsigned long)ZB_COMMISSIONING_JOIN_RETRY_MS);
    /* During the power-up commissioning window keep the receiver searching.
       A short scheduler yield preserves BLE/IPCC processing, then the next
       all-channel scan begins without the former five-second dead time. */
    ZbApp_StartTimer(ZbApp.join_retry_timer,
                     ZB_COMMISSIONING_JOIN_RETRY_MS);
  }
  else if ((rejoin_existing == 0U) &&
           (ZbApp.rtc_join_active == 0U) &&
           (ZbApp.join_blocked_until_reset == 0U))
  {
    /* A factory-new node must keep commissioning after BLE advertising has
       stopped.  A bounded interval avoids a tight sequence of 20+ second
       synchronous all-channel scans while still allowing a coordinator that
       opens PermitJoin later to be discovered without resetting the board. */
    ZbApp_Log("ZIGBEE: join failed status=0x%02x; retrying all channels in %lu ms\r\n",
              status, (unsigned long)ZB_JOIN_RETRY_MS);
    ZbApp_StartTimer(ZbApp.join_retry_timer, ZB_JOIN_RETRY_MS);
  }
  else if ((ZbApp.rtc_join_active != 0U) &&
           (ZbApp.rtc_join_attempts < ZB_RTC_JOIN_MAX_ATTEMPTS))
  {
    ZbApp_Log("ZIGBEE: RTC join failed status=0x%02x; retry in 5 s\r\n",
              status);
    ZbApp_StartTimer(ZbApp.join_retry_timer, ZB_JOIN_RETRY_MS);
  }
  else if (ZbApp.rtc_join_active != 0U)
  {
    ZbApp_Log("ZIGBEE: RTC join failed status=0x%02x; 3 attempts exhausted\r\n",
              status);
    ZbApp_CompleteJoinFailureCycle();
  }
}

static enum ZclStatusCodeT ZbApp_CommandHandler(struct ZbZclClusterT *cluster,
                                                struct ZbZclHeaderT *header,
                                                struct ZbApsdeDataIndT *data_indication)
{
  if (admission_expired || ZbApp.join_blocked_until_reset)
    return ZCL_STATUS_FAILURE;
  const uint8_t *payload = (data_indication != NULL) ?
                           data_indication->asdu : NULL;
  uint16_t length = (data_indication != NULL) ?
                    data_indication->asduLength : 0U;
  uint8_t trusted_coordinator = ZbApp_IsTrustedCoordinator(data_indication);

  if (header->cmdId == 0x11U || header->cmdId == 0x12U) {
    if (!trusted_coordinator || payload == NULL ||
        admission_phase != 1U || ZbApp.restore_recovery_active == 0U ||
        ZbApp.join_status != ZB_STATUS_SUCCESS)
      return ZCL_STATUS_INVALID_FIELD;
    if (header->cmdId == 0x12U) {
      if (length != 2U || ((uint16_t)payload[0] | ((uint16_t)payload[1] << 8U)) != ZbShortAddress(ZbApp.zb))
        return ZCL_STATUS_INVALID_FIELD;
      /* Announce receipt prompts a token-bearing request in task context. */
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ANNOUNCE, CFG_SCH_PRIO_0);
      return ZCL_STATUS_SUCCESS;
    }
    if (length != 10U || !recovery_token_valid ||
        memcmp(payload, recovery_token, sizeof(recovery_token)) != 0 ||
        ((uint16_t)payload[8] | ((uint16_t)payload[9] << 8U)) != ZbShortAddress(ZbApp.zb))
      return ZCL_STATUS_INVALID_FIELD;
    ZbApp_Log("ZIGBEE: recovery ACK received; old network retained\r\n");
    ZbApp_SetParentOnline(1U, ZB_STATUS_SUCCESS);
    return ZCL_STATUS_SUCCESS;
  }

  if (header->cmdId == 0x10U) {
    if (!trusted_coordinator || length != 2U || payload == NULL)
      return ZCL_STATUS_INVALID_FIELD;
    /* Probe only: never authorizes acquisition or changes monitoring cadence. */
    if (!ZbApp.config_valid && !ZbApp.radio_exclusive)
      (void)ZbNwkFastPollRequest(ZbApp.zb, 250U, 10000U);
    return ZbApp_SendCommand(0x9BU, payload, 2U);
  }

  if ((header->cmdId == V2_ACK) ||
      (header->cmdId == ZB_CMD_MONITOR_CONTROL) ||
      (header->cmdId == ZB_CMD_MONITOR_CONFIG) ||
      (header->cmdId == ZB_CMD_REPORT_START) ||
      (header->cmdId == ZB_CMD_REPORT_STOP))
  {
    if (trusted_coordinator == 0U)
    {
      return ZCL_STATUS_INVALID_FIELD;
    }
    /* A valid application command from endpoint 8 is stronger evidence than
       an older queued APS probe. Keep the restored association online. */
    if (ZbApp.rejoin_probe_pending != 0U)
    {
      ZbApp.rejoin_probe_pending = 0U;
      ZbApp.rejoin_probe_rtc = 0U;
      ZbApp.rejoin_via_startup = 0U;
      ZbApp.rtc_join_active = 0U;
      ZbApp.rtc_join_attempts = 0U;
    }
    ZbApp_SetParentOnline(1U, ZB_STATUS_SUCCESS);
  }

  if ((header->cmdId == V2_ACK) && (payload != NULL) &&
      (length == V2_ACK_LEN))
  {
    if (payload[0] == V2_VERSION && payload[1] == V2_KIND_FEATURES)
    {
      if (ZbDeferred.active && ZbDeferred.raw_done &&
          ZbDeferred.features_ready && ZbDeferred.queued &&
          v2_u32(payload+2U) == ZbDeferred.batch)
      {
        ZbDeferred.confirmed=(payload[8] == V2_OK);
        if (!ZbDeferred.confirmed)
          ZbDeferred.deadline=HAL_GetTick()+1000U;
      }
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_UPLOAD, CFG_SCH_PRIO_0);
      return ZCL_STATUS_SUCCESS;
    }
    v2_tx_receive(&ZbVibrationUpload, payload, length, HAL_GetTick());
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_UPLOAD, CFG_SCH_PRIO_0);
    return ZCL_STATUS_SUCCESS;
  }

  if ((header->cmdId == ZB_CMD_MONITOR_CONTROL) &&
      (payload != NULL) && (length >= 1U))
  {
    if ((payload[0] == 0x01U) && (length >= 2U))
    {
      uint8_t was_enabled = ZbApp.monitor_enabled;
      ZbApp.monitor_enabled = (payload[1] != 0U) ? 1U : 0U;
      ZbApp_CheckAuthorizationBudget();
      if (ZbApp.monitor_enabled == 0U)
      {
        sampling_cadence.active=0;sampling_due=0;
        if(sampling_timer!=0xFFU)HW_TS_Stop(sampling_timer);
        ZbApp.monitor_state = ZB_MONITOR_SLEEP;
        if ((ZbApp.report_timer != 0xFFU) &&
            (ZbApp.audio_waiting == 0U))
        {
          HW_TS_Stop(ZbApp.report_timer);
        }
      }
      else if (ZbApp.monitor_state == ZB_MONITOR_SLEEP)
      {
        ZbApp.monitor_state = ZB_MONITOR_WAITING;
      }
      (void)ZbApp_SendMonitorState(ZbApp.monitor_state);
      if ((ZbApp.monitor_enabled != 0U) && (was_enabled == 0U))
      {
        sampling_cadence.active=0;sampling_due=0;
        if (ZbApp.config_valid != 0U)
        {
          UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
        }
        else
        {
          ZbApp_RequestConfig(ZB_CONFIG_QUERY_JOIN);
        }
      }
      return ZCL_STATUS_SUCCESS;
    }
    if (payload[0] == 0x02U)
    {
      if (ZbApp.monitor_enabled == 0U)
      {
        return ZCL_STATUS_FAILURE;
      }
      if (ZbApp.report_timer != 0xFFU)
      {
        HW_TS_Stop(ZbApp.report_timer);
      }
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
      return ZCL_STATUS_SUCCESS;
    }
    return ZCL_STATUS_INVALID_FIELD;
  }

  if ((header->cmdId == ZB_CMD_MONITOR_CONFIG) &&
      (payload != NULL) &&
      ((length == 20U) || (length == 26U) || (length == 28U) ||
       (length == 29U)))
  {
    ZbMonitorConfig_t next;
    uint8_t had_pending_query = ZbApp.config_query_pending;
    next.auto_interval_sec = (uint32_t)payload[0] |
        ((uint32_t)payload[1] << 8U) | ((uint32_t)payload[2] << 16U) |
        ((uint32_t)payload[3] << 24U);
    next.poll_interval_sec = (uint16_t)payload[4] |
        ((uint16_t)payload[5] << 8U);
    next.vibration_rate_hz = (uint16_t)payload[6] |
        ((uint16_t)payload[7] << 8U);
    next.digital_mic_rate_hz = (uint16_t)payload[8] |
        ((uint16_t)payload[9] << 8U);
    next.analog_mic_rate_hz = (uint32_t)payload[10] |
        ((uint32_t)payload[11] << 8U) | ((uint32_t)payload[12] << 16U) |
        ((uint32_t)payload[13] << 24U);
    next.vibration_window_ms = (uint16_t)payload[14] |
        ((uint16_t)payload[15] << 8U);
    next.digital_mic_window_ms = (uint16_t)payload[16] |
        ((uint16_t)payload[17] << 8U);
    next.analog_mic_window_ms = (uint16_t)payload[18] |
        ((uint16_t)payload[19] << 8U);
    next.raw_upload_every_cycles = 1U;
    next.raw_upload_generation = 0U;
    next.sensor_enable_mask = CUSTOM_APP_ZB_SENSOR_ALL;
    uint8_t transaction_flags = 0U;
    uint8_t migration_target = 0U;
    if (length >= 26U)
    {
      next.raw_upload_every_cycles = (uint16_t)payload[20] |
          ((uint16_t)payload[21] << 8U);
      next.raw_upload_generation = (uint32_t)payload[22] |
          ((uint32_t)payload[23] << 8U) |
          ((uint32_t)payload[24] << 16U) |
          ((uint32_t)payload[25] << 24U);
    }
    if (length >= 28U)
    {
      transaction_flags = payload[26];
      migration_target = payload[27];
    }
    if (length == 29U)
    {
      next.sensor_enable_mask = payload[28];
    }
    if ((next.auto_interval_sec < 300U) || (next.auto_interval_sec > 2147483U) ||
        (next.poll_interval_sec == 0U) ||
        (next.vibration_rate_hz != 26667U) ||
        (next.digital_mic_rate_hz != 46875U) ||
        (next.analog_mic_rate_hz != 256000U) ||
        ((next.vibration_window_ms != 500U) &&
         (next.vibration_window_ms != 1000U) &&
         (next.vibration_window_ms != 1500U)) ||
        ((next.digital_mic_window_ms != 500U) &&
         (next.digital_mic_window_ms != 1000U) &&
         (next.digital_mic_window_ms != 1500U) &&
         (next.digital_mic_window_ms != 2000U)) ||
        ((next.analog_mic_window_ms != 500U) &&
         (next.analog_mic_window_ms != 1000U) &&
         (next.analog_mic_window_ms != 1500U) &&
         (next.analog_mic_window_ms != 2000U)) ||
         (next.raw_upload_every_cycles == 0U) ||
         ((next.sensor_enable_mask & (uint8_t)~CUSTOM_APP_ZB_SENSOR_ALL) != 0U) ||
         (((transaction_flags & ZB_CONFIG_FLAG_CHANNEL_MIGRATION) != 0U) &&
          ((migration_target < 11U) || (migration_target > 26U))))
    {
      ZbApp_Log("ZIGBEE: invalid sampling configuration length=%u\r\n",
                (unsigned int)length);
      return ZCL_STATUS_INVALID_VALUE;
    }
    if ((next.raw_upload_generation != ZbApp.monitor_config.raw_upload_generation) ||
        (next.raw_upload_every_cycles != ZbApp.monitor_config.raw_upload_every_cycles))
    {
      ZbApp.raw_upload_cycle_counter = 0U;
      ZbApp_Log("ZIGBEE: raw waveform cadence reset generation=%lu every=%u cycles\r\n",
                (unsigned long)next.raw_upload_generation,
                (unsigned int)next.raw_upload_every_cycles);
    }
    ZbApp.monitor_config = next;
    sampling_cadence_change(&sampling_cadence,next.auto_interval_sec*1000U);
    ZbApp_ArmSamplingTimer();
    ZbApp_Log("ZIGBEE: sensor policy mask=0x%02x vibration=%u dmic=%u amic=%u\r\n",
              (unsigned int)next.sensor_enable_mask,
              (unsigned int)((next.sensor_enable_mask & CUSTOM_APP_ZB_SENSOR_VIBRATION) != 0U),
              (unsigned int)((next.sensor_enable_mask & CUSTOM_APP_ZB_SENSOR_DIGITAL_MIC) != 0U),
              (unsigned int)((next.sensor_enable_mask & CUSTOM_APP_ZB_SENSOR_ANALOG_MIC) != 0U));
    ZbApp.config_valid = 1U;
    ZbApp_CheckAuthorizationBudget();
    uint8_t was_monitoring = ZbApp.monitor_enabled;
    if ((transaction_flags & ZB_CONFIG_FLAG_MONITOR_PRESENT) != 0U) {
      ZbApp.monitor_enabled = (transaction_flags & ZB_CONFIG_FLAG_MONITOR_ENABLED) != 0U;
      if (!ZbApp.monitor_enabled) {
        sampling_cadence.active = 0; sampling_due = 0;
        if (sampling_timer != 0xFFU) HW_TS_Stop(sampling_timer);
        ZbApp.monitor_state = ZB_MONITOR_SLEEP;
      } else {
        monitor_standby_enabled(&monitor_standby);
        UTIL_LPM_SetStopMode(1U << CFG_LPM_MONITOR_STANDBY, UTIL_LPM_ENABLE);
        ZbApp.monitor_state = ZB_MONITOR_WAITING;
      }
    }
    /* Legacy 0x0C only updates sampling parameters. Authorization and the enable
       state are carried by 0x0B, so an unconfirmed node cannot start merely
       because it received a configuration frame. */
    if (ZbApp.config_query_pending != 0U)
    {
      uint8_t reason = ZbApp.config_query_reason;
      if (ZbApp.report_timer != 0xFFU)
      {
        HW_TS_Stop(ZbApp.report_timer);
      }
      if ((reason == ZB_CONFIG_QUERY_COLLECTION) &&
          ((transaction_flags & ZB_CONFIG_FLAG_CHANNEL_MIGRATION) != 0U))
      {
        ZbApp.channel_migration_wait = 1U;
        ZbApp.channel_migration_target = migration_target;
        ZbApp.config_query_attempts = 0U;
        ZbApp_StartTimer(ZbApp.report_timer, ZB_CONFIG_QUERY_RETRY_MS);
        ZbApp_Log("ZIGBEE: channel migration pending target=%u; radio remains active until gateway confirms\r\n",
                  (unsigned int)migration_target);
      }
      else
      {
        ZbApp.config_query_pending = 0U;
        ZbApp.channel_migration_wait = 0U;
        ZbApp.channel_migration_target = 0U;
        ZbApp_StopConfigTransaction();
      }
      if (((reason == ZB_CONFIG_QUERY_JOIN) || (reason == ZB_CONFIG_QUERY_IDLE)) &&
          (ZbApp.monitor_enabled != 0U) &&
          (ZbApp.config_query_pending == 0U))
      {
        /* Covers the defensive case where the explicit enable command was
           delivered before its preceding configuration frame. */
        UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
      }
      else if ((reason == ZB_CONFIG_QUERY_COLLECTION) &&
               (ZbApp.monitor_enabled != 0U) &&
               (ZbApp.config_query_pending == 0U))
      {
        ZbApp_ScheduleNextReport();
      }
    }
    if ((transaction_flags & ZB_CONFIG_FLAG_MONITOR_PRESENT) != 0U && !ZbApp.config_query_pending) {
      if (!ZbApp.monitor_enabled) ZbApp_ScheduleMonitorStandby();
      else if (!had_pending_query && !was_monitoring) {
        if (ZbApp.report_timer != 0xFFU) HW_TS_Stop(ZbApp.report_timer);
        UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
      }
      ZbApp_Log("ZIGBEE: gateway monitoring switch=%u (read with configuration)\r\n", ZbApp.monitor_enabled);
    }
    UNUSED(had_pending_query);
    ZbApp_Log("ZIGBEE: sampling configuration accepted interval=%lus vibration=%ums dmic=%ums amic=%ums raw_every=%u transaction=0x%02x target=%u\r\n",
              (unsigned long)next.auto_interval_sec,
              (unsigned int)next.vibration_window_ms,
              (unsigned int)next.digital_mic_window_ms,
              (unsigned int)next.analog_mic_window_ms,
              (unsigned int)next.raw_upload_every_cycles,
              (unsigned int)transaction_flags,
              (unsigned int)migration_target);
    return ZCL_STATUS_SUCCESS;
  }

  if (header->cmdId == ZB_CMD_REPORT_START)
  {
    ZbApp.monitor_enabled = 1U;
    ZbApp_CheckAuthorizationBudget();
    if (ZbApp.config_valid != 0U)
    {
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
    }
    else
    {
      ZbApp_RequestConfig(ZB_CONFIG_QUERY_JOIN);
    }
    return ZCL_STATUS_SUCCESS;
  }
  if (header->cmdId == ZB_CMD_REPORT_STOP)
  {
    sampling_cadence.active=0;sampling_due=0;
    if(sampling_timer!=0xFFU)HW_TS_Stop(sampling_timer);
    ZbApp.monitor_enabled = 0U;
    ZbApp.monitor_state = ZB_MONITOR_SLEEP;
    ZbApp.config_query_pending = 0U;
    ZbApp.channel_migration_wait = 0U;
    ZbApp_StopConfigTransaction();
    return ZCL_STATUS_SUCCESS;
  }

  if (ZbApp.temperature_command != NULL)
  {
    return ZbApp.temperature_command(cluster, header, data_indication);
  }
  return ZCL_STATUS_UNSUPP_COMMAND;
}

static void ZbApp_ReportTask(void)
{
  Custom_APP_ZigbeeSnapshot_t snapshot;
  uint8_t capture_without_network = ZbApp.offline_capture_requested;
  uint8_t raw_upload_due = 0U;

  if (sampling_due != 0U)
  {
    sampling_due=0;
    unsigned due=sampling_cadence_due(&sampling_cadence,APP_ElapsedMs());
    ZbApp_ArmSamplingTimer();
    if (!due) return;
    if (ZbApp.monitor_enabled==0U || ZbApp.audio_waiting ||
        ZbApp.upload_allowed || v2_tx_active(&ZbVibrationUpload) ||
        ZbApp.config_query_pending || ZbApp.restore_pending || ZbApp.radio_exclusive)
    {
      ZbApp_Log("ZIGBEE: sampling deadline skipped busy=%u count=%u next_rtc=%lu ms\r\n",
                ZbApp.monitor_state,due,(unsigned long)sampling_cadence.next);
      /* A coalesced report task may also be completing the current capture. */
      if (!ZbApp.audio_waiting) return;
    }
    if (due>1U)ZbApp_Log("ZIGBEE: sampling missed deadlines=%u; no catch-up burst\r\n",due-1U);
  }

  if (recovery_config_pending != 0U) {
    recovery_config_pending = 0U;
    ZbApp_RequestConfig(ZB_CONFIG_QUERY_JOIN);
    return;
  }

  if (ZbApp.radio_exclusive != 0U)
  {
    return;
  }
  if (ZbApp.commissioning_window_open != 0U)
  {
    /* Configuration and authorization must finish before acquiring sensors. */
    if (ZbApp.monitor_enabled != 0U)
    {
      ZbApp.monitor_state = ZB_MONITOR_WAITING;
    }
    return;
  }
  if ((ZbApp.monitor_enabled == 0U) && (ZbApp.audio_waiting == 0U))
  {
    ZbApp.monitor_state = ZB_MONITOR_SLEEP;
    if (network_authorized && !ZbApp.config_query_pending)
    {
      /* Reuse the existing idle-query cadence (3 s during the first awake
         standby, 5 min after STOP2). No extra timer may wake a sleeping node.
         TI appends the RSSI/LQI measured from this fresh uplink; DC is the
         real board supply type, not a fabricated battery percentage.
         Status is best-effort: never enqueue it behind an active upload. */
      if (APP_ZIGBEE_IsConnected() && !APP_ZIGBEE_IsTransmitting() &&
          !v2_tx_active(&ZbVibrationUpload) && !ZbDeferred.active &&
          !ZbApp.parent_sync_pending)
        (void)ZbApp_SendMonitorState(ZB_MONITOR_SLEEP);
      ZbApp_RequestConfig(ZB_CONFIG_QUERY_IDLE);
    }
    return;
  }

  if ((ZbApp.audio_waiting == 0U) &&
      ((ZbApp.upload_allowed != 0U) || v2_tx_active(&ZbVibrationUpload)))
  {
    /* A duplicate enable/report event must not start a second capture while
       the previous Flash batch still owns the uploader. */
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_UPLOAD, CFG_SCH_PRIO_0);
    return;
  }
  if ((ZbApp.audio_waiting == 0U) && (ZbApp.config_valid == 0U)) return;
  if ((ZbApp.audio_waiting != 0U) && (APP_ZIGBEE_IsConnected() == 0U))
    capture_without_network = 1U;

  if (ZbApp.restore_pending != 0U)
  {
    /* The RTC deadline must not turn an unfinished persisted startup into a
       concurrent factory/rejoin request. Its callback owns the next step. */
    ZbApp_ScheduleNextReport();
    return;
  }

  if ((capture_without_network == 0U) &&
      (APP_ZIGBEE_IsConnected() == 0U))
  {
    if ((ZbApp.rtc_join_active == 0U) &&
        (ZbApp.join_blocked_until_reset == 0U))
    {
      ZbApp.rtc_join_active = 1U;
      ZbApp.rtc_join_attempts = 0U;
      if (ZbApp.join_status != ZB_STATUS_SUCCESS)
      {
        ZbApp_PrepareFreshJoin();
      }
      ZbApp_Log("ZIGBEE: RTC sample due; coordinator offline, starting rejoin/join cycle\r\n");
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
    }
    else if (ZbApp.join_blocked_until_reset != 0U)
    {
      ZbApp_ScheduleNextReport();
    }
    return;
  }

  if (ZbApp.audio_waiting == 0U)
  {
    if (!sampling_cadence.active)
      sampling_cadence_start(&sampling_cadence,APP_ElapsedMs(),ZbApp.monitor_config.auto_interval_sec*1000U);
    ZbApp_ArmSamplingTimer();
    ZbApp_Log("ZIGBEE: sampling start_rtc=%lu next_rtc=%lu period=%lu ms\r\n",
              (unsigned long)APP_ElapsedMs(),(unsigned long)sampling_cadence.next,
              (unsigned long)sampling_cadence.period);
    ZbApp.collection_requested = 0U;
    if (capture_without_network == 0U)
    {
      ZbApp_SendPendingSnapshot();
    }
    ZbApp.batch_id++;
    if (ZbApp.batch_id == 0U)
    {
      ZbApp.batch_id = 1U;
    }
    ZbApp.monitor_state = ZB_MONITOR_CAPTURING;
    Custom_APP_ZigbeeActivityChanged();
    if (capture_without_network == 0U)
    {
      ZbApp_SendMonitorState(ZbApp.monitor_state);
    }
    if (Custom_APP_ZigbeeBeginSnapshot(
            ZbApp.monitor_config.vibration_rate_hz,
            ZbApp.monitor_config.vibration_window_ms,
            ZbApp.monitor_config.digital_mic_window_ms,
            ZbApp.monitor_config.analog_mic_window_ms,
            ZbApp.monitor_config.sensor_enable_mask,
            ZbApp.batch_id) != 0U)
    {
      ZbApp.audio_waiting = 1U;
      ZbApp_StartTimer(ZbApp.report_timer, ZB_AUDIO_SETTLE_MS);
      ZbApp_Log("ZIGBEE: snapshot batch=%lu, audio then vibration capture started\r\n",
                (unsigned long)ZbApp.batch_id);
      return;
    }
    ZbApp_Log("ZIGBEE: BLE owns streaming sensors; using available values\r\n");
  }

  if (Custom_APP_ZigbeeSnapshotReady() == 0U)
  {
    ZbApp_StartTimer(ZbApp.report_timer, 20U);
    return;
  }
  ZbApp.audio_waiting = 0U;
  Custom_APP_ZigbeeFinishSnapshot(&snapshot);
  if ((snapshot.valid_mask & CUSTOM_APP_ZB_VALID_VIBRATION) != 0U)
  {
    raw_upload_due = ((ZbApp.raw_upload_cycle_counter == 0U) ||
        (ZbApp.raw_upload_cycle_counter >=
         ZbApp.monitor_config.raw_upload_every_cycles)) ? 1U : 0U;
    if (raw_upload_due != 0U)
      ZbApp.raw_upload_cycle_counter = 1U;
    else
      ZbApp.raw_upload_cycle_counter++;
  }
  if ((snapshot.valid_mask & CUSTOM_APP_ZB_VALID_VIBRATION) != 0U &&
      !ZigbeeVibrationStore_SetRawPolicy(ZbApp.batch_id, raw_upload_due))
  {
    snapshot.valid_mask &= (uint8_t)~CUSTOM_APP_ZB_VALID_VIBRATION;
    ZbApp_Log("ZIGBEE: Flash upload policy save failed\r\n");
  }
  if (capture_without_network != 0U)
  {
    ZbApp.pending_snapshot = snapshot;
    ZbApp.pending_batch_id = ZbApp.batch_id;
    ZbApp.pending_snapshot_valid = 1U;
    ZbApp.offline_capture_requested = 0U;
    ZbApp_Log("ZIGBEE: snapshot batch=%lu saved in SRAM for next RTC cycle valid=0x%02x\r\n",
              (unsigned long)ZbApp.batch_id,
              (unsigned int)snapshot.valid_mask);
  }
  else
  {
    ZbApp_SendSnapshot(&snapshot, ZbApp.batch_id);
    if ((snapshot.valid_mask & CUSTOM_APP_ZB_VALID_VIBRATION) != 0U)
    {
      ZbApp.monitor_state = ZB_MONITOR_UPLOADING;
      (void)ZbApp_SendMonitorState(ZbApp.monitor_state);
      ZbApp.upload_allowed = 1U;
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_UPLOAD, CFG_SCH_PRIO_0);
    }
    else
    {
      ZbApp_FinishCollectionCycle(
          ((ZbApp.monitor_config.sensor_enable_mask & CUSTOM_APP_ZB_SENSOR_VIBRATION) == 0U) ? 1U : 0U);
    }
    ZbApp_Log("ZIGBEE: snapshot batch=%lu queued; raw_due=%u valid=0x%02x\r\n",
              (unsigned long)ZbApp.batch_id,
              (unsigned int)raw_upload_due,
              (unsigned int)snapshot.valid_mask);
  }
  /* PA8 ends with acquisition. The queued APS frames keep only the network
     indication and PA4 run indication active until transmission completes. */
  Custom_APP_ZigbeeActivityChanged();
  if (capture_without_network != 0U)
  {
    ZbApp.monitor_state = ZB_MONITOR_WAITING;
    ZbApp_ScheduleNextReport();
  }
}

static void ZbApp_SendSnapshot(const Custom_APP_ZigbeeSnapshot_t *snapshot,
                               uint32_t batch_id)
{
  uint8_t payload[ZB_TX_MAX_PAYLOAD];
  int16_t value;

  ZbApp.report_has_summary_confirm = 0U;

  payload[0] = (uint8_t)batch_id;
  payload[1] = (uint8_t)(batch_id >> 8U);
  payload[2] = (uint8_t)(batch_id >> 16U);
  payload[3] = (uint8_t)(batch_id >> 24U);
  if ((snapshot->valid_mask & CUSTOM_APP_ZB_VALID_TEMPERATURE) != 0U)
  {
    payload[4] = (uint8_t)snapshot->temperature_centi_c;
    payload[5] = (uint8_t)((uint16_t)snapshot->temperature_centi_c >> 8U);
    (void)ZbApp_SendCommand(ZB_CMD_TEMP_REPORT, payload, 6U);
  }
  if ((snapshot->valid_mask & CUSTOM_APP_ZB_VALID_MAGNETIC) != 0U)
  {
    for (uint32_t axis = 0U; axis < 3U; axis++)
    {
      value = ZbApp_Magnetic0p1uTToRaw(snapshot->magnetic_0p1_ut[axis]);
      payload[4U + axis * 2U] = (uint8_t)value;
      payload[5U + axis * 2U] = (uint8_t)((uint16_t)value >> 8U);
    }
    (void)ZbApp_SendCommand(ZB_CMD_MAG_REPORT, payload, 10U);
  }
  if ((snapshot->valid_mask & CUSTOM_APP_ZB_VALID_DIGITAL_MIC) != 0U)
  {
    value = (int16_t)((uint16_t)snapshot->digital_mic_db_spl * 100U);
    payload[4] = (uint8_t)value;
    payload[5] = (uint8_t)((uint16_t)value >> 8U);
    (void)ZbApp_SendCommand(ZB_CMD_DIGITAL_MIC_REPORT, payload, 6U);
  }
  if ((snapshot->valid_mask & CUSTOM_APP_ZB_VALID_ANALOG_MIC) != 0U)
  {
    value = (int16_t)((uint16_t)snapshot->analog_mic_db_spl * 100U);
    payload[4] = (uint8_t)value;
    payload[5] = (uint8_t)((uint16_t)value >> 8U);
    (void)ZbApp_SendCommand(ZB_CMD_ANALOG_MIC_REPORT, payload, 6U);
  }
  if (((snapshot->valid_mask & CUSTOM_APP_ZB_VALID_VIBRATION) != 0U) &&
      (snapshot->vibration_features_ready != 0U))
  {
    uint32_t offset = 10U;

    payload[4] = (uint8_t)snapshot->vibration_rate_hz;
    payload[5] = (uint8_t)(snapshot->vibration_rate_hz >> 8U);
    payload[6] = (uint8_t)snapshot->vibration_sample_count;
    payload[7] = (uint8_t)(snapshot->vibration_sample_count >> 8U);
    payload[8] = (uint8_t)(snapshot->vibration_sample_count >> 16U);
    payload[9] = (uint8_t)(snapshot->vibration_sample_count >> 24U);
    for (uint32_t axis = 0U; axis < 3U; axis++)
    {
      for (uint32_t feature = 0U; feature < 5U; feature++)
      {
        uint32_t encoded =
            (uint32_t)snapshot->vibration_features_milli[axis][feature];
        payload[offset++] = (uint8_t)encoded;
        payload[offset++] = (uint8_t)(encoded >> 8U);
        payload[offset++] = (uint8_t)(encoded >> 16U);
        payload[offset++] = (uint8_t)(encoded >> 24U);
      }
    }
    if (ZbApp_SendCommand(ZB_CMD_VIBRATION_SUMMARY, payload, offset) ==
        ZCL_STATUS_SUCCESS)
    {
      ZbApp.report_has_summary_confirm = 1U;
    }
    ZbApp_Log("ZIGBEE: vibration summary queued samples=%lu\r\n",
              (unsigned long)snapshot->vibration_sample_count);
  }
}

static int16_t ZbApp_Magnetic0p1uTToRaw(int16_t value)
{
  int32_t scaled = (int32_t)value * 2;
  scaled += (scaled >= 0) ? 1 : -1;
  return (int16_t)(scaled / 3);
}

static enum ZclStatusCodeT ZbApp_SendCommand(uint8_t command_id,
                                             const uint8_t *payload,
                                             uint16_t length)
{
  ZbTxFrame_t *frame;

  if ((ZbApp.temperature_server == NULL) ||
      (ZbApp.join_status != ZB_STATUS_SUCCESS) ||
      (payload == NULL) ||
      (length > ZB_TX_MAX_PAYLOAD))
  {
    return ZCL_STATUS_FAILURE;
  }
  if (ZbApp.tx_count >= ZB_TX_QUEUE_DEPTH)
  {
    ZbApp_Log("ZIGBEE: tx queue full, cmd=0x%02x dropped\r\n",
              (unsigned int)command_id);
    return ZCL_STATUS_INSUFFICIENT_SPACE;
  }

  frame = &ZbTxQueue[ZbApp.tx_head];
  frame->command_id = command_id;
  frame->length = (uint8_t)length;
  /* Keep the proven ZCL/APS stack path. Only requests with a real response
     need a callback: the coordinator explicitly ACKs the vibration summary,
     while ordinary sensor and monitor-state reports do not get a ZCL reply.
     The parent probe must retain its original callback for link validation. */
  frame->await_confirm =
      ((command_id == ZB_CMD_VIBRATION_SUMMARY) ||
       ((command_id == ZB_CMD_MONITOR_STATE) &&
        ((ZbApp.parent_sync_pending != 0U) ||
         ((length >= 2U) && (payload[1] == ZB_MONITOR_COMPLETE) &&
          (ZbApp.report_has_summary_confirm == 0U))))) ? 1U : 0U;
  memcpy(frame->payload, payload, length);
  ZbApp.tx_head = (uint8_t)((ZbApp.tx_head + 1U) % ZB_TX_QUEUE_DEPTH);
  ZbApp.tx_count++;
  Custom_APP_ZigbeeActivityChanged();
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_TX, CFG_SCH_PRIO_1);
  return ZCL_STATUS_SUCCESS;
}

static void ZbApp_TxTask(void)
{
  struct ZbZclClusterCommandReqT request;
  ZbTxFrame_t *frame;
  enum ZclStatusCodeT status;

  if (ZbApp.persist_tx_hold != 0U) return;

  if ((ZbApp.radio_exclusive != 0U) ||
      (ZbApp.tx_in_flight >= ZB_TX_MAX_IN_FLIGHT) ||
      (ZbApp.tx_count == 0U))
  {
    return;
  }
  if ((ZbApp.temperature_server == NULL) ||
      (ZbApp.join_status != ZB_STATUS_SUCCESS))
  {
    ZbApp_ResetTxQueue();
    return;
  }

  while ((ZbApp.tx_count != 0U) &&
         (ZbApp.tx_in_flight < ZB_TX_MAX_IN_FLIGHT))
  {
    frame = &ZbTxQueue[ZbApp.tx_tail];
    memset(&request, 0, sizeof(request));
    request.dst.mode = ZB_APSDE_ADDRMODE_SHORT;
    request.dst.nwkAddr = 0x0000U;
    request.dst.endpoint = ZB_COORDINATOR_ENDPOINT;
    request.cmdId = frame->command_id;
    request.noDefaultResp = ZCL_NO_DEFAULT_RESPONSE_TRUE;
    request.payload = frame->payload;
    request.length = frame->length;
    if (frame->await_confirm != 0U)
    {
      ZbApp.tx_in_flight++;
    }
    status = ZbZclClusterCommandReq(ZbApp.temperature_server,
                                    &request,
                                    (frame->await_confirm != 0U) ?
                                        ZbApp_CommandConfirm : NULL,
                                    (void *)(uintptr_t)((command_confirm_generation << 8) | frame->command_id));
    if (status != ZCL_STATUS_SUCCESS)
    {
      if (frame->await_confirm != 0U)
      {
        ZbApp.tx_in_flight--;
      }
      if ((frame->command_id == ZB_CMD_MONITOR_STATE) &&
          (ZbApp.parent_sync_pending != 0U))
      {
        ZbApp.parent_sync_pending = 0U;
        if (ZbApp.rejoin_probe_pending != 0U)
        {
          ZbApp.rejoin_probe_pending = 0U;
          ZbApp.rejoin_probe_rtc = 0U;
          if (ZbApp.commissioning_window_open != 0U)
          {
            ZbApp_StartTimer(ZbApp.join_retry_timer,
                             ZB_COMMISSIONING_JOIN_RETRY_MS);
          }
          else if ((ZbApp.rtc_join_active != 0U) &&
                   (ZbApp.rtc_join_attempts < ZB_RTC_JOIN_MAX_ATTEMPTS))
          {
            ZbApp_StartTimer(ZbApp.join_retry_timer, ZB_JOIN_RETRY_MS);
          }
          else if (ZbApp.rtc_join_active != 0U)
          {
            ZbApp_CompleteJoinFailureCycle();
          }
        }
      }
      ZbApp_Log("ZIGBEE: tx cmd=0x%02x submit failed zcl=0x%02x\r\n",
                (unsigned int)frame->command_id,
                (unsigned int)status);
      if (ZbApp.radio_control_deferred != 0U)
      {
        UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_RADIO_CONTROL,
                         CFG_SCH_PRIO_0);
      }
    }
    ZbApp.tx_tail = (uint8_t)((ZbApp.tx_tail + 1U) % ZB_TX_QUEUE_DEPTH);
    ZbApp.tx_count--;
  }
  Custom_APP_ZigbeeActivityChanged();
}

static void ZbApp_UploadConfirm(struct ZbApsdeDataConfT *confirm, void *arg)
{
  ZbUploadRadioSlot_t *slot = (ZbUploadRadioSlot_t *)arg;
  if (confirm != NULL)
  {
    v2_tx_radio_result(&ZbVibrationUpload, confirm->status == ZB_STATUS_SUCCESS, HAL_GetTick());
    if (confirm->status != ZB_STATUS_SUCCESS)
      ZbApp_Log("ZIGBEE: V2 APS confirm failed=0x%02x batch=%lu block=%u pace=%lu ms\r\n",
                (unsigned int)confirm->status, (unsigned long)ZbDeferred.batch,
                (unsigned int)ZbVibrationUpload.index,
                (unsigned long)ZbVibrationUpload.pace_ms);
  }
  if (slot != NULL)
  {
    slot->busy = 0U;
  }
  Custom_APP_ZigbeeActivityChanged();
}

static int ZbApp_UploadSend(void *context, uint8_t command,
                            const uint8_t *data, size_t length)
{
  struct ZbApsdeDataReqT request;
  ZbUploadRadioSlot_t *slot = NULL;
  enum ZbStatusCodeT status;
  UNUSED(context);

  if ((data == NULL) || (length > 79U) ||
      (ZbApp.radio_exclusive != 0U) ||
      (APP_ZIGBEE_IsConnected() == 0U))
  {
    return 0;
  }
  /* One outstanding upload frame: controls and window queries cannot overtake
     fragments still owned by APS. Leave the ordinary control queue available. */
  for (unsigned index = 0U; index < 3U; index++)
    if (ZbUploadSlots[index].busy != 0U) return 0;
  for (unsigned index = 0U; index < 3U; index++)
  {
    if (ZbUploadSlots[index].busy == 0U)
    {
      slot = &ZbUploadSlots[index];
      break;
    }
  }
  if (slot == NULL)
  {
    return 0;
  }

  slot->frame[0] = 0x19U;
  slot->frame[1] = ZbApp.upload_sequence++;
  slot->frame[2] = command;
  memcpy(slot->frame + 3U, data, length);
  memset(&request, 0, sizeof(request));
  request.dst.mode = ZB_APSDE_ADDRMODE_SHORT;
  request.dst.nwkAddr = 0x0000U;
  request.dst.endpoint = ZB_COORDINATOR_ENDPOINT;
  request.srcEndpt = ZB_APP_ENDPOINT;
  request.profileId = 0x0104U;
  request.clusterId = 0x0402U;
  request.asdu = slot->frame;
  request.asduLength = (uint16_t)(length + 3U);
  request.txOptions = ZB_APSDE_DATAREQ_TXOPTIONS_SECURITY |
                      ZB_APSDE_DATAREQ_TXOPTIONS_NWKKEY;
  /* 0x94 is recovered by the gateway's missing-fragment bitmap. Keep APS
     ACK only on batch/block/query/final control packets. */
  if (command != V2_DATA)
  {
    request.txOptions |= ZB_APSDE_DATAREQ_TXOPTIONS_ACK;
  }
  /* A waveform slot must not initiate a long route discovery. Control traffic
     obtains/repairs the route before the next scheduled waveform attempt. */
  request.discoverRoute = (command != V2_DATA);
  slot->busy = 1U;
  slot->submitted_ms = HAL_GetTick();
  status = ZbApsdeDataReqCallback(ZbApp.zb, &request,
                                  ZbApp_UploadConfirm, slot);
  if (status != ZB_STATUS_SUCCESS)
  {
    slot->busy = 0U;
    v2_tx_radio_result(&ZbVibrationUpload, 0, HAL_GetTick());
    return 0;
  }
  Custom_APP_ZigbeeActivityChanged();
  return 1;
}

static int ZbApp_UploadRead(void *context, uint32_t batch, uint32_t first,
                            unsigned points, uint8_t *raw)
{
  UNUSED(context);
  return ZigbeeVibrationStore_ReadBatch(batch, first, points, raw) &&
      vib_features_add(&ZbDeferred.stats, batch, first, points, raw);
}

static void ZbApp_StartUploadFastPoll(void)
{
  if (ZbUploadFastPoll!=NULL || ZbApp.zb==NULL ||
      !APP_ZIGBEE_IsConnected()) return;
  /* Stack-managed parent polling delivers block ACKs to the sleepy node.
     The first argument is the initial delay, NOT a poll interval. No extra
     NLME-SYNC requests or clock/RTT calibration are submitted by the uploader. */
  ZbUploadFastPoll=ZbNwkFastPollRequest(ZbApp.zb,ZB_NWK_SYNC_DEFAULT_DELAY_MS,0U);
  if (ZbUploadFastPoll==NULL)
    ZbApp_Log("ZIGBEE: V2 upload fast-poll request failed\r\n");
}

static void ZbApp_StopUploadFastPoll(void)
{
  if (ZbApp.zb!=NULL && ZbUploadFastPoll!=NULL)
    (void)ZbNwkFastPollRelease(ZbApp.zb,ZbUploadFastPoll);
  ZbUploadFastPoll=NULL;
}

static void ZbApp_UploadTask(void)
{
  static const V2TxIO io = {NULL, ZbApp_UploadRead, ZbApp_UploadSend};
  static uint16_t logged_block;
  uint32_t now = HAL_GetTick();

  if (ZbUploadQueueSleeping != 0U)
  {
    if (ZbUploadQueueWake == 0U) return;
    ZbUploadQueueWake=0U;
    ZbUploadQueueSleeping=0U;
    v2_tx_resume(&ZbVibrationUpload,now);
  }
  if (ZbApp.persist_tx_hold != 0U) return;

  if (ZbApp.radio_exclusive != 0U) return;
  if (APP_ZIGBEE_IsConnected() == 0U)
  {
    if ((ZbApp.upload_allowed != 0U) || ZbDeferred.active ||
        v2_tx_active(&ZbVibrationUpload))
    {
      ZbApp_StartLinkRecovery();
      ZbApp_StartTimer(ZbApp.upload_timer, 1000U);
    }
    return;
  }

  /* A raw FINAL ACK alone never releases the record. Summary must also ACK. */
  if (ZbDeferred.active && ZbDeferred.confirmed)
  {
    if (!ZigbeeVibrationStore_Acknowledge(ZbDeferred.batch))
      goto transfer_failed;
    ZbApp_Log("ZIGBEE: batch=%lu raw/features confirmed; Flash record released\r\n",
              (unsigned long)ZbDeferred.batch);
    memset(&ZbDeferred, 0, sizeof(ZbDeferred));
    ZbVibrationUpload.phase = V2_TX_IDLE;
    uint8_t pending[V2_BATCH_LEN];
    if (!ZigbeeVibrationStore_Pending(pending))
    {
      ZbApp.upload_allowed = 0U;
      ZbApp_StopUploadFastPoll();
      ZbApp_FinishCollectionCycle(1U);
      return;
    }
  }

  if (ZbVibrationUpload.phase == V2_TX_ERROR) goto transfer_failed;
  if (ZbVibrationUpload.phase == V2_TX_DONE)
  {
    ZbApp_Log("ZIGBEE: V2 batch=%lu waveform committed bytes=%lu packets=%lu; finalizing features\r\n",
              (unsigned long)ZbDeferred.batch,
              (unsigned long)ZbVibrationUpload.sent_bytes,
              (unsigned long)ZbVibrationUpload.sent_packets);
    ZbApp_Log("ZIGBEE: V2 stats waits=%lu ack_timeouts=%lu aps_failures=%lu pace=%lu ms\r\n",
              (unsigned long)ZbVibrationUpload.wait_count,
              (unsigned long)ZbVibrationUpload.timeout_count,
              (unsigned long)ZbVibrationUpload.radio_failures,
              (unsigned long)ZbVibrationUpload.pace_ms);
    ZbVibrationUpload.phase = V2_TX_IDLE;
    ZbDeferred.raw_done = 1U;
  }

  if (!ZbDeferred.active && ZbApp.upload_allowed)
  {
    uint8_t descriptor[V2_BATCH_LEN];
    if (!ZigbeeVibrationStore_Pending(descriptor))
    {
      ZbApp.upload_allowed=0U;
      ZbApp_StopUploadFastPoll();
      ZbApp_FinishCollectionCycle(1U);
      return;
    }
    memset(&ZbDeferred,0,sizeof(ZbDeferred));
    ZbDeferred.batch=v2_u32(descriptor+1U);
    ZbDeferred.rate=v2_u16(descriptor+5U);
    ZbDeferred.count=v2_u32(descriptor+7U);
    ZbDeferred.expected_crc=v2_u32(descriptor+13U);
    ZbDeferred.deadline=now;
    int policy=ZigbeeVibrationStore_RawPolicy(ZbDeferred.batch);
    if(policy<0) goto transfer_failed;
    ZbDeferred.feature_only=(policy==0);
    ZbDeferred.active=1U;
    vib_features_begin(&ZbDeferred.stats,ZbDeferred.batch,ZbDeferred.count);
    if(!ZbDeferred.feature_only &&
       !v2_tx_start(&ZbVibrationUpload,descriptor,now)) goto transfer_failed;
    ZbApp_StartUploadFastPoll();
    logged_block=0U;
    ZbApp_Log("ZIGBEE: Flash batch=%lu points=%lu mode=%s\r\n",
              (unsigned long)ZbDeferred.batch,(unsigned long)ZbDeferred.count,
              ZbDeferred.feature_only ? "features-only" : "raw-and-features");
  }

  if (ZbDeferred.active && ZbDeferred.feature_only && !ZbDeferred.raw_done)
  {
    uint32_t first=ZbDeferred.stats.sample_count;
    unsigned points=ZbDeferred.count-first;
    if(points>V2_POINTS) points=V2_POINTS;
    if(points && !ZbApp_UploadRead(NULL,ZbDeferred.batch,first,points,
                                  ZbVibrationUpload.raw)) goto transfer_failed;
    if(ZbDeferred.stats.sample_count==ZbDeferred.count) ZbDeferred.raw_done=1U;
  }
  else if (v2_tx_active(&ZbVibrationUpload))
  {
    if (!ZbVibrationUpload.deferred || v2_due(now,ZbVibrationUpload.next_ms))
      ZbApp_StartUploadFastPoll();
    now=HAL_GetTick();
    v2_tx_poll(&ZbVibrationUpload,&io,now);
    if(ZbVibrationUpload.index!=logged_block)
    {
      logged_block=ZbVibrationUpload.index;
      ZbApp_Log("ZIGBEE: V2 verified window %u/%u\r\n",
                (unsigned int)logged_block,v2_blocks(ZbDeferred.count));
    }
  }

  if (ZbDeferred.active && ZbDeferred.raw_done)
  {
    if(!ZbDeferred.features_ready)
    {
      if((ZbDeferred.stats.raw_crc ^ UINT32_MAX)!=ZbDeferred.expected_crc ||
         !vib_features_finish(&ZbDeferred.stats,ZbDeferred.rate,ZbDeferred.features) ||
         !ZigbeeVibrationStore_SaveFeatures(ZbDeferred.batch,ZbDeferred.features))
        goto transfer_failed;
      ZbDeferred.features_ready=1U;
    }
    if(v2_due(now,ZbDeferred.deadline))
    {
      if(ZbDeferred.tries>=V2_RETRIES) goto transfer_failed;
      Custom_APP_ZigbeeSnapshot_t summary;
      memset(&summary,0,sizeof(summary));
      summary.valid_mask=CUSTOM_APP_ZB_VALID_VIBRATION;
      summary.vibration_features_ready=1U;
      summary.vibration_rate_hz=ZbDeferred.rate;
      summary.vibration_sample_count=ZbDeferred.count;
      memcpy(summary.vibration_features_milli,ZbDeferred.features,sizeof(ZbDeferred.features));
      ZbApp_SendSnapshot(&summary,ZbDeferred.batch);
      ZbDeferred.queued=ZbApp.report_has_summary_confirm;
      ++ZbDeferred.tries;
      ZbDeferred.deadline=now+(ZbDeferred.queued ? V2_SEND_STALL_MS : 1000U);
    }
  }
  if (ZbVibrationUpload.deferred && !v2_due(now,ZbVibrationUpload.next_ms) &&
      ZbApp.upload_timer != 0xFFU)
  {
    uint8_t busy = 0U;
    for (unsigned index=0U; index<3U; index++) busy |= ZbUploadSlots[index].busy;
    if (!busy)
    {
      uint32_t delay = ZbVibrationUpload.next_ms-now;
      ZbUploadQueueSleeping = 1U;
      ZbUploadQueueWake = 0U;
      ZbApp_StopUploadFastPoll();
      ZbApp_StartTimer(ZbApp.upload_timer,delay);
      Custom_APP_ZigbeeActivityChanged();
      return;
    }
  }
  if(ZbDeferred.active || v2_tx_active(&ZbVibrationUpload))
    ZbApp_StartTimer(ZbApp.upload_timer,V2_PACE_MS);
  return;

transfer_failed:
  {
    uint32_t failed_batch=ZbDeferred.batch ? ZbDeferred.batch : ZbApp.batch_id;
    ZbApp_Log("ZIGBEE: raw/features transfer failed; Flash batch=%lu retained\r\n",
              (unsigned long)failed_batch);
    ZbApp_Log("ZIGBEE: V2 failure phase=%u block=%u retries=%u waits=%lu ack_timeouts=%lu aps_failures=%lu\r\n",
              ZbVibrationUpload.phase,ZbVibrationUpload.index,ZbVibrationUpload.tries,
              (unsigned long)ZbVibrationUpload.wait_count,
              (unsigned long)ZbVibrationUpload.timeout_count,
              (unsigned long)ZbVibrationUpload.radio_failures);
    ZbUploadQueueSleeping=0U;
    memset(&ZbDeferred,0,sizeof(ZbDeferred));
    ZbVibrationUpload.phase=V2_TX_IDLE;
    ZbApp.upload_allowed=0U;
    ZbApp_StopUploadFastPoll();
    ZbApp.monitor_state=ZB_MONITOR_ERROR;
    (void)ZbApp_SendMonitorStateForBatch(ZbApp.monitor_state,failed_batch);
    ZbApp.monitor_state=ZbApp.monitor_enabled ? ZB_MONITOR_WAITING : ZB_MONITOR_SLEEP;
    Custom_APP_ZigbeeActivityChanged();
    if(ZbApp.monitor_enabled) ZbApp_RequestConfig(ZB_CONFIG_QUERY_COLLECTION);
  }
}

static void ZbApp_StartConfigTransaction(void)
{
  if ((ZbConfigFastPoll != NULL) || (ZbApp.zb == NULL))
  {
    return;
  }
  ZbConfigFastPoll = ZbNwkFastPollRequest(
      ZbApp.zb, ZB_NWK_SYNC_DEFAULT_DELAY_MS, 0U);
  if (ZbConfigFastPoll != NULL)
  {
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_DISABLE);
  }
  else
  {
    ZbApp_Log("ZIGBEE: configuration transaction fast-poll request failed\r\n");
  }
}

static void ZbApp_StopConfigTransaction(void)
{
  if (ZbConfigFastPoll != NULL)
  {
    if (ZbApp.zb != NULL)
    {
      (void)ZbNwkFastPollRelease(ZbApp.zb, ZbConfigFastPoll);
    }
    ZbConfigFastPoll = NULL;
  }
  if (network_authorized != 0U)
  {
    UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
  }
  Custom_APP_ZigbeeActivityChanged();
}

static void ZbApp_RequestConfig(uint8_t reason)
{
  struct nwk_fastpoll_entry_t *poll_handle;

  if (APP_ZIGBEE_IsConnected() == 0U)
  {
    if (reason == ZB_CONFIG_QUERY_COLLECTION) ZbApp_ScheduleNextReport();
    if (reason == ZB_CONFIG_QUERY_IDLE) ZbApp_ScheduleMonitorStandby();
    return;
  }
  ZbApp.config_query_pending = 1U;
  ZbApp.config_query_reason = reason;
  ZbApp.config_query_attempts = 1U;
  ZbApp.channel_migration_wait = 0U;
  ZbApp.channel_migration_target = 0U;
  (void)ZbApp_SendCommand(ZB_CMD_CONFIG_QUERY, &reason, 1U);
  if (reason == ZB_CONFIG_QUERY_JOIN)
  {
    poll_handle = ZbNwkFastPollRequest(
        ZbApp.zb, ZB_NWK_SYNC_DEFAULT_DELAY_MS, 30000U);
    UNUSED(poll_handle);
    ZbApp_StartTimer(ZbApp.report_timer, 30000U);
  }
  else
  {
    ZbApp_StartConfigTransaction();
    ZbApp_StartTimer(ZbApp.report_timer, ZB_CONFIG_QUERY_RETRY_MS);
    ZbApp_Log("ZIGBEE: post-upload configuration transaction started\r\n");
  }
}

static void ZbApp_FinishCollectionCycle(uint8_t success)
{
  ZbApp.monitor_state = (success != 0U) ?
                        ZB_MONITOR_COMPLETE : ZB_MONITOR_ERROR;
  (void)ZbApp_SendMonitorState(ZbApp.monitor_state);
  ZbApp.monitor_state = (ZbApp.monitor_enabled != 0U) ?
                        ZB_MONITOR_WAITING : ZB_MONITOR_SLEEP;
  Custom_APP_ZigbeeActivityChanged();
  if (ZbApp.monitor_enabled != 0U)
  {
    ZbApp_RequestConfig(ZB_CONFIG_QUERY_COLLECTION);
  }
}

static void ZbApp_CommandConfirm(struct ZbZclCommandRspT *response, void *arg)
{
  if (admission_expired || ZbApp.join_blocked_until_reset ||
      ((uint32_t)(uintptr_t)arg >> 8) != command_confirm_generation) return;
  uint8_t was_parent_verification =
      ((ZbApp.parent_sync_pending != 0U) &&
       ((uint8_t)(uintptr_t)arg == ZB_CMD_MONITOR_STATE)) ? 1U : 0U;
  uint8_t candidate_probe = ((was_parent_verification != 0U) &&
                             (ZbApp.rejoin_probe_pending != 0U)) ? 1U : 0U;
  uint8_t candidate_rtc = ZbApp.rejoin_probe_rtc;
  uint8_t defer_join = 0U;

  if (was_parent_verification != 0U)
  {
    ZbApp.parent_sync_pending = 0U;
  }
  if (candidate_probe != 0U)
  {
    ZbApp.rejoin_probe_pending = 0U;
    ZbApp.rejoin_probe_rtc = 0U;
  }
  else if (ZbApp.rejoin_probe_pending != 0U)
  {
    /* An older queued frame may complete before the probe. It cannot
       establish the status of this new rejoin candidate. */
    if (ZbApp.tx_in_flight != 0U)
    {
      ZbApp.tx_in_flight--;
    }
    if ((ZbApp.tx_count != 0U) &&
        (ZbApp.tx_in_flight < ZB_TX_MAX_IN_FLIGHT))
    {
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_TX, CFG_SCH_PRIO_1);
    }
    Custom_APP_ZigbeeActivityChanged();
    if (ZbApp.radio_control_deferred != 0U)
    {
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_RADIO_CONTROL,
                       CFG_SCH_PRIO_0);
    }
    return;
  }
  if ((response == NULL) || (response->aps_status != ZB_STATUS_SUCCESS))
  {
    ZbApp_Log("ZIGBEE: tx cmd=0x%02x failed aps=0x%02x\r\n",
              (unsigned int)(uint8_t)(uintptr_t)arg,
              response ? (unsigned int)response->aps_status : 0xFFU);
    if (candidate_probe != 0U)
    {
      ZbApp_SetParentOnline(0U,
                            response ? response->aps_status :
                                       (enum ZbStatusCodeT)0xFF);
      ZbApp_Log("ZIGBEE: rejoin candidate rejected; saved network retained\r\n");
      defer_join = 1U;
      if ((ZbApp.restore_recovery_active != 0U) &&
          (ZbApp.saved_channel_rejoin_attempts >=
           ZB_SAVED_CHANNEL_REJOIN_ATTEMPTS))
      {
        ZbApp_StartFreshJoinFallback();
      }
      else if (ZbApp.commissioning_window_open != 0U)
      {
        ZbApp_StartTimer(ZbApp.join_retry_timer,
                         ZB_COMMISSIONING_JOIN_RETRY_MS);
      }
      else if ((ZbApp.rtc_join_active != 0U) &&
               (ZbApp.rtc_join_attempts < ZB_RTC_JOIN_MAX_ATTEMPTS))
      {
        ZbApp_StartTimer(ZbApp.join_retry_timer, ZB_JOIN_RETRY_MS);
      }
      else if (ZbApp.rtc_join_active != 0U)
      {
        ZbApp_CompleteJoinFailureCycle();
      }
    }
    else if ((was_parent_verification != 0U) &&
        (ZbApp.commissioning_window_open != 0U) &&
        (ZbApp.radio_exclusive == 0U) &&
        (ZbApp.join_blocked_until_reset == 0U))
    {
      /* The first APS probe can race the stack's automatic end-device rejoin.
         Preserve the restored credentials and retry as a secure rejoin after
         five seconds instead of resetting into a duplicate factory join. */
      ZbApp_Log("ZIGBEE: restored parent unavailable; secure rejoin within remaining recovery budget\r\n");
      ZbApp_SetParentOnline(0U, response ? response->aps_status :
                            (enum ZbStatusCodeT)0xFF);
      ZbApp_StartTimer(ZbApp.join_retry_timer,
                       ZB_COMMISSIONING_JOIN_RETRY_MS);
    }
    else if (was_parent_verification != 0U)
    {
      ZbApp_SetParentOnline(0U,
                            response ? response->aps_status :
                                       (enum ZbStatusCodeT)0xFF);
      ZbApp_StartLinkRecovery();
    }
    else
    {
      /* One failed application confirmation (for example 0xE9 while CPU2 is
         busy) does not prove that the parent disappeared. Verify separately
         and let the active V2 batch remain recoverable. */
      ZbApp_VerifyParent();
    }
  }
  else
  {
    ZbApp_SetParentOnline(1U, response->aps_status);
    if ((uint8_t)(uintptr_t)arg == ZB_CMD_VIBRATION_SUMMARY)
    {
      if (ZbDeferred.active && ZbDeferred.queued)
      {
        /* ZCL ACK is diagnostic only; kind FEATURES confirms durable SQL. */
        if(response->status != ZCL_STATUS_SUCCESS) {
          ZbDeferred.queued=0U;
          ZbDeferred.deadline=HAL_GetTick()+1000U;
        }
        UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_UPLOAD, CFG_SCH_PRIO_0);
      }
      ZbApp_Log("ZIGBEE: vibration summary confirm aps=0x%02x zcl=0x%02x\r\n",
                (unsigned int)response->aps_status,
                (unsigned int)response->status);
      if (response->status != ZCL_STATUS_SUCCESS)
      {
        ZbApp_Log("ZIGBEE: vibration summary ZCL acknowledgement missing; check gateway batch\r\n");
      }
    }
    if (candidate_probe != 0U)
    {
      ZbApp.rejoin_via_startup = 0U;
      ZbApp.rtc_join_active = 0U;
      ZbApp.rtc_join_attempts = 0U;
      ZbApp_Log("ZIGBEE: rejoin confirmed, endpoint=1 coordinator_endpoint=8\r\n");
      ZbApp.manual_announce_required = 1U;
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ANNOUNCE, CFG_SCH_PRIO_0);
      if (candidate_rtc != 0U)
      {
        ZbApp.reporting_active = 1U;
        ZbApp.monitor_state = (ZbApp.monitor_enabled != 0U) ?
                              ZB_MONITOR_WAITING : ZB_MONITOR_SLEEP;
        ZbApp.offline_capture_requested = 0U;
        ZbApp_SendPendingSnapshot();
        if ((ZbApp.upload_allowed != 0U) || v2_tx_active(&ZbVibrationUpload))
        {
          UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_UPLOAD, CFG_SCH_PRIO_0);
          ZbApp_Log("ZIGBEE: RTC rejoin confirmed; pending V2 upload resumed\r\n");
        }
        else if (ZbApp.monitor_enabled != 0U)
        {
          UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
          ZbApp_Log("ZIGBEE: RTC rejoin confirmed; pending and current records queued\r\n");
        }
      }
      else
      {
        ZbApp_StartAutonomousReporting();
      }
#if ((CFG_ZIGBEE_NVM_ENABLED != 0U) && \
     (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U))
      if (ZbApp.nvm_ready != 0U)
      {
        ZbApp_SchedulePersistSave(ZB_PERSIST_SAVE_DELAY_MS);
      }
#endif
    }
  }
  if (ZbApp.tx_in_flight != 0U)
  {
    ZbApp.tx_in_flight--;
  }
  if ((ZbApp.tx_count != 0U) &&
      (ZbApp.tx_in_flight < ZB_TX_MAX_IN_FLIGHT))
  {
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_TX, CFG_SCH_PRIO_1);
  }
  if ((defer_join == 0U) && (APP_ZIGBEE_IsTransmitting() == 0U))
  {
    ZbApp_Log("ZIGBEE: transmission complete; STOP2 allowed\r\n");
  }
  Custom_APP_ZigbeeActivityChanged();
  if (ZbApp.radio_control_deferred != 0U)
  {
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_RADIO_CONTROL,
                     CFG_SCH_PRIO_0);
  }
}

static void ZbApp_ResetTxQueue(void)
{
  command_confirm_generation = (command_confirm_generation + 1U) & 0x00FFFFFFU;
  uint8_t was_busy = APP_ZIGBEE_IsTransmitting();

  ZbApp.tx_head = 0U;
  ZbApp.tx_tail = 0U;
  ZbApp.tx_count = 0U;
  ZbApp.tx_in_flight = 0U;
  if (was_busy != 0U)
  {
    Custom_APP_ZigbeeActivityChanged();
  }
}

static void ZbApp_SendPendingSnapshot(void)
{
  if ((ZbApp.pending_snapshot_valid == 0U) ||
      (APP_ZIGBEE_IsConnected() == 0U))
  {
    return;
  }

  ZbApp_SendSnapshot(&ZbApp.pending_snapshot, ZbApp.pending_batch_id);
  ZbApp_Log("ZIGBEE: pending snapshot batch=%lu queued after rejoin\r\n",
            (unsigned long)ZbApp.pending_batch_id);
  ZbApp.pending_snapshot_valid = 0U;
}

static enum ZclStatusCodeT ZbApp_SendMonitorState(uint8_t state)
{
  return ZbApp_SendMonitorStateForBatch(state, ZbApp.batch_id);
}

static enum ZclStatusCodeT ZbApp_SendMonitorStateForBatch(uint8_t state,
                                                          uint32_t batch_id)
{
  uint8_t payload[7];
  payload[0] = ZbApp.monitor_enabled;
  payload[1] = state;
  payload[2] = (uint8_t)batch_id;
  payload[3] = (uint8_t)(batch_id >> 8U);
  payload[4] = (uint8_t)(batch_id >> 16U);
  payload[5] = (uint8_t)(batch_id >> 24U);
  payload[6] = ZCL_BASIC_POWER_DC;
  return ZbApp_SendCommand(ZB_CMD_MONITOR_STATE, payload, sizeof(payload));
}

static void ZbApp_ScheduleMonitorStandby(void)
{
  if (!network_authorized || ZbApp.monitor_enabled || ZbApp.config_query_pending) return;
  uint32_t now = APP_ElapsedMs(); /* RTC-backed, advances through STOP2. */
  if (!monitor_standby.seen) {
    ZbApp_Log("ZIGBEE: monitoring off; first authorized standby is 10 minutes\r\n");
  }
  int awake;
  uint32_t delay = monitor_standby_delay(&monitor_standby, now, &awake);
  if (awake) {
    UTIL_LPM_SetStopMode(1U << CFG_LPM_MONITOR_STANDBY, UTIL_LPM_DISABLE);
  } else {
    UTIL_LPM_SetStopMode(1U << CFG_LPM_MONITOR_STANDBY, UTIL_LPM_ENABLE);
    ZbApp_Log("ZIGBEE: monitoring off; STOP2 allowed, query again in 5 minutes\r\n");
  }
  ZbApp_StartTimer(ZbApp.report_timer, delay);
  Custom_APP_ZigbeeActivityChanged();
}

static void ZbApp_ScheduleNextReport(void)
{
  if (ZbApp.radio_exclusive != 0U)
  {
    return;
  }
  if ((ZbApp.reporting_active == 0U) || (ZbApp.monitor_enabled == 0U))
  {
    return;
  }
  if (!sampling_cadence.active)
    sampling_cadence_start(&sampling_cadence,APP_ElapsedMs(),ZbApp.monitor_config.auto_interval_sec*1000U);
  ZbApp_ArmSamplingTimer();
}

static void ZbApp_ArmSamplingTimer(void)
{
  if (sampling_cadence.active && ZbApp.monitor_enabled)
    ZbApp_StartTimer(sampling_timer,sampling_cadence_delay(&sampling_cadence,APP_ElapsedMs()));
}

static void ZbApp_SamplingTimerCallback(void)
{
  if (!sampling_cadence.active || !ZbApp.monitor_enabled) return;
  sampling_due=1U;
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
}

static void ZbApp_StartAutonomousReporting(void)
{
  ZbApp.reporting_active = 1U;
  ZbApp.monitor_enabled = 0U;
  ZbApp.monitor_state = ZB_MONITOR_SLEEP;
  ZbApp.collection_requested = 0U;
  ZbApp_Log("ZIGBEE: joined/restored; requesting gateway sampling configuration\r\n");
  if (ZbApp.radio_exclusive == 0U)
  {
    ZbApp_RequestConfig(ZB_CONFIG_QUERY_JOIN);
  }
}

static void ZbApp_AnnounceTask(void)
{
  struct ZbZdoDeviceAnnceT announcement;
  struct nwk_fastpoll_entry_t *poll_handle = NULL;
  uint8_t capability = MCP_ASSOC_CAP_ALLOC_ADDR;
  uint16_t local_pan_id = 0xFFFFU;
  uint16_t local_manager_address = 0U;
  uint64_t local_ext_pan_id = 0U;

  if ((ZbApp.radio_exclusive != 0U) ||
      (ZbApp.zb == NULL) ||
      (ZbApp.join_status != ZB_STATUS_SUCCESS) ||
      (ZbApp.join_blocked_until_reset != 0U))
  {
    return;
  }

  /* Restore may load an older interface power setting. Apply in task context,
   * before the recovery handshake/announcement, never in a CPU2 callback. */
  ZbApp_SetMaximumTxPower();
  (void)ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_CapabilityInformation,
                 &capability, sizeof(capability));
  if ((ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_PanId,
                &local_pan_id, sizeof(local_pan_id)) == ZB_STATUS_SUCCESS) &&
      (ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_ExtendedPanId,
                &local_ext_pan_id, sizeof(local_ext_pan_id)) == ZB_STATUS_SUCCESS) &&
      (local_pan_id != 0xFFFFU) && (local_ext_pan_id != 0U))
  {
    (void)ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_ManagerAddr,
                   &local_manager_address, sizeof(local_manager_address));
    ZbApp.rejoin_pan_id = local_pan_id;
    ZbApp.rejoin_ext_pan_id = local_ext_pan_id;
    ZbApp.rejoin_short_address = ZbShortAddress(ZbApp.zb);
    ZbApp.rejoin_manager_address = local_manager_address;
    ZbApp.rejoin_identity_valid = 1U;
    ZbApp_Log("ZIGBEE: local PAN=0x%04x extPAN=%08lx%08lx; compare gateway\r\n",
              (unsigned int)local_pan_id,
              (unsigned long)(local_ext_pan_id >> 32U),
              (unsigned long)local_ext_pan_id);
  }

  /* A fresh, unapproved node already keeps a zero-timeout fast-poll handle
   * until authorization or Leave. Restored/authorized nodes only need the
   * original short window around their announcement. */
  if (network_authorized == 0U)
  {
    ZbApp_StartAdmissionFastPoll();
  }
  else
  {
    poll_handle = ZbNwkFastPollRequest(ZbApp.zb,
                                       ZB_NWK_SYNC_DEFAULT_DELAY_MS,
                                       ZB_COMMISSIONING_FAST_POLL_MS);
    if (poll_handle == NULL)
    {
      ZbApp_Log("ZIGBEE: commissioning fast-poll request failed\r\n");
    }
    else
    {
      ZbApp_Log("ZIGBEE: commissioning fast-poll active for %lu ms\r\n",
                (unsigned long)ZB_COMMISSIONING_FAST_POLL_MS);
    }
  }
  if (ZbApp.manual_announce_required != 0U)
  {
    memset(&announcement, 0, sizeof(announcement));
    announcement.nwkAddr = ZbShortAddress(ZbApp.zb);
    announcement.extAddr = ZbExtendedAddress(ZbApp.zb);
    announcement.capability = capability;
    ZbZdoDeviceAnnce(ZbApp.zb, &announcement);
  ZbApp_Log("ZIGBEE: self announcement submitted short=0x%04x (delivery unverified)\r\n",
              (unsigned int)announcement.nwkAddr);
  }
  ZbApp.manual_announce_required = 0U;
  if (admission_phase == 1U && ZbApp.restore_recovery_active != 0U) {
    uint8_t hello[10];
    uint16_t address = ZbShortAddress(ZbApp.zb);
    if (!recovery_token_valid) {
      uint32_t random_words[2];
      if (HAL_RNG_GenerateRandomNumber(&hrng, &random_words[0]) != HAL_OK ||
          HAL_RNG_GenerateRandomNumber(&hrng, &random_words[1]) != HAL_OK) {
        ZbApp_Log("ZIGBEE: recovery token RNG unavailable; retry later\r\n");
        return;
      }
      memcpy(recovery_token, random_words, sizeof(recovery_token));
      recovery_token_valid = 1U;
    }
    memcpy(hello, recovery_token, sizeof(recovery_token));
    hello[8] = (uint8_t)address; hello[9] = (uint8_t)(address >> 8U);
    (void)ZbNwkFastPollRequest(ZbApp.zb, 250U, 10000U);
    enum ZclStatusCodeT hello_status = ZbApp_SendCommand(0x9CU, hello, sizeof(hello));
    ZbApp_Log("ZIGBEE: recovery hello queued status=0x%02x short=0x%04x\r\n",
              (unsigned int)hello_status, (unsigned int)address);
  }
}

static void ZbApp_StartTimer(uint8_t timer_id, uint32_t delay_ms)
{
  uint64_t ticks;

  if (timer_id == 0xFFU)
  {
    return;
  }
  ticks = ((uint64_t)delay_ms * 1000ULL + CFG_TS_TICK_VAL - 1ULL) /
          CFG_TS_TICK_VAL;
  if (ticks == 0U)
  {
    ticks = 1U;
  }
  if (ticks > UINT32_MAX)
  {
    ticks = UINT32_MAX;
  }
  HW_TS_Stop(timer_id);
  HW_TS_Start(timer_id, (uint32_t)ticks);
}

static void ZbApp_JoinRetryTimerCallback(void)
{
  if (ZbApp.radio_retry_pending != 0U)
  {
    ZbApp.radio_retry_pending = 0U;
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_RADIO_CONTROL, CFG_SCH_PRIO_0);
  }
  else
  {
    if (ZbApp.security_join_cleanup_pending != 0U)
    {
      /* A security-failed association can leave volatile APS/NWK state in
       * CPU2 even though no network was admitted. Reset only that live state
       * before retrying; the original admission deadline remains unchanged. */
      ZbApp.security_join_cleanup_pending = 0U;
      ZbReset(ZbApp.zb);
      ZbApp_SetMaximumTxPower();
      ZbApp_PrepareFreshJoin();
      ZbApp_Log("ZIGBEE: failed-join security state reset before retry\r\n");
    }
    ZbApp_Log("ZIGBEE: join retry timer fired\r\n");
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
  }
}

static void ZbApp_ReportTimerCallback(void)
{
  if (ZbApp.config_query_pending != 0U)
  {
    uint8_t reason = ZbApp.config_query_reason;
    if (reason == ZB_CONFIG_QUERY_JOIN)
    {
      ZbApp.config_query_pending = 0U;
      /* An unconfirmed node must not collect with defaults or stale values. */
      ZbApp.monitor_enabled = 0U;
      ZbApp.monitor_state = ZB_MONITOR_SLEEP;
      ZbApp_Log("ZIGBEE: initial configuration query timed out; collection remains disabled\r\n");
      ZbApp_ScheduleMonitorStandby();
    }
    else if (ZbApp.channel_migration_wait != 0U)
    {
      /* A successful reply after the coordinator has moved to the target
       * channel is the end-to-end proof that it is safe to sleep. */
      ZbApp.config_query_attempts++;
      (void)ZbApp_SendCommand(ZB_CMD_CONFIG_QUERY, &reason, 1U);
      ZbApp_StartTimer(ZbApp.report_timer, ZB_CONFIG_QUERY_RETRY_MS);
      ZbApp_Log("ZIGBEE: waiting for channel %u gateway confirmation; config query retry=%u\r\n",
                (unsigned int)ZbApp.channel_migration_target,
                (unsigned int)ZbApp.config_query_attempts);
    }
    else if (ZbApp.config_query_attempts < ZB_CONFIG_QUERY_MAX_ATTEMPTS)
    {
      ZbApp.config_query_attempts++;
      (void)ZbApp_SendCommand(ZB_CMD_CONFIG_QUERY, &reason, 1U);
      ZbApp_StartTimer(ZbApp.report_timer, ZB_CONFIG_QUERY_RETRY_MS);
      ZbApp_Log("ZIGBEE: post-upload configuration query retry=%u\r\n",
                (unsigned int)ZbApp.config_query_attempts);
    }
    else
    {
      ZbApp.config_query_pending = 0U;
      ZbApp_StopConfigTransaction();
      ZbApp_Log("ZIGBEE: post-upload configuration query timed out; cached configuration retained\r\n");
      ZbApp_ScheduleNextReport();
      if (!ZbApp.monitor_enabled) ZbApp_ScheduleMonitorStandby();
    }
    return;
  }
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
}

static void ZbApp_UploadTimerCallback(void)
{
  if (ZbUploadQueueSleeping != 0U)
  {
    ZbUploadQueueWake=1U; /* state machine is resumed in the sequencer task */
  }
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_UPLOAD, CFG_SCH_PRIO_0);
}

#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
static void ZbApp_PersistTimerCallback(void)
{
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_PERSIST, CFG_SCH_PRIO_1);
}
#endif

static void ZbApp_SetParentOnline(uint8_t online, enum ZbStatusCodeT status)
{
  online = (online != 0U) ? 1U : 0U;
  ZbApp.parent_sync_pending = 0U;
  if ((ZbApp.parent_state_known != 0U) &&
      (ZbApp.parent_online == online))
  {
    return;
  }

  ZbApp.parent_online = online;
  ZbApp.parent_state_known = 1U;
  if (online != 0U)
  {
    if (admission_phase == 1U && admission_expired == 0U) {
      HW_TS_Stop(admission_timer_id);
      admission_phase = 0U;
      ZbApp.commissioning_window_open = 0U;
      recovery_config_pending = 1U;
      recovery_token_valid = 0U;
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
      ZbApp_Log("ZIGBEE: restored coordinator verified; saved network retained (%s)\r\n",
                network_authorized != 0U ? "authorized" : "pending authorization");
    }
    /* A real APS confirmation proves that the restored parent is reachable.
       Cancel the startup-only fallback before normal monitoring begins. */
    ZbApp.restore_recovery_active = 0U;
    ZbApp.saved_channel_rejoin_attempts = 0U;
    ZbApp_Log("ZIGBEE: coordinator link online status=0x%02x\r\n",
              (unsigned int)status);
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
    if ((ZbApp.persist_save_pending != 0U) &&
        (ZbApp.persist_timer != 0xFFU))
    {
      ZbApp_StartTimer(ZbApp.persist_timer, ZB_PERSIST_RETRY_DELAY_MS);
    }
#endif
  }
  else
  {
    ZbApp_Log("ZIGBEE: coordinator link offline status=0x%02x\r\n",
              (unsigned int)status);
  }
}

static uint8_t ZbApp_IsTrustedCoordinator(const struct ZbApsdeDataIndT *data_indication)
{
  return ((data_indication != NULL) &&
          (data_indication->src.mode == ZB_APSDE_ADDRMODE_SHORT) &&
          (data_indication->src.nwkAddr == 0x0000U) &&
          (data_indication->src.endpoint == ZB_COORDINATOR_ENDPOINT)) ? 1U : 0U;
}

static void ZbApp_StartLinkRecovery(void)
{
  if ((ZbApp.radio_exclusive != 0U) ||
      (ZbApp.restore_pending != 0U) ||
      (ZbApp.rtc_join_active != 0U) ||
      (ZbApp.join_blocked_until_reset != 0U) ||
      (ZbApp.zb == NULL))
  {
    return;
  }
  if ((ZbApp.restore_recovery_active != 0U) &&
      (ZbApp.rejoin_channel_mask == 0U))
  {
    /* Never broaden an old-network recovery to all channels when its saved
       channel cannot be read. The parent probe has already failed, so enter
       the normal commissioning path directly. */
    ZbApp_StartFreshJoinFallback();
    return;
  }
  ZbApp.rtc_join_active = 1U;
  ZbApp.rtc_join_attempts = 0U;
  if (ZbApp.join_status != ZB_STATUS_SUCCESS)
  {
    ZbApp_PrepareFreshJoin();
  }
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
}

static void ZbApp_VerifyParent(void)
{
  enum ZclStatusCodeT status;

  if ((ZbApp.parent_sync_pending != 0U) ||
      (ZbApp.radio_exclusive != 0U) ||
      (ZbApp.zb == NULL) ||
      (ZbApp.join_status != ZB_STATUS_SUCCESS) ||
      (ZbApp.join_blocked_until_reset != 0U))
  {
    return;
  }

  ZbApp.parent_sync_pending = 1U;
  status = ZbApp_SendMonitorState(ZB_MONITOR_WAITING);
  if (status == ZCL_STATUS_SUCCESS)
  {
    ZbApp_Log("ZIGBEE: coordinator link verification queued\r\n");
  }
  else
  {
    ZbApp.parent_sync_pending = 0U;
    ZbApp_Log("ZIGBEE: coordinator link verification queue failed zcl=0x%02x\r\n",
              (unsigned int)status);
  }
}

static void ZbApp_PrepareFreshJoin(void)
{
  if (ZbApp.zb == NULL)
  {
    return;
  }

  if (ZbApp.join_retry_timer != 0xFFU)
  {
    HW_TS_Stop(ZbApp.join_retry_timer);
  }
  if (ZbApp.join_status == ZB_STATUS_SUCCESS)
  {
    /* A persisted association can be locally valid while its parent has
       already forgotten the child. Reset only the live APS/NWK state; the
       flash record is replaced after the next successful join. */
    ZbReset(ZbApp.zb);
    ZbApp_SetMaximumTxPower();
  }
  ZbApp.join_status = (enum ZbStatusCodeT)0x01;
  ZbApp.parent_online = 0U;
  ZbApp.parent_state_known = 1U;
  ZbApp.parent_sync_pending = 0U;
  ZbApp.rejoin_identity_valid = 0U;
  ZbApp.rejoin_via_startup = 0U;
  ZbApp.restore_recovery_active = 0U;
  ZbApp.saved_channel_rejoin_attempts = 0U;
  ZbApp.rejoin_channel_mask = 0U;
  ZbApp_ResetTxQueue();
}

static void ZbApp_StartFreshJoinFallback(void)
{
  uint32_t failed_channel_mask = ZbApp.rejoin_channel_mask;

  if ((ZbApp.zb == NULL) || (ZbApp.restore_recovery_active == 0U))
  {
    return;
  }

  ZbApp_Log("ZIGBEE: saved network verification pending on mask=0x%08lx; credentials retained\r\n",
            (unsigned long)failed_channel_mask);
  ZbApp.rtc_join_active = 0U;
  ZbApp.rtc_join_attempts = 0U;
  admission_phase = 1U;
  admission_expired = 0U;
  ZbApp.restore_pending = 0U;
  ZbApp_StartTimer(admission_timer_id, ZB_RESTORED_PARENT_VERIFY_MS);
  ZbApp.manual_announce_required = 1U;
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ANNOUNCE, CFG_SCH_PRIO_0);
}

static void ZbApp_CompleteJoinFailureCycle(void)
{
  ZbApp.rtc_join_active = 0U;
  ZbApp.rtc_join_attempts = 0U;
  if ((ZbApp.upload_allowed != 0U) || v2_tx_active(&ZbVibrationUpload))
  {
    ZbApp.upload_allowed = 0U;
    ZbVibrationUpload.phase = V2_TX_IDLE;
    ZbUploadQueueSleeping = 0U;
    memset(&ZbDeferred, 0, sizeof(ZbDeferred));
    if (ZbApp.upload_timer != 0xFFU) HW_TS_Stop(ZbApp.upload_timer);
    ZbApp.monitor_state = ZB_MONITOR_WAITING;
    ZbApp_Log("ZIGBEE: upload recovery exhausted; Flash batch retained until next cycle\r\n");
    ZbApp_ScheduleNextReport();
    Custom_APP_ZigbeeActivityChanged();
    return;
  }

  if (ZbApp.pending_snapshot_valid != 0U)
  {
    ZbApp_Log("ZIGBEE: pending snapshot retained; next retry in 60 s\r\n");
    ZbApp_ScheduleNextReport();
    return;
  }

  /* No older record is waiting, so acquire this RTC cycle once and retain it
     in SRAM. Sensor completion will schedule the next 60-second cycle. */
  ZbApp.offline_capture_requested = 1U;
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_REPORT, CFG_SCH_PRIO_1);
}

static enum zb_msg_filter_rc ZbApp_LeaveFilter(struct ZigBeeT *zb,
                                               uint32_t id,
                                               void *message,
                                               void *arg)
{
  struct ZbNlmeLeaveIndT *leave_indication;
  UNUSED(zb);
  UNUSED(arg);
  if ((id != ZB_MSG_FILTER_LEAVE_IND) || (message == NULL))
  {
    return ZB_MSG_CONTINUE;
  }
  leave_indication = (struct ZbNlmeLeaveIndT *)message;
  if (!leave_indication->rejoin)
  {
    /* A failed fresh join may be followed by a local LEAVE.indication while
     * the stack rolls back the incomplete security association.  That is not
     * a coordinator-requested Leave for an admitted child: no network was
     * successfully joined, so keep the original 30-second admission budget
     * and the already scheduled all-channel retry intact. */
    if (ZbApp.join_status != ZB_STATUS_SUCCESS)
    {
      ZbApp.security_join_cleanup_pending = 1U;
      ZbApp_Log("ZIGBEE: leave during unsuccessful join ignored; original admission deadline retained\r\n");
      return ZB_MSG_CONTINUE;
    }

    if (admission_timer_id != 0xFFU)
    {
      HW_TS_Stop(admission_timer_id);
    }
    network_authorized = 0U;
    if (ZbApp.join_retry_timer != 0xFFU)
    {
      HW_TS_Stop(ZbApp.join_retry_timer);
    }
    if (ZbApp.report_timer != 0xFFU)
    {
      HW_TS_Stop(ZbApp.report_timer);
    }
    if (ZbApp.upload_timer != 0xFFU)
    {
      HW_TS_Stop(ZbApp.upload_timer);
    }
    ZbApp.join_status = (enum ZbStatusCodeT)0x01;
    ZbApp.parent_online = 0U;
    ZbApp.parent_state_known = 1U;
    ZbApp.parent_sync_pending = 0U;
    /* The stack delivered a Leave indication. Erase saved
     * credentials in task context and immediately try a fresh join for 30 s.
     * Only an unsuccessful fresh window enters the five-minute sleep. */
    admission_phase = 0U;
    admission_expired = 0U;
    ZbApp.commissioning_window_open = 0U;
    ZbApp.join_blocked_until_reset = 1U;
    ZbApp.reporting_active = 0U;
    ZbApp.monitor_enabled = 0U;
    ZbApp.monitor_state = ZB_MONITOR_SLEEP;
    ZbApp.rtc_join_active = 1U;
    ZbApp.rtc_join_attempts = 0U;
    ZbApp.offline_capture_requested = 0U;
    ZbApp.persist_save_pending = 0U;
    ZbApp.persist_tx_hold = 0U;
    ZbApp.config_query_pending = 0U;
    ZbApp.channel_migration_wait = 0U;
    ZbApp.upload_allowed = 0U;
    ZbVibrationUpload.phase = V2_TX_IDLE;
    ZbUploadQueueSleeping = 0U;
    memset(&ZbDeferred, 0, sizeof(ZbDeferred));
    memset(ZbUploadSlots, 0, sizeof(ZbUploadSlots));
    ZigbeeVibrationStore_Abort();
    leave_cleanup_pending = 1U;
    ZbApp.collection_requested = 0U;
    ZbApp.audio_waiting = 0U;
    recovery_config_pending = recovery_token_valid = 0U;
    ZbApp_ResetTxQueue();
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
    if (ZbApp.persist_timer != 0xFFU)
    {
      HW_TS_Stop(ZbApp.persist_timer);
    }
#endif
    ZbApp.config_valid = 0U;
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NVM_ERASE, CFG_SCH_PRIO_0);
    ZbApp_Log("ZIGBEE: leave received; network erase and fresh commissioning queued\r\n");
  }
  return ZB_MSG_CONTINUE;
}

static bool ZbApp_NvmInit(void)
{
  int status;

  ZbApp_Log("ZIGBEE: NVM region 0x%08lx-0x0807ffff\r\n",
            (unsigned long)(HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS));
  status = EE_Init(0, HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS);
  if (status != EE_OK)
  {
    status = EE_Init(1, HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS);
    ZbApp_Log("ZIGBEE: NVM first-use format status=%d\r\n", status);
  }
  else
  {
    ZbApp_Log("ZIGBEE: existing NVM loaded\r\n");
  }
  return status == EE_OK;
}

static bool ZbApp_NvmRead(void)
{
  uint32_t length;
  uint32_t marker;
  uint32_t authorization = ZB_PERSIST_AUTHORIZED;
  uint16_t data_words;
  uint16_t first_data_word;

  memset(&ZbPersistCache, 0, sizeof(ZbPersistCache));
  if (EE_Read(0, ZIGBEE_DB_START_ADDR, &length) != EE_OK)
  {
    return false;
  }
  if (EE_Read(0, (uint16_t)(ZIGBEE_DB_START_ADDR + 1U), &marker) != EE_OK)
  {
    return false;
  }
  if (marker == ZB_PERSIST_SLEEPY_POLL_MARKER)
  {
    if (EE_Read(0, (uint16_t)(ZIGBEE_DB_START_ADDR + 2U),
                &authorization) != EE_OK)
    {
      return false;
    }
    if ((authorization != ZB_PERSIST_AUTH_PENDING) &&
        (authorization != ZB_PERSIST_AUTHORIZED))
    {
      ZbApp_Log("ZIGBEE: persisted authorization marker invalid\r\n");
      return false;
    }
    first_data_word = 3U;
    network_persist_format_v2 = 1U;
  }
  else if (marker == ZB_PERSIST_SLEEPY_POLL_MARKER_V1)
  {
    /* Existing deployments had no separate business-authorization marker.
     * Preserve compatibility by treating those records as authorized and
     * rewrite them in v2 format on the next save. */
    first_data_word = 2U;
    authorization = ZB_PERSIST_AUTHORIZED;
    network_persist_format_v2 = 0U;
  }
  else
  {
    ZbApp_Log("ZIGBEE: persisted format marker unknown; fresh join required\r\n");
    return false;
  }
  if ((length == 0U) ||
      (length > ZB_PERSIST_MAX_DATA_BYTES))
  {
    return false;
  }
  ZbPersistCache.words[0] = length;
  ZbPersistCache.words[1] = marker;
  data_words = (uint16_t)((length + sizeof(uint32_t) - 1U) / sizeof(uint32_t));
  for (uint16_t offset = 0U; offset < data_words; offset++)
  {
    if (EE_Read(0, (uint16_t)(ZIGBEE_DB_START_ADDR + first_data_word + offset),
                &ZbPersistCache.words[3U + offset]) != EE_OK)
    {
      return false;
    }
  }
  ZbPersistCache.words[2] = authorization;
  network_authorized = authorization == ZB_PERSIST_AUTHORIZED ? 1U : 0U;
  ZbApp_Log("ZIGBEE: persisted authorization=%s\r\n",
            network_authorized != 0U ? "authorized" : "pending");
  return true;
}

#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
static bool ZbApp_NvmWrite(void)
{
  uint32_t length = ZbPersistCache.words[0];
  uint16_t word_count = (uint16_t)(3U +
      (length + sizeof(uint32_t) - 1U) / sizeof(uint32_t));

  for (uint16_t index = 0U; index < word_count; index++)
  {
    int status = EE_Write(0, (uint16_t)(ZIGBEE_DB_START_ADDR + index),
                          ZbPersistCache.words[index]);
    if (status == EE_CLEAN_NEEDED)
    {
      status = EE_Clean(0, 0);
    }
    if (status != EE_OK)
    {
      ZbApp_Log("ZIGBEE: NVM write failed word=%u status=%d\r\n",
                (unsigned int)index, status);
      return false;
    }
  }
  network_persist_format_v2 = 1U;
  return true;
}

static bool ZbApp_NvmWriteAuthorization(void)
{
  int status;
  if ((ZbApp.nvm_ready == 0U) || (network_persist_format_v2 == 0U))
  {
    return false;
  }
  status = EE_Write(0, (uint16_t)(ZIGBEE_DB_START_ADDR + 2U),
                    network_authorized != 0U ?
                    ZB_PERSIST_AUTHORIZED : ZB_PERSIST_AUTH_PENDING);
  if (status == EE_CLEAN_NEEDED)
  {
    status = EE_Clean(0, 0);
  }
  if (status != EE_OK)
  {
    ZbApp_Log("ZIGBEE: authorization marker write failed status=%d\r\n",
              status);
    return false;
  }
  ZbApp_Log("ZIGBEE: authorization marker saved=%s\r\n",
            network_authorized != 0U ? "authorized" : "pending");
  return true;
}
#endif

static void ZbApp_NvmErase(void)
{
  /* This task runs outside the CPU2 message-filter callback, so it is safe
   * to release the stack-owned fast-poll handle here. */
  ZbApp_StopAdmissionFastPoll();
  ZbApp_StopConfigTransaction();
  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_DISABLE);
  Custom_APP_ZigbeeAbortSnapshot();
#if (CFG_ZIGBEE_NVM_ENABLED != 0U)
  if (ZbApp.nvm_ready == 0U)
  {
    ZbApp_Log("ZIGBEE: leave cleanup blocked: NVM unavailable\r\n");
    return;
  }
  if (leave_cleanup_pending != 0U)
  {
    int status = EE_Write(0, ZIGBEE_DB_START_ADDR, ZB_CLEANUP_PENDING_MARKER);
    if (status == EE_CLEAN_NEEDED) status = EE_Clean(0, 0);
    if (status != EE_OK)
    {
      ZbApp_Log("ZIGBEE: leave cleanup marker failed; commissioning blocked\r\n");
      return;
    }
  }
#endif
  if (ZigbeeVibrationStore_ResetRecords() != HAL_OK)
  {
    ZbApp_Log("ZIGBEE: Flash record reset failed; commissioning blocked, retry on reboot\r\n");
    return;
  }
  ZbApp_Log("ZIGBEE: Flash records invalidated; no chip erase, slots reusable\r\n");
#if (CFG_ZIGBEE_NVM_ENABLED != 0U)
  if (ZbApp.nvm_ready != 0U)
  {
    int status = EE_Init(1, HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS);
    if (status != EE_OK)
    {
      ZbApp.nvm_ready = 0U;
      ZbApp_Log("ZIGBEE: network erase failed status=%d; recommissioning blocked until reset\r\n", status);
      return;
    }
  }
#endif
  /* The Leave filter deliberately marks join_status failed before this task
   * runs, so ZbApp_PrepareFreshJoin() would otherwise skip its conditional
   * reset. Always clear CPU2's volatile APS/NWK security state after a real
   * Leave before starting factory-new commissioning. */
  ZbReset(ZbApp.zb);
  ZbApp_SetMaximumTxPower();
  leave_cleanup_pending = 0U;
  network_authorized = 0U;
  monitor_standby = (MonitorStandby){0};
  UTIL_LPM_SetStopMode(1U << CFG_LPM_MONITOR_STANDBY, UTIL_LPM_ENABLE);
  ZbApp.config_valid = ZbApp.monitor_enabled = ZbApp.reporting_active = 0U;
  ZbApp.pending_snapshot_valid = ZbApp.report_has_summary_confirm = 0U;
  ZbApp.raw_upload_cycle_counter = 0U;
  ZbApp.persist_save_pending = ZbApp.persist_tx_hold = 0U;
  memset(&ZbApp.pending_snapshot, 0, sizeof(ZbApp.pending_snapshot));
  memset(&ZbPersistCache, 0, sizeof(ZbPersistCache));
  ZbApp.monitor_config = (ZbMonitorConfig_t){300U, 5U, 26667U, 46875U, 256000U,
      500U, 1000U, 1000U, 1U, 0U, CUSTOM_APP_ZB_SENSOR_ALL};
  ZbApp.security_join_cleanup_pending = 0U;
  network_persist_format_v2 = 0U;
  ZbApp.join_blocked_until_reset = 0U;
  ZbApp.rtc_join_active = 0U;
  ZbApp_PrepareFreshJoin();
  ZbApp_StartAdmissionBudget(2U);
  ZbApp_Log("ZIGBEE: network data erased; fresh 30-second commissioning started\r\n");
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
}

#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
static bool ZbApp_PersistSave(void)
{
  uint32_t length;
  bool saved = false;

  if ((leave_cleanup_pending != 0U) || (ZbApp.join_blocked_until_reset != 0U) ||
      (ZbApp.zb == NULL) || (ZbApp.nvm_ready == 0U))
  {
    return false;
  }
  /* ZbPersistGet is synchronous. Its nested CPU2 acknowledgement wait must
     not switch CPU1 to STOP2 or change the UART/IPCC clock mid-command. */
  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_DISABLE);
  ZbApp_Log("ZIGBEE: exporting network state\r\n");
  memset(&ZbPersistCache, 0, sizeof(ZbPersistCache));
  length = ZbPersistGet(ZbApp.zb, NULL, 0U);
  if ((length == 0U) ||
      (length > ZB_PERSIST_MAX_DATA_BYTES))
  {
    ZbApp_Log("ZIGBEE: persistence size invalid=%lu\r\n",
              (unsigned long)length);
    goto done;
  }
  length = ZbPersistGet(ZbApp.zb,
                        &ZbPersistCache.bytes[ZB_PERSIST_DATA_OFFSET],
                        length);
  if ((length == 0U) || (length > ZB_PERSIST_MAX_DATA_BYTES))
  {
    ZbApp_Log("ZIGBEE: persistence export failed\r\n");
    goto done;
  }
  ZbPersistCache.words[0] = length;
  ZbPersistCache.words[1] = ZB_PERSIST_SLEEPY_POLL_MARKER;
  ZbPersistCache.words[2] = network_authorized != 0U ?
      ZB_PERSIST_AUTHORIZED : ZB_PERSIST_AUTH_PENDING;
  if (!ZbApp_NvmWrite())
  {
    goto done;
  }
  ZbApp_Log("ZIGBEE: persisted network saved, %lu bytes\r\n",
            (unsigned long)length);
  {
    uint32_t outgoing = 0U, persisted = 0U;
    enum ZbStatusCodeT a = ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_OutgoingCounter, &outgoing, sizeof(outgoing));
    enum ZbStatusCodeT b = ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_PersistCounter, &persisted, sizeof(persisted));
    ZbApp_Log("ZIGBEE: persistence counters outgoing=%lu status=0x%02x reserved=%lu status=0x%02x\r\n",
              (unsigned long)outgoing, (unsigned int)a, (unsigned long)persisted, (unsigned int)b);
  }
  saved = true;
done:
  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP_ZIGBEE, UTIL_LPM_ENABLE);
  return saved;
}

static void ZbApp_SchedulePersistSave(uint32_t delay_ms)
{
  if (leave_cleanup_pending != 0U) return;
  /* Repeated notifications must not postpone an already scheduled save. */
  if (ZbApp.persist_save_pending != 0U) return;
  ZbApp.persist_save_pending = 1U;
  if (ZbApp.persist_timer != 0xFFU)
  {
    ZbApp_StartTimer(ZbApp.persist_timer, delay_ms);
  }
  else
  {
    /* Never export synchronously in a join callback if the optional timer
       could not be allocated. The next normal report can retry scheduling. */
    ZbApp_Log("ZIGBEE: persistence timer unavailable; save deferred\r\n");
  }
}

static void ZbApp_PersistNotify(struct ZigBeeT *zb, void *arg)
{
  UNUSED(zb);
  UNUSED(arg);
  if (leave_cleanup_pending != 0U) return;
  /* CPU2 callback: no synchronous stack commands or flash operations here. */
  ZbApp.persist_save_pending = 1U;
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_PERSIST, CFG_SCH_PRIO_1);
}

static void ZbApp_EnablePersistence(void)
{
  bool registered = ZbPersistNotifyRegister(ZbApp.zb, ZbApp_PersistNotify, NULL);
  ZbApp_Log("ZIGBEE: stack persistence notifications registered=%u\r\n", registered ? 1U : 0U);
  ZbApp_PersistNotify(ZbApp.zb, NULL);
}

static void ZbApp_PersistTask(void)
{
  if (ZbApp.persist_save_pending == 0U)
  {
    return;
  }
  if ((ZbApp.zb == NULL) ||
      (ZbApp.join_status != ZB_STATUS_SUCCESS) ||
      (ZbApp.join_blocked_until_reset != 0U) ||
      (ZbApp.nvm_ready == 0U))
  {
    ZbApp.persist_save_pending = 0U;
    ZbApp.persist_tx_hold = 0U;
    return;
  }
  if ((ZbApp.radio_exclusive != 0U) ||
      (ZbApp.startup_wait_active != 0U) ||
      (ZbApp.restore_pending != 0U) ||
      (ZbApp.audio_waiting != 0U) ||
      (ZbApp.monitor_state == ZB_MONITOR_CAPTURING))
  {
    ZbApp_StartTimer(ZbApp.persist_timer, ZB_PERSIST_RETRY_DELAY_MS);
    return;
  }
  /* Stop submitting new application frames while existing confirmations drain.
   * Pending waveform data remains queued. A long upload cannot starve NVM. */
  ZbApp.persist_tx_hold = 1U;
  if (ZbApp.tx_in_flight != 0U)
  {
    ZbApp_StartTimer(ZbApp.persist_timer, ZB_PERSIST_SAVE_DELAY_MS);
    return;
  }

  ZbApp.persist_save_pending = 0U;
  if (!ZbApp_PersistSave())
  {
    ZbApp_Log("ZIGBEE: persistence save deferred for retry\r\n");
    ZbApp_SchedulePersistSave(ZB_PERSIST_SAVE_DELAY_MS);
  }
  ZbApp.persist_tx_hold = 0U;
  UTIL_SEQ_SetTask((1U << CFG_TASK_ZIGBEE_TX) | (1U << CFG_TASK_ZIGBEE_UPLOAD), CFG_SCH_PRIO_0);
}

#endif

static enum ZbStatusCodeT ZbApp_StartPersistedNetwork(void)
{
  uint64_t extended_pan_id = 0U;
  enum ZbStatusCodeT status;

  if (!ZbApp_NvmRead())
  {
    return ZB_STATUS_ALLOC_FAIL;
  }
  (void)ZbNwkSet(ZbApp.zb, ZB_NWK_NIB_ID_ExtendedPanId,
                 &extended_pan_id, sizeof(extended_pan_id));
  /* The CPU2 response and the eventual restoration notification are
     separate. Do not start a factory join while the latter is pending. */
  ZbApp.restore_pending = 1U;
  ZbApp_StartAdmissionBudget(1U);
  ++persist_restore_generation;
  persist_restore_request_generation = persist_restore_generation;
  persist_restore_inflight = 1U;
  status = ZbStartupPersist(ZbApp.zb,
                            &ZbPersistCache.bytes[ZB_PERSIST_DATA_OFFSET],
                            ZbPersistCache.words[0],
                            NULL,
                            ZbApp_PersistRestoreCallback,
                            (void *)(uintptr_t)persist_restore_generation);
  if (status != ZB_STATUS_SUCCESS)
  {
    persist_restore_inflight = 0U;
    ZbApp.restore_pending = 0U;
  }
  return status;
}

static void ZbApp_PersistRestoreCallback(enum ZbStatusCodeT status, void *arg)
{
  struct ZbChannelListT active_channels;

  ZbApp_Log("ZIGBEE-DIAG: restore callback status=0x%02x gen=%lu current=%lu expired=%u blocked=%u pending=%u\r\n",
            (unsigned int)status, (unsigned long)(uintptr_t)arg,
            (unsigned long)persist_restore_generation, (unsigned int)admission_expired,
            (unsigned int)ZbApp.join_blocked_until_reset, (unsigned int)ZbApp.restore_pending);
  if (persist_restore_inflight != 0U &&
      (uint32_t)(uintptr_t)arg == persist_restore_request_generation)
  {
    persist_restore_inflight = 0U;
    if (admission_phase == 5U && admission_expired == 0U)
    {
      HW_TS_Stop(admission_timer_id);
      /* Reset only after this notification has returned to the sequencer. */
      admission_phase = 1U;
      admission_expired = 1U;
      ZbApp_Log("ZIGBEE-DIAG: expired restore completed status=0x%02x; reset queued\r\n",
                (unsigned int)status);
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ADMISSION_TIMEOUT, CFG_SCH_PRIO_0);
      return;
    }
  }
  if ((uint32_t)(uintptr_t)arg != persist_restore_generation ||
      admission_expired != 0U || ZbApp.join_blocked_until_reset != 0U)
  {
    ZbApp_Log("ZIGBEE-DIAG: restore callback ignored\r\n");
    return;
  }
  ZbApp.restore_pending = 0U;
  if (status == ZB_STATUS_SUCCESS)
  {
    /* Restart only the recovery timer; preserve authorization and network.
     * Phase 1 is closed by a current parent confirmation, not restore alone. */
    HW_TS_Stop(admission_timer_id);
    ZbApp_StartTimer(admission_timer_id, ZB_RESTORED_PARENT_VERIFY_MS);
    ZbApp_Log("ZIGBEE: restored-parent verification window started (%lu ms)\r\n",
              (unsigned long)ZB_RESTORED_PARENT_VERIFY_MS);
    ZbApp.join_status = ZB_STATUS_SUCCESS;
    recovery_token_valid = 0U;
    recovery_config_pending = 0U;
    recovery_retry_count = 0U;
    /* Restored parameters prove only that a former association existed.
       Keep the gateway offline until a current APS transaction is confirmed. */
    ZbApp.parent_online = 0U;
    ZbApp.parent_state_known = 0U;
    ZbApp.parent_sync_pending = 0U;
    ZbApp.restore_recovery_active = 1U;
    ZbApp.saved_channel_rejoin_attempts = 0U;
    ZbApp.rejoin_channel_mask = 0U;
    memset(&active_channels, 0, sizeof(active_channels));
    if (ZbNwkGet(ZbApp.zb, ZB_NWK_NIB_ID_ActiveChannelList,
                 &active_channels, sizeof(active_channels)) == ZB_STATUS_SUCCESS)
    {
      for (uint8_t index = 0U; index < active_channels.count; index++)
      {
        if (active_channels.list[index].page == 0U)
        {
          ZbApp.rejoin_channel_mask |=
              active_channels.list[index].channelMask & ZB_ALL_CHANNELS_MASK;
        }
      }
    }
    ZbApp_Log("ZIGBEE: saved-network recovery restricted to mask=0x%08lx\r\n",
              (unsigned long)ZbApp.rejoin_channel_mask);
#if (ZB_SLEEPY_PERSIST_AUTOSAVE_ENABLED != 0U)
    /* Save the advanced security state even if the parent is unavailable.
       Repeatedly restoring the same old blob can reuse a counter window. */
    if (ZbApp.nvm_ready != 0U)
    {
      ZbApp_EnablePersistence();
      ZbApp_Log("ZIGBEE: post-restore network-state refresh scheduled\r\n");
    }
#else
    ZbApp_Log("ZIGBEE: sleepy-node persistence autosave deferred; scheduler ready\r\n");
#endif
    ZbApp_Log("ZIGBEE: persisted network restore success; coordinator link unverified\r\n");
    ZbApp_StartAutonomousReporting();
    if (ZbApp.radio_exclusive == 0U)
    {
      /* The token handshake owns restored-parent verification. Do not race
       * it with an APS probe that can initiate another startup operation. */
      ZbApp.manual_announce_required = 1U;
      UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_ANNOUNCE, CFG_SCH_PRIO_0);
    }
    return;
  }
  ZbApp_Log("ZIGBEE: persisted network restore failed status=0x%02x\r\n", status);
  ZbApp_StartAdmissionBudget(2U);
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
}

static bool ZbApp_CheckWirelessFirmware(void)
{
  WirelessFwInfo_t firmware_info;

  memset(&firmware_info, 0, sizeof(firmware_info));
  if (SHCI_GetWirelessFwInfo(&firmware_info) != SHCI_Success)
  {
    ZbApp_Log("ZIGBEE: cannot read CPU2 firmware information\r\n");
    return false;
  }
  ZbApp_Log("ZIGBEE: CPU2 FW %u.%u.%u stack=0x%02x\r\n",
            firmware_info.VersionMajor, firmware_info.VersionMinor,
            firmware_info.VersionSub, firmware_info.StackType);
  switch (firmware_info.StackType)
  {
    case INFO_STACK_TYPE_BLE_ZIGBEE_RFD_DYNAMIC:
    case INFO_STACK_TYPE_BLE_ZIGBEE_RFD_STATIC:
    case INFO_STACK_TYPE_BLE_ZIGBEE_FFD_DYNAMIC:
    case INFO_STACK_TYPE_BLE_ZIGBEE_FFD_STATIC:
    case INFO_STACK_TYPE_ZIGBEE_RFD:
    case INFO_STACK_TYPE_ZIGBEE_FFD:
      return true;

    default:
      return false;
  }
}

static void ZbStartupWaitCb(enum ZbStatusCodeT status, void *callback_arg)
{
  ZbApp_Log("ZIGBEE-DIAG: startup callback status=0x%02x gen=%lu current=%lu expired=%u\r\n",
            (unsigned int)status, (unsigned long)(uintptr_t)callback_arg,
            (unsigned long)startup_wait_generation, (unsigned int)admission_expired);
  ZbApp_StartupOperationCompleted((uint32_t)(uintptr_t)callback_arg);
  if ((uint32_t)(uintptr_t)callback_arg != startup_wait_generation || admission_expired)
  {
    ZbApp_Log("ZIGBEE-DIAG: startup callback ignored\r\n");
    return;
  }
  struct ZbStartupWaitInfo *info = &startup_wait_info;
  info->status = status;
  UTIL_SEQ_SetEvt(EVENT_ZIGBEE_STARTUP_ENDED);
}

static void ZbStartupRejoinWaitCb(struct ZbNlmeJoinConfT *confirm,
                                 void *callback_arg)
{
  ZbApp_Log("ZIGBEE-DIAG: rejoin callback status=0x%02x gen=%lu current=%lu expired=%u\r\n",
            (unsigned int)(confirm != NULL ? confirm->status : (enum ZbStatusCodeT)0xFF),
            (unsigned long)(uintptr_t)callback_arg,
            (unsigned long)startup_wait_generation, (unsigned int)admission_expired);
  ZbApp_StartupOperationCompleted((uint32_t)(uintptr_t)callback_arg);
  if ((uint32_t)(uintptr_t)callback_arg != startup_wait_generation || admission_expired) return;
  struct ZbStartupWaitInfo *info = &startup_wait_info;

  info->status = (confirm != NULL) ? confirm->status :
                 (enum ZbStatusCodeT)0xFF;
  UTIL_SEQ_SetEvt(EVENT_ZIGBEE_STARTUP_ENDED);
}

enum ZbStatusCodeT ZbStartupWait(struct ZigBeeT *zb, struct ZbStartupT *config)
{
  struct ZbStartupWaitInfo *info;
  enum ZbStatusCodeT status;

  info = &startup_wait_info;
  ++startup_wait_generation;
  ZbApp_ReadSecurityDiagnostic(0U);
  info->status = ZB_STATUS_ALLOC_FAIL;
  startup_operation_inflight = 1U;
  ZbApp_Log("ZIGBEE-DIAG: ZbStartup call gen=%lu control=%u restore_pending=%u startup_active=%u join=0x%02x phase=%u\r\n",
            (unsigned long)startup_wait_generation, (unsigned int)config->startupControl,
            (unsigned int)ZbApp.restore_pending, (unsigned int)ZbApp.startup_wait_active,
            (unsigned int)ZbApp.join_status, (unsigned int)admission_phase);
  ZbApp_SetMaximumTxPower();
  status = ZbStartup(zb, config, ZbStartupWaitCb, (void *)(uintptr_t)startup_wait_generation);
  ZbApp_Log("ZIGBEE-DIAG: ZbStartup immediate status=0x%02x gen=%lu\r\n",
            (unsigned int)status, (unsigned long)startup_wait_generation);
  if (status != ZB_STATUS_SUCCESS)
  {
    startup_operation_inflight = 0U;
    return status;
  }
  UTIL_SEQ_WaitEvt(EVENT_ZIGBEE_STARTUP_ENDED);
  status = info->status;
  if (admission_expired) status = (enum ZbStatusCodeT)0xFF;
  return status;
}

static enum ZbStatusCodeT ZbStartupRejoinWait(struct ZigBeeT *zb)
{
  struct ZbStartupWaitInfo *info;
  enum ZbStatusCodeT status;

  info = &startup_wait_info;
  ++startup_wait_generation;
  info->status = ZB_STATUS_ALLOC_FAIL;
  startup_operation_inflight = 1U;
  ZbApp_Log("ZIGBEE-DIAG: ZbStartupRejoin call gen=%lu phase=%u\r\n",
            (unsigned long)startup_wait_generation, (unsigned int)admission_phase);
  ZbApp_SetMaximumTxPower();
  status = ZbStartupRejoin(zb, ZbStartupRejoinWaitCb, (void *)(uintptr_t)startup_wait_generation);
  ZbApp_Log("ZIGBEE-DIAG: ZbStartupRejoin immediate status=0x%02x gen=%lu\r\n",
            (unsigned int)status, (unsigned long)startup_wait_generation);
  if (status != ZB_STATUS_SUCCESS)
  {
    startup_operation_inflight = 0U;
    return status;
  }
  UTIL_SEQ_WaitEvt(EVENT_ZIGBEE_STARTUP_ENDED);
  status = info->status;
  if (admission_expired) status = (enum ZbStatusCodeT)0xFF;
  return status;
}

void APP_ZIGBEE_Error(uint32_t error_id, uint32_t error_code)
{
  ZbApp_Log("ZIGBEE: error id=%lu code=%lu\r\n",
            (unsigned long)error_id, (unsigned long)error_code);
}

void APP_ZIGBEE_RegisterCmdBuffer(TL_CmdPacket_t *buffer)
{
  p_ZIGBEE_otcmdbuffer = buffer;
}

Zigbee_Cmd_Request_t *ZIGBEE_Get_OTCmdPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)p_ZIGBEE_otcmdbuffer->cmdserial.cmd.payload;
}

Zigbee_Cmd_Request_t *ZIGBEE_Get_OTCmdRspPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)((TL_EvtPacket_t *)p_ZIGBEE_otcmdbuffer)->evtserial.evt.payload;
}

Zigbee_Cmd_Request_t *ZIGBEE_Get_NotificationPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)p_ZIGBEE_notif_M0_to_M4->evtserial.evt.payload;
}

Zigbee_Cmd_Request_t *ZIGBEE_Get_M0RequestPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)p_ZIGBEE_request_M0_to_M4->evtserial.evt.payload;
}

void ZIGBEE_CmdTransfer(void)
{
  Zigbee_Cmd_Request_t *request =
      (Zigbee_Cmd_Request_t *)p_ZIGBEE_otcmdbuffer->cmdserial.cmd.payload;
  p_ZIGBEE_otcmdbuffer->cmdserial.cmd.cmdcode = 0x280U;
  p_ZIGBEE_otcmdbuffer->cmdserial.cmd.plen = 8U + (request->Size * 4U);
  TL_ZIGBEE_SendM4RequestToM0();
  Wait_Getting_Ack_From_M0();
}

void ZIGBEE_CmdTransferWithNotif(void)
{
  g_ot_notification_allowed = 1U;
  ZIGBEE_CmdTransfer();
}

void TL_ZIGBEE_CmdEvtReceived(TL_EvtPacket_t *buffer)
{
  UNUSED(buffer);
  Receive_Ack_From_M0();
}

void TL_ZIGBEE_NotReceived(TL_EvtPacket_t *buffer)
{
  p_ZIGBEE_notif_M0_to_M4 = buffer;
  Receive_Notification_From_M0();
}

void Pre_ZigbeeCmdProcessing(void)
{
  UTIL_SEQ_WaitEvt(EVENT_SYNCHRO_BYPASS_IDLE);
}

static void Wait_Getting_Ack_From_M0(void)
{
  UTIL_SEQ_WaitEvt(EVENT_ACK_FROM_M0_EVT);
}

static void Receive_Ack_From_M0(void)
{
  UTIL_SEQ_SetEvt(EVENT_ACK_FROM_M0_EVT);
}

static void Receive_Notification_From_M0(void)
{
  CptReceiveNotifyFromM0++;
  UTIL_SEQ_SetTask(1U << CFG_TASK_NOTIFY_FROM_M0_TO_M4, CFG_SCH_PRIO_0);
}

void TL_ZIGBEE_M0RequestReceived(TL_EvtPacket_t *buffer)
{
  p_ZIGBEE_request_M0_to_M4 = buffer;
  CptReceiveRequestFromM0++;
  UTIL_SEQ_SetTask(1U << CFG_TASK_REQUEST_FROM_M0_TO_M4, CFG_SCH_PRIO_0);
}

void APP_ZIGBEE_TL_INIT(void)
{
  ZigbeeConfigBuffer.p_ZigbeeOtCmdRspBuffer = (uint8_t *)&ZigbeeOtCmdBuffer;
  ZigbeeConfigBuffer.p_ZigbeeNotAckBuffer = ZigbeeNotifRspEvtBuffer;
  ZigbeeConfigBuffer.p_ZigbeeNotifRequestBuffer = ZigbeeNotifRequestBuffer;
  TL_ZIGBEE_Init(&ZigbeeConfigBuffer);
}

static void APP_ZIGBEE_ProcessNotifyM0ToM4(void)
{
  if (CptReceiveNotifyFromM0 != 0U)
  {
    CptReceiveNotifyFromM0 = 0U;
    Zigbee_CallBackProcessing();
  }
}

static void APP_ZIGBEE_ProcessRequestM0ToM4(void)
{
  if (CptReceiveRequestFromM0 != 0U)
  {
    CptReceiveRequestFromM0 = 0U;
    Zigbee_M0RequestProcessing();
  }
}
