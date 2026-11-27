#include "gateway_exchange.h"
#include "control_sync.h"
#include "monitor_standby.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
/* Real state machine + extracted production poll/settings code; only hardware and radio are mocked. */
#define RTC_BKP_DR0 0U
#define PWR_CR1_DBP 0x100U
#define CUSTOM_APP_ZB_SENSOR_ALL 7U
#define CFG_LPM_GATEWAY_EXCHANGE 7U
#define CFG_LPM_MONITOR_STANDBY 8U
#define CFG_TASK_ZIGBEE_REPORT 10U
#define CFG_SCH_PRIO_1 1U
#define ZB_CONFIG_FLAG_MONITOR_PRESENT 4U
#define ZB_CONFIG_FLAG_MONITOR_ENABLED 2U
#define ZB_CONFIG_QUERY_JOIN 1U
#define ZB_MONITOR_SLEEP 0U
#define ZB_MONITOR_WAITING 1U
#define UTIL_LPM_DISABLE 0U
#define UTIL_LPM_ENABLE 1U
#define CFG_TASK_ZIGBEE_UPLOAD 9U
#define CFG_SCH_PRIO_0 0U
#define ZB_NWK_NIB_ID_FastPollPeriod 0U
#define ZB_NWK_SYNC_DEFAULT_DELAY_MS 1000U
#define V2_TX_IDLE 0U
#define V2_BATCH_LEN 77U
typedef int RTC_HandleTypeDef;
static RTC_HandleTypeDef hrtc;
static struct {uint32_t CR1;} power;
#define PWR (&power)
static uint32_t regs[20],now,fail_write,changed_during_write,save_writes;
static uint8_t capture,stream,ready=1,radio_fail,stop_locked;
static unsigned sends,releases,resends,upload_wakes,legacy_replies;
static uint8_t gateway_legacy_reply_pending,legacy_reply_result;
enum ZclStatusCodeT { ZCL_STATUS_SUCCESS=0,ZCL_STATUS_FAILURE=1 };
#define ZCL_BASIC_POWER_DC 4U
static uint8_t coordinator_endpoint=1,recovery_announce_pending;
static uint32_t applied_config_hash=0x11223344;
static uint8_t legacy_frame[CONTROL_FINAL_SIZE];
static enum ZclStatusCodeT ZbApp_SendCommand(uint8_t code,const uint8_t *p,uint16_t n){
 assert(code==CONTROL_STATUS_CMD);
 assert(n==CONTROL_STATUS_SIZE || n==CONTROL_FINAL_SIZE);assert(control_status_valid(p,n));
 if(radio_fail)return ZCL_STATUS_FAILURE;
 legacy_replies++;legacy_reply_result=(p[1]&CONTROL_APPLIED)?1:((p[1]&CONTROL_SAVE_FAILED)?2:0);
 memcpy(legacy_frame,p,n);return ZCL_STATUS_SUCCESS;
}
static uint8_t latest[82];
static struct {
 struct {
  uint32_t auto_interval_sec,analog_mic_rate_hz,raw_upload_generation;
  uint16_t poll_interval_sec,vibration_rate_hz,digital_mic_rate_hz;
  uint16_t vibration_window_ms,digital_mic_window_ms,analog_mic_window_ms,raw_upload_every_cycles;
  uint8_t sensor_enable_mask;
 } monitor_config;
 uint8_t radio_exclusive,restore_pending,restore_recovery_active,monitor_state;
 uint8_t config_query_pending,monitor_enabled,config_valid,tx_count,tx_in_flight,persist_save_pending,persist_tx_hold;
 uint8_t upload_allowed,report_timer;
 void *zb;
} ZbApp;
static struct {uint8_t busy;} ZbUploadSlots[3];
static struct {uint8_t phase;} ZbVibrationUpload;
static struct {uint8_t active,confirmed;uint32_t batch;} ZbDeferred;
static uint8_t hy_zdo_inflight;
static MonitorStandby monitor_standby;
static uint8_t network_authorized=1,ZbUploadQueueSleeping,sampling_due,sampling_timer=0xff;
static struct {uint8_t active;} sampling_cadence;
static uint32_t stop_disabled_mask;
static uint32_t upload_generation,retained_batch;
static int ZigbeeVibrationStore_DescribeBatch(uint32_t batch,uint8_t *descriptor){(void)descriptor;return batch && batch==retained_batch;}
static GatewayExchange gateway_exchange;
static uint8_t gateway_settings_dirty,gateway_mode_save_failed,gateway_loaded_valid,gateway_loaded_enabled;
static uint32_t gateway_save_retry;
static uint32_t CommWindows[5];static uint16_t CommFactor=1;
static struct nwk_fastpoll_entry_t {int value;} fast_poll;
static struct nwk_fastpoll_entry_t *gateway_fast_poll;
static struct {int dummy;} gateway_scalar_cache;
static uint8_t gateway_scalar_cache_valid;
static uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *r,uint32_t i){(void)r;assert(i<16);return regs[i];}
static void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *r,uint32_t i,uint32_t v){
 (void)r;assert(i<16 && (PWR->CR1&PWR_CR1_DBP));save_writes++;
 if(changed_during_write){gateway_exchange.revision++;changed_during_write=0;}
 if(!fail_write)regs[i]=v;
}
static void HAL_PWR_EnableBkUpAccess(void){PWR->CR1|=PWR_CR1_DBP;}
static void HAL_PWR_DisableBkUpAccess(void){PWR->CR1&=~PWR_CR1_DBP;}
static void ZbApp_Log(const char *fmt,...){(void)fmt;}
static uint32_t APP_ElapsedMs(void){return now;}
static uint8_t APP_ZIGBEE_IsMeasuring(void){return capture;}
static uint8_t Custom_APP_ProtocolStreaming(void){return stream;}
static uint8_t APP_ZIGBEE_ProtocolReady(void){return ready;}
static uint8_t APP_ZIGBEE_IsConnected(void){return ready;}
static uint8_t v2_tx_active(const void *p){(void)p;return ZbVibrationUpload.phase!=0;}
static void UTIL_LPM_SetStopMode(uint32_t bit,uint8_t mode){
 assert(bit==1U<<CFG_LPM_GATEWAY_EXCHANGE || bit==1U<<CFG_LPM_MONITOR_STANDBY);
 if(mode==UTIL_LPM_DISABLE)stop_disabled_mask|=bit;else stop_disabled_mask&=~bit;
 if(bit==1U<<CFG_LPM_GATEWAY_EXCHANGE)stop_locked=mode==UTIL_LPM_DISABLE;
}
static void ZbApp_ClearDeferredUpload(void){memset(&ZbDeferred,0,sizeof(ZbDeferred));}
static void UTIL_SEQ_SetTask(uint32_t bit,unsigned priority){(void)bit;(void)priority;upload_wakes++;}
static void ZbApp_SendSnapshot(const void *p,uint32_t batch){(void)p;(void)batch;resends++;}
static int ZbNwkFastPollRelease(void *zb,void *handle){(void)zb;assert(handle==&fast_poll);releases++;return 0;}
static void ZbApp_StopConfigTransaction(void){}
static void ZbApp_ScheduleNextReport(void){}
static void ZbApp_ScheduleMonitorStandby(void);
static void ZbApp_StartConfigTransaction(void){}
static void ZbApp_StartTimer(uint8_t timer,uint32_t delay){(void)timer;(void)delay;}
static void ZbApp_CheckAuthorizationBudget(void){}
static void ZbApp_OpenControlWindow(uint8_t reason){(void)reason;}
static void Custom_APP_ZigbeeActivityChanged(void){}
static void HW_TS_Stop(uint8_t timer){(void)timer;}
static int ZbNwkSet(void *a,unsigned b,const void *c,unsigned d){(void)a;(void)b;(void)c;(void)d;return 0;}
static struct nwk_fastpoll_entry_t *ZbNwkFastPollRequest(void *a,unsigned b,unsigned c){(void)a;(void)b;(void)c;return &fast_poll;}
#include "gateway_settings_under_test.inc"
static CommResult transport(const CommSource *from,const uint8_t *p,uint16_t n){
 assert(from->link==COMM_LINK_ZIGBEE&&n<=82);if(radio_fail)return COMM_BUSY;
 memcpy(latest,p,n);sends++;return COMM_OK;
}
static CommResult execute(const CommRequest *r,uint16_t *active){*active=0;if(r->code==COMM_CMD_DECIMATION)CommFactor=r->factor;return COMM_OK;}
static int decision(uint8_t result,uint16_t mask,uint16_t seq,uint32_t revision){
 uint8_t p[15];Comm_WriteU32(p,gateway_exchange.token);Comm_WriteU32(p+4,gateway_exchange.batch);
 Comm_WriteU32(p+8,revision);p[12]=result;Comm_WriteU16(p+13,mask);
 return gateway_decision(&gateway_exchange,p,sizeof(p),seq);
}
static void tick(void){now+=2100;APP_ZIGBEE_GatewayProcess();}
static void test_standby_lock_cleanup(void){
 /* Reproduce: OFF's first ten-minute awake window -> real 0x08 -> final gateway permit. */
 memset(&gateway_exchange,0,sizeof(gateway_exchange));memset(&monitor_standby,0,sizeof(monitor_standby));
 gateway_settings_dirty=gateway_mode_save_failed=gateway_legacy_reply_pending=0;
 capture=stream=radio_fail=fail_write=0;ready=network_authorized=1;
 ZbApp.monitor_enabled=0;ZbApp.config_query_pending=ZbApp.upload_allowed=0;
 memset(&ZbDeferred,0,sizeof(ZbDeferred));ZbVibrationUpload.phase=0;
 stop_disabled_mask=0;
 ZbApp_ScheduleMonitorStandby();
 assert(monitor_standby.seen==1 && monitor_standby.until==now+600000U);
 assert(stop_disabled_mask&(1U<<CFG_LPM_MONITOR_STANDBY));
 ZbApp_GatewayBegin(700);tick();assert(decision(0,2,1,0));
 assert(invoke_legacy_report_start()==ZCL_STATUS_SUCCESS && ZbApp.monitor_enabled==1);
 assert(!(stop_disabled_mask&(1U<<CFG_LPM_MONITOR_STANDBY)));
 assert(stop_disabled_mask&(1U<<CFG_LPM_GATEWAY_EXCHANGE)); /* not permission to sleep early */
 assert(monitor_standby.seen==2 && monitor_standby.until==0);
 tick();assert(decision(0,1,2,gateway_exchange.revision));tick();
 assert(!gateway_exchange.active && stop_disabled_mask==0);
 /* Re-enabling an already enabled monitor must clear a stale lock without resetting its state. */
 ZbApp.monitor_state=2;monitor_standby.seen=1;monitor_standby.until=now+600000U;
 UTIL_LPM_SetStopMode(1U<<CFG_LPM_MONITOR_STANDBY,UTIL_LPM_DISABLE);
 assert(ZbApp_ApplyMonitorSwitch(ZB_CONFIG_FLAG_MONITOR_PRESENT|ZB_CONFIG_FLAG_MONITOR_ENABLED)==1);
 assert(!(stop_disabled_mask&(1U<<CFG_LPM_MONITOR_STANDBY)) && ZbApp.monitor_state==2);
 /* Final permission also sweeps a stale standby lock on either ON or OFF branch, without waiting. */
 for(unsigned enabled=0;enabled<2;enabled++) {
  ZbApp.monitor_enabled=(uint8_t)enabled;monitor_standby.seen=1;monitor_standby.until=now+600000U;
  UTIL_LPM_SetStopMode(1U<<CFG_LPM_MONITOR_STANDBY,UTIL_LPM_DISABLE);
  ZbApp_GatewayBegin(701+enabled);tick();assert(decision(0,1,1,0));
  APP_ZIGBEE_GatewayProcess(); /* same timestamp: no fixed wake interval */
  assert(!gateway_exchange.active && stop_disabled_mask==0 && monitor_standby.until==0);
 }
 /* Cleanup owns only MONITOR_STANDBY; an unrelated BLE lock must remain. */
 stop_disabled_mask|=1U<<1;
 UTIL_LPM_SetStopMode(1U<<CFG_LPM_MONITOR_STANDBY,UTIL_LPM_DISABLE);
 ZbApp_ReleaseMonitorStandbyLock();assert(stop_disabled_mask==(1U<<1));stop_disabled_mask=0;
 puts("standby regression: legacy 0x08, repeat enable, ON/OFF final-permit cleanup passed; no ten-minute wait");
}
int main(void){
 CommSource source={COMM_LINK_ZIGBEE,20};uint8_t command[14],p[2]={4,0};uint16_t n;unsigned before;
 Comm_Init(execute);Comm_RegisterTransport(COMM_LINK_ZIGBEE,transport);Comm_SetSession(COMM_LINK_ZIGBEE,20);
 Comm_RegisterExecutedObserver(ZbApp_CommonCommandDone);
 Comm_RegisterCommandFinalizer(ZbApp_FinalizeCommand,ZbApp_FinalResponseSubmitted);
 ZbApp.monitor_config.auto_interval_sec=300;ZbApp.monitor_config.poll_interval_sec=5;
 ZbApp.monitor_config.vibration_rate_hz=26667;ZbApp.monitor_config.digital_mic_rate_hz=46875;
 ZbApp.monitor_config.analog_mic_rate_hz=256000;ZbApp.monitor_config.vibration_window_ms=500;
 ZbApp.monitor_config.digital_mic_window_ms=1000;ZbApp.monitor_config.analog_mic_window_ms=1000;
 ZbApp.monitor_config.raw_upload_every_cycles=1;ZbApp.monitor_config.sensor_enable_mask=7;
 ZbApp.config_valid=ZbApp.monitor_enabled=1;ZbApp.report_timer=0xff;
 /* No implicit sleep: no ACK, timeout, failed/inconsistent mask all stay locked. */
 ZbApp_GatewayBegin(42);assert(stop_locked);assert(!decision(0,1,1,0));
 tick();assert(latest[0]==1&&latest[1]==4&&latest[2]==0x90&&Comm_ReadU16(latest+6)==13);
 const uint8_t golden[]={1,4,0x90,0,0,0,13,0,1,0,0,0,42,0,0,0,0,0,0,0,0};
 assert(!memcmp(latest,golden,sizeof(golden)));
 assert(Comm_ReadU32(latest+12)==42&&latest[20]==0&&gateway_exchange.advertised);
 for(unsigned i=0;i<8;i++)tick();assert(gateway_exchange.active&&stop_locked);
 assert(!decision(0,3,1,0)&&!decision(COMM_BAD_VALUE,1,1,0));
 assert(decision(0,2,1,0));tick();assert(gateway_exchange.commands&&stop_locked);
 /* Final response must include save failure and revision before any gateway final decision. */
 n=Comm_EncodeFrame(command,sizeof(command),COMM_KIND_COMMAND,COMM_CMD_DECIMATION,99,p,2);
 assert(Comm_Receive(&source,command,n)==COMM_OK);before=save_writes;fail_write=1;
 Comm_Process();assert(CommFactor==4&&gateway_exchange.revision==1&&gateway_settings_dirty);
 assert(save_writes>before&&gateway_exchange.advertised&&gateway_exchange.response_ready);
 assert(latest[1]==COMM_KIND_RESPONSE&&latest[2]==COMM_CMD_DECIMATION&&Comm_ReadU16(latest+6)==8);
 assert(latest[8]==COMM_SAVE_FAILED&&Comm_ReadU32(latest+12)==1);
 before=save_writes;unsigned replies=sends;
 assert(Comm_Receive(&source,command,n)==COMM_OK);Comm_Process();
 assert(gateway_exchange.revision==1&&save_writes==before&&latest[8]==COMM_SAVE_FAILED);
 assert(!decision(0,1,2,0)); /* old revision cannot authorize */
 assert(decision(0,1,2,1));tick();assert(gateway_exchange.active&&stop_locked);
 assert(sends==replies+1); /* only replayed final response; no supplemental ready event */
 fail_write=0;ZbApp.tx_count=1;tick();assert(!gateway_settings_dirty&&gateway_exchange.active);
 ZbApp.tx_count=0;tick();assert(!gateway_exchange.active&&!stop_locked&&releases==1);
 assert(regs[0]==GATEWAY_SETTINGS_MAGIC&&!(PWR->CR1&PWR_CR1_DBP)&&regs[19]==0);
 /* Successful configuration is committed before OK; transmit BUSY never re-saves/re-executes. */
 ZbApp_GatewayBegin(45);tick();assert(decision(0,2,1,0));
 p[0]=8;n=Comm_EncodeFrame(command,sizeof(command),COMM_KIND_COMMAND,COMM_CMD_DECIMATION,101,p,2);
 assert(Comm_Receive(&source,command,n)==COMM_OK);radio_fail=1;Comm_Process();
 assert(CommFactor==8&&!gateway_settings_dirty&&Comm_HasPending()&&!gateway_exchange.response_ready);
 before=save_writes;Comm_Process();assert(save_writes==before);
 radio_fail=0;Comm_Process();assert(latest[8]==COMM_OK&&Comm_ReadU32(latest+12)==1&&gateway_exchange.response_ready);
 assert(decision(0,1,2,1));tick();assert(!gateway_exchange.active);
 CommFactor=4;gateway_settings_dirty=1;ZbApp_GatewaySaveSettings();
 /* Restore actual settings functions, CRC failure and partial commits. */
 CommFactor=1;ZbApp.monitor_config.auto_interval_sec=999;ZbApp_GatewayLoadSettings();
 assert(CommFactor==4&&ZbApp.monitor_config.auto_interval_sec==300&&gateway_loaded_valid&&gateway_loaded_enabled);
 regs[2]^=1;CommFactor=2;ZbApp_GatewayLoadSettings();assert(CommFactor==2);regs[2]^=1;
 regs[0]=0;CommFactor=3;ZbApp_GatewayLoadSettings();assert(CommFactor==3);regs[0]=GATEWAY_SETTINGS_MAGIC;
 PWR->CR1=PWR_CR1_DBP;gateway_settings_dirty=1;ZbApp_GatewaySaveSettings();assert(PWR->CR1&PWR_CR1_DBP);
 gateway_settings_dirty=1;changed_during_write=1;ZbApp_GatewaySaveSettings();assert(gateway_settings_dirty);
 ZbApp_GatewaySaveSettings();assert(!gateway_settings_dirty);
 /* New exchange invalidates prior decisions; older decision sequence cannot revoke final grant. */
 ZbApp_GatewayBegin(43);tick();assert(!decision(0,1,1,99));
 assert(decision(0,2,1,0));assert(decision(0,1,2,0));assert(!decision(0,2,1,0));
 capture=1;tick();assert(gateway_exchange.active);capture=0;
 ZbApp.persist_save_pending=1;tick();assert(gateway_exchange.active);ZbApp.persist_save_pending=0;
 tick();assert(!gateway_exchange.active);
 /* Scalar receipt does not wait for unrelated older Flash backlog to start uploading. */
 ZbApp_GatewayBegin(70);gateway_scalar_cache_valid=1;ZbApp.upload_allowed=1;tick();
 assert(gateway_exchange.advertised&&decision(0,1,1,0));tick();assert(!gateway_exchange.active);ZbApp.upload_allowed=0;
 /* A final permit overtaking a queued command cannot bypass execution/save. */
 ZbApp_GatewayBegin(80);tick();assert(decision(0,2,1,0));
 n=Comm_EncodeFrame(command,sizeof(command),COMM_KIND_COMMAND,COMM_CMD_STATUS,102,NULL,0);
 assert(Comm_Receive(&source,command,n)==COMM_OK);assert(decision(0,1,2,0));
 assert(!gateway_can_finish(&gateway_exchange,Comm_HasPending(),0));
 Comm_Process();assert(gateway_exchange.revision==1&&gateway_exchange.mask==0&&gateway_exchange.advertised&&gateway_exchange.response_ready);
 assert(Comm_ReadU32(latest+12)==1);assert(decision(0,1,3,1));tick();assert(!gateway_exchange.active);
 /* A command-triggered acquisition changes token/batch but keeps the approved command window. */
 ZbApp_GatewayBegin(100);tick();assert(decision(0,2,1,0));
 uint32_t previous_token=gateway_exchange.token;
 ZbApp_GatewayBegin(101);assert(gateway_exchange.token!=previous_token&&gateway_exchange.commands&&!gateway_exchange.valid);
 tick();assert(decision(0,1,1,0));tick();assert(!gateway_exchange.active);
 /* Keep the exchange alive until the uploader acknowledges/releases the retained Flash record. */
 ZbApp_GatewayBegin(102);ZbDeferred.active=ZbDeferred.confirmed=1;ZbDeferred.batch=102;
 tick();assert(decision(0,1,1,0));tick();assert(gateway_exchange.active&&stop_locked);
 ZbApp_ClearDeferredUpload();tick();assert(!gateway_exchange.active);
 /* Unknown coordinator/offline never grants permission. Rejection preserves/requeues raw batch. */
 ZbApp_GatewayBegin(44);ready=0;tick();assert(stop_locked&&!gateway_exchange.advertised);ready=1;
 ZbDeferred.active=ZbDeferred.confirmed=1;ZbDeferred.batch=44;ZbApp.upload_allowed=1;retained_batch=44;
 tick();assert(gateway_exchange.advertised);uint32_t token=gateway_exchange.token;
 assert(decision(COMM_BAD_VALUE,0,1,0));tick();assert(gateway_exchange.active&&gateway_exchange.token!=token);
 assert(!ZbDeferred.active&&ZbApp.upload_allowed&&upload_wakes);
 tick();assert(!gateway_exchange.advertised); /* pending raw upload is not DATA_END */
 /* A failed uploader may already have cleared its RAM context; Flash still retries. */
 ZbApp.upload_allowed=0;tick();assert(gateway_exchange.advertised);
 token=gateway_exchange.token;assert(decision(COMM_BAD_VALUE,0,1,0));tick();
 assert(gateway_exchange.token!=token&&ZbApp.upload_allowed);
 retained_batch=0;ZbApp.upload_allowed=0;ZbApp_GatewayChanged();tick();gateway_mode_save_failed=1;
 assert(legacy_replies&&legacy_reply_result==1);
 assert(Comm_ReadU32(legacy_frame+8)==gateway_exchange.revision);
 assert(decision(0,1,1,gateway_exchange.revision));tick();assert(stop_locked&&gateway_exchange.active);
 /* Legacy configuration gets a final failure reply, not an early APPLIED status. */
 gateway_mode_save_failed=0;fail_write=1;CommFactor=9;ZbApp_GatewayChanged();tick();
 assert(legacy_reply_result==2&&gateway_settings_dirty&&gateway_exchange.response_ready&&stop_locked);
 assert((legacy_frame[1]&CONTROL_SAVE_FAILED)&&!(legacy_frame[1]&CONTROL_APPLIED)&&Comm_ReadU32(legacy_frame+8)==gateway_exchange.revision);
 test_standby_lock_cleanup();
 puts("gateway exchange: real poll/observer/backup passed permission, pending commands, stale ACK, queue replay, readback failure, partial commit, restore, local busy and rejected-data retry tests");
 return 0;
}
