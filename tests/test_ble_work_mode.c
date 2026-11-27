/* Exercise production mode functions through the real common protocol queue. */
#include "comm_protocol.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
typedef int RTC_HandleTypeDef;
typedef uint8_t tBleStatus;
typedef enum { CUSTOM_APP_WORK_MODE_ONLINE=0, CUSTOM_APP_WORK_MODE_INSPECTION=1 } Custom_APP_WorkMode_t;
#define RTC_BKP_DR19 19
#define PWR_CR1_DBP 0x100U
#define BLE_STATUS_SUCCESS 0
#define BLE_STATUS_BUSY 0x46
#define BLE_STATUS_INSUFFICIENT_RESOURCES 0x64
#define CUSTOM_STM_SENSOR_CTRL 1
#define CUSTOM_STM_SENSOR_STATUS 2
#define CUSTOM_STM_VIB_WAVE 3
#define CFG_TASK_DT_SEND_ID 5U
#define CFG_SCH_PRIO_1 1U
#define CFG_TS_TICK_VAL 500U
static RTC_HandleTypeDef hrtc;
static struct { uint32_t CR1; } power;
#define PWR (&power)
static struct { uint8_t IsConnected,Sensor_status_Notification_Status,SensorSeq,SensorEnableMask,TxFlowOn,Vib_wave_Notification_Status; } Custom_App_Context;
static uint8_t DT_ActiveWorkMode,DT_ConfiguredWorkMode,DT_WorkModeResult;
static uint8_t DT_WorkModeSendPending,DT_WorkModeReadPending,DT_WorkModeSendAttempts,DT_WorkModeReadAttempts;
static uint32_t DT_WorkModeReplyTick,backup;
static uint8_t DT_ModeRestartOnNotify,DT_WorkModeTimerId,DT_WorkModeTimerReady;
static uint32_t tick;
static uint32_t CommBleSession=11U;
static uint16_t CommBleCapacity=20U;
static uint8_t DT_ModeRestartPending,DT_ModeRestartScheduled;
static uint32_t DT_ModeRestartTick;
static unsigned resets,flash_busy;
#define FLASH_FLAG_BSY 1U
#define FLASH_FLAG_CFGBSY 2U
#define __HAL_FLASH_GET_FLAG(flag) (flash_busy & (flag))
static void NVIC_SystemReset(void){resets++;}
static unsigned task_requests, timer_starts;
static unsigned fail_save,fail_gatt,read_count,notify_count,save_count;
static uint8_t read_bytes[11],notify_bytes[16],previous_bytes[16];
static CommSource last_source;
static uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *h,unsigned r){(void)h;assert(r==19);return backup;}
static void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *h,unsigned r,uint32_t v){(void)h;assert(r==19);assert(PWR->CR1&PWR_CR1_DBP);save_count++;if(!fail_save)backup=v;}
static void HAL_PWR_EnableBkUpAccess(void){PWR->CR1|=PWR_CR1_DBP;}
static void HAL_PWR_DisableBkUpAccess(void){PWR->CR1&=~PWR_CR1_DBP;}
static void DT_Log(const char *s){(void)s;}
static uint32_t HAL_GetTick(void){return tick;}
static void UTIL_SEQ_SetTask(uint32_t task,unsigned priority){
 assert(task==(1U<<CFG_TASK_DT_SEND_ID)&&priority==CFG_SCH_PRIO_1);task_requests++;
}
static void HW_TS_Start(uint8_t timer,uint32_t ticks){
 assert(timer==DT_WorkModeTimerId&&ticks==40U);timer_starts++;
}
static uint8_t DT_SensorIdToMask(uint8_t id){return id>=1&&id<=4?(uint8_t)(1U<<(id-1)):0;}
static void DT_ApplySensorEnableMask(uint8_t mask){Custom_App_Context.SensorEnableMask=mask;}
static tBleStatus Custom_STM_App_Update_Char_Variable_Length(unsigned c,uint8_t *p,uint8_t n){
 if (fail_gatt) return 0x0a;
 if (c==CUSTOM_STM_SENSOR_CTRL) {
  assert(n==11);read_count++;memset(read_bytes,0,sizeof(read_bytes));memcpy(read_bytes,p,n);
 } else {
  assert(c==CUSTOM_STM_SENSOR_STATUS&&(n==11||n==15||n==16));notify_count++;
  memset(notify_bytes,0,sizeof(notify_bytes));memcpy(notify_bytes,p,n);
 }
 return BLE_STATUS_SUCCESS;
}
static CommResult transport(const CommSource *s,const uint8_t *p,uint16_t n){
 assert(p[0]==COMM_VERSION && p[3]==0);
 assert(p[1]==COMM_KIND_RESPONSE ? (n==15 && Comm_ReadU16(p+6)==7) : (n==11 && Comm_ReadU16(p+6)==3));
 if(fail_gatt)return COMM_BUSY;
 memcpy(previous_bytes,notify_bytes,sizeof(notify_bytes));memcpy(notify_bytes,p,n);last_source=*s;notify_count++;return COMM_OK;
}
#include "mode-under-test.inc"
typedef struct { uint8_t address_type,address[6]; } Bonded_Device_Entry_t;
static struct { struct { uint16_t connectionHandle; } BleApplicationContext_legacy; } BleApplicationContext;
static uint16_t BondSecurityHandle=0xffffU,BondRebondHandle=0xffffU;
static uint8_t BondReportPending;
static unsigned security_requests,rebond_requests,bond_reports;
static void DT_BLE_TimestampLog(const char *text){(void)text;}
static tBleStatus aci_gap_allow_rebond(uint16_t handle){
 assert(handle==BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
 rebond_requests++;return BLE_STATUS_SUCCESS;
}
static tBleStatus aci_gap_peripheral_security_req(uint16_t handle){
 assert(handle==BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
 security_requests++;return BLE_STATUS_SUCCESS;
}
static tBleStatus aci_gap_get_bonded_devices(uint8_t *count,Bonded_Device_Entry_t *entries){
 *count=1;entries[0].address_type=0;bond_reports++;return BLE_STATUS_SUCCESS;
}
#include "bond-under-test.inc"
static void legacy(uint8_t a,uint8_t b,unsigned n){uint8_t p[3];p[0]=a;p[1]=b;p[2]=0;DT_HandleSensorCtrl(p,(uint8_t)n);}
static uint16_t packet(uint8_t *frame,uint8_t code,uint16_t sequence,uint8_t mode){
 return Comm_EncodeFrame(frame,COMM_COMMAND_MAX,COMM_KIND_COMMAND,code,sequence,&mode,code==COMM_CMD_WORK_MODE_QUERY?0:1);
}
int main(void){
 CommSource ble={COMM_LINK_BLE,10},zb={COMM_LINK_ZIGBEE,20};
 uint8_t frame[COMM_COMMAND_MAX];uint16_t n;unsigned before;
 backup=0;DT_LoadWorkMode();assert(Custom_APP_GetWorkMode()==0&&Custom_APP_GetConfiguredWorkMode()==0);
 Comm_Init(NULL);Comm_RegisterWorkModeHandler(DT_CommWorkMode);
 Comm_RegisterTransport(COMM_LINK_BLE,transport);Comm_RegisterTransport(COMM_LINK_ZIGBEE,transport);
 Comm_SetSession(ble.link,ble.session);Comm_SetSession(zb.link,zb.session);
 Custom_App_Context.IsConnected=1;Custom_App_Context.Sensor_status_Notification_Status=1;Custom_App_Context.TxFlowOn=0x33;
 /* A new subscription defaults to the 11-byte common event. */
 DT_QueueWorkModeStatus();DT_ProcessWorkMode(0);
 const uint8_t event_golden[]={1,4,0x85,0,0,0,3,0,0,0,0};
 assert(!memcmp(notify_bytes,event_golden,sizeof(event_golden)));
 assert(read_count==1&&notify_count==1&&read_bytes[0]==1&&read_bytes[1]==COMM_KIND_EVENT&&read_bytes[2]==COMM_DATA_WORK_MODE);
 assert(notify_bytes[1]==COMM_KIND_EVENT&&notify_bytes[2]==COMM_DATA_WORK_MODE&&notify_bytes[8]==COMM_OK);
 assert(read_bytes[8]==0&&read_bytes[9]==0&&read_bytes[10]==0);
 before=task_requests;
 legacy(0xf0,0,1);assert(task_requests==before+1U);
 DT_WorkModeTimerReady=1;tick=20;fail_gatt=1;
 DT_ServiceWorkModeTask();assert(timer_starts==1&&notify_count==1);
 fail_gatt=0;DT_ScheduleWorkMode();tick=40;DT_ServiceWorkModeTask();
 assert(timer_starts==1&&notify_count==2);
 const uint8_t query_event[]={1,4,0x85,0,1,0,3,0,0,0,0};
 assert(!memcmp(notify_bytes,query_event,sizeof(query_event)));
 assert(!memcmp(read_bytes,event_golden,sizeof(event_golden)));
 /* SET executes only in Comm_Process; BUSY and replay do not repeat the write. */
 n=packet(frame,COMM_CMD_WORK_MODE_SET,0x1234,1);before=save_count;
 const uint8_t command_golden[]={1,1,0x25,0,0x34,0x12,1,0,1};
 assert(n==sizeof(command_golden)&&!memcmp(frame,command_golden,n));
 assert(Comm_Receive(&ble,frame,n)==COMM_OK&&save_count==before);
 fail_gatt=1;Comm_Process();assert(save_count==before+1&&backup==0x57423501U&&Comm_HasPending());
 assert(Custom_APP_GetWorkMode()==0&&Custom_APP_GetConfiguredWorkMode()==1&&!(PWR->CR1&PWR_CR1_DBP));
 Comm_Process();assert(save_count==before+1);fail_gatt=0;Comm_Process();
 const uint8_t response_golden[]={1,2,0x25,0,0x34,0x12,7,0,0,0,1,0,0,0,0};
 assert(!memcmp(notify_bytes,response_golden,sizeof(response_golden)));
 assert(notify_bytes[1]==COMM_KIND_RESPONSE&&notify_bytes[2]==COMM_CMD_WORK_MODE_SET&&Comm_ReadU16(notify_bytes+4)==0x1234);
 assert(notify_bytes[8]==COMM_OK&&notify_bytes[9]==0&&notify_bytes[10]==1&&last_source.link==COMM_LINK_BLE);
 DT_ProcessWorkMode(50);assert(read_bytes[2]==COMM_DATA_WORK_MODE&&read_bytes[10]==1);
 assert(Comm_Receive(&ble,frame,n)==COMM_OK);Comm_Process();assert(save_count==before+1);
 frame[8]=0;assert(Comm_Receive(&ble,frame,n)==COMM_BAD_VALUE);
 /* Query from Zigbee uses identical payload and returns over Zigbee. */
 n=packet(frame,COMM_CMD_WORK_MODE_QUERY,1,0);assert(Comm_Receive(&zb,frame,n)==COMM_OK);Comm_Process();
 assert(last_source.link==COMM_LINK_ZIGBEE&&notify_bytes[9]==0&&notify_bytes[10]==1);
 /* SET from Zigbee also saves without changing current BLE advertising policy. */
 n=packet(frame,COMM_CMD_WORK_MODE_SET,2,0);assert(Comm_Receive(&zb,frame,n)==COMM_OK);Comm_Process();
 assert(last_source.link==COMM_LINK_ZIGBEE&&notify_bytes[10]==0&&backup==0x57423500U);
 n=packet(frame,COMM_CMD_WORK_MODE_SET,3,1);fail_save=1;assert(Comm_Receive(&zb,frame,n)==COMM_OK);Comm_Process();
 assert(notify_bytes[8]==COMM_SAVE_FAILED&&notify_bytes[10]==0);fail_save=0;
 /* Invalid values and lengths respond without invoking the persistence handler. */
 before=save_count;n=packet(frame,COMM_CMD_WORK_MODE_SET,4,2);
 assert(Comm_Receive(&zb,frame,n)==COMM_OK);Comm_Process();
 assert(save_count==before&&notify_bytes[8]==COMM_BAD_VALUE&&notify_bytes[9]==COMM_WORK_MODE_UNKNOWN&&notify_bytes[10]==COMM_WORK_MODE_UNKNOWN);
 n=Comm_EncodeFrame(frame,sizeof(frame),COMM_KIND_COMMAND,COMM_CMD_WORK_MODE_SET,5,NULL,0);
 assert(Comm_Receive(&zb,frame,n)==COMM_OK);Comm_Process();assert(notify_bytes[8]==COMM_BAD_LENGTH&&save_count==before);
 n=packet(frame,COMM_CMD_WORK_MODE_SET,6,0);frame[2]=COMM_CMD_WORK_MODE_QUERY;
 assert(Comm_Receive(&zb,frame,n)==COMM_OK);Comm_Process();assert(notify_bytes[8]==COMM_BAD_LENGTH);
 /* Different queued settings must each preserve their own execution snapshot. */
 n=packet(frame,COMM_CMD_WORK_MODE_SET,7,1);assert(Comm_Receive(&zb,frame,n)==COMM_OK);
 n=packet(frame,COMM_CMD_WORK_MODE_SET,8,0);assert(Comm_Receive(&zb,frame,n)==COMM_OK);
 fail_gatt=1;Comm_Process();fail_gatt=0;Comm_Process();assert(notify_bytes[10]==1);Comm_Process();assert(notify_bytes[10]==0);
 /* Disconnect invalidates queued SET; the next connection cannot execute it. */
 before=save_count;n=packet(frame,COMM_CMD_WORK_MODE_SET,50,1);assert(Comm_Receive(&ble,frame,n)==COMM_OK);
 Comm_SetSession(COMM_LINK_BLE,11);Comm_Process();assert(save_count==before&&!Comm_HasPending());ble.session=11;
 /* Short legacy inputs also return the common mode event and snapshot. */
 const uint8_t legacy_set_read[]={1,4,0x85,0,0,0,3,0,0,0,1};
 legacy(0xf1,1,2);assert(backup==0x57423501U);DT_ProcessWorkMode(60);
 assert(notify_bytes[1]==COMM_KIND_EVENT&&notify_bytes[2]==COMM_DATA_WORK_MODE);
 assert(Comm_ReadU16(notify_bytes+6)==3&&notify_bytes[8]==COMM_OK&&notify_bytes[9]==0&&notify_bytes[10]==1);
 assert(!memcmp(read_bytes,legacy_set_read,sizeof(legacy_set_read)));
 fail_save=1;
 legacy(0xf1,0,2);DT_ProcessWorkMode(80);
 assert(notify_bytes[10]==1&&notify_bytes[8]==COMM_SAVE_FAILED);
 assert(read_bytes[10]==1&&read_bytes[8]==COMM_SAVE_FAILED);
 fail_save=0;
 DT_LoadWorkMode();assert(Custom_APP_GetWorkMode()==1&&Custom_APP_GetConfiguredWorkMode()==1);
 legacy(0xf1,2,2);assert(DT_WorkModeResult==COMM_BAD_VALUE);legacy(0xf1,0,1);assert(DT_WorkModeResult==COMM_BAD_VALUE);
 PWR->CR1=PWR_CR1_DBP;legacy(0xf1,0,2);assert(PWR->CR1&PWR_CR1_DBP);DT_LoadWorkMode();assert(Custom_APP_GetWorkMode()==0);
 legacy(4,1,2);assert(Custom_App_Context.SensorEnableMask==8);legacy(0xf0,0,1);assert(Custom_App_Context.SensorEnableMask==8);
 legacy(4,0,2);assert(Custom_App_Context.SensorEnableMask==0);legacy(1,0,1);assert(Custom_App_Context.SensorEnableMask==15);legacy(0,0,1);assert(Custom_App_Context.SensorEnableMask==0);
 fail_gatt=1;legacy(0xf0,0,1);for(unsigned i=0;i<20;i++)DT_ProcessWorkMode(100+i*20);
 assert(DT_WorkModeSendPending==0&&DT_WorkModeReadPending==0&&DT_WorkModeSendAttempts==20);
 assert(Custom_App_Context.TxFlowOn==0x33&&Custom_App_Context.SensorSeq==0);fail_gatt=0;legacy(0xf0,0,1);DT_ProcessWorkMode(600);assert(DT_WorkModeSendPending==0);
 Custom_App_Context.Sensor_status_Notification_Status=0;legacy(0xf0,0,1);DT_ProcessWorkMode(700);assert(DT_WorkModeReadPending==0&&DT_WorkModeSendAttempts==0);
 Custom_App_Context.IsConnected=0;DT_ProcessWorkMode(800);assert(DT_WorkModeSendAttempts==0);
 backup=0x57423502U;DT_LoadWorkMode();assert(Custom_APP_GetWorkMode()==0&&backup==0x57423502U);
 /* A legacy successful notification arms one delayed reset, even if the
    phone disconnects. Busy Flash postpones reset until the next task. */
 Custom_App_Context.IsConnected=Custom_App_Context.Sensor_status_Notification_Status=1;
 tick=1000;legacy(0xf1,1,2);fail_gatt=1;DT_ProcessWorkMode(tick);
 assert(!DT_ModeRestartScheduled);
 fail_gatt=0;tick=1020;DT_ProcessWorkMode(tick);
 assert(DT_ModeRestartScheduled&&DT_ModeRestartTick==2520);
 tick=1100;legacy(0xf0,0,1);DT_ProcessWorkMode(tick);
 assert(DT_ModeRestartTick==2520);
 Custom_App_Context.IsConnected=0;
 tick=2519;DT_ServiceWorkModeTask();assert(resets==0);
 tick=2520;flash_busy=FLASH_FLAG_BSY;DT_ServiceWorkModeTask();assert(resets==0);
 flash_busy=FLASH_FLAG_CFGBSY;DT_ServiceWorkModeTask();assert(resets==0);
 flash_busy=0;DT_ServiceWorkModeTask();assert(resets==1&&!DT_ModeRestartScheduled);
 tick=2600;DT_ServiceWorkModeTask();assert(resets==1);
 DT_LoadWorkMode();assert(Custom_APP_GetWorkMode()==1);
 /* Save failure and setting the current mode never arm reset. */
 Custom_App_Context.IsConnected=1;tick=3000;
 legacy(0xf1,1,2);DT_ProcessWorkMode(tick);assert(!DT_ModeRestartScheduled);
 fail_save=1;legacy(0xf1,0,2);DT_ProcessWorkMode(tick+20);
 assert(!DT_ModeRestartPending&&!DT_ModeRestartScheduled);fail_save=0;
 /* Real BLE transport must submit a successful SET response, and an
    expired connection or BUSY transport must not start the timer. */
 CommRequest request;memset(&request,0,sizeof(request));
 request.source=ble;request.code=COMM_CMD_WORK_MODE_SET;request.mode=0;
 uint8_t active,configured,payload[7]={0};
 assert(DT_CommWorkMode(&request,&active,&configured)==COMM_OK);
 assert(DT_ModeRestartPending&&!DT_ModeRestartScheduled);
 payload[1]=active;payload[2]=configured;
 uint8_t response_frame[15];
 n=Comm_EncodeFrame(response_frame,sizeof(response_frame),COMM_KIND_RESPONSE,COMM_CMD_WORK_MODE_SET,77,payload,7);
 assert(n==sizeof(response_frame));
 ble.session=10;assert(DT_CommBleSend(&ble,response_frame,n)==COMM_NOT_READY&&!DT_ModeRestartScheduled);
 ble.session=11;fail_gatt=1;assert(DT_CommBleSend(&ble,response_frame,n)==COMM_NOT_READY&&!DT_ModeRestartScheduled);
 fail_gatt=0;tick=0xffffff00U;
 assert(DT_CommBleSend(&ble,response_frame,n)==COMM_OK&&DT_ModeRestartScheduled);
 uint32_t deadline=DT_ModeRestartTick;
 tick=0x10U;assert(DT_CommBleSend(&ble,response_frame,n)==COMM_OK&&DT_ModeRestartTick==deadline);
 tick=deadline-1;DT_ProcessWorkMode(tick);assert(resets==1);
 tick=deadline;DT_ProcessWorkMode(tick);assert(resets==2);
 DT_LoadWorkMode();
 /* Zigbee mode changes keep their existing configuration workflow. */
 request.source=zb;request.mode=1;assert(DT_CommWorkMode(&request,&active,&configured)==COMM_OK);
 assert(!DT_ModeRestartPending&&!DT_ModeRestartScheduled);
 /* Production bonding task ignores stale handles and consumes each event. */
 BleApplicationContext.BleApplicationContext_legacy.connectionHandle=0x801;
 BondReportPending=1;BondSecurityHandle=0x801;
 DT_BLE_BondSecurityTask();assert(bond_reports==1&&security_requests==1);
 DT_BLE_BondSecurityTask();assert(security_requests==1);
 BondRebondHandle=0x801;DT_BLE_BondSecurityTask();assert(rebond_requests==1);
 BondSecurityHandle=BondRebondHandle=0x802;DT_BLE_BondSecurityTask();
 assert(security_requests==1&&rebond_requests==1);
 puts("PASS: legacy/unified modes, delayed reset, failed save/send, disconnect, Flash busy, tick wrap, replay, bonding and stale handles.");return 0;
}
