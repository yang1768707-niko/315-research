/* The runner extracts DT_CommExecute verbatim from custom_app.c. Only drivers,
 * clock and radios are mocked: this exercises the production control function. */
#include "comm_protocol.h"
#include "custom_app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define CFG_TASK_DT_SEND_ID 0
#define CFG_SCH_PRIO_1 0
typedef uint8_t tBleStatus;
#define BLE_STATUS_SUCCESS 0U
#define BLE_STATUS_BUSY 0x46U
#define BLE_STATUS_INSUFFICIENT_RESOURCES 0x64U
#define CUSTOM_STM_SENSOR_STATUS 1U
#define CUSTOM_STM_VIB_WAVE 2U
static struct {
  uint8_t IsConnected,Sensor_status_Notification_Status,Vib_wave_Notification_Status;
} Custom_App_Context={1,1,1};
static uint16_t fake_active,CommFactor=1,CommBleCapacity=244,CommFrameLength,CommStatusMask;
static uint32_t CommBleSession=17U;
static uint8_t ble_reply[16],ble_busy;
static uint8_t zigbee_reply[16];
static uint16_t ble_reply_length;
static unsigned ble_sent;
static CommResult final_result=COMM_OK;
static void DT_ConfirmModeRestart(uint8_t mode) {(void)mode;assert(0);}
static tBleStatus Custom_STM_App_Update_Char_Variable_Length(unsigned characteristic,uint8_t *frame,uint8_t length) {
  assert(characteristic==CUSTOM_STM_SENSOR_STATUS);
  if(ble_busy)return BLE_STATUS_BUSY;
  assert(length<=sizeof(ble_reply));memcpy(ble_reply,frame,length);
  ble_reply_length=length;ble_sent++;return BLE_STATUS_SUCCESS;
}
static CommResult zigbee_transport(const CommSource *source,const uint8_t *frame,uint16_t length) {
  assert(source->link==COMM_LINK_ZIGBEE&&length==16U&&Comm_ReadU16(frame+6)==8U);
  memcpy(zigbee_reply,frame,length);
  assert(Comm_ReadU32(frame+12)==7U);return COMM_OK;
}
static CommResult finalize(const CommRequest *request,CommResult result,uint32_t *revision) {
  (void)request;*revision=7U;
  return result==COMM_OK?final_result:result;
}
static uint16_t CommActiveFactor=1,CommPhase,CommTimedMask,CommStoredMask,CommStoredRequested,ZigbeeCaptureMask;
static uint32_t CommWindows[5],CommDeadlines[5],CommBatchId,CommRawIndex,CommRetryAt;
static uint8_t CommStreamMode,CommDraining,CommFinalPending,CommStoredCapture,CommBatchPending;
static uint8_t ZigbeeAudioWindowActive,ZigbeeVibrationWindowActive,ZigbeeSensorEnableMask;
static uint8_t ZigbeeVibrationStoreActive,ZigbeeDigitalMicStarted,ZigbeeAnalogMicStarted;
static CommLink CommStreamLink;
static unsigned starts,stops,batches,captures;
static uint8_t fail_hardware,radio_ready=1,scheduled_busy;
static uint16_t captured_vib,captured_dmic,captured_amic;
static uint8_t captured_mask;
static uint16_t DT_CommActiveMask(void) { return CommStoredCapture?CommStoredMask:fake_active; }
static void DT_CommQueueStatus(uint8_t status,uint16_t mask) { (void)status;(void)mask; }
static void DT_CommStop(uint16_t mask) {fake_active&=(uint16_t)~mask;stops++;}
static void DT_StartVibStream(void) {starts++;if(!fail_hardware)fake_active|=COMM_SENSOR_VIBRATION;}
static void DT_ApplySensorEnableMask(uint8_t mask) {
  starts++;if(!fail_hardware)fake_active=(fake_active&COMM_SENSOR_VIBRATION)|Comm_MaskFromBle(mask);
}
static uint8_t APP_ZIGBEE_IsMeasuring(void) {return scheduled_busy;}
static uint8_t APP_ZIGBEE_IsPendingAuthorization(void) {return 0;}
static uint8_t APP_ZIGBEE_IsTransmitting(void) {return 0;}
static uint8_t APP_ZIGBEE_ProtocolReady(void) {return radio_ready;}
static uint32_t APP_ZIGBEE_ProtocolNextBatch(void) {return ++batches;}
static void UTIL_SEQ_SetTask(uint32_t task,uint32_t priority) {(void)task;(void)priority;}
static uint32_t HAL_GetTick(void) {return 1000;}
static void DT_AbortZigbeeSnapshot(void) {ZigbeeAudioWindowActive=ZigbeeVibrationWindowActive=0;}
static void Vibrometer_Stop(void) {}
static void ZigbeeVibrationStore_Abort(void) {}
static void DigitalMicrophone_Stop(void) {}
static void AnalogMicrophone_Stop(void) {}
static void Thermometer_EnterLowPower(void) {}
static void Magnetometer_EnterLowPower(void) {}
uint8_t Custom_APP_ZigbeeBeginSnapshot(uint16_t rate,uint16_t vib,uint16_t dmic,
    uint16_t amic,uint8_t mask,uint32_t batch) {
  assert(rate==26667 && batch!=0);captures++;
  captured_vib=vib;captured_dmic=dmic;captured_amic=amic;captured_mask=mask;
  return !fail_hardware;
}
#include "comm_executor_under_test.inc"

int main(void) {
  CommRequest r;memset(&r,0,sizeof(r));uint16_t active=0;
  r.source.link=COMM_LINK_BLE;r.code=COMM_CMD_WINDOW;r.mask=COMM_SENSOR_VIBRATION;r.duration_ms=500;
  assert(DT_CommExecute(&r,&active)==COMM_OK && CommWindows[4]==500);
  r.code=COMM_CMD_SENSOR_ENABLE;r.state=1;
  assert(DT_CommExecute(&r,&active)==COMM_OK && active==16 && CommActiveFactor==1);
  assert(CommDeadlines[4]==1500 && CommTimedMask==16);
  unsigned before=starts;
  assert(DT_CommExecute(&r,&active)==COMM_OK && starts==before && CommDeadlines[4]==1500);
  r.code=COMM_CMD_DECIMATION;r.factor=4;
  assert(DT_CommExecute(&r,&active)==COMM_BUSY && CommFactor==1);
  r.code=COMM_CMD_WINDOW;r.duration_ms=1000;
  assert(DT_CommExecute(&r,&active)==COMM_BUSY && CommWindows[4]==500);
  r.code=COMM_CMD_SENSOR_ENABLE;r.source.link=COMM_LINK_ZIGBEE;r.state=0;
  assert(DT_CommExecute(&r,&active)==COMM_OK && active==0); /* cross-link stop */
  r.code=COMM_CMD_DECIMATION;
  assert(DT_CommExecute(&r,&active)==COMM_OK && CommFactor==4);
  CommFactor=1;CommWindows[4]=0;CommTimedMask=0;
  r.code=COMM_CMD_SENSOR_ENABLE;r.mask=COMM_SENSOR_TEMPERATURE|COMM_SENSOR_MAGNETIC;r.state=1;
  assert(DT_CommExecute(&r,&active)==COMM_OK && active==3);
  r.mask=COMM_SENSOR_TEMPERATURE;r.state=0;
  assert(DT_CommExecute(&r,&active)==COMM_OK && active==2);
  r.mask=COMM_SENSOR_MAGNETIC;
  assert(DT_CommExecute(&r,&active)==COMM_OK && active==0);
  r.source.link=COMM_LINK_BLE;r.mask=COMM_SENSOR_VIBRATION;r.state=1;CommBleCapacity=20;
  assert(DT_CommExecute(&r,&active)==COMM_NOT_READY && active==0);
  CommBleCapacity=244;scheduled_busy=1;
  assert(DT_CommExecute(&r,&active)==COMM_BUSY && active==0);scheduled_busy=0;
  fail_hardware=1;
  assert(DT_CommExecute(&r,&active)==COMM_HARDWARE_ERROR && active==0);fail_hardware=0;
  CommWindows[4]=600;r.source.link=COMM_LINK_ZIGBEE;
  assert(DT_CommExecute(&r,&active)==COMM_BAD_VALUE && captures==0);
  CommWindows[4]=500;r.mask=COMM_SENSOR_VIBRATION|COMM_SENSOR_DIGITAL_MIC;
  assert(DT_CommExecute(&r,&active)==COMM_OK && active==20 && captures==1);
  assert(captured_vib==500 && captured_dmic==500 && captured_amic==500 && captured_mask==3);
  assert(CommStoredCapture && CommStoredRequested==20);
  r.mask=COMM_SENSOR_DIGITAL_MIC;r.state=0;
  assert(DT_CommExecute(&r,&active)==COMM_OK && active==16 && CommStoredRequested==16);
  r.mask=COMM_SENSOR_VIBRATION;
  assert(DT_CommExecute(&r,&active)==COMM_OK && active==0 && CommStoredRequested==0);
  /* Exercise factor commands through the production executor, BLE transport
     and common finalization. Both links must retain the revision field. */
  CommSource ble={COMM_LINK_BLE,17U},zigbee={COMM_LINK_ZIGBEE,18U};
  uint8_t command[COMM_COMMAND_MAX],factor_bytes[2];
  uint16_t length;
  fake_active=0;CommStoredCapture=0;CommDraining=0;CommTimedMask=0;
  Comm_Init(DT_CommExecute);Comm_SetSession(ble.link,ble.session);Comm_SetSession(zigbee.link,zigbee.session);
  Comm_RegisterTransport(COMM_LINK_BLE,DT_CommBleSend);
  Comm_RegisterTransport(COMM_LINK_ZIGBEE,zigbee_transport);
  Comm_RegisterCommandFinalizer(finalize,NULL);
  FILE *responses=fopen("Debug/comm-tests/ble-sampling-responses.bin","wb");assert(responses);
  for(unsigned factor=1;factor<=4;factor*=2) {
    Comm_WriteU16(factor_bytes,(uint16_t)factor);
    length=Comm_EncodeFrame(command,sizeof(command),COMM_KIND_COMMAND,COMM_CMD_DECIMATION,
                           (uint16_t)(0x40U+factor),factor_bytes,2U);
    assert(Comm_Receive(&ble,command,length)==COMM_OK);ble_busy=1;
    unsigned sent_before=ble_sent;Comm_Process();
    assert(CommFactor==factor&&Comm_HasPending()&&ble_sent==sent_before);
    ble_busy=0;Comm_Process();
    assert(!Comm_HasPending()&&ble_reply_length==16U&&Comm_ReadU16(ble_reply+6)==8U);
    assert(Comm_ReadU32(ble_reply+12)==7U);
    assert(ble_reply[2]==COMM_CMD_DECIMATION&&ble_reply[8]==COMM_OK);
    assert(Comm_ReadU16(ble_reply+4)==0x40U+factor);
    assert(fwrite(ble_reply,1,ble_reply_length,responses)==ble_reply_length);
    /* The configured factor is latched by the subsequent production START. */
    r.source=ble;r.code=COMM_CMD_SENSOR_ENABLE;r.mask=COMM_SENSOR_VIBRATION;r.state=1;
    CommWindows[4]=0;assert(DT_CommExecute(&r,&active)==COMM_OK&&CommActiveFactor==factor);
    r.state=0;assert(DT_CommExecute(&r,&active)==COMM_OK&&active==0);
  }
  /* Busy and save errors retain the full final-response layout. */
  fake_active=COMM_SENSOR_VIBRATION;factor_bytes[0]=2;
  length=Comm_EncodeFrame(command,sizeof(command),COMM_KIND_COMMAND,COMM_CMD_DECIMATION,80,factor_bytes,2U);
  assert(Comm_Receive(&ble,command,length)==COMM_OK);Comm_Process();
  assert(ble_reply_length==16U&&ble_reply[8]==COMM_BUSY&&CommFactor==4);
  assert(fwrite(ble_reply,1,ble_reply_length,responses)==ble_reply_length);
  fake_active=0;final_result=COMM_SAVE_FAILED;
  length=Comm_EncodeFrame(command,sizeof(command),COMM_KIND_COMMAND,COMM_CMD_DECIMATION,81,factor_bytes,2U);
  assert(Comm_Receive(&ble,command,length)==COMM_OK);Comm_Process();
  assert(ble_reply_length==16U&&ble_reply[8]==COMM_SAVE_FAILED);
  assert(fwrite(ble_reply,1,ble_reply_length,responses)==ble_reply_length);
  assert(fclose(responses)==0);final_result=COMM_OK;
  length=Comm_EncodeFrame(command,sizeof(command),COMM_KIND_COMMAND,COMM_CMD_DECIMATION,82,factor_bytes,2U);
  assert(Comm_Receive(&zigbee,command,length)==COMM_OK);Comm_Process();
  /* All acquisition responses must be byte-identical across transports
     when result, active mask, request sequence and revision are equal. */
  for(unsigned code=COMM_CMD_SENSOR_ENABLE;code<=COMM_CMD_STATUS;code++) {
    uint8_t payload[6]={0x10U,0U,0U,0U,0U,0U};
    uint16_t payload_length=3U;
    if(code==COMM_CMD_WINDOW) {
      Comm_WriteU32(payload+2,500U);payload_length=6U;
    } else if(code==COMM_CMD_DECIMATION) {
      Comm_WriteU16(payload,2U);payload_length=2U;
    } else if(code==COMM_CMD_STATUS) payload_length=0U;
    length=Comm_EncodeFrame(command,sizeof(command),COMM_KIND_COMMAND,(uint8_t)code,
                           (uint16_t)(100U+code),payload,payload_length);
    assert(Comm_Receive(&ble,command,length)==COMM_OK);Comm_Process();
    assert(Comm_Receive(&zigbee,command,length)==COMM_OK);Comm_Process();
    assert(ble_reply_length==16U&&!memcmp(ble_reply,zigbee_reply,16U));
  }
  puts("BLE sampling: factors 1/2/4, acquisition start, common 16-byte replies, BUSY, save error, BLE/Zigbee revisions passed");
  puts("comm acquisition: production executor passed shared start/stop, window, idempotence, MTU, BUSY, failure and Flash routing tests");
  return 0;
}
