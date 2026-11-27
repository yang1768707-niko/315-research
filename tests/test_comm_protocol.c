#include "comm_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned calls, sent;
static uint8_t busy, last_frame[COMM_FRAME_MAX];
static CommSource last_source;
static uint16_t active;
static CommResult transport(const CommSource *s,const uint8_t *p,uint16_t n) {
  if(busy)return COMM_BUSY;
  assert(n<=sizeof(last_frame));memcpy(last_frame,p,n);last_source=*s;sent++;return COMM_OK;
}
static CommResult executor(const CommRequest *r,uint16_t *mask) {
  calls++;
  if(r->code==COMM_CMD_SENSOR_ENABLE) {
    if(r->state)active|=r->mask;else active&=(uint16_t)~r->mask;
  }
  *mask=active;return COMM_OK;
}
static uint16_t command(uint8_t *out,uint8_t code,uint16_t seq,const uint8_t *p,uint16_t n) {
  return Comm_EncodeFrame(out,COMM_COMMAND_MAX,COMM_KIND_COMMAND,code,seq,p,n);
}
int main(void) {
  uint8_t frame[COMM_FRAME_MAX],payload[64],out[COMM_FRAME_MAX];
  CommSource ble={COMM_LINK_BLE,10},zb={COMM_LINK_ZIGBEE,20};CommRequest r;
  Comm_Init(executor);Comm_RegisterTransport(COMM_LINK_BLE,transport);
  Comm_RegisterTransport(COMM_LINK_ZIGBEE,transport);
  Comm_SetSession(ble.link,ble.session);Comm_SetSession(zb.link,zb.session);
  payload[0]=0x10;payload[1]=0;payload[2]=1;
  uint16_t n=command(frame,COMM_CMD_SENSOR_ENABLE,0x1234,payload,3);
  const uint8_t expected[]={1,1,0x20,0,0x34,0x12,3,0,0x10,0,1};
  assert(n==sizeof(expected) && !memcmp(frame,expected,n));
  for(unsigned i=0;i<n;i++)assert(Comm_DecodeCommand(&ble,frame,(uint16_t)i,&r)==COMM_BAD_LENGTH);
  assert(Comm_DecodeCommand(&ble,frame,n,&r)==COMM_OK && r.mask==16 && r.state==1 && r.sequence==0x1234);
  assert(Comm_DecodeCommand(&zb,frame,n,&r)==COMM_OK && r.source.link==COMM_LINK_ZIGBEE);
  frame[9]=0x80;assert(Comm_DecodeCommand(&ble,frame,n,&r)==COMM_BAD_VALUE);frame[9]=0;
  frame[10]=2;assert(Comm_DecodeCommand(&ble,frame,n,&r)==COMM_BAD_VALUE);frame[10]=1;
  assert(Comm_Receive(&ble,frame,n)==COMM_OK);
  memset(frame,0,sizeof(frame)); /* RX buffer lifetime ends after Receive */
  busy=1;Comm_Process();assert(calls==1 && active==16 && sent==0);
  payload[2]=0;n=command(frame,COMM_CMD_SENSOR_ENABLE,0x1235,payload,3);
  assert(Comm_Receive(&ble,frame,n)==COMM_OK);
  Comm_Process();assert(calls==2 && active==0); /* STOP not blocked by TX congestion */
  busy=0;Comm_Process();Comm_Process();assert(sent==2 && !Comm_HasPending());
  assert(last_frame[1]==COMM_KIND_RESPONSE && last_frame[8]==COMM_OK && last_frame[9]==COMM_STATUS_STOPPED);
  assert(Comm_Receive(&ble,frame,n)==COMM_OK);Comm_Process();assert(calls==2); /* replay */
  frame[10]=1;assert(Comm_Receive(&ble,frame,n)==COMM_BAD_VALUE); /* conflicting ID */

  n=command(frame,COMM_CMD_STATUS,4,NULL,0);
  assert(Comm_Receive(&zb,frame,n)==COMM_OK);Comm_Process();
  assert(last_source.link==COMM_LINK_ZIGBEE && last_source.session==20);
  n=command(frame,COMM_CMD_SENSOR_ENABLE,5,payload,3);
  assert(Comm_Receive(&ble,frame,n)==COMM_OK);
  Comm_SetSession(COMM_LINK_BLE,11);unsigned before=calls;Comm_Process();assert(calls==before);
  assert(Comm_Receive(&ble,frame,n)==COMM_NOT_READY);ble.session=11;

  payload[0]=0;payload[1]=0;
  n=command(frame,COMM_CMD_DECIMATION,6,payload,2);
  assert(Comm_DecodeCommand(&ble,frame,n,&r)==COMM_BAD_VALUE);
  assert(Comm_Receive(&ble,frame,n)==COMM_OK);Comm_Process();
  assert(calls==before && last_frame[8]==COMM_BAD_VALUE);
  payload[0]=16;n=command(frame,COMM_CMD_DECIMATION,7,payload,2);
  assert(Comm_DecodeCommand(&ble,frame,n,&r)==COMM_OK && r.factor==16);
  payload[0]=17;command(frame,COMM_CMD_DECIMATION,7,payload,2);
  assert(Comm_DecodeCommand(&ble,frame,n,&r)==COMM_BAD_VALUE);
  payload[0]=31;payload[1]=0;Comm_WriteU32(payload+2,0);
  n=command(frame,COMM_CMD_WINDOW,8,payload,6);
  assert(Comm_DecodeCommand(&ble,frame,n,&r)==COMM_OK && r.duration_ms==0);
  Comm_WriteU32(frame+10,0x80000000UL);
  assert(Comm_DecodeCommand(&ble,frame,n,&r)==COMM_BAD_VALUE);
  for(unsigned i=0;i<4;i++) {
    n=command(frame,COMM_CMD_STATUS,(uint16_t)(20+i),NULL,0);
    assert(Comm_Receive(&ble,frame,n)==COMM_OK);
  }
  n=command(frame,COMM_CMD_STATUS,25,NULL,0);assert(Comm_Receive(&ble,frame,n)==COMM_BUSY);
  for(unsigned i=0;i<4;i++)Comm_Process();assert(!Comm_HasPending());

  CommScalars v={0};v.valid_mask=15;v.temperature_centi_c=-1234;
  v.magnetic_0p1_ut[0]=-32768;v.magnetic_0p1_ut[1]=32767;v.magnetic_0p1_ut[2]=-1;
  v.digital_centi_db=7000;v.analog_centi_db=8100;
  memset(out,0xA5,sizeof(out));assert(!Comm_EncodeScalars(out,23,0x12345678,&v));assert(out[0]==0xA5);
  assert(Comm_EncodeScalars(out,sizeof(out),0x12345678,&v)==24);
  assert(Comm_ReadU32(out)==0x12345678 && out[4]==1 && out[5]==2 && (int16_t)Comm_ReadU16(out+6)==-1234);
  assert(out[8]==2 && out[9]==6 && (int16_t)Comm_ReadU16(out+10)==-32768);
  assert(out[16]==3 && Comm_ReadU16(out+18)==7000 && out[20]==4 && Comm_ReadU16(out+22)==8100);
  assert(Comm_SendScalars(COMM_LINK_ZIGBEE,1,&v)==COMM_OK && last_frame[2]==COMM_DATA_SCALARS);
  int16_t xyz[2][3]={{-1,2,-3},{32767,-32768,0}};
  assert(Comm_EncodeWave(out,21,9,100,xyz,2)==0);
  assert(Comm_EncodeWave(out,sizeof(out),9,100,xyz,2)==22);
  assert(Comm_ReadU32(out+4)==100 && Comm_ReadU16(out+8)==2 && (int16_t)Comm_ReadU16(out+18)==-32768);
  int32_t features[3][5]={{-1000,2,3,4,5},{6,7,8,9,10},{11,12,13,14,2147483647}};
  assert(Comm_EncodeFeatures(out,sizeof(out),42,features)==64);
  assert((int32_t)Comm_ReadU32(out+4)==-1000 && Comm_ReadU32(out+60)==2147483647);
  assert(Comm_SendFeatures(COMM_LINK_BLE,42,features)==COMM_OK && Comm_ReadU16(last_frame+6)==64);
  assert(Comm_SendBatch(COMM_LINK_BLE,42,1)==COMM_OK && Comm_ReadU16(last_frame+16)==1);
  for(unsigned i=0;i<16;i++)assert(Comm_MaskToBle(Comm_MaskFromBle((uint8_t)i))==i);
  assert(Comm_MaskFromZigbee(7)==31);
  puts("comm_protocol: all codec, validation, replay, queue, backpressure and session tests passed");
  return 0;
}
