#ifndef SENSOR_ZCL_IMPL_H
#define SENSOR_ZCL_IMPL_H
/* Included once by app_zigbee.c, after its state and forward declarations. */
static enum ZclStatusCodeT ZbApp_ControlAttribute(struct ZbZclClusterT *cluster,
                                                 struct ZbZclAttrCbInfoT *info)
{
  (void)cluster;
  uint32_t value=0; unsigned size=1;
  if(info->type!=ZCL_ATTR_CB_TYPE_READ) return ZCL_STATUS_READ_ONLY;
  switch(info->info->attributeId) {
    case 0: value=HY_PROTOCOL_VERSION; size=2; break;
    case 1: value=HY_CAPABILITIES; size=2; break;
    case 2: value=ZbApp.monitor_enabled; break;
    case 3: value=ZbApp.monitor_config.auto_interval_sec; size=4; break;
    case 4: value=ZbApp.monitor_config.sensor_enable_mask; break;
    case 5: value=ZbApp.monitor_state; break;
    case 6: value=ZbApp.config_valid; break;
    case 7: value=ZbApp.monitor_config.vibration_rate_hz; size=2; break;
    default: return ZCL_STATUS_UNSUPP_ATTRIBUTE;
  }
  if(info->zcl_len<size) return ZCL_STATUS_INVALID_FIELD;
  for(unsigned i=0;i<size;i++) info->zcl_data[i]=(uint8_t)(value>>(8U*i));
  return ZCL_STATUS_SUCCESS;
}
#define HY_ATTR(id,type) { .attributeId=id, .dataType=type, \
  .flags=ZCL_ATTR_FLAG_CB_READ, .callback=ZbApp_ControlAttribute }
static const struct ZbZclAttrT hy_control_attrs[]={
  HY_ATTR(0,ZCL_DATATYPE_UNSIGNED_16BIT), HY_ATTR(1,ZCL_DATATYPE_BITMAP_16BIT),
  HY_ATTR(2,ZCL_DATATYPE_BOOLEAN), HY_ATTR(3,ZCL_DATATYPE_UNSIGNED_32BIT),
  HY_ATTR(4,ZCL_DATATYPE_BITMAP_8BIT), HY_ATTR(5,ZCL_DATATYPE_ENUMERATION_8BIT),
  HY_ATTR(6,ZCL_DATATYPE_BOOLEAN), HY_ATTR(7,ZCL_DATATYPE_UNSIGNED_16BIT)
};
#undef HY_ATTR
static const struct ZbZclAttrT hy_data_attrs[]={
  {.attributeId=0,.dataType=ZCL_DATATYPE_UNSIGNED_32BIT}, /* batch */
  {.attributeId=1,.dataType=ZCL_DATATYPE_BITMAP_8BIT}, /* valid mask */
  {.attributeId=2,.dataType=ZCL_DATATYPE_SIGNED_16BIT}, /* magnetic 0.1 uT */
  {.attributeId=3,.dataType=ZCL_DATATYPE_SIGNED_16BIT},
  {.attributeId=4,.dataType=ZCL_DATATYPE_SIGNED_16BIT},
  {.attributeId=5,.dataType=ZCL_DATATYPE_UNSIGNED_8BIT}, /* digital dB SPL */
  {.attributeId=6,.dataType=ZCL_DATATYPE_UNSIGNED_8BIT}, /* analog dB SPL */
  {.attributeId=7,.dataType=ZCL_DATATYPE_UNSIGNED_32BIT} /* vibration points */
};
static void ZbApp_RegisterPrivateClusters(void)
{
  ZbApp.identify_server=ZbZclIdentifyServerAlloc(ZbApp.zb,ZB_APP_ENDPOINT,NULL);
  if(ZbApp.identify_server) ZbZclClusterEndpointRegister(ZbApp.identify_server);
  struct ZbZclClusterT **clusters[]={&ZbApp.control_server,&ZbApp.data_server};
  const unsigned ids[]={HY_CONTROL_CLUSTER,HY_DATA_CLUSTER};
  for(unsigned i=0;i<2;i++) {
    *clusters[i]=ZbZclClusterAlloc(ZbApp.zb,sizeof(struct ZbZclClusterT),
        (enum ZbZclClusterIdT)ids[i],ZB_APP_ENDPOINT,ZCL_DIRECTION_TO_SERVER);
    if(!*clusters[i]) continue;
    (*clusters[i])->command=ZbApp_CommandHandler;
    enum ZclStatusCodeT status=ZbZclAttrAppendList(*clusters[i],
      i?hy_data_attrs:hy_control_attrs,i?ZCL_ATTR_LIST_LEN(hy_data_attrs):ZCL_ATTR_LIST_LEN(hy_control_attrs));
    if(status!=ZCL_STATUS_SUCCESS || ZbZclClusterAttach(*clusters[i])!=ZCL_STATUS_SUCCESS) {
      ZbZclClusterFree(*clusters[i]); *clusters[i]=NULL; continue;
    }
    ZbZclClusterEndpointRegister(*clusters[i]);
  }
}
static void ZbApp_UpdateStandardTemperature(void)
{
  /* Called only from sequencer work while Zigbee owns radio and sensors.
   * Never reads the sensor from a ZCL/CPU2 or timer callback. */
  if(!ZbApp.temperature_server || ZbApp.audio_waiting || ZbApp.radio_exclusive) return;
  uint8_t bytes[2]; int16_t value=(int16_t)ZCL_TEMP_MEAS_UNKNOWN;
  if(Thermometer_AppendPayload(bytes,sizeof(bytes))==2U)
    value=(int16_t)((uint16_t)bytes[0]|((uint16_t)bytes[1]<<8));
  (void)ZbZclAttrIntegerWrite(ZbApp.temperature_server,ZCL_TEMP_MEAS_ATTR_MEAS_VAL,value);
}
static void ZbApp_UpdateClusterSnapshot(const Custom_APP_ZigbeeSnapshot_t *s,uint32_t batch)
{
  if(ZbApp.temperature_server)
    (void)ZbZclAttrIntegerWrite(ZbApp.temperature_server,ZCL_TEMP_MEAS_ATTR_MEAS_VAL,
      s->valid_mask&1U?s->temperature_centi_c:(int16_t)ZCL_TEMP_MEAS_UNKNOWN);
  if(!ZbApp.data_server) return;
  long long values[]={batch,s->valid_mask,s->magnetic_0p1_ut[0],s->magnetic_0p1_ut[1],
    s->magnetic_0p1_ut[2],s->digital_mic_db_spl,s->analog_mic_db_spl,s->vibration_sample_count};
  for(unsigned i=0;i<8;i++) (void)ZbZclAttrIntegerWrite(ZbApp.data_server,i,values[i]);
}
static volatile uint8_t hy_parent_response;
static void ZbApp_ParentDescriptor(struct ZbZdoNodeDescRspT *rsp,void *arg)
{
  hy_zdo_inflight=0U;
  if((uint32_t)(uintptr_t)arg!=command_confirm_generation || !ZbApp.parent_sync_pending) return;
  hy_parent_response=(rsp && rsp->status==ZB_STATUS_SUCCESS && rsp->nwkAddr==0U)?1U:2U;
  UTIL_SEQ_SetTask(1U<<CFG_TASK_ZIGBEE_REPORT,CFG_SCH_PRIO_0);
}
#endif
