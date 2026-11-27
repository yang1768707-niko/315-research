/* 统一协议实现：负责字节编解码、命令校验/排队/响应和链路分发。
 * 本文件不直接控制传感器；实际采集由 custom_app.c 注册的 DT_CommExecute 执行。
 * 按现有协作式调度使用，内部没有中断锁，不能作为并发安全队列调用。
 */
#include "comm_protocol.h"
#include <string.h>

/* 命令和待发响应共用 4 个槽位，不使用动态内存。 */
#define COMM_QUEUE_DEPTH 4U

/* wire/length 保存原帧以校验重发；executed 防止重复操作硬件；
 * mask/result 保存执行后的活动掩码及结果，供响应使用。 */
typedef struct {
  CommRequest request;
  uint8_t wire[COMM_COMMAND_MAX];
  uint8_t length, executed;
  uint16_t mask;
  CommResult result;
  uint8_t mode_active, mode_configured; /* 保存执行时快照，重发不能变成后续命令的模式 */
  uint32_t revision; /* 最终响应对应的命令版本，重复请求复用此快照 */
} CommQueued;

/* 各链路独立保存发送回调、连接会话和主动上传序号。 */
static CommTransport transports[COMM_LINK_COUNT];
static uint32_t sessions[COMM_LINK_COUNT];
static uint16_t sequences[COMM_LINK_COUNT];
static CommExecutor execute;
static CommWorkModeHandler work_mode_handler;
static CommExecutedObserver executed_observer;
static CommCommandFinalizer command_finalizer;
static CommResponseSubmitted response_submitted;

void Comm_RegisterCommandFinalizer(CommCommandFinalizer finalizer,CommResponseSubmitted submitted) {
  command_finalizer=finalizer;response_submitted=submitted;
}

void Comm_RegisterExecutedObserver(CommExecutedObserver observer) {
  executed_observer=observer;
}

/* 环形队列及每链路最近一次执行记录：去重范围有限，并非完整历史。 */
static CommQueued queue[COMM_QUEUE_DEPTH], last[COMM_LINK_COUNT];
static uint8_t head, count, last_valid[COMM_LINK_COUNT];

/* 读取小端 u16；调用方保证输入至少有 2 字节。 */
uint16_t Comm_ReadU16(const uint8_t *p) { return (uint16_t)(p[0]|((uint16_t)p[1]<<8)); }

/* 读取小端 u32；调用方保证输入至少有 4 字节。 */
uint32_t Comm_ReadU32(const uint8_t *p) {
  return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}

/* 将 u16 写成 2 字节小端格式，不依赖内存对齐。 */
void Comm_WriteU16(uint8_t *p,uint16_t n) { p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8); }

/* 将 u32 写成 4 字节小端格式，调用方保证输出容量。 */
void Comm_WriteU32(uint8_t *p,uint32_t n) {
  for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(n>>(8*i));
}

/* 绑定公共命令执行器，清空队列、重复记录及发送序号。
 * 保留已注册的传输回调和会话号，兼容 BLE/Zigbee 各自的初始化顺序。 */
void Comm_Init(CommExecutor executor) {
  execute=executor;head=count=0;
  memset(last_valid,0,sizeof(last_valid));memset(sequences,0,sizeof(sequences));
}

/* 注册链路发送回调：BLE 对接 GATT 通知，Zigbee 对接原发送队列。 */
void Comm_RegisterTransport(CommLink link,CommTransport transport) {
  if((unsigned)link<COMM_LINK_COUNT)transports[link]=transport;
}

/* 绑定应用层模式保存函数；备份寄存器访问不进入无线接收回调。 */
void Comm_RegisterWorkModeHandler(CommWorkModeHandler handler) {
  work_mode_handler=handler;
}

/* 更新连接会话并清除该链路重复记录；排队的旧会话命令在处理时丢弃。 */
void Comm_SetSession(CommLink link,uint32_t session) {
  if((unsigned)link<COMM_LINK_COUNT && sessions[link]!=session) {
    sessions[link]=session;last_valid[link]=0;
  }
}

/* 拼装 8 字节公共帧头和载荷，flags 固定为 0；成功返回完整帧长度，失败返回 0。
 * 支持载荷与输出缓冲区重叠；序号由调用方指定，可用于保留待重发帧。 */
uint16_t Comm_EncodeFrame(uint8_t *out,uint16_t capacity,uint8_t kind,
                         uint8_t code,uint16_t sequence,const uint8_t *payload,uint16_t length) {
  if(!out || (length && !payload) || (uint32_t)length+COMM_HEADER_SIZE>capacity)return 0;

  /* memmove 支持载荷原本就位于 out+8 的原地编码。 */
  if(length)memmove(out+COMM_HEADER_SIZE,payload,length);
  out[0]=COMM_VERSION;out[1]=kind;out[2]=code;out[3]=0;
  Comm_WriteU16(out+4,sequence);Comm_WriteU16(out+6,length);
  return (uint16_t)(length+COMM_HEADER_SIZE);
}

/* 只解析和校验命令，不操作硬件；检查帧头、长度、掩码、状态、窗口和抽取倍数。
 * 返回 COMM_OK 才表示参数有效，输出 request 为内部对象，不能直接发送。 */
CommResult Comm_DecodeCommand(const CommSource *source,const uint8_t *p,uint16_t n,CommRequest *r) {
  if(!source || !p || !r || (unsigned)source->link>=COMM_LINK_COUNT)return COMM_BAD_VALUE;
  if(n<COMM_HEADER_SIZE || n>COMM_COMMAND_MAX || Comm_ReadU16(p+6)!=n-COMM_HEADER_SIZE)return COMM_BAD_LENGTH;
  if(p[0]!=COMM_VERSION || p[1]!=COMM_KIND_COMMAND || p[3])return COMM_BAD_VALUE;
  memset(r,0,sizeof(*r));r->source=*source;r->sequence=Comm_ReadU16(p+4);r->code=p[2];
  p+=COMM_HEADER_SIZE;n-=COMM_HEADER_SIZE;
  switch(r->code) {
    case COMM_CMD_SENSOR_ENABLE:
      if(n!=3)return COMM_BAD_LENGTH;
      r->mask=Comm_ReadU16(p);r->state=p[2];
      if(r->state>COMM_START)return COMM_BAD_VALUE;
      break;
    case COMM_CMD_WINDOW:
      if(n!=6)return COMM_BAD_LENGTH;
      r->mask=Comm_ReadU16(p);r->duration_ms=Comm_ReadU32(p+2);
      if(r->duration_ms>COMM_MAX_WINDOW_MS)return COMM_BAD_VALUE;
      break;
    case COMM_CMD_DECIMATION:
      if(n!=2)return COMM_BAD_LENGTH;
      r->factor=Comm_ReadU16(p);
      return r->factor && r->factor<=COMM_MAX_DECIMATION ? COMM_OK:COMM_BAD_VALUE;
    case COMM_CMD_STATUS:return n?COMM_BAD_LENGTH:COMM_OK;
    case COMM_CMD_WORK_MODE_QUERY:return n?COMM_BAD_LENGTH:COMM_OK;
    case COMM_CMD_WORK_MODE_SET:
      if(n!=1)return COMM_BAD_LENGTH;
      r->mode=p[0];
      return r->mode<=COMM_WORK_MODE_INSPECTION?COMM_OK:COMM_BAD_VALUE;
    default:return COMM_UNSUPPORTED;
  }
  return r->mask && !(r->mask & ~COMM_SENSOR_ALL)?COMM_OK:COMM_BAD_VALUE;
}

/* 公共接收入口：将命令复制入队，返回后原接收缓冲区可复用。
 * 同会话同序号同内容重发复用结果，同序号不同内容拒绝，队满返回 COMM_BUSY。
 * 完整帧头下的参数错误排队响应，残缺帧直接拒绝；实际执行由 Comm_Process 完成。 */
CommResult Comm_Receive(const CommSource *source,const uint8_t *p,uint16_t n) {
  CommRequest r;CommResult result=Comm_DecodeCommand(source,p,n,&r);
  if(result!=COMM_OK) {

    if(!source || (unsigned)source->link>=COMM_LINK_COUNT || !p || n<COMM_HEADER_SIZE ||
       n>COMM_COMMAND_MAX || p[0]!=COMM_VERSION || p[1]!=COMM_KIND_COMMAND || p[3] ||
       Comm_ReadU16(p+6)!=n-COMM_HEADER_SIZE)return result;
  }
  if(source->session!=sessions[source->link])return COMM_NOT_READY;
  for(unsigned i=0;i<count;i++) {
    CommQueued *q=&queue[(head+i)%COMM_QUEUE_DEPTH];
    if(q->request.source.link==source->link && q->request.source.session==source->session && q->request.sequence==r.sequence)
      return q->length==n && !memcmp(q->wire,p,n)?COMM_OK:COMM_BAD_VALUE;
  }
  if(count==COMM_QUEUE_DEPTH)return COMM_BUSY;
  CommQueued *q=&queue[(head+count)%COMM_QUEUE_DEPTH];
  memset(q,0,sizeof(*q));q->request=r;q->length=(uint8_t)n;memcpy(q->wire,p,n);
  q->mode_active=q->mode_configured=COMM_WORK_MODE_UNKNOWN;
  q->revision=COMM_REVISION_UNKNOWN;
  if(result!=COMM_OK) {q->executed=1;q->result=result;}
  /* 最近一次已执行命令重发时复用结果，不能再次启动或停止硬件。 */
  if(last_valid[source->link] && last[source->link].request.sequence==r.sequence) {
    CommQueued *old=&last[source->link];
    if(old->length!=n || memcmp(old->wire,p,n))return COMM_BAD_VALUE;
    q->executed=1;q->result=old->result;q->mask=old->mask;
    q->mode_active=old->mode_active;q->mode_configured=old->mode_configured;
    q->revision=old->revision;
  }
  count++;return COMM_OK;
}

/* 返回是否还有待执行命令或待发送响应，供任务调度使用。 */
uint8_t Comm_HasPending(void) { return count!=0; }

/* 在主循环/任务上下文执行命令并发送响应，不在中断中操作硬件。
 * 先执行有效会话的待处理命令，再发送队首响应，避免响应拥塞阻塞 STOP。
 * 发送返回 BUSY 保留队首且不重复执行，其他发送结果使队首出队。 */
void Comm_Process(void) {
  if(!count)return;

  for(unsigned i=0;i<count;i++) {
    CommQueued *pending=&queue[(head+i)%COMM_QUEUE_DEPTH];
    CommSource *from=&pending->request.source;
    if(!pending->executed && from->session==sessions[from->link]) {
      if(pending->request.code==COMM_CMD_WORK_MODE_QUERY || pending->request.code==COMM_CMD_WORK_MODE_SET)
        pending->result=work_mode_handler?work_mode_handler(&pending->request,
            &pending->mode_active,&pending->mode_configured):COMM_UNSUPPORTED;
      else pending->result=execute?execute(&pending->request,&pending->mask):COMM_UNSUPPORTED;
      if(executed_observer)executed_observer(&pending->request,pending->result);
      pending->revision=0U;
      if(command_finalizer)pending->result=command_finalizer(&pending->request,pending->result,&pending->revision);
      pending->executed=1;
      last[from->link]=*pending;last_valid[from->link]=1;
    }
  }
  CommQueued *q=&queue[head];CommSource *s=&q->request.source;
  if(s->session==sessions[s->link]) {
    uint8_t frame[16],payload[8]={0};
    payload[0]=(uint8_t)q->result;
    uint16_t length=4U;
    if(q->request.code==COMM_CMD_WORK_MODE_QUERY || q->request.code==COMM_CMD_WORK_MODE_SET) {
      payload[1]=q->mode_active;payload[2]=q->mode_configured;length=3U;
    } else {
      payload[1]=q->mask?COMM_STATUS_SAMPLING:COMM_STATUS_STOPPED;
      Comm_WriteU16(payload+2,q->mask);
    }
    /* 单次最终响应：结果码、原业务字段、命令版本。不再另发“保存完成”来补充成功条件。 */
    Comm_WriteU32(payload+length,q->revision);length+=4U;
    uint16_t bytes=Comm_EncodeFrame(frame,sizeof(frame),COMM_KIND_RESPONSE,q->request.code,q->request.sequence,payload,length);
    CommResult sent=transports[s->link]?transports[s->link](s,frame,bytes):COMM_NOT_READY;
    if(sent==COMM_BUSY)return; /* 只重试响应，不重复执行命令。 */
    if(sent==COMM_OK && response_submitted)response_submitted(&q->request,q->result,q->revision);
  }
  head=(uint8_t)((head+1)%COMM_QUEUE_DEPTH);count--;
}

/* 提交已编码完整帧并带上当前会话，不自动拆包、不缓存失败帧。
 * BUSY 时调用方保留帧重试；链路适配器还会检查实际可发送长度。 */
CommResult Comm_SendFrame(CommLink link,const uint8_t *frame,uint16_t length) {
  if((unsigned)link>=COMM_LINK_COUNT || !frame || length<COMM_HEADER_SIZE ||
     length>COMM_FRAME_MAX || Comm_ReadU16(frame+6)!=length-COMM_HEADER_SIZE)return COMM_BAD_LENGTH;
  CommSource s;s.link=link;s.session=sessions[link];
  return transports[link]?transports[link](&s,frame,length):COMM_NOT_READY;
}

/* 给载荷添加帧头，仅提交成功后递增该链路主动上传序号。
 * 使用局部缓冲区，传输回调返回前必须复制或消费数据。 */
CommResult Comm_Send(CommLink link,uint8_t kind,uint8_t code,const uint8_t *payload,uint16_t length) {
  uint8_t frame[COMM_FRAME_MAX];
  if((unsigned)link>=COMM_LINK_COUNT)return COMM_BAD_VALUE;
  uint16_t n=Comm_EncodeFrame(frame,sizeof(frame),kind,code,sequences[link],payload,length);
  if(!n)return COMM_BAD_LENGTH;
  CommResult r=Comm_SendFrame(link,frame,n);
  if(r==COMM_OK)sequences[link]++;
  return r;
}

/* 编码载荷：批次 u32 + 多个 TLV；T、L 各 1 字节，L 为值的字节数。
 * 仅输出 valid_mask 选中的温度、磁场、数字声压、模拟声压，按此顺序排列。
 * 返回载荷长度，失败返回 0，最多需要 24 字节。 */
uint16_t Comm_EncodeScalars(uint8_t *out,uint16_t capacity,uint32_t batch,const CommScalars *v) {
  if(!out || !v || (v->valid_mask & ~COMM_SENSOR_SCALARS))return 0;
  unsigned need=4;
  for(unsigned i=0;i<4;i++)if(v->valid_mask&(1U<<i))need+=i==1?8:4;
  if(capacity<need)return 0;
  Comm_WriteU32(out,batch);unsigned at=4;
  for(unsigned i=0;i<4;i++)if(v->valid_mask&(1U<<i)) {
    out[at++]=(uint8_t)(i+1);out[at++]=(uint8_t)(i==1?6:2);
    if(i==1)for(unsigned a=0;a<3;a++){Comm_WriteU16(out+at,(uint16_t)v->magnetic_0p1_ut[a]);at+=2;}
    else {uint16_t value=i==0?(uint16_t)v->temperature_centi_c:(i==2?v->digital_centi_db:v->analog_centi_db);
      Comm_WriteU16(out+at,value);at+=2;}
  }
  return (uint16_t)at;
}

/* 编码载荷：批次 u32 + 原始起点 u32 + 点数 u16 + XYZ i16。
 * first 为原始采样序号，count 为本包实际点数，每点按 X/Y/Z 排列占 6 字节。
 * 此处只打包不抽取，抽取倍数通过批次消息说明。返回载荷长度，失败返回 0。 */
uint16_t Comm_EncodeWave(uint8_t *out,uint16_t capacity,uint32_t batch,uint32_t first,
                         const int16_t (*xyz)[3],uint16_t count) {
  uint32_t n=COMM_WAVE_PREFIX_SIZE+(uint32_t)count*6;
  if(!out || !xyz || !count || n>capacity)return 0;
  Comm_WriteU32(out,batch);Comm_WriteU32(out+4,first);Comm_WriteU16(out+8,count);
  for(unsigned i=0;i<count;i++)for(unsigned a=0;a<3;a++)Comm_WriteU16(out+10+i*6+a*2,(uint16_t)xyz[i][a]);
  return (uint16_t)n;
}

/* 编码载荷：批次 u32 + X/Y/Z 各 5 个 i32 特征，共 64 字节。
 * 按调用方提供的特征顺序写入，不在这里计算特征或改变缩放值。 */
uint16_t Comm_EncodeFeatures(uint8_t *out,uint16_t capacity,uint32_t batch,const int32_t features[3][5]) {
  if(!out || !features || capacity<COMM_FEATURE_PAYLOAD_SIZE)return 0;
  Comm_WriteU32(out,batch);
  for(unsigned a=0;a<3;a++)for(unsigned f=0;f<5;f++)Comm_WriteU32(out+4+(a*5+f)*4,(uint32_t)features[a][f]);
  return COMM_FEATURE_PAYLOAD_SIZE;
}

/* 标量发送便捷接口：编码批次和 TLV 后调用公共发送函数。 */
CommResult Comm_SendScalars(CommLink link,uint32_t batch,const CommScalars *v) {
  uint8_t p[24];uint16_t n=Comm_EncodeScalars(p,sizeof(p),batch,v);
  return n?Comm_Send(link,COMM_KIND_DATA,COMM_DATA_SCALARS,p,n):COMM_BAD_VALUE;
}

/* 波形发送便捷接口；当前实时流用 EncodeWave + SendFrame 保留 BUSY 待发帧。 */
CommResult Comm_SendWave(CommLink link,uint32_t batch,uint32_t first,const int16_t (*xyz)[3],uint16_t count) {
  uint8_t p[COMM_FRAME_MAX-COMM_HEADER_SIZE];
  uint16_t n=Comm_EncodeWave(p,sizeof(p),batch,first,xyz,count);
  return n?Comm_Send(link,COMM_KIND_DATA,COMM_DATA_WAVE,p,n):COMM_BAD_VALUE;
}

/* 发送已有振动特征，当前由 Zigbee 快照发布路径调用。 */
CommResult Comm_SendFeatures(CommLink link,uint32_t batch,const int32_t f[3][5]) {
  uint8_t p[COMM_FEATURE_PAYLOAD_SIZE];uint16_t n=Comm_EncodeFeatures(p,sizeof(p),batch,f);
  return n?Comm_Send(link,COMM_KIND_DATA,COMM_DATA_FEATURES,p,n):COMM_BAD_VALUE;
}

/* 发送状态事件，载荷为状态 u8 + 传感器掩码 u16。 */
CommResult Comm_SendStatus(CommLink link,uint8_t status,uint16_t mask) {
  uint8_t p[3]={0};p[0]=status;Comm_WriteU16(p+1,mask);
  return Comm_Send(link,COMM_KIND_EVENT,COMM_DATA_STATUS,p,sizeof(p));
}

/* 发送批次元信息：批次、原始采样率、抽取倍数、灵敏度。
 * 采样率与灵敏度取协议宏，必须与振动硬件配置一致。 */
CommResult Comm_SendBatch(CommLink link,uint32_t batch,uint16_t factor) {
  uint8_t p[12];Comm_WriteU32(p,batch);Comm_WriteU32(p+4,COMM_VIB_RATE_HZ);
  Comm_WriteU16(p+8,factor);Comm_WriteU16(p+10,COMM_VIB_SENSITIVITY_UG);
  return Comm_Send(link,COMM_KIND_EVENT,COMM_DATA_BATCH,p,sizeof(p));
}

/* 连接后主动状态与旧短模式指令也只发布统一事件，避免 FE81 混入 v3 帧。 */
CommResult Comm_SendWorkMode(CommLink link,CommResult result,uint8_t active,uint8_t configured) {
  uint8_t payload[3];payload[0]=(uint8_t)result;payload[1]=active;payload[2]=configured;
  return Comm_Send(link,COMM_KIND_EVENT,COMM_DATA_WORK_MODE,payload,sizeof(payload));
}

/* 旧 BLE 标量掩码转公共掩码：旧 bit3 温度，bit0 磁场，bit1/2 两类麦克风。
 * 振动使用独立状态，不在 BLE 标量掩码映射中处理。 */
uint16_t Comm_MaskFromBle(uint8_t m) { return (uint16_t)(((m&8)>>3)|((m&7)<<1)); }

/* 公共标量掩码转旧 BLE 驱动位图；振动位不参与转换。 */
uint8_t Comm_MaskToBle(uint16_t m) { return (uint8_t)(((m&1)<<3)|((m&14)>>1)); }

/* 转换 A 版旧配置：bit0 振动，bit1/2 数字/模拟麦克风。
 * 旧采集默认包含温度与磁场，返回值总带这两位；不能用于解释新协议掩码。 */
uint16_t Comm_MaskFromZigbee(uint8_t m) {
  return COMM_SENSOR_TEMPERATURE|COMM_SENSOR_MAGNETIC|((m&1)?COMM_SENSOR_VIBRATION:0)|((m&6)<<1);
}
