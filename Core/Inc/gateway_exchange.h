#ifndef GATEWAY_EXCHANGE_H
#define GATEWAY_EXCHANGE_H
#include <stdint.h>
#include "comm_protocol.h"

/* 节点主动发起一轮上报。token 隔离轮次，revision 隔离本轮命令后的配置。
 * 普通 APS/V2 ACK 只确认传输，不拥有本状态机的休眠放行权。 */
typedef struct {
  uint32_t token, batch, revision, next;
  uint16_t decision, mask;
  uint8_t active, advertised, valid, commands, rejected, decision_seen, response_ready;
} GatewayExchange;

static inline void gateway_begin(GatewayExchange *s,uint32_t batch) {
  uint8_t commands=s->active && s->commands;
  if(s->active && (s->batch==batch || batch==0U ||
      (s->batch!=0U && !(s->valid && s->commands))))return;
  if(!++s->token)++s->token;
  s->batch=batch;s->revision=0;s->next=0;
  s->active=1;s->advertised=s->valid=s->commands=s->rejected=s->decision_seen=0;
  /* 已获准的命令可触发下一次采集，延续命令接收窗口，但旧休眠许可必须失效。 */
  s->commands=commands;
  s->mask=0;
  s->response_ready=0;
}
/* 仅实际执行过的命令递增版本，重传请求不重复递增；任何新命令撤销旧许可。 */
static inline void gateway_changed(GatewayExchange *s) {
  s->revision++;s->mask=0;s->advertised=0;s->commands=1;s->next=0;
  s->response_ready=0;
}
/* 新 ACK 载荷：token:u32,batch:u32,revision:u32,result:u8,sleep_mask:u16。
 * 同版本决策序号必须递增；迟到的“可睡”及“有命令”都不能覆盖新决策。 */
static inline int gateway_decision(GatewayExchange *s,const uint8_t *p,uint16_t n,uint16_t seq) {
  uint16_t mask;
  if(!p || n!=COMM_GATEWAY_ACK_SIZE || !s->active || !s->advertised ||
      Comm_ReadU32(p)!=s->token || Comm_ReadU32(p+4)!=s->batch ||
      Comm_ReadU32(p+8)!=s->revision)return 0;
  mask=Comm_ReadU16(p+13);
  if(p[12]>COMM_SAVE_FAILED || (mask & ~(COMM_GATEWAY_SLEEP_ALLOWED|COMM_GATEWAY_COMMANDS_PENDING)) ||
      mask==(COMM_GATEWAY_SLEEP_ALLOWED|COMM_GATEWAY_COMMANDS_PENDING) ||
      (p[12]!=COMM_OK && mask))return 0;
  if(s->decision_seen && (int16_t)(seq-s->decision)<=0)return 0;
  s->decision_seen=1;s->decision=seq;s->mask=mask;
  s->valid=p[12]==COMM_OK;s->rejected=!s->valid;
  s->commands=s->valid && (mask & COMM_GATEWAY_COMMANDS_PENDING);
  return 1;
}
static inline int gateway_can_finish(const GatewayExchange *s,int busy,int dirty) {
  return s->active && s->advertised && s->valid &&
      s->mask==COMM_GATEWAY_SLEEP_ALLOWED && !busy && !dirty;
}
#endif
