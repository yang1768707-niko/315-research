#ifndef SCALAR_TX_H
#define SCALAR_TX_H
#include "scalar_delivery.h"
#include "vibration_wire.h"
#define SCALAR_RETRIES 5U
#define SCALAR_ACK_MS 3000U
#define SCALAR_STALL_MS 10000U
typedef struct {
    uint8_t manifest[SCALAR_MANIFEST_LEN];
    uint32_t next_ms,deadline_ms;
    uint8_t pending,waiting,attempts,done,error,restart;
} ScalarTx;
typedef struct {
    void *context;
    int (*item)(void *,unsigned);
    int (*manifest)(void *,const uint8_t *);
} ScalarTxIO;
static inline int scalar_tx_start(ScalarTx *t,const uint8_t *manifest,uint32_t now)
{
    if(!scalar_manifest_valid(manifest,SCALAR_MANIFEST_LEN))return 0;
    uint8_t copy[SCALAR_MANIFEST_LEN];
    for(unsigned i=0;i<SCALAR_MANIFEST_LEN;i++)copy[i]=manifest[i];
    *t=(ScalarTx){0};
    for(unsigned i=0;i<SCALAR_MANIFEST_LEN;i++)t->manifest[i]=copy[i];
    t->next_ms=now;t->deadline_ms=now+SCALAR_STALL_MS;
    return 1;
}
static inline void scalar_tx_receive(ScalarTx *t,const uint8_t *p,size_t length,uint32_t now)
{
    if(!t->waiting || length!=V2_ACK_LEN || p[0]!=V2_VERSION || p[1]!=SCALAR_ACK_KIND ||
       v2_u32(p+2)!=scalar_u32(t->manifest+1) || v2_u16(p+6)!=0)return;
    uint64_t missing=v2_u64(p+9);
    if(p[8]==V2_OK && !missing) {t->waiting=0;t->done=1;return;}
    if(p[8]==V2_MISSING && missing && !(missing&~(uint64_t)(t->manifest[6]&t->manifest[7]))) {
        t->pending=(uint8_t)missing;t->waiting=0;t->next_ms=now+50U;t->deadline_ms=now+SCALAR_STALL_MS;
    } else if(p[8]==V2_STORE_FAILED) {
        t->waiting=0;t->next_ms=now+500U;t->deadline_ms=now+SCALAR_STALL_MS;
    } else if(p[8]==V2_NEED_BATCH) {t->waiting=0;t->restart=1;}
    else if(p[8]==V2_REJECT) {t->waiting=0;t->error=1;}
}
/* Probe first. The receiver asks for only absent/corrupt items. Lost probes or
 * replies never authorize waveform data; retry count and enqueue stalls bound
 * the awake interval. Return 1 confirmed, 0 pending, -1 retained failure. */
static inline int scalar_tx_poll(ScalarTx *t,const ScalarTxIO *io,uint32_t now)
{
    if(t->done)return 1;
    if(t->error || t->restart)return -1;
    if((int32_t)(now-t->next_ms)<0)return 0;
    if(t->waiting) {
        t->waiting=0;t->deadline_ms=now+SCALAR_STALL_MS;
    }
    if(t->attempts>=SCALAR_RETRIES || (int32_t)(now-t->deadline_ms)>=0) {
        t->error=1;return -1;
    }
    for(unsigned i=0;i<4;i++)if(t->pending&(1U<<i)) {
        if(!io->item(io->context,i))return 0;
        t->pending&=(uint8_t)~(1U<<i);
        return 0; /* Let the ordinary TX queue service this item first. */
    }
    if(!io->manifest(io->context,t->manifest))return 0;
    t->attempts++;t->waiting=1;t->next_ms=now+SCALAR_ACK_MS;
    return 0;
}
#endif
