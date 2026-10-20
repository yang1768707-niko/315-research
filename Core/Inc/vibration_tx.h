#ifndef VIBRATION_TX_H
#define VIBRATION_TX_H
#include "vibration_codec.h"
/* Block completion is pushed; BATCH/WAIT supplies a relative queue delay. */

#define V2_TX_IDLE 0
#define V2_TX_BATCH 1
#define V2_TX_PREPARE 2
#define V2_TX_BLOCK 3
#define V2_TX_STREAM 4
#define V2_TX_WINDOW 5
#define V2_TX_FINAL 6
#define V2_TX_DONE 7
#define V2_TX_ERROR 8
#define V2_ACK_TIMEOUT_MS 3000U
#define V2_SEND_STALL_MS 10000U
#define V2_PACE_MS 12U
#define V2_RETRIES 5U

typedef struct {
    uint8_t batch[V2_BATCH_LEN], block[V2_BLOCK_LEN];
    uint8_t raw[V2_RAW_BYTES], encoded[V2_RAW_BYTES];
    uint64_t missing;
    uint32_t next_ms, deadline_ms;
    uint16_t index, encoded_len;
    uint8_t phase, waiting, tries, restarts, fragment, fragments;
    uint32_t sent_bytes, sent_packets;
    uint32_t pace_ms, timeout_count, wait_count, radio_failures;
    uint8_t deferred, block_repairs, block_timeouts;
} V2Tx;
typedef struct {
    void *context;
    /* read exact raw XYZ little endian bytes for this batch; 1 success */
    int (*read)(void *, uint32_t batch, uint32_t first, unsigned points, uint8_t *);
    /* 1 accepted, 0 backpressure, -1 permanent error; owns/copies bytes before return */
    int (*send)(void *, uint8_t command, const uint8_t *, size_t);
} V2TxIO;
static inline int v2_due(uint32_t now, uint32_t target) {return (int32_t)(now-target)>=0;}
static inline int v2_tx_active(const V2Tx *t) {return t->phase>=V2_TX_BATCH && t->phase<=V2_TX_FINAL;}
/* Diagnostics do not change the fixed single-node pacing. APS owns accepted buffers. */
static inline void v2_tx_radio_result(V2Tx *t,int success,uint32_t now) {
    (void)now;
    if(v2_tx_active(t) && !success)++t->radio_failures;
}
/* The RTC wake event calls resume: it still sends BATCH to reapply, not data. */
static inline void v2_tx_resume(V2Tx *t,uint32_t now) {
    if(t->deferred){t->deferred=0;t->next_ms=now;t->deadline_ms=now+V2_SEND_STALL_MS;}
}
static inline int v2_tx_start(V2Tx *t, const uint8_t *batch, uint32_t now) {
    if(!v2_valid_batch(batch,V2_BATCH_LEN))return 0;
    memset(t,0,sizeof(*t));memcpy(t->batch,batch,V2_BATCH_LEN);
    t->pace_ms=V2_PACE_MS;
    t->phase=V2_TX_BATCH;t->next_ms=now;t->deadline_ms=now+V2_SEND_STALL_MS;return 1;
}
static inline void v2_tx_phase(V2Tx *t, uint8_t phase, uint32_t now) {
    t->phase=phase;t->waiting=0;t->tries=0;t->deferred=0;t->next_ms=now;
    t->deadline_ms=now+V2_SEND_STALL_MS;
}
static inline void v2_tx_next_block(V2Tx *t, uint32_t now) {
    t->block_repairs=0;t->block_timeouts=0;
    ++t->index;
    v2_tx_phase(t,t->index<v2_blocks(v2_u32(t->batch+7))?V2_TX_PREPARE:V2_TX_FINAL,now);
}
static inline void v2_tx_receive(V2Tx *t,const uint8_t *p,size_t n,uint32_t now) {
    if(!v2_tx_active(t) || n!=V2_ACK_LEN || p[0]!=V2_VERSION ||
       v2_u32(p+2)!=v2_u32(t->batch+1))return;
    unsigned kind=p[1],status=p[8],block=v2_u16(p+6);
    unsigned expected=t->phase==V2_TX_BATCH?V2_KIND_BATCH:
        t->phase==V2_TX_BLOCK?V2_KIND_BLOCK:
        (t->phase==V2_TX_WINDOW || t->phase==V2_TX_STREAM)?V2_KIND_WINDOW:
        t->phase==V2_TX_FINAL?V2_KIND_FINAL:0;
    /* A complete block can arrive before the final APS send callback. */
    if((kind!=expected && !(kind==V2_KIND_BLOCK && t->phase==V2_TX_STREAM)) ||
       (!t->waiting && t->phase!=V2_TX_STREAM) ||
       ((kind==V2_KIND_BLOCK || kind==V2_KIND_WINDOW) && block!=t->index))return;
    if(status==V2_WAIT && (kind==V2_KIND_BATCH || kind==V2_KIND_BLOCK)) {
        uint32_t delay=v2_u32(p+9);
        if(delay<100U)delay=100U;
        if(delay>1800000U)delay=1800000U;
        t->waiting=0;t->tries=0;t->deferred=1;++t->wait_count;
        t->next_ms=now+delay;t->deadline_ms=t->next_ms+V2_SEND_STALL_MS;return;
    }
    if(status==V2_NEED_BATCH) {
        if(++t->restarts>V2_RETRIES){t->phase=V2_TX_ERROR;return;}
        t->block_repairs=0;t->block_timeouts=0;
        t->index=0;v2_tx_phase(t,V2_TX_BATCH,now);return;
    }
    if(status==V2_REJECT){t->phase=V2_TX_ERROR;return;}
    if(status==V2_STORE_FAILED)return; /* retry at bounded ACK timeout */
    if(kind==V2_KIND_BATCH && status==V2_OK){v2_tx_phase(t,V2_TX_PREPARE,now);return;}
    if(kind==V2_KIND_BLOCK && status==V2_BLOCK_DONE){v2_tx_next_block(t,now);return;}
    if(kind==V2_KIND_BLOCK && status==V2_OK && t->phase==V2_TX_BLOCK) {
        t->missing=v2_mask(t->fragments);t->fragment=0;
        v2_tx_phase(t,V2_TX_STREAM,now);return;
    }
    if(kind==V2_KIND_WINDOW && status==V2_NEED_BLOCK) {
        if(++t->restarts>V2_RETRIES)t->phase=V2_TX_ERROR;
        else v2_tx_phase(t,V2_TX_BLOCK,now);
        return;
    }
    if(kind==V2_KIND_WINDOW && status==V2_MISSING) {
        uint64_t missing=v2_u64(p+9);
        if(!missing || (missing&~v2_mask(t->fragments)))return;
        if(t->phase==V2_TX_STREAM && t->block_repairs && missing==t->missing)return;
        if(++t->block_repairs>V2_RETRIES){t->phase=V2_TX_ERROR;return;}
        t->missing=missing;t->fragment=0;t->waiting=0;t->phase=V2_TX_STREAM;
        t->next_ms=now;t->deadline_ms=now+V2_SEND_STALL_MS;return;
    }
    if(kind==V2_KIND_WINDOW && status==V2_OK){v2_tx_next_block(t,now);return;}
    if(kind==V2_KIND_FINAL && status==V2_OK)t->phase=V2_TX_DONE;
}
static inline void v2_tx_poll(V2Tx *t, const V2TxIO *io,uint32_t now) {
    if(!v2_tx_active(t) || !v2_due(now,t->next_ms))return;
    t->deferred=0;
    if(t->waiting) {
        if(!v2_due(now,t->deadline_ms))return;
        if(t->phase==V2_TX_WINDOW && ++t->block_timeouts>V2_RETRIES){t->phase=V2_TX_ERROR;return;}
        if(++t->tries>V2_RETRIES){t->phase=V2_TX_ERROR;return;}
        ++t->timeout_count;
        t->waiting=0;t->deadline_ms=now+V2_SEND_STALL_MS;
    }
    if(t->phase==V2_TX_PREPARE) {
        unsigned points=v2_block_points(v2_u32(t->batch+7),t->index);
        uint8_t codec=0;
        if(!points || !io->read(io->context,v2_u32(t->batch+1),(uint32_t)t->index*V2_POINTS,points,t->raw)){
            t->phase=V2_TX_ERROR;return;
        }
        t->encoded_len=(uint16_t)v2_encode(t->raw,points,t->encoded,sizeof(t->encoded),&codec);
        if(!t->encoded_len){t->phase=V2_TX_ERROR;return;}
        t->fragments=(uint8_t)v2_fragments(t->encoded_len);
        t->block[0]=V2_VERSION;v2_w32(t->block+1,v2_u32(t->batch+1));
        v2_w16(t->block+5,t->index);v2_w16(t->block+7,(uint16_t)points);
        t->block[9]=codec;v2_w16(t->block+10,t->encoded_len);
        v2_w32(t->block+12,v2_crc(t->raw,(size_t)points*6));
        v2_tx_phase(t,V2_TX_BLOCK,now);
    }
    uint8_t packet[79],command=0;
    const uint8_t *payload=packet;
    size_t length=0;
    if(t->phase==V2_TX_BATCH){command=V2_BATCH;payload=t->batch;length=V2_BATCH_LEN;}
    else if(t->phase==V2_TX_BLOCK){command=V2_BLOCK;payload=t->block;length=V2_BLOCK_LEN;}
    else if(t->phase==V2_TX_STREAM) {
        while(t->fragment<t->fragments && !(t->missing&(UINT64_C(1)<<t->fragment)))++t->fragment;
        if(t->fragment>=t->fragments) {
            /* Normal path waits for the receiver's pushed bitmap/OK, not a query. */
            t->phase=V2_TX_WINDOW;t->waiting=1;t->next_ms=now;
            t->deadline_ms=now+V2_ACK_TIMEOUT_MS;return;
        }
        unsigned offset=(unsigned)t->fragment*V2_FRAGMENT_BYTES;
        unsigned count=t->encoded_len-offset;
        if(count>V2_FRAGMENT_BYTES)count=V2_FRAGMENT_BYTES;
        v2_w32(packet,v2_u32(t->batch+1));v2_w16(packet+4,t->index);packet[6]=t->fragment;
        memcpy(packet+7,t->encoded+offset,count);command=V2_DATA;length=7+count;
    } else if(t->phase==V2_TX_WINDOW || t->phase==V2_TX_FINAL) {
        packet[0]=V2_VERSION;packet[1]=t->phase==V2_TX_WINDOW?V2_KIND_WINDOW:V2_KIND_FINAL;
        v2_w32(packet+2,v2_u32(t->batch+1));v2_w16(packet+6,t->index);
        command=V2_QUERY;length=8;
    }
    if(!command)return;
    int submitted=io->send(io->context,command,payload,length);
    if(submitted<0){t->phase=V2_TX_ERROR;return;}
    t->next_ms=now+V2_PACE_MS;
    if(!submitted) {
        if(v2_due(now,t->deadline_ms))t->phase=V2_TX_ERROR;
        return;
    }
    t->sent_bytes+=(uint32_t)length;++t->sent_packets;
    if(command==V2_DATA){++t->fragment;t->deadline_ms=now+V2_SEND_STALL_MS;}
    else if(command==V2_BLOCK && t->index!=0U) {
        /* The preceding block OK already permits this block. Keep metadata,
         * but do not perform another ordinary permission round trip. */
        t->missing=v2_mask(t->fragments);t->fragment=0;
        v2_tx_phase(t,V2_TX_STREAM,now);t->next_ms=now+V2_PACE_MS;
    } else {t->waiting=1;t->deadline_ms=now+V2_ACK_TIMEOUT_MS;}
}
#endif
