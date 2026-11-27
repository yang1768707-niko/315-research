#ifndef UPLOAD_LEASE_H
#define UPLOAD_LEASE_H
#include "vibration_wire.h"
/* Nonce-bearing query: version, kind, batch LE32, nonce LE32 (10 bytes).
 * Reply: ordinary ACK, bytes 9..12 nonce, 13..16 validity in ms.
 * Deadline starts at REQUEST SUBMISSION, never at delayed reply arrival.
 * Backend reserves >=60s after receipt; node stops after 25s (35s margin). */
#define UPLOAD_LEASE_KIND 7U
#define UPLOAD_LEASE_MS 25000U
#define UPLOAD_LEASE_RENEW_MS 8000U
#define UPLOAD_LEASE_RETRY_MS 3000U
typedef struct {
    uint32_t batch,nonce,sent,expires,renew,next,started;
    uint8_t waiting,valid,attempts,lost;
} UploadLease;
static inline void upload_lease_start(UploadLease *l,uint32_t batch,uint32_t nonce,uint32_t now) {
    *l=(UploadLease){0};l->batch=batch;l->nonce=nonce;l->next=now;l->started=now;
}
static inline int upload_lease_valid(const UploadLease *l,uint32_t now) {
    return l->valid && (int32_t)(now-l->expires)<0;
}
static inline int upload_lease_query(UploadLease *l,uint32_t now,uint8_t p[10]) {
    if((!l->valid && (uint32_t)(now-l->started)>=20000U) ||
       (l->valid && !upload_lease_valid(l,now) && (uint32_t)(now-l->expires)>=10000U)) {l->lost=1;return 0;}
    if((int32_t)(now-l->next)<0 || (upload_lease_valid(l,now) && (int32_t)(now-l->renew)<0))return 0;
    if(l->attempts>=5U){l->lost=1;return 0;}
    /* Retries keep their nonce AND original send time. Slow replies must not
     * be invalidated every 3s or extend a lease from a later retransmission. */
    if(!l->waiting){if(!++l->nonce)++l->nonce;l->sent=now;l->waiting=1;}
    l->attempts++;l->next=now+UPLOAD_LEASE_RETRY_MS;
    p[0]=V2_VERSION;p[1]=UPLOAD_LEASE_KIND;v2_w32(p+2,l->batch);v2_w32(p+6,l->nonce);return 1;
}
static inline int upload_lease_receive(UploadLease *l,const uint8_t *p,size_t n,uint32_t now) {
    if(!l->waiting || n!=V2_ACK_LEN || p[0]!=V2_VERSION || p[1]!=UPLOAD_LEASE_KIND ||
       v2_u32(p+2)!=l->batch || v2_u32(p+9)!=l->nonce || v2_u16(p+6)!=0)return 0;
    /* All replies, including NEED_BATCH, must match the live nonce. */
    if(p[8]==V2_NEED_BATCH){l->lost=1;l->valid=0;l->waiting=0;return -1;}
    if(p[8]!=V2_OK || v2_u32(p+13)!=UPLOAD_LEASE_MS || (uint32_t)(now-l->sent)>=UPLOAD_LEASE_MS)return 0;
    l->valid=1;l->waiting=0;l->attempts=0;l->expires=l->sent+UPLOAD_LEASE_MS;
    l->renew=now+UPLOAD_LEASE_RENEW_MS;l->next=l->renew;return 1;
}
#endif
