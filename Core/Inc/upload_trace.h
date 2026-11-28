#ifndef UPLOAD_TRACE_H
#define UPLOAD_TRACE_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
/* UART 0x9f/v3: bounded diagnostic metadata, never sensor bytes or keys.
 * Identity = address + batch + block/fragment + payload fingerprint.
 * A retry can have the same fingerprint; only a source ZCL sequence identifies
 * its individual radio attempt. Clocks/ordinals are LOCAL to each source. */
#define UT_LEN 40U
#define UT_COORD_RX 1U
#define UT_COORD_FORWARD 2U
#define UT_COORD_DROP 3U
#define UT_HOST_RX 4U
#define UT_HOST_REPLY 5U
#define UT_COORD_SUBMIT 6U
#define UT_COORD_RESULT 7U
#define UT_NODE_RX 8U
#define UT_SNAPSHOT 9U
#define UT_SNAPSHOT_END 10U
#define UT_NODE_TX 11U
#define UT_NODE_RESULT 12U
#define UT_HOST_WRITE 13U
#define UT_CAPABILITY 14U
#define UT_SNAPSHOT_FLAG 1U
#define UT_SEQ_VALID 2U
#define UT_FRAGMENT_VALID 4U
#ifndef UT_RING_SIZE
#define UT_RING_SIZE 96U
#endif
#define UT_DUMP_MAX 48U
#define UT_COOLDOWN_MS 10000U
typedef struct { uint8_t p[UT_LEN]; } UploadTraceRecord;
typedef struct {
    UploadTraceRecord records[UT_RING_SIZE];
    uint32_t serial,overwritten;
    unsigned head,count;
} UploadTraceRing;
/* Per-address throttling: a noisy device must not suppress another node. */
typedef struct {uint16_t address;uint8_t used;uint32_t tick;} UploadTraceLimit;
static inline uint16_t ut_u16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static inline uint32_t ut_u32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static inline void ut_w16(uint8_t *p,uint16_t n){p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static inline void ut_w32(uint8_t *p,uint32_t n){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(n>>(8*i));}
static inline uint32_t ut_fingerprint(const uint8_t *p,size_t n){
    uint32_t hash=2166136261U;
    for(size_t i=0;i<n;i++){hash^=p[i];hash*=16777619U;}
    return hash;
}
static inline int ut_packet(UploadTraceRecord *r,uint8_t stage,uint32_t tick,uint16_t address,
                            uint8_t command,int sequence,const uint8_t *p,size_t n){
    if(!r || !p || n>255U)return 0;
    memset(r,0,sizeof(*r));r->p[0]=3;r->p[1]=stage;ut_w32(r->p+2,tick);ut_w16(r->p+6,address);
    r->p[8]=command;r->p[9]=(uint8_t)sequence;r->p[18]=255;
    ut_w16(r->p+16,UINT16_MAX);ut_w16(r->p+28,(uint16_t)n);
    if(sequence>=0)r->p[19]|=UT_SEQ_VALID;
    if(command==0x94 && n>=7U){ut_w32(r->p+12,ut_u32(p));ut_w16(r->p+16,ut_u16(p+4));r->p[18]=p[6];r->p[19]|=UT_FRAGMENT_VALID;}
    else if((command==0x92 || command==0x93 || command==0xA5) && n>=5U && p[0]==2U){
        ut_w32(r->p+12,ut_u32(p+1));
        if(command==0x93 && n>=16U)ut_w16(r->p+16,ut_u16(p+5));
    } else if(command==0x95 && n>=8U && p[0]==2U){
        r->p[10]=p[1];ut_w32(r->p+12,ut_u32(p+2));
        if(p[1]==2U || p[1]==3U)ut_w16(r->p+16,ut_u16(p+6));
        if(p[1]==7U && n>=10U)ut_w32(r->p+24,ut_u32(p+6));
    } else if(command==0x0D && n==17U && p[0]==2U){
        r->p[10]=p[1];r->p[11]=p[8];ut_w32(r->p+12,ut_u32(p+2));
        if(p[1]==2U || p[1]==3U)ut_w16(r->p+16,ut_u16(p+6));
        ut_w32(r->p+24,ut_u32(p+9)); /* bitmap low word or lease nonce */
    } else if((command>=0x86 && command<=0x89) && n>=6U)ut_w32(r->p+12,ut_u32(p));
    else if(command==0x8C && n>=70U)ut_w32(r->p+12,ut_u32(p));
    else if(command==0x8D && n>=6U){ut_w32(r->p+12,ut_u32(p+2));r->p[11]=p[1];}
    else return 0;
    ut_w32(r->p+20,ut_fingerprint(p,command==0x8D?6U:n));return 1;
}
static inline void ut_push(UploadTraceRing *ring,UploadTraceRecord *r){
    ut_w32(r->p+32,++ring->serial);ring->records[ring->head]=*r;
    ring->head=(ring->head+1U)%UT_RING_SIZE;
    if(ring->count<UT_RING_SIZE)ring->count++;else if(ring->overwritten!=UINT32_MAX)ring->overwritten++;
}
static inline int ut_allow(UploadTraceLimit *limits,unsigned count,uint16_t address,uint32_t now){
    if(!limits || !count)return 0;
    unsigned slot=0;uint32_t oldest=0;
    for(unsigned i=0;i<count;i++){
        if(limits[i].used && limits[i].address==address){
            if((uint32_t)(now-limits[i].tick)<UT_COOLDOWN_MS)return 0;
            slot=i;break;
        }
        if(!limits[i].used){slot=i;break;}
        if((uint32_t)(now-limits[i].tick)>=oldest){oldest=now-limits[i].tick;slot=i;}
    }
    limits[slot].used=1;limits[slot].address=address;limits[slot].tick=now;return 1;
}
static inline int ut_valid(const uint8_t *p,size_t n){return p && n==UT_LEN && p[0]==3U && p[1]>=1U && p[1]<=14U;}
#endif
