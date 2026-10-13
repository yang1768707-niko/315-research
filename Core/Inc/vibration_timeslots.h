#ifndef VIBRATION_TIMESLOTS_H
#define VIBRATION_TIMESLOTS_H
#include "vibration_wire.h"

/* Application submission windows, not hardware TDMA. A closed APS stack can
 * still retry after a window; measure confirm latency on the real radio. */
#define V2_TIMED_BLOCK_LEN 25U
#define V2_SCHEDULE_VERSION 3U
#define V2_TIMED_ACK_LEN 37U
#define V2_TIMED_OK 10U
#define V2_SLOT_MS 160U
#define V2_CONTROL_MS 160U
#define V2_SLOT_OFFSET_MS 60U
#define V2_SLOT_SUBMIT_MS 20U
#define V2_SINGLE_FRAME_MS 40U
#define V2_SYNC_RTT_MAX_MS 100U
#define V2_SYNC_VALID_MS 10000U
#define V2_EPOCH_MS 12000U
#define V2_EPOCH_GUARD_MS 1000U
#define V2_GROUP_MAX 4U
#define V2_TIMED_WAITERS 16U

typedef struct {
    uint64_t ticket, seen;
} V2TimedWaiter;
typedef struct {
    V2TimedWaiter waiters[V2_TIMED_WAITERS];
    uint64_t sequence, start, end, quiet_until;
    unsigned members[V2_GROUP_MAX], count;
    uint32_t epochs, waits;
    int initialized, draining;
} V2TimedScheduler;

static inline void v2_timed_init(V2TimedScheduler *s,uint64_t now) {
    if(!s->initialized) {
        s->initialized=1;
        s->quiet_until=now+V2_EPOCH_MS+V2_EPOCH_GUARD_MS;
    }
}
static inline int v2_timed_member(const V2TimedScheduler *s,unsigned id) {
    for(unsigned i=0;i<s->count;i++)if(s->members[i]==id+1U)return (int)i;
    return -1;
}
static inline void v2_timed_forget(V2TimedScheduler *s,unsigned id) {
    if(id>=V2_TIMED_WAITERS)return;
    s->waiters[id].ticket=0;
    /* Do not reuse a lane early: its previous node may hold a delayed grant. */
    for(unsigned i=0;i<s->count;i++)if(s->members[i]==id+1U){s->members[i]=0;s->draining=1;}
}
static inline int v2_timed_request(V2TimedScheduler *s,unsigned id,uint64_t now) {
    v2_timed_init(s,now);
    if(id>=V2_TIMED_WAITERS)return -1;
    s->waiters[id].seen=now;
    int member=v2_timed_member(s,id);
    /* Stop extending issued leases as soon as membership needs to change.
       An unresponsive member must not retain a lane indefinitely. */
    for(unsigned i=0;i<s->count;i++)
        if(!s->members[i] || now-s->waiters[s->members[i]-1U].seen>V2_EPOCH_MS)
            s->draining=1;
    if(now<s->end && member>=0) {
        if(!s->draining && now+V2_EPOCH_MS>s->end)s->end=now+V2_EPOCH_MS;
        return member;
    }
    if(member<0 && s->count)s->draining=1;
    V2TimedWaiter *w=&s->waiters[id];
    if(!w->ticket)w->ticket=++s->sequence;
    w->seen=now;
    if(now>=s->quiet_until && now>=s->end+V2_EPOCH_GUARD_MS) {
        s->count=0;s->draining=0;
        for(unsigned k=0;k<V2_GROUP_MAX;k++) {
            unsigned first=V2_TIMED_WAITERS;
            for(unsigned i=0;i<V2_TIMED_WAITERS;i++) {
                V2TimedWaiter *q=&s->waiters[i];
                if(q->ticket && now-q->seen>20000U)q->ticket=0;
                if(q->ticket && (first==V2_TIMED_WAITERS || q->ticket<s->waiters[first].ticket))first=i;
            }
            if(first==V2_TIMED_WAITERS)break;
            s->members[s->count++]=first+1U;s->waiters[first].ticket=0;
        }
        s->start=now+500U;s->end=s->start+V2_EPOCH_MS;++s->epochs;
        for(unsigned i=0;i<V2_TIMED_WAITERS;i++)if(s->waiters[i].ticket)s->draining=1;
    }
    int lane=v2_timed_member(s,id);
    if(now>=s->end)lane=-1;
    if(lane<0)++s->waits;
    return lane;
}
static inline int v2_timed_active(const V2TimedScheduler *s,unsigned id,uint64_t now) {
    return now>=s->start && now<s->end+V2_EPOCH_GUARD_MS && v2_timed_member(s,id)>=0;
}
/* Checked wire schedule. Epoch/start and server time are modulo 2^32 ms. */
typedef struct {
    uint32_t start, offset, end, sync_until, last_cycle;
    uint16_t slot_ms;
    uint8_t lane, count, valid, sent_cycle;
} V2TimedClock;
static inline uint32_t v2_timed_period(const V2TimedClock *c) {
    return c->count*c->slot_ms+V2_CONTROL_MS;
}
/* Distinguish timing uncertainty from a malformed/incompatible schedule. */
static inline unsigned v2_timed_reject_reason(const uint8_t *p,uint32_t now,
                                              uint32_t sent,uint32_t cookie) {
    uint32_t server=v2_u32(p+25),start=v2_u32(p+29);
    if(v2_u32(p+17)!=cookie || v2_u32(p+21)!=sent)return 1;
    if(now-sent>V2_SYNC_RTT_MAX_MS)return 2;
    if(!p[33] || p[33]>V2_GROUP_MAX || p[34]>=p[33] ||
       v2_u16(p+35)!=V2_SLOT_MS)return 3;
    uint32_t lease=v2_u32(p+9);
    if((int32_t)(server-start)<-500 || !lease || lease>V2_EPOCH_MS+500U ||
       lease<=(now-sent)/2U)return 4;
    return 0;
}
static inline int v2_timed_accept(V2TimedClock *c,const uint8_t *p,uint32_t now,
                                 uint32_t sent,uint32_t cookie) {
    uint32_t rtt=now-sent,server=v2_u32(p+25),start=v2_u32(p+29);
    if(v2_timed_reject_reason(p,now,sent,cookie))return 0;
    uint32_t estimate=server+rtt/2U;
    /* Keep the per-cycle send latch when only refreshing synchronization. */
    if(!c->valid || c->start!=start || c->lane!=p[34])c->sent_cycle=0;
    c->offset=estimate-now;c->start=start;c->end=server+v2_u32(p+9);
    c->sync_until=now+V2_SYNC_VALID_MS;c->slot_ms=v2_u16(p+35);
    c->count=p[33];c->lane=p[34];c->valid=1;return 1;
}
static inline int v2_timed_live(const V2TimedClock *c,uint32_t now) {
    return c->valid && (int32_t)(now-c->sync_until)<0 &&
        (int32_t)(now+c->offset-c->end)<0;
}
static inline int v2_timed_data_ready(V2TimedClock *c,uint32_t now) {
    uint32_t server=now+c->offset;
    if(!v2_timed_live(c,now) || (int32_t)(server-c->start)<0)return 0;
    uint32_t elapsed=server-c->start,period=v2_timed_period(c);
    uint32_t cycle=elapsed/period,phase=elapsed%period;
    if(c->count==1U) {
        /* One uploader owns this lease: no need to wait through empty lanes.
           Its old lease still expires before a new member can start. */
        cycle=elapsed/V2_SINGLE_FRAME_MS;
        if(c->sent_cycle && c->last_cycle==cycle)return 0;
        c->last_cycle=cycle;c->sent_cycle=1;return 1;
    }
    uint32_t begin=c->lane*c->slot_ms+V2_SLOT_OFFSET_MS;
    if(phase<begin || phase>=begin+V2_SLOT_SUBMIT_MS ||
       (c->sent_cycle && c->last_cycle==cycle))return 0;
    c->last_cycle=cycle;c->sent_cycle=1;return 1;
}
static inline int v2_timed_control_ready(const V2TimedClock *c,uint32_t now) {
    uint32_t server=now+c->offset;
    if(!c->valid || c->count==1U || (int32_t)(server-c->start)<0 || (int32_t)(server-c->end)>=0)return 1;
    uint32_t phase=(server-c->start)%v2_timed_period(c);
    return phase>=c->count*c->slot_ms+40U && phase<v2_timed_period(c)-80U;
}
#endif
