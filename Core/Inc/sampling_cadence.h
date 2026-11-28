#ifndef SAMPLING_CADENCE_H
#define SAMPLING_CADENCE_H
#include <stdint.h>
/* RTC milliseconds, modulo 2^32; intervals must be below INT32_MAX. */
typedef struct { uint32_t next, period; unsigned active; } SamplingCadence;
static inline void sampling_cadence_start(SamplingCadence *c,uint32_t now,uint32_t period) {
    c->period=period;c->next=now+period;c->active=1;
}
static inline unsigned sampling_cadence_due(SamplingCadence *c,uint32_t now) {
    if(!c->active || (int32_t)(now-c->next)<0)return 0;
    unsigned count=(now-c->next)/c->period+1U;
    c->next+=count*c->period;return count;
}
static inline void sampling_cadence_change(SamplingCadence *c,uint32_t period) {
    if(c->active && c->period!=period){c->next=c->next-c->period+period;c->period=period;}
}
static inline uint32_t sampling_cadence_delay(const SamplingCadence *c,uint32_t now) {
    return (int32_t)(c->next-now)>0?c->next-now:1U;
}
/* A reconnect plan changes only the next sampling deadline. Upload scheduling
 * and the external Flash queue are deliberately independent. */
static inline void sampling_cadence_resume(SamplingCadence *c,uint32_t now,uint32_t period,uint32_t delay) {
    c->period=period;c->next=now+delay;c->active=1U;
}
#endif
