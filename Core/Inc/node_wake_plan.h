#ifndef NODE_WAKE_PLAN_H
#define NODE_WAKE_PLAN_H
#include <stdint.h>
#define NODE_WAKE_SAMPLE 1U
#define NODE_WAKE_QUEUE 2U
/* RTC milliseconds advance in STOP2. Independent causes, including both at
 * once; callers must service SAMPLE before QUEUE when both bits are set. */
static inline unsigned node_wake_causes(uint32_t now,int sample_enabled,uint32_t sample_at,
                                        int queue_enabled,uint32_t queue_at) {
    unsigned causes=0;
    if(sample_enabled && (int32_t)(now-sample_at)>=0)causes|=NODE_WAKE_SAMPLE;
    if(queue_enabled && (int32_t)(now-queue_at)>=0)causes|=NODE_WAKE_QUEUE;
    return causes;
}
#endif
