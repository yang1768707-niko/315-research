#ifndef LINK_HEALTH_H
#define LINK_HEALTH_H
#include <stdint.h>
#define UPLOAD_RECOVERY_BACKOFF_MS 60000U
#define UPLOAD_RECOVERY_RESTART_MS 180000U
typedef struct {
    uint32_t deadline,restart_at;
    uint8_t failures,pending,verifying,backoff,rounds,restarted;
} LinkHealth;
static inline void link_health_result(LinkHealth *h,int success) {
    if(success)h->failures=0;
    else if(h->failures<255U && ++h->failures>=4U)h->pending=1;
}
/* Only completed radio requests with a missing peer acknowledgement count.
 * Numeric values are the STM32WB Zigbee SDK statuses, not application commands:
 * 0xe9 = MAC NO_ACK; 0xa7 = APS NO_ACK. CCA, routing, resource, security,
 * parameter and submission errors are deliberately neutral, not peer evidence. */
static inline int link_health_missing_ack(unsigned status) {
    return status==0xe9U || status==0xa7U;
}
static inline void link_health_confirm(LinkHealth *h,unsigned status) {
    if(status==0U)link_health_result(h,1);
    else if(link_health_missing_ack(status))link_health_result(h,0);
}
#endif
