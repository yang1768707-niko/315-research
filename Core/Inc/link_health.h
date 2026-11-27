#ifndef LINK_HEALTH_H
#define LINK_HEALTH_H
#include <stdint.h>
typedef struct {uint32_t deadline;uint8_t failures,pending,verifying;} LinkHealth;
static inline void link_health_result(LinkHealth *h,int success) {
    if(success)h->failures=0;
    else if(h->failures<255U && ++h->failures>=4U)h->pending=1;
}
#endif
