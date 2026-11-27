#ifndef MONITOR_STANDBY_H
#define MONITOR_STANDBY_H
#include <stdint.h>
typedef struct { uint8_t seen; uint32_t until, random; } MonitorStandby;
/* Seed once from hardware RNG + unique ID, then jitter every interval. No RNG
 * peripheral access is required from timer/low-power callbacks. */
static inline uint32_t monitor_standby_random(MonitorStandby *s, uint32_t low, uint32_t high)
{
    uint32_t x=s->random ? s->random : UINT32_C(0x6d2b79f5);
    x^=x<<13; x^=x>>17; x^=x<<5; s->random=x;
    return low+x%(high-low+1U);
}
/* A bounded RTC-backed deadline; repeated OFF replies never extend it. */
static inline uint32_t monitor_standby_delay(MonitorStandby *s, uint32_t now, int *awake)
{
    if (!s->seen) { s->seen = 1; s->until = now + 600000U; }
    uint32_t remaining = s->until - now;
    if (s->seen == 1 && (int32_t)remaining > 0) {
        uint32_t delay=monitor_standby_random(s,3000U,10000U);
        *awake = 1; return remaining < delay ? remaining : delay;
    }
    s->seen = 2; s->until = 0; *awake = 0;
    return monitor_standby_random(s,285000U,315000U);
}
static inline void monitor_standby_enabled(MonitorStandby *s) { s->seen = 2; s->until = 0; }
#endif
