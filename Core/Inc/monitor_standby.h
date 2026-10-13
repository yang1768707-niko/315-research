#ifndef MONITOR_STANDBY_H
#define MONITOR_STANDBY_H
#include <stdint.h>
typedef struct { uint8_t seen; uint32_t until; } MonitorStandby;
/* A bounded RTC-backed deadline; repeated OFF replies never extend it. */
static inline uint32_t monitor_standby_delay(MonitorStandby *s, uint32_t now, int *awake)
{
    if (!s->seen) { s->seen = 1; s->until = now + 600000U; }
    uint32_t remaining = s->until - now;
    if (s->seen == 1 && (int32_t)remaining > 0) {
        *awake = 1; return remaining < 3000U ? remaining : 3000U;
    }
    s->seen = 2; s->until = 0; *awake = 0; return 300000U;
}
static inline void monitor_standby_enabled(MonitorStandby *s) { s->seen = 2; s->until = 0; }
#endif
