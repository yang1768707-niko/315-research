#ifndef CONTROL_SYNC_H
#define CONTROL_SYNC_H
#include <stdint.h>
/* FC10 0xA4: version, flags, applied configuration fingerprint[4], power, state.
 * An observation, never a configuration request. Only mismatches need a push. */
#define CONTROL_STATUS_CMD 0xA4U
#define CONTROL_STATUS_VERSION 1U
#define CONTROL_STATUS_SIZE 8U
#define CONTROL_FINAL_SIZE 12U
/* Bit 3 is SAVE_FAILED only in the extended final response. */
#define CONTROL_SAVE_FAILED 0x08U
#define CONTROL_VALID 0x01U
#define CONTROL_ENABLED 0x02U
#define CONTROL_APPLIED 0x04U
#define CONTROL_NEEDS_SCHEDULE 0x08U
#define CONTROL_UPLOAD_CONFIG_WINDOW 0x10U
/* FC10 and UART command for restoring a deadline without changing sampling policy. */
#define CONTROL_RESUME_SCHEDULE_CMD 0x19U
#define CONTROL_RESUME_SCHEDULE_SIZE 9U
#define CONTROL_CONFIG_SIZE 29U
/* Recovery-only extension: future deadline (Unix seconds, also dedup token),
 * then delay milliseconds. Neither field belongs to the policy fingerprint. */
#define CONTROL_RESUME_CONFIG_SIZE 37U
/* UART-only envelope: short address[2], config[29 or 37], delivery mode.
 * The mode is never sent over Zigbee and is excluded from the fingerprint. */
#define CONTROL_CONFIG_SYNC_CMD 0x16U
#define CONTROL_DELIVERY_CACHE 0U
#define CONTROL_DELIVERY_WINDOW 1U
#define CONTROL_DELIVERY_URGENT 2U
#define CONTROL_RECEIVE_WINDOW_MS 8000U
static inline uint32_t control_fingerprint(const uint8_t *p, unsigned n)
{
    uint32_t hash = UINT32_C(2166136261);
    while (n--) { hash ^= *p++; hash *= UINT32_C(16777619); }
    return hash;
}
static inline uint32_t control_read_hash(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int control_status_valid(const uint8_t *p, unsigned n)
{
    return p && n == CONTROL_STATUS_SIZE && p[0] == CONTROL_STATUS_VERSION &&
           !(p[1] & (uint8_t)~31U) && p[7] <= 5U;
}
static inline int control_final_status_valid(const uint8_t *p,unsigned n)
{
    return n==CONTROL_FINAL_SIZE && control_status_valid(p,CONTROL_STATUS_SIZE) &&
           ((p[1]&(CONTROL_APPLIED|CONTROL_SAVE_FAILED))==CONTROL_APPLIED ||
            (p[1]&(CONTROL_APPLIED|CONTROL_SAVE_FAILED))==CONTROL_SAVE_FAILED);
}
/* Account for coordinator caching/retries. Missed slots are skipped, never
 * turned into an immediate capture or a catch-up burst. */
static inline uint32_t control_resume_delay(uint32_t delay,uint32_t age,uint32_t period)
{
    if (!period || !delay || delay>period) return 0;
    return age<delay ? delay-age : period-(age-delay)%period;
}
#endif
