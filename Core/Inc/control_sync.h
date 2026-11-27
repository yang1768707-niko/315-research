#ifndef CONTROL_SYNC_H
#define CONTROL_SYNC_H
#include <stdint.h>
/* FC10 0xA4: version, flags, applied configuration fingerprint[4], power, state.
 * An observation, never a configuration request. Only mismatches need a push. */
#define CONTROL_STATUS_CMD 0xA4U
#define CONTROL_STATUS_VERSION 1U
#define CONTROL_STATUS_SIZE 8U
#define CONTROL_FINAL_SIZE 12U /* 最终配置回复追加 revision:u32 */
#define CONTROL_VALID 0x01U
#define CONTROL_ENABLED 0x02U
#define CONTROL_APPLIED 0x04U
#define CONTROL_SAVE_FAILED 0x08U /* 旧配置命令的最终失败回复，不能同时置 APPLIED */
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
    return p && (n == CONTROL_STATUS_SIZE || (n == CONTROL_FINAL_SIZE &&
           (p[1] & (CONTROL_APPLIED|CONTROL_SAVE_FAILED)))) && p[0] == CONTROL_STATUS_VERSION &&
           !(p[1] & (uint8_t)~15U) &&
           (p[1] & (CONTROL_APPLIED|CONTROL_SAVE_FAILED))!=(CONTROL_APPLIED|CONTROL_SAVE_FAILED) && p[7] <= 5U;
}
#endif
