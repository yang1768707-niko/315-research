#ifndef SCALAR_DELIVERY_H
#define SCALAR_DELIVERY_H
#include <stdint.h>
#include <stddef.h>
#define SCALAR_MANIFEST_CMD 0xA5U
#define SCALAR_MANIFEST_LEN 24U
#define SCALAR_ACK_KIND 6U
#define SCALAR_POLICY_VERSION 1U
#define SCALAR_ALL 0x0FU
/* Bits: temperature, magnetic XYZ together, digital SPL, analog SPL. */
static inline uint8_t scalar_expected_mask(uint8_t sensor_mask)
{ return 3U | ((sensor_mask & 2U)?4U:0U) | ((sensor_mask & 4U)?8U:0U); }
static inline uint32_t scalar_u32(const uint8_t *p)
{ return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static inline void scalar_w32(uint8_t *p,uint32_t value)
{ for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(value>>(8*i)); }
static inline uint32_t scalar_crc(const uint8_t *p,size_t length)
{
    uint32_t crc=UINT32_MAX;
    while(length--) {crc^=*p++;for(unsigned b=0;b<8;b++)crc=(crc>>1)^((crc&1)?UINT32_C(0xedb88320):0);}
    return crc^UINT32_MAX;
}
/* Checksum includes item identity, batch and exact legacy 0x86..0x89 bytes. */
static inline uint32_t scalar_value_crc(unsigned item,const uint8_t *payload)
{
    uint8_t bytes[11];unsigned size=item==1U?10U:6U;
    bytes[0]=(uint8_t)(0x86U+item);
    for(unsigned i=0;i<size;i++)bytes[1+i]=payload[i];
    return scalar_crc(bytes,size+1U);
}
static inline int scalar_manifest_valid(const uint8_t *p,size_t length)
{
    return p && length==SCALAR_MANIFEST_LEN && p[0]==2U && scalar_u32(p+1)!=0U &&
           p[5]==SCALAR_POLICY_VERSION && !(p[6]&~SCALAR_ALL) && !(p[7]&~SCALAR_ALL);
}
#endif
