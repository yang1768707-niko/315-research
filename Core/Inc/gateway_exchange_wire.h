#ifndef GATEWAY_EXCHANGE_WIRE_H
#define GATEWAY_EXCHANGE_WIRE_H

#include <stdint.h>

#define GX_CONTROL 0xE0U
#define GX_DATA 0xE1U
#define GX_UART_DOWN 0x1AU
#define GX_UART_UP 0xACU
#define GX_READY 0x90U
/* Context is a separate event so the 21-byte READY and 23-byte decision stay unchanged. */
#define GX_CONTEXT 0x91U
#define GX_SAMPLING 0x26U
#define GX_EVENT 4U
#define GX_RESPONSE 2U
#define GX_COMMAND 1U
#define GX_HEADER 8U
#define GX_KEY_SIZE 12U
#define GX_READY_SIZE 13U
#define GX_DECISION_SIZE 15U
#define GX_CONTEXT_SIZE 17U
#define GX_SCALAR_SIZE 24U
#define GX_SAMPLING_SIZE 41U
/* Final sampling reply: token, batch, result, resulting revision. */
#define GX_SAMPLING_RESULT_SIZE 13U
#define GX_COMMAND_FRAME_SIZE (GX_HEADER+GX_SAMPLING_SIZE)
#define GX_INVALID_VALUE 3U
#define GX_BUSY 1U
#define GX_SAVE_FAILED 7U
#define GX_SLEEP_ALLOWED 1U
#define GX_COMMANDS_PENDING 2U
#define GX_NOT_READY 5U
#define GX_HARDWARE_ERROR 6U
#define GX_CONTEXT_COMMANDS 1U
#define GX_CONTEXT_SCALAR 2U
#define GX_CONTEXT_FINAL 4U

static inline uint16_t gx_get16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t gx_get32(const uint8_t *p)
{
    return (uint32_t)gx_get16(p) | ((uint32_t)gx_get16(p+2) << 16);
}

static inline void gx_put16(uint8_t *p, uint16_t value)
{
    p[0]=(uint8_t)value;p[1]=(uint8_t)(value>>8);
}

static inline void gx_put32(uint8_t *p, uint32_t value)
{
    gx_put16(p,(uint16_t)value);gx_put16(p+2,(uint16_t)(value>>16));
}

static inline void gx_header(uint8_t *p, uint8_t kind, uint8_t code, uint16_t seq, uint16_t size)
{
    p[0]=1;p[1]=kind;p[2]=code;p[3]=0;
    gx_put16(p+4,seq);gx_put16(p+6,size);
}

static inline int gx_frame(const uint8_t *p, unsigned n, uint8_t kind, uint8_t code, unsigned size)
{
    return p && n==GX_HEADER+size && p[0]==1 && p[1]==kind &&
           p[2]==code && p[3]==0 && gx_get16(p+6)==size;
}

static inline int gx_newer(uint16_t candidate, uint16_t last)
{
    uint16_t distance=(uint16_t)(candidate-last);
    return distance && distance<32768U;
}

static inline int gx_decision_valid(uint16_t sequence,uint16_t previous,uint8_t result,uint16_t mask)
{
    return sequence && (!previous || gx_newer(sequence,previous)) &&
           result<=GX_SAVE_FAILED && mask<=GX_COMMANDS_PENDING && (!result || !mask);
}

#endif
