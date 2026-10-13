#ifndef VIBRATION_CODEC_H
#define VIBRATION_CODEC_H
#include "vibration_wire.h"

/* RAW: XYZ int16 LE. RICE: k[3], first XYZ[6], then axis-major deltas.
 * Bits are MSB first, unary = q zeroes followed by one, then k remainder bits.
 * Delta uses int32 (not int16); zigzag maps [-65535,65535] losslessly. */
static inline int32_t v2_sample(const uint8_t *p) {
    uint32_t n=v2_u16(p); return n>=32768U?(int32_t)n-65536:(int32_t)n;
}
static inline uint32_t v2_delta(const uint8_t *raw, unsigned i, unsigned axis) {
    int32_t d=v2_sample(raw+(i*3+axis)*2)-v2_sample(raw+((i-1)*3+axis)*2);
    return d<0?(uint32_t)(-d)*2-1:(uint32_t)d*2;
}
static inline void v2_putbit(uint8_t *p, size_t *bit, unsigned value) {
    if(value) p[*bit/8]|=(uint8_t)(1U<<(7-(*bit%8)));
    ++*bit;
}
static inline int v2_getbit(const uint8_t *p, size_t bytes, size_t *bit) {
    if(*bit>=bytes*8) return -1;
    int n=(p[*bit/8]>>(7-(*bit%8)))&1; ++*bit; return n;
}
static inline size_t v2_encode(const uint8_t *raw, unsigned points, uint8_t *out,
                                size_t capacity, uint8_t *codec) {
    size_t raw_bytes=(size_t)points*6, bits=72;
    unsigned ks[3]={0,0,0};
    if(!raw || !out || !codec || !points || points>V2_POINTS || capacity<raw_bytes) return 0;
    for(unsigned a=0;a<3;a++) {
        uint32_t best=UINT32_MAX;
        for(unsigned k=0;k<=16;k++) {
            uint32_t cost=0;
            for(unsigned i=1;i<points;i++) cost+=(v2_delta(raw,i,a)>>k)+1+k;
            if(cost<best) {best=cost;ks[a]=k;}
        }
        bits+=best;
    }
    if((bits+7)/8>=raw_bytes) {memcpy(out,raw,raw_bytes);*codec=V2_RAW;return raw_bytes;}
    size_t bytes=(bits+7)/8, at=72;
    memset(out,0,bytes);
    for(unsigned a=0;a<3;a++) out[a]=(uint8_t)ks[a];
    memcpy(out+3,raw,6);
    for(unsigned a=0;a<3;a++) for(unsigned i=1;i<points;i++) {
        uint32_t u=v2_delta(raw,i,a), q=u>>ks[a];
        while(q--) v2_putbit(out,&at,0);
        v2_putbit(out,&at,1);
        for(unsigned j=ks[a];j>0;j--) v2_putbit(out,&at,(u>>(j-1))&1);
    }
    *codec=V2_RICE;return bytes;
}
static inline int v2_decode(uint8_t codec, const uint8_t *in, size_t bytes,
                             unsigned points, uint8_t *raw, size_t capacity) {
    if(!in || !raw || !points || points>V2_POINTS || capacity<(size_t)points*6) return 0;
    if(codec==V2_RAW) {if(bytes!=(size_t)points*6)return 0;memcpy(raw,in,bytes);return 1;}
    if(codec!=V2_RICE || bytes<9 || bytes>V2_RAW_BYTES) return 0;
    size_t at=72;
    memcpy(raw,in+3,6);
    for(unsigned a=0;a<3;a++) {
        unsigned k=in[a];
        if(k>16)return 0;
        int32_t previous=v2_sample(raw+a*2);
        for(unsigned i=1;i<points;i++) {
            uint32_t q=0,u=0;
            int bit;
            do {
                bit=v2_getbit(in,bytes,&at);
                if(bit<0)return 0;
                if(bit==0 && ++q>(131070U>>k))return 0;
            } while(bit==0);
            for(unsigned j=0;j<k;j++) {
                bit=v2_getbit(in,bytes,&at);if(bit<0)return 0;u=(u<<1)|(unsigned)bit;
            }
            u|=q<<k;
            if(u>131070U)return 0;
            int32_t d=(u&1)?-(int32_t)((u+1)/2):(int32_t)(u/2);
            int32_t value=previous+d;
            if(value<-32768 || value>32767)return 0;
            v2_w16(raw+(i*3+a)*2,(uint16_t)value);previous=value;
        }
    }
    /* Reject trailing bytes and nonzero padding: one canonical block length. */
    if((at+7)/8!=bytes)return 0;
    while(at<bytes*8) if(v2_getbit(in,bytes,&at)!=0)return 0;
    return 1;
}
#endif

