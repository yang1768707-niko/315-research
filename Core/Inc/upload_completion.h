#ifndef UPLOAD_COMPLETION_H
#define UPLOAD_COMPLETION_H
#include "vibration_wire.h"
#include "scalar_delivery.h"
#include "upload_lease.h"

/* Opt-in extensions: old 77-byte BATCH / 10-byte lease frames stay valid.
 * The coordinator still forwards 0x92/0x95 and the existing 17-byte 0x0D. */
#define UPLOAD_COMPLETION_VERSION 1U
#define UPLOAD_COMPLETION_KIND 8U
#define UPLOAD_COMPLETION_BATCH_LEN (V2_BATCH_LEN+2U)
#define UPLOAD_COMPLETION_LEASE_LEN 11U
#define UPLOAD_IDENTITY_LEN 17U

/* Feature words are excluded: they change from deferred to computed in Flash.
 * Identity still binds batch, rate, point count, waveform CRC, scalar policy
 * and every captured scalar checksum, plus whether raw upload was required. */
static inline uint32_t upload_completion_fingerprint(const uint8_t *identity,
    const uint8_t manifest[SCALAR_MANIFEST_LEN],unsigned raw_required)
{
    uint8_t proof[UPLOAD_IDENTITY_LEN+SCALAR_MANIFEST_LEN+1U];
    memcpy(proof,identity,UPLOAD_IDENTITY_LEN);
    memcpy(proof+UPLOAD_IDENTITY_LEN,manifest,SCALAR_MANIFEST_LEN);
    proof[sizeof(proof)-1U]=(uint8_t)raw_required;
    return v2_crc(proof,sizeof(proof));
}
static inline int upload_completion_matches(const uint8_t *p,size_t n,
    const uint8_t descriptor[V2_BATCH_LEN],const uint8_t manifest[SCALAR_MANIFEST_LEN],
    unsigned raw_required,const UploadLease *lease,int requesting_batch)
{
    if(n!=V2_ACK_LEN || p[0]!=V2_VERSION || p[1]!=UPLOAD_COMPLETION_KIND ||
       p[8]!=V2_OK || v2_u16(p+6)!=0 || v2_u32(p+2)!=v2_u32(descriptor+1) ||
       v2_u32(p+13)!=upload_completion_fingerprint(descriptor,manifest,raw_required))return 0;
    uint32_t nonce=v2_u32(p+9);
    return nonce ? lease->waiting && lease->batch==v2_u32(p+2) && lease->nonce==nonce
                 : requesting_batch;
}
#endif
