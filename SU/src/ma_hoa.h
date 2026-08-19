#ifndef MA_HOA_H
#define MA_HOA_H

#include <Arduino.h>
#include "mbedtls/gcm.h"

// =====================================================
// AES-128-GCM
//
// 4 FRAME / VOICE PACKET:
//
// AAD       = 8 byte header
// PLAINTEXT = 80 byte = 4 x 20B Speex
// TAG       = 8 byte
//
// IV 96-bit:
// SESSION_ID 64-bit || SEQ32
// =====================================================

static const unsigned char KHOA_BIMAT_AES[16] =
{
    0x2B, 0x7E, 0x15, 0x16,
    0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88,
    0x09, 0xCF, 0x4F, 0x3C
};

#define AES_GCM_AAD_LEN       8
#define AES_GCM_VOICE_LEN    80
#define AES_GCM_TAG_LEN       8
#define AES_GCM_IV_LEN       12


inline void Tao_IV_GCM(
    uint8_t iv[AES_GCM_IV_LEN],
    uint64_t session_id,
    uint32_t seq)
{
    iv[0]  = (session_id >> 56) & 0xFF;
    iv[1]  = (session_id >> 48) & 0xFF;
    iv[2]  = (session_id >> 40) & 0xFF;
    iv[3]  = (session_id >> 32) & 0xFF;
    iv[4]  = (session_id >> 24) & 0xFF;
    iv[5]  = (session_id >> 16) & 0xFF;
    iv[6]  = (session_id >> 8)  & 0xFF;
    iv[7]  =  session_id        & 0xFF;

    iv[8]  = (seq >> 24) & 0xFF;
    iv[9]  = (seq >> 16) & 0xFF;
    iv[10] = (seq >> 8)  & 0xFF;
    iv[11] =  seq        & 0xFF;
}


inline void MaHoa_GCM(
    uint8_t *header,
    uint8_t *payload_vao,
    uint8_t *payload_ra,
    uint8_t *auth_tag,
    uint64_t session_id,
    uint32_t seq)
{
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);

    mbedtls_gcm_setkey(
        &gcm,
        MBEDTLS_CIPHER_ID_AES,
        KHOA_BIMAT_AES,
        128
    );

    uint8_t iv[AES_GCM_IV_LEN];

    Tao_IV_GCM(
        iv,
        session_id,
        seq
    );

    mbedtls_gcm_crypt_and_tag(
        &gcm,
        MBEDTLS_GCM_ENCRYPT,

        AES_GCM_VOICE_LEN,

        iv,
        AES_GCM_IV_LEN,

        header,
        AES_GCM_AAD_LEN,

        payload_vao,
        payload_ra,

        AES_GCM_TAG_LEN,
        auth_tag
    );

    mbedtls_gcm_free(&gcm);
}


inline bool GiaiMa_GCM(
    uint8_t *header,
    uint8_t *payload_vao,
    uint8_t *payload_ra,
    uint8_t *auth_tag,
    uint64_t session_id,
    uint32_t seq)
{
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);

    mbedtls_gcm_setkey(
        &gcm,
        MBEDTLS_CIPHER_ID_AES,
        KHOA_BIMAT_AES,
        128
    );

    uint8_t iv[AES_GCM_IV_LEN];

    Tao_IV_GCM(
        iv,
        session_id,
        seq
    );

    int ket_qua =
        mbedtls_gcm_auth_decrypt(
            &gcm,

            AES_GCM_VOICE_LEN,

            iv,
            AES_GCM_IV_LEN,

            header,
            AES_GCM_AAD_LEN,

            auth_tag,
            AES_GCM_TAG_LEN,

            payload_vao,
            payload_ra
        );

    mbedtls_gcm_free(&gcm);

    return (
        ket_qua
        == 0
    );
}

#endif
