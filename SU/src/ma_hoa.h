#ifndef MA_HOA_H
#define MA_HOA_H

#include <Arduino.h>
#include "mbedtls/gcm.h"

// =====================================================
// AES-128-GCM — NATIVE 8-FRAME + CIPHERTEXT/TAG FEC
//
// VOICE:
//   AAD        = 8B header
//   plaintext  = 160B Speex
//   ciphertext = 160B
//   tag        = 8B
//
// FEC:
//   parity input = 168B
//                = XOR(
//                    VOICE ciphertext 160B
//                    + original VOICE GCM tag 8B
//                  )
//
//   FEC parity 168B cũng được AES-GCM mã hóa/xác thực.
//   FEC packet có tag riêng 8B.
//
// IV 96-bit:
//   SESSION_ID64 || NONCE_SEQ32
//
// Domain separation:
//   VOICE: NONCE_SEQ32 = voice_seq
//   FEC:   NONCE_SEQ32 = 0x80000000 | group_start_seq
//
// Không reuse IV giữa VOICE và FEC.
// =====================================================

static const unsigned char KHOA_BIMAT_AES[16] =
{
    0x2B, 0x7E, 0x15, 0x16,
    0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88,
    0x09, 0xCF, 0x4F, 0x3C
};

#define AES_GCM_AAD_LEN            8
#define AES_GCM_VOICE_LEN        160
#define AES_GCM_FEC_LEN          168
#define AES_GCM_TAG_LEN            8
#define AES_GCM_IV_LEN            12

#define FEC_IV_DOMAIN 0x80000000UL


inline void Tao_IV_GCM(
    uint8_t iv[AES_GCM_IV_LEN],
    uint64_t session_id,
    uint32_t nonce_seq)
{
    iv[0]  = (session_id >> 56) & 0xFF;
    iv[1]  = (session_id >> 48) & 0xFF;
    iv[2]  = (session_id >> 40) & 0xFF;
    iv[3]  = (session_id >> 32) & 0xFF;
    iv[4]  = (session_id >> 24) & 0xFF;
    iv[5]  = (session_id >> 16) & 0xFF;
    iv[6]  = (session_id >> 8)  & 0xFF;
    iv[7]  =  session_id        & 0xFF;

    iv[8]  = (nonce_seq >> 24) & 0xFF;
    iv[9]  = (nonce_seq >> 16) & 0xFF;
    iv[10] = (nonce_seq >> 8)  & 0xFF;
    iv[11] =  nonce_seq        & 0xFF;
}


// =====================================================
// CORE GCM VỚI PAYLOAD LENGTH BIẾN ĐỔI
// =====================================================

inline void MaHoa_GCM_Len(
    uint8_t *header,
    const uint8_t *payload_vao,
    uint8_t *payload_ra,
    uint8_t *auth_tag,
    size_t payload_len,
    uint64_t session_id,
    uint32_t nonce_seq)
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
        nonce_seq
    );

    mbedtls_gcm_crypt_and_tag(
        &gcm,
        MBEDTLS_GCM_ENCRYPT,
        payload_len,
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


inline bool GiaiMa_GCM_Len(
    uint8_t *header,
    const uint8_t *payload_vao,
    uint8_t *payload_ra,
    const uint8_t *auth_tag,
    size_t payload_len,
    uint64_t session_id,
    uint32_t nonce_seq)
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
        nonce_seq
    );

    int ket_qua =
        mbedtls_gcm_auth_decrypt(
            &gcm,
            payload_len,
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

    return ket_qua == 0;
}


// =====================================================
// VOICE WRAPPER — 160B
// =====================================================

inline void MaHoa_GCM(
    uint8_t *header,
    const uint8_t *payload_vao,
    uint8_t *payload_ra,
    uint8_t *auth_tag,
    uint64_t session_id,
    uint32_t nonce_seq)
{
    MaHoa_GCM_Len(
        header,
        payload_vao,
        payload_ra,
        auth_tag,
        AES_GCM_VOICE_LEN,
        session_id,
        nonce_seq
    );
}


inline bool GiaiMa_GCM(
    uint8_t *header,
    const uint8_t *payload_vao,
    uint8_t *payload_ra,
    const uint8_t *auth_tag,
    uint64_t session_id,
    uint32_t nonce_seq)
{
    return GiaiMa_GCM_Len(
        header,
        payload_vao,
        payload_ra,
        auth_tag,
        AES_GCM_VOICE_LEN,
        session_id,
        nonce_seq
    );
}


// =====================================================
// FEC WRAPPER — 168B
// =====================================================

inline void MaHoa_GCM_FEC(
    uint8_t *header,
    const uint8_t *parity_vao_168,
    uint8_t *parity_ma_hoa_168,
    uint8_t *fec_auth_tag,
    uint64_t session_id,
    uint32_t fec_nonce)
{
    MaHoa_GCM_Len(
        header,
        parity_vao_168,
        parity_ma_hoa_168,
        fec_auth_tag,
        AES_GCM_FEC_LEN,
        session_id,
        fec_nonce
    );
}


inline bool GiaiMa_GCM_FEC(
    uint8_t *header,
    const uint8_t *parity_ma_hoa_168,
    uint8_t *parity_sach_168,
    const uint8_t *fec_auth_tag,
    uint64_t session_id,
    uint32_t fec_nonce)
{
    return GiaiMa_GCM_Len(
        header,
        parity_ma_hoa_168,
        parity_sach_168,
        fec_auth_tag,
        AES_GCM_FEC_LEN,
        session_id,
        fec_nonce
    );
}

#endif
