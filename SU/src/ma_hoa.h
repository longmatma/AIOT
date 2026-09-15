#ifndef MA_HOA_H
#define MA_HOA_H

#include <Arduino.h>
#include <string.h>
#include "mbedtls/gcm.h"
#include "mbedtls/md.h"
#include "khoa_goc_local.h"

// =====================================================
// ADAPTIVE SECURITY - STAGE 1
// KHONG AI - CHUA TU DONG DOI PROFILE
//
// Muc tieu cua Stage 1:
//   1) Tao nen tang 3 SECURITY PROFILE.
//   2) Giu NGUYEN packet format hien tai:
//        VOICE = 176B
//        FEC   = 184B
//        TAG   = 8B
//   3) Dung 1 ROOT KEY 256-bit rieng cua he thong.
//   4) Tu dong sinh epoch key theo SESSION + PROFILE + EPOCH.
//   5) Kiem tra day du return code AES-GCM (fail-closed).
//
// Stage 1 MAC DINH = NORMAL de test an toan.
// Chua thay doi profile qua radio/rBS.
// Sau khi Stage 1 PASS moi lam Stage 2: rBS gui profile theo SESSION.
// =====================================================

#define AES_GCM_AAD_LEN              8
#define AES_GCM_VOICE_LEN          160
#define AES_GCM_FEC_LEN            168
#define AES_GCM_TAG_LEN              8
#define AES_GCM_IV_LEN              12

#define FEC_IV_DOMAIN 0x80000000UL

// Profile IDs se giu co dinh tu day ve sau.
#define BAOMAT_PROFILE_NORMAL         0U
#define BAOMAT_PROFILE_STRONG         1U
#define BAOMAT_PROFILE_HIGH           2U

// Stage 1: khoa profile NORMAL de khong thay doi giao thuc dieu khien.
#define BAOMAT_PROFILE_MAC_DINH       BAOMAT_PROFILE_NORMAL

#if VEDC_ROOT_KEY_LEN < 32
#error "VEDC_ROOT_KEY_LEN phai >= 32 byte de ho tro AES-256"
#endif


// =====================================================
// PROFILE STATE
// =====================================================

inline uint8_t &BaoMat_Profile_Storage()
{
    static uint8_t profile =
        BAOMAT_PROFILE_MAC_DINH;

    return profile;
}


inline bool Profile_BaoMat_HopLe(
    uint8_t profile)
{
    return (
        profile == BAOMAT_PROFILE_NORMAL
        ||
        profile == BAOMAT_PROFILE_STRONG
        ||
        profile == BAOMAT_PROFILE_HIGH
    );
}


inline bool Dat_Profile_BaoMat(
    uint8_t profile)
{
    if (!Profile_BaoMat_HopLe(profile))
    {
        return false;
    }

    BaoMat_Profile_Storage() =
        profile;

    return true;
}


inline uint8_t Lay_Profile_BaoMat()
{
    return BaoMat_Profile_Storage();
}


inline const char *Ten_Profile_BaoMat(
    uint8_t profile)
{
    switch (profile)
    {
        case BAOMAT_PROFILE_NORMAL:
            return "NORMAL";

        case BAOMAT_PROFILE_STRONG:
            return "STRONG";

        case BAOMAT_PROFILE_HIGH:
            return "HIGH";

        default:
            return "INVALID";
    }
}


inline uint16_t So_Bit_AES_Theo_Profile(
    uint8_t profile)
{
    if (profile == BAOMAT_PROFILE_NORMAL)
    {
        return 128U;
    }

    return 256U;
}


// Chu ky epoch deu la boi so cua FEC group 8 packet.
// Nhu vay 1 FEC group khong bi cat ngang 2 epoch key.
inline uint32_t ChuKy_DoiKhoa_Theo_Profile(
    uint8_t profile)
{
    switch (profile)
    {
        case BAOMAT_PROFILE_NORMAL:
            return 256UL;

        case BAOMAT_PROFILE_STRONG:
            return 128UL;

        case BAOMAT_PROFILE_HIGH:
            return 64UL;

        default:
            return 128UL;
    }
}


// =====================================================
// XOA BO NHO NHAY CAM
// =====================================================

inline void Xoa_BoNho_NhayCam(
    void *ptr,
    size_t len)
{
    volatile uint8_t *p =
        reinterpret_cast<volatile uint8_t *>(ptr);

    while (len--)
    {
        *p++ = 0;
    }
}


// =====================================================
// IV 96-bit GIU NGUYEN KIEN TRUC HIEN TAI:
//
//   SESSION_ID64 || NONCE_SEQ32
//
// VOICE:
//   nonce_seq = seq
//
// FEC:
//   nonce_seq = 0x80000000 | group_start_seq
// =====================================================

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
// DERIVE EPOCH KEY BANG HMAC-SHA256
//
// K_epoch = HMAC-SHA256(
//     ROOT_KEY,
//     "VEDC-ADAPT-AES-V1"
//     || SESSION_ID64
//     || PROFILE8
//     || EPOCH32
// )
//
// NORMAL:
//   dung 16 byte dau -> AES-128
//
// STRONG/HIGH:
//   dung 32 byte -> AES-256
//
// FEC nonce co bit31=1, nen khi tinh epoch ta bo bit domain:
//   seq_thuan = nonce_seq & 0x7FFFFFFF
// =====================================================

inline bool Lay_Khoa_Adaptive(
    uint64_t session_id,
    uint32_t nonce_seq,
    uint8_t khoa_32[32],
    uint16_t &key_bits)
{
    if (
        session_id == 0
        ||
        khoa_32 == nullptr
    )
    {
        return false;
    }

    const uint8_t profile =
        Lay_Profile_BaoMat();

    if (!Profile_BaoMat_HopLe(profile))
    {
        memset(khoa_32, 0, 32);
        return false;
    }

    const uint32_t seq_thuan =
        nonce_seq
        &
        0x7FFFFFFFUL;

    const uint32_t chu_ky =
        ChuKy_DoiKhoa_Theo_Profile(
            profile
        );

    const uint32_t epoch =
        seq_thuan
        /
        chu_ky;

    key_bits =
        So_Bit_AES_Theo_Profile(
            profile
        );

    // Cache epoch hien tai de khong HMAC lai moi packet.
    static uint64_t cache_session = 0;
    static uint32_t cache_epoch = 0xFFFFFFFFUL;
    static uint8_t cache_profile = 0xFFU;
    static uint16_t cache_key_bits = 0;
    static uint8_t cache_key[32] = {0};
    static bool cache_ok = false;

    if (
        cache_ok
        &&
        cache_session == session_id
        &&
        cache_epoch == epoch
        &&
        cache_profile == profile
        &&
        cache_key_bits == key_bits
    )
    {
        memcpy(
            khoa_32,
            cache_key,
            sizeof(cache_key)
        );

        return true;
    }

    static const uint8_t domain[] =
    {
        'V','E','D','C','-','A','D','A','P','T',
        '-','A','E','S','-','V','1'
    };

    uint8_t context[
        sizeof(domain)
        +
        8
        +
        1
        +
        4
    ];

    size_t o = 0;

    memcpy(
        &context[o],
        domain,
        sizeof(domain)
    );

    o += sizeof(domain);

    context[o++] = (session_id >> 56) & 0xFF;
    context[o++] = (session_id >> 48) & 0xFF;
    context[o++] = (session_id >> 40) & 0xFF;
    context[o++] = (session_id >> 32) & 0xFF;
    context[o++] = (session_id >> 24) & 0xFF;
    context[o++] = (session_id >> 16) & 0xFF;
    context[o++] = (session_id >> 8)  & 0xFF;
    context[o++] =  session_id        & 0xFF;

    context[o++] =
        profile;

    context[o++] = (epoch >> 24) & 0xFF;
    context[o++] = (epoch >> 16) & 0xFF;
    context[o++] = (epoch >> 8)  & 0xFF;
    context[o++] =  epoch        & 0xFF;

    const mbedtls_md_info_t *md_info =
        mbedtls_md_info_from_type(
            MBEDTLS_MD_SHA256
        );

    if (md_info == nullptr)
    {
        memset(khoa_32, 0, 32);

        Xoa_BoNho_NhayCam(
            context,
            sizeof(context)
        );

        return false;
    }

    uint8_t digest[32];

    const int rc =
        mbedtls_md_hmac(
            md_info,
            VEDC_ROOT_KEY,
            VEDC_ROOT_KEY_LEN,
            context,
            sizeof(context),
            digest
        );

    Xoa_BoNho_NhayCam(
        context,
        sizeof(context)
    );

    if (rc != 0)
    {
        memset(khoa_32, 0, 32);

        Xoa_BoNho_NhayCam(
            digest,
            sizeof(digest)
        );

        return false;
    }

    memcpy(
        cache_key,
        digest,
        sizeof(cache_key)
    );

    cache_session =
        session_id;

    cache_epoch =
        epoch;

    cache_profile =
        profile;

    cache_key_bits =
        key_bits;

    cache_ok =
        true;

    memcpy(
        khoa_32,
        cache_key,
        sizeof(cache_key)
    );

    Xoa_BoNho_NhayCam(
        digest,
        sizeof(digest)
    );

    return true;
}


// =====================================================
// AES-GCM CORE - FAIL CLOSED
// =====================================================

inline bool MaHoa_GCM_Len(
    const uint8_t *header,
    const uint8_t *payload_vao,
    uint8_t *payload_ra,
    uint8_t *auth_tag,
    size_t payload_len,
    uint64_t session_id,
    uint32_t nonce_seq)
{
    if (
        header == nullptr
        ||
        payload_vao == nullptr
        ||
        payload_ra == nullptr
        ||
        auth_tag == nullptr
        ||
        payload_len == 0
    )
    {
        return false;
    }

    uint8_t khoa[32];
    uint16_t key_bits = 0;

    if (
        !Lay_Khoa_Adaptive(
            session_id,
            nonce_seq,
            khoa,
            key_bits
        )
    )
    {
        memset(payload_ra, 0, payload_len);
        memset(auth_tag, 0, AES_GCM_TAG_LEN);
        return false;
    }

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);

    int rc =
        mbedtls_gcm_setkey(
            &gcm,
            MBEDTLS_CIPHER_ID_AES,
            khoa,
            key_bits
        );

    Xoa_BoNho_NhayCam(
        khoa,
        sizeof(khoa)
    );

    if (rc != 0)
    {
        mbedtls_gcm_free(&gcm);
        memset(payload_ra, 0, payload_len);
        memset(auth_tag, 0, AES_GCM_TAG_LEN);
        return false;
    }

    uint8_t iv[AES_GCM_IV_LEN];

    Tao_IV_GCM(
        iv,
        session_id,
        nonce_seq
    );

    rc =
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

    Xoa_BoNho_NhayCam(
        iv,
        sizeof(iv)
    );

    if (rc != 0)
    {
        memset(payload_ra, 0, payload_len);
        memset(auth_tag, 0, AES_GCM_TAG_LEN);
        return false;
    }

    return true;
}


inline bool GiaiMa_GCM_Len(
    const uint8_t *header,
    const uint8_t *payload_vao,
    uint8_t *payload_ra,
    const uint8_t *auth_tag,
    size_t payload_len,
    uint64_t session_id,
    uint32_t nonce_seq)
{
    if (
        header == nullptr
        ||
        payload_vao == nullptr
        ||
        payload_ra == nullptr
        ||
        auth_tag == nullptr
        ||
        payload_len == 0
    )
    {
        return false;
    }

    uint8_t khoa[32];
    uint16_t key_bits = 0;

    if (
        !Lay_Khoa_Adaptive(
            session_id,
            nonce_seq,
            khoa,
            key_bits
        )
    )
    {
        memset(payload_ra, 0, payload_len);
        return false;
    }

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);

    int rc =
        mbedtls_gcm_setkey(
            &gcm,
            MBEDTLS_CIPHER_ID_AES,
            khoa,
            key_bits
        );

    Xoa_BoNho_NhayCam(
        khoa,
        sizeof(khoa)
    );

    if (rc != 0)
    {
        mbedtls_gcm_free(&gcm);
        memset(payload_ra, 0, payload_len);
        return false;
    }

    uint8_t iv[AES_GCM_IV_LEN];

    Tao_IV_GCM(
        iv,
        session_id,
        nonce_seq
    );

    rc =
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

    Xoa_BoNho_NhayCam(
        iv,
        sizeof(iv)
    );

    if (rc != 0)
    {
        memset(payload_ra, 0, payload_len);
        return false;
    }

    return true;
}


// =====================================================
// WRAPPERS GIU NGUYEN TEN API CU
// Nen main/dong_goi hien tai khong can doi ngay o Stage 1.
// =====================================================

inline bool MaHoa_GCM(
    const uint8_t *header,
    const uint8_t *payload_vao,
    uint8_t *payload_ra,
    uint8_t *auth_tag,
    uint64_t session_id,
    uint32_t nonce_seq)
{
    return MaHoa_GCM_Len(
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
    const uint8_t *header,
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


inline bool MaHoa_GCM_FEC(
    const uint8_t *header,
    const uint8_t *parity_vao_168,
    uint8_t *parity_ma_hoa_168,
    uint8_t *fec_auth_tag,
    uint64_t session_id,
    uint32_t fec_nonce)
{
    return MaHoa_GCM_Len(
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
    const uint8_t *header,
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
