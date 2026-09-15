from pathlib import Path
from datetime import datetime
import shutil

ROOT = Path(__file__).resolve().parent

FILES = [
    ROOT / "SU" / "src" / "ma_hoa.h",
    ROOT / "DU" / "src" / "ma_hoa.h",
]

for p in FILES:
    if not p.exists():
        raise SystemExit(f"[FAIL] Khong tim thay: {p}")

for p in FILES:
    old = p.read_text(encoding="utf-8")

    required = [
        "khoa_goc_local.h",
        "AES_GCM_TAG_LEN",
        "Lay_Khoa_Adaptive",
        "MaHoa_GCM_Len",
        "GiaiMa_GCM_Len",
    ]

    for marker in required:
        if marker not in old:
            raise SystemExit(
                f"[FAIL] {p} khong dung baseline Adaptive AES hien tai; "
                f"thieu marker {marker}"
            )

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup_dir = ROOT / f"backup_before_fixed_aes128_stageA_{stamp}"
backup_dir.mkdir(parents=True, exist_ok=False)

FIXED_HEADER = r"""#ifndef MA_HOA_H
#define MA_HOA_H

#include <Arduino.h>
#include <string.h>
#include "mbedtls/gcm.h"
#include "mbedtls/md.h"
#include "khoa_goc_local.h"

// =====================================================
// VEDC - FIXED AES-128-GCM
//
// Muc tieu:
// - AES co dinh: AES-128-GCM.
// - KHONG con adaptive crypto theo NORMAL/STRONG/HIGH.
// - KHONG rekey theo 64/128/256 packet.
// - Moi SESSION_ID sinh 1 SESSION KEY rieng:
//
//   K_session = first_128_bits(
//       HMAC-SHA256(
//           ROOT_KEY,
//           "VEDC-FIXED-AES128-V1" || SESSION_ID64
//       )
//   )
//
// - K_session giu nguyen trong ca session.
// - SESSION moi -> key moi.
// - IV van giu:
//       SESSION_ID64 || NONCE_SEQ32
//   VOICE nonce = seq
//   FEC nonce   = 0x80000000 | group_start
//
// Packet format GIU NGUYEN:
// - VOICE payload 160B
// - FEC payload 168B
// - GCM tag 8B
//
// CAC HAM PROFILE BEN DUOI CHI GIU TAM THOI DE TUONG THICH
// VOI CODE HANDSHAKE CU. PROFILE KHONG CON ANH HUONG DEN AES.
// =====================================================

#define AES_GCM_AAD_LEN              8
#define AES_GCM_VOICE_LEN          160
#define AES_GCM_FEC_LEN            168
#define AES_GCM_TAG_LEN              8
#define AES_GCM_IV_LEN              12

#define FEC_IV_DOMAIN 0x80000000UL

#define BAOMAT_PROFILE_NORMAL         0U
#define BAOMAT_PROFILE_STRONG         1U
#define BAOMAT_PROFILE_HIGH           2U
#define BAOMAT_PROFILE_MAC_DINH       BAOMAT_PROFILE_NORMAL

#if VEDC_ROOT_KEY_LEN < 16
#error "VEDC_ROOT_KEY_LEN phai >= 16 byte"
#endif

// =====================================================
// COMPATIBILITY PROFILE STATE
// Chi de packet SESSION cu van bat tay duoc.
// KHONG anh huong key size / rekey.
// =====================================================

inline uint8_t &BaoMat_Profile_Storage()
{
    static uint8_t profile = BAOMAT_PROFILE_MAC_DINH;
    return profile;
}

inline bool Profile_BaoMat_HopLe(uint8_t profile)
{
    return (
        profile == BAOMAT_PROFILE_NORMAL
        || profile == BAOMAT_PROFILE_STRONG
        || profile == BAOMAT_PROFILE_HIGH
    );
}

inline bool Dat_Profile_BaoMat(uint8_t profile)
{
    if (!Profile_BaoMat_HopLe(profile))
        return false;

    BaoMat_Profile_Storage() = profile;
    return true;
}

inline uint8_t Lay_Profile_BaoMat()
{
    return BaoMat_Profile_Storage();
}

inline const char *Ten_Profile_BaoMat(uint8_t profile)
{
    switch (profile)
    {
        case BAOMAT_PROFILE_NORMAL: return "NORMAL";
        case BAOMAT_PROFILE_STRONG: return "STRONG";
        case BAOMAT_PROFILE_HIGH:   return "HIGH";
        default:                    return "INVALID";
    }
}

// Tuong thich log cu: AES LUON 128.
inline uint16_t So_Bit_AES_Theo_Profile(uint8_t)
{
    return 128U;
}

// 0 = khong con packet-based rekey.
inline uint32_t ChuKy_DoiKhoa_Theo_Profile(uint8_t)
{
    return 0UL;
}


// =====================================================
// XOA BO NHO NHAY CAM
// =====================================================

inline void Xoa_BoNho_NhayCam(void *ptr, size_t len)
{
    volatile uint8_t *p =
        reinterpret_cast<volatile uint8_t *>(ptr);

    while (len--)
        *p++ = 0;
}


// =====================================================
// IV 96-bit
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
// FIXED SESSION KEY
//
// Giu ten Lay_Khoa_Adaptive TAM THOI de code cu khong can sua
// trong Stage A. Ham nay KHONG con adaptive.
//
// nonce_seq khong tham gia KDF. No chi tham gia IV.
// =====================================================

inline bool Lay_Khoa_Adaptive(
    uint64_t session_id,
    uint32_t nonce_seq,
    uint8_t khoa_32[32],
    uint16_t &key_bits)
{
    (void)nonce_seq;

    if (session_id == 0 || khoa_32 == nullptr)
        return false;

    key_bits = 128U;

    static uint64_t cache_session = 0;
    static uint8_t cache_key[16] = {0};
    static bool cache_ok = false;

    if (cache_ok && cache_session == session_id)
    {
        memset(khoa_32, 0, 32);
        memcpy(khoa_32, cache_key, sizeof(cache_key));
        return true;
    }

    static const uint8_t domain[] =
    {
        'V','E','D','C','-','F','I','X','E','D','-',
        'A','E','S','1','2','8','-','V','1'
    };

    uint8_t context[sizeof(domain) + 8];
    size_t o = 0;

    memcpy(&context[o], domain, sizeof(domain));
    o += sizeof(domain);

    context[o++] = (session_id >> 56) & 0xFF;
    context[o++] = (session_id >> 48) & 0xFF;
    context[o++] = (session_id >> 40) & 0xFF;
    context[o++] = (session_id >> 32) & 0xFF;
    context[o++] = (session_id >> 24) & 0xFF;
    context[o++] = (session_id >> 16) & 0xFF;
    context[o++] = (session_id >> 8)  & 0xFF;
    context[o++] =  session_id        & 0xFF;

    const mbedtls_md_info_t *md_info =
        mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    if (md_info == nullptr)
    {
        memset(khoa_32, 0, 32);
        Xoa_BoNho_NhayCam(context, sizeof(context));
        return false;
    }

    uint8_t digest[32] = {0};

    const int rc =
        mbedtls_md_hmac(
            md_info,
            VEDC_ROOT_KEY,
            VEDC_ROOT_KEY_LEN,
            context,
            sizeof(context),
            digest
        );

    Xoa_BoNho_NhayCam(context, sizeof(context));

    if (rc != 0)
    {
        memset(khoa_32, 0, 32);
        Xoa_BoNho_NhayCam(digest, sizeof(digest));
        return false;
    }

    memcpy(cache_key, digest, sizeof(cache_key));
    cache_session = session_id;
    cache_ok = true;

    memset(khoa_32, 0, 32);
    memcpy(khoa_32, cache_key, sizeof(cache_key));

    Xoa_BoNho_NhayCam(digest, sizeof(digest));
    return true;
}


// =====================================================
// AES-GCM ENCRYPT - FAIL CLOSED
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
        || payload_vao == nullptr
        || payload_ra == nullptr
        || auth_tag == nullptr
        || payload_len == 0
    )
    {
        return false;
    }

    uint8_t khoa[32] = {0};
    uint16_t key_bits = 0;

    if (!Lay_Khoa_Adaptive(
            session_id,
            nonce_seq,
            khoa,
            key_bits))
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
            128
        );

    Xoa_BoNho_NhayCam(khoa, sizeof(khoa));

    if (rc != 0)
    {
        mbedtls_gcm_free(&gcm);
        memset(payload_ra, 0, payload_len);
        memset(auth_tag, 0, AES_GCM_TAG_LEN);
        return false;
    }

    uint8_t iv[AES_GCM_IV_LEN];
    Tao_IV_GCM(iv, session_id, nonce_seq);

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
    Xoa_BoNho_NhayCam(iv, sizeof(iv));

    if (rc != 0)
    {
        memset(payload_ra, 0, payload_len);
        memset(auth_tag, 0, AES_GCM_TAG_LEN);
        return false;
    }

    return true;
}


// =====================================================
// AES-GCM DECRYPT - FAIL CLOSED
// =====================================================

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
        || payload_vao == nullptr
        || payload_ra == nullptr
        || auth_tag == nullptr
        || payload_len == 0
    )
    {
        return false;
    }

    uint8_t khoa[32] = {0};
    uint16_t key_bits = 0;

    if (!Lay_Khoa_Adaptive(
            session_id,
            nonce_seq,
            khoa,
            key_bits))
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
            128
        );

    Xoa_BoNho_NhayCam(khoa, sizeof(khoa));

    if (rc != 0)
    {
        mbedtls_gcm_free(&gcm);
        memset(payload_ra, 0, payload_len);
        return false;
    }

    uint8_t iv[AES_GCM_IV_LEN];
    Tao_IV_GCM(iv, session_id, nonce_seq);

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
    Xoa_BoNho_NhayCam(iv, sizeof(iv));

    if (rc != 0)
    {
        memset(payload_ra, 0, payload_len);
        return false;
    }

    return true;
}


// =====================================================
// WRAPPERS - GIU NGUYEN API HIEN TAI
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
    uint8_t *parity_ra_168,
    const uint8_t *fec_auth_tag,
    uint64_t session_id,
    uint32_t fec_nonce)
{
    return GiaiMa_GCM_Len(
        header,
        parity_ma_hoa_168,
        parity_ra_168,
        fec_auth_tag,
        AES_GCM_FEC_LEN,
        session_id,
        fec_nonce
    );
}

#endif
"""

for p in FILES:
    rel = p.relative_to(ROOT)
    dst = backup_dir / rel
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dst)

    p.write_text(FIXED_HEADER, encoding="utf-8")

    print(f"[PATCH] {p}")

print()
print("[DONE] FIXED AES-128-GCM STAGE A")
print(f"[BACKUP] {backup_dir}")
print("[CRYPTO] AES-128-GCM co dinh")
print("[KDF] HMAC-SHA256(ROOT_KEY, DOMAIN || SESSION_ID)")
print("[REKEY] chi doi key khi SESSION_ID doi")
print("[PACKET] VOICE/FEC/TAG/IV giu nguyen kich thuoc")
print("[COMPAT] profile handshake cu tam thoi van ton tai nhung KHONG anh huong AES")
print("[NEXT] Build SU + DU. Neu SUCCESS thi CHUA upload; gui log build de lam Stage B.")
