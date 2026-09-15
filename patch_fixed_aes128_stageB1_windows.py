from pathlib import Path
from datetime import datetime
import shutil

ROOT = Path(__file__).resolve().parent
SU = ROOT / "SU" / "src"
DU = ROOT / "DU" / "src"

PHAT = SU / "phat_lora.cpp"
DU_MAIN = DU / "main.cpp"
NHAN_CPP = DU / "nhan_lora.cpp"
SU_CRYPTO = SU / "ma_hoa.h"
DU_CRYPTO = DU / "ma_hoa.h"

for p in (PHAT, DU_MAIN, NHAN_CPP, SU_CRYPTO, DU_CRYPTO):
    if not p.exists():
        raise SystemExit(f"[FAIL] Khong tim thay: {p}")

for p in (SU_CRYPTO, DU_CRYPTO):
    txt = p.read_text(encoding="utf-8")
    if "VEDC - FIXED AES-128-GCM" not in txt:
        raise SystemExit(f"[FAIL] {p} chua o Stage A")

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = ROOT / f"backup_before_fixed_aes128_stageB1_{stamp}"
backup.mkdir(parents=True, exist_ok=False)

for p in (PHAT, DU_MAIN, NHAN_CPP, SU_CRYPTO, DU_CRYPTO):
    q = backup / p.relative_to(ROOT)
    q.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, q)

print(f"[BACKUP] {backup}")


def rep(text, old, new, label):
    n = text.count(old)
    if n != 1:
        raise RuntimeError(f"[STOP] {label}: tim thay {n}, can 1")
    print(f"[PATCH] {label}")
    return text.replace(old, new, 1)


# SU
t = PHAT.read_text(encoding="utf-8")

t = rep(
    t,
    '#include "phat_lora.h"\n#include "ma_hoa.h"\n',
    '#include "phat_lora.h"\n',
    "SU remove profile include",
)

old = '''                    if (buffer[2] == TYPE_SESSION_READY)
                    {
                        const uint8_t profile_duoc_chon =
                            buffer[3] & 0x03U;

                        if (
                            !Profile_BaoMat_HopLe(
                                profile_duoc_chon
                            )
                            ||
                            !Dat_Profile_BaoMat(
                                profile_duoc_chon
                            )
                        )
                        {
                            Serial.printf(
                                "[SU CRYPTO DROP] SESSION_READY profile khong hop le=%u | SESSION=%016llX\\n",
                                (unsigned int)profile_duoc_chon,
                                (unsigned long long)session_id
                            );

                            LoRa.receive();
                            continue;
                        }

                        Serial.printf(
                            "[SU SESSION] SESSION_READY OK | SESSION=%016llX\\n",
                            (unsigned long long)session_id
                        );

                        Serial.printf(
                            "[SU CRYPTO] PROFILE=%s(%u) | AES=%u | REKEY_EVERY=%u packet\\n",
                            Ten_Profile_BaoMat(
                                profile_duoc_chon
                            ),
                            (unsigned int)profile_duoc_chon,
                            (unsigned int)So_Bit_AES_Theo_Profile(
                                profile_duoc_chon
                            ),
                            (unsigned int)ChuKy_DoiKhoa_Theo_Profile(
                                profile_duoc_chon
                            )
                        );

                        LoRa.idle();

                        // Giu guard da duoc chung minh: rBS can quay lai RX
                        // truoc khi SU phat VOICE dau tien.
                        delay(POST_READY_TX_GUARD_MS);

                        return true;
                    }
'''

new = '''                    if (buffer[2] == TYPE_SESSION_READY)
                    {
                        Serial.printf(
                            "[SU SESSION] SESSION_READY OK | SESSION=%016llX\\n",
                            (unsigned long long)session_id
                        );

                        Serial.println(
                            "[SU CRYPTO] FIXED AES-128-GCM | SESSION KEY THEO SESSION_ID"
                        );

                        LoRa.idle();

                        // Giu guard da duoc chung minh: rBS can quay lai RX
                        // truoc khi SU phat VOICE dau tien.
                        delay(POST_READY_TX_GUARD_MS);

                        return true;
                    }
'''
t = rep(t, old, new, "SU SESSION_READY fixed AES")
PHAT.write_text(t, encoding="utf-8")


# DU nhan_lora
t = NHAN_CPP.read_text(encoding="utf-8")

t = rep(
    t,
    '#include "nhan_lora.h"\n#include "ma_hoa.h"\n',
    '#include "nhan_lora.h"\n',
    "DU remove profile include",
)

t = rep(
    t,
    '    packet[3] = Lay_Profile_BaoMat() & 0x03U;\n',
    '    packet[3] = 0x00;\n',
    "DU SESSION_READY flags=0",
)

old = '''    Serial.printf(
        "[DU SESSION] SESSION_READY -> rBS | SESSION=%016llX | PROFILE=%s(%u) | TX=%s\\n",
        (unsigned long long)session_id,
        Ten_Profile_BaoMat(
            Lay_Profile_BaoMat()
        ),
        (unsigned int)Lay_Profile_BaoMat(),
        ok == 1 ? "OK" : "FAIL"
    );
'''

new = '''    Serial.printf(
        "[DU SESSION] SESSION_READY -> rBS | SESSION=%016llX | TX=%s\\n",
        (unsigned long long)session_id,
        ok == 1 ? "OK" : "FAIL"
    );
'''
t = rep(t, old, new, "DU SESSION_READY no profile")
NHAN_CPP.write_text(t, encoding="utf-8")


# DU main
t = DU_MAIN.read_text(encoding="utf-8")

old = '''            if (session_moi == 0)
            {
                Serial.println(
                    "[DU DROP] SESSION_ID = 0!"
                );

                continue;
            }

            const uint8_t session_meta =
                goi_tin[3];

            const uint8_t profile_moi =
                (session_meta >> 6) & 0x03U;

            // bits5..0 hien chi cho phep payload length = 8.
            if (
                (session_meta & 0x3FU) != 8U
                ||
                !Profile_BaoMat_HopLe(
                    profile_moi
                )
            )
            {
                Serial.printf(
                    "[DU CRYPTO DROP] SESSION META/PROFILE sai | META=0x%02X | PROFILE=%u\\n",
                    (unsigned int)session_meta,
                    (unsigned int)profile_moi
                );

                continue;
            }

            if (
                da_co_session
                &&
                session_moi
                    == session_id_hien_tai
            )
'''

new = '''            if (session_moi == 0)
            {
                Serial.println(
                    "[DU DROP] SESSION_ID = 0!"
                );

                continue;
            }

            if (goi_tin[3] != 8U)
            {
                Serial.printf(
                    "[DU DROP] SESSION_START META sai | META=0x%02X\\n",
                    (unsigned int)goi_tin[3]
                );

                continue;
            }

            if (
                da_co_session
                &&
                session_moi
                    == session_id_hien_tai
            )
'''
t = rep(t, old, new, "DU remove profile parser")

old = '''            {
                if (
                    profile_moi
                    != Lay_Profile_BaoMat()
                )
                {
                    Serial.printf(
                        "[DU CRYPTO DROP] CUNG SESSION NHUNG DOI PROFILE | SESSION=%016llX | CU=%u | MOI=%u\\n",
                        (unsigned long long)session_moi,
                        (unsigned int)Lay_Profile_BaoMat(),
                        (unsigned int)profile_moi
                    );

                    continue;
                }

                HMI_DU_TamDung_Beacon(1500UL);

                Serial.printf(
                    "[DU] SESSION LAP LAI = %016llX -> GUI LAI SESSION_READY\\n",
                    (unsigned long long)session_moi
                );

                DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
                Gui_GPS_REPORT_DU(session_moi, du_lieu_gps_du);
                Gui_SESSION_READY_RBS(session_moi);
                continue;
            }

            // Nếu vì lỗi control session cũ còn group,
'''

new = '''            {
                HMI_DU_TamDung_Beacon(1500UL);

                Serial.printf(
                    "[DU] SESSION LAP LAI = %016llX -> GUI LAI SESSION_READY\\n",
                    (unsigned long long)session_moi
                );

                DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
                Gui_GPS_REPORT_DU(session_moi, du_lieu_gps_du);
                Gui_SESSION_READY_RBS(session_moi);
                continue;
            }

            // Nếu vì lỗi control session cũ còn group,
'''
t = rep(t, old, new, "DU remove profile lock")

old = '''            if (
                !Dat_Profile_BaoMat(
                    profile_moi
                )
            )
            {
                Serial.printf(
                    "[DU CRYPTO DROP] Khong dat duoc PROFILE=%u\\n",
                    (unsigned int)profile_moi
                );

                continue;
            }

            Serial.printf(
                "[DU CRYPTO] SESSION PROFILE=%s(%u) | AES=%u | REKEY_EVERY=%u packet\\n",
                Ten_Profile_BaoMat(
                    profile_moi
                ),
                (unsigned int)profile_moi,
                (unsigned int)So_Bit_AES_Theo_Profile(
                    profile_moi
                ),
                (unsigned int)ChuKy_DoiKhoa_Theo_Profile(
                    profile_moi
                )
            );

            session_id_hien_tai =
                session_moi;

            da_co_session =
                true;
'''

new = '''            Serial.println(
                "[DU CRYPTO] FIXED AES-128-GCM | SESSION KEY THEO SESSION_ID"
            );

            session_id_hien_tai =
                session_moi;

            da_co_session =
                true;
'''
t = rep(t, old, new, "DU fixed AES session log")
DU_MAIN.write_text(t, encoding="utf-8")


FINAL_HEADER = '#ifndef MA_HOA_H\n#define MA_HOA_H\n\n#include <Arduino.h>\n#include <string.h>\n#include "mbedtls/gcm.h"\n#include "mbedtls/md.h"\n#include "khoa_goc_local.h"\n\n// =====================================================\n// VEDC - FIXED AES-128-GCM FINAL\n//\n// - AES-128-GCM co dinh.\n// - KHONG NORMAL / STRONG / HIGH.\n// - KHONG AES-256.\n// - KHONG rekey theo packet.\n// - Moi SESSION_ID sinh mot session key 128-bit:\n//     first_16_bytes(HMAC-SHA256(ROOT_KEY, DOMAIN || SESSION_ID64))\n// - Session moi -> key moi.\n// - Trong mot session -> key giu nguyen.\n// - IV = SESSION_ID64 || NONCE_SEQ32.\n// - Packet VOICE/FEC va GCM tag 8B giu nguyen.\n// =====================================================\n\n#define AES_GCM_AAD_LEN              8\n#define AES_GCM_VOICE_LEN          160\n#define AES_GCM_FEC_LEN            168\n#define AES_GCM_TAG_LEN              8\n#define AES_GCM_IV_LEN              12\n\n#define FEC_IV_DOMAIN 0x80000000UL\n\n#if VEDC_ROOT_KEY_LEN < 16\n#error "VEDC_ROOT_KEY_LEN phai >= 16 byte"\n#endif\n\ninline void Xoa_BoNho_NhayCam(void *ptr, size_t len)\n{\n    volatile uint8_t *p =\n        reinterpret_cast<volatile uint8_t *>(ptr);\n\n    while (len--)\n        *p++ = 0;\n}\n\ninline void Tao_IV_GCM(\n    uint8_t iv[AES_GCM_IV_LEN],\n    uint64_t session_id,\n    uint32_t nonce_seq)\n{\n    iv[0]  = (session_id >> 56) & 0xFF;\n    iv[1]  = (session_id >> 48) & 0xFF;\n    iv[2]  = (session_id >> 40) & 0xFF;\n    iv[3]  = (session_id >> 32) & 0xFF;\n    iv[4]  = (session_id >> 24) & 0xFF;\n    iv[5]  = (session_id >> 16) & 0xFF;\n    iv[6]  = (session_id >> 8)  & 0xFF;\n    iv[7]  =  session_id        & 0xFF;\n\n    iv[8]  = (nonce_seq >> 24) & 0xFF;\n    iv[9]  = (nonce_seq >> 16) & 0xFF;\n    iv[10] = (nonce_seq >> 8)  & 0xFF;\n    iv[11] =  nonce_seq        & 0xFF;\n}\n\ninline bool Lay_Khoa_Session_AES128(\n    uint64_t session_id,\n    uint8_t khoa_16[16])\n{\n    if (session_id == 0 || khoa_16 == nullptr)\n        return false;\n\n    static uint64_t cache_session = 0;\n    static uint8_t cache_key[16] = {0};\n    static bool cache_ok = false;\n\n    if (cache_ok && cache_session == session_id)\n    {\n        memcpy(khoa_16, cache_key, sizeof(cache_key));\n        return true;\n    }\n\n    static const uint8_t domain[] =\n    {\n        \'V\',\'E\',\'D\',\'C\',\'-\',\'F\',\'I\',\'X\',\'E\',\'D\',\'-\',\n        \'A\',\'E\',\'S\',\'1\',\'2\',\'8\',\'-\',\'V\',\'1\'\n    };\n\n    uint8_t context[sizeof(domain) + 8];\n    size_t o = 0;\n\n    memcpy(&context[o], domain, sizeof(domain));\n    o += sizeof(domain);\n\n    context[o++] = (session_id >> 56) & 0xFF;\n    context[o++] = (session_id >> 48) & 0xFF;\n    context[o++] = (session_id >> 40) & 0xFF;\n    context[o++] = (session_id >> 32) & 0xFF;\n    context[o++] = (session_id >> 24) & 0xFF;\n    context[o++] = (session_id >> 16) & 0xFF;\n    context[o++] = (session_id >> 8)  & 0xFF;\n    context[o++] =  session_id        & 0xFF;\n\n    const mbedtls_md_info_t *md_info =\n        mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);\n\n    if (md_info == nullptr)\n    {\n        Xoa_BoNho_NhayCam(context, sizeof(context));\n        return false;\n    }\n\n    uint8_t digest[32] = {0};\n\n    int rc =\n        mbedtls_md_hmac(\n            md_info,\n            VEDC_ROOT_KEY,\n            VEDC_ROOT_KEY_LEN,\n            context,\n            sizeof(context),\n            digest\n        );\n\n    Xoa_BoNho_NhayCam(context, sizeof(context));\n\n    if (rc != 0)\n    {\n        Xoa_BoNho_NhayCam(digest, sizeof(digest));\n        return false;\n    }\n\n    memcpy(cache_key, digest, sizeof(cache_key));\n    cache_session = session_id;\n    cache_ok = true;\n\n    memcpy(khoa_16, cache_key, sizeof(cache_key));\n    Xoa_BoNho_NhayCam(digest, sizeof(digest));\n\n    return true;\n}\n\ninline bool MaHoa_GCM_Len(\n    const uint8_t *header,\n    const uint8_t *payload_vao,\n    uint8_t *payload_ra,\n    uint8_t *auth_tag,\n    size_t payload_len,\n    uint64_t session_id,\n    uint32_t nonce_seq)\n{\n    if (!header || !payload_vao || !payload_ra || !auth_tag || payload_len == 0)\n        return false;\n\n    uint8_t khoa[16] = {0};\n\n    if (!Lay_Khoa_Session_AES128(session_id, khoa))\n    {\n        memset(payload_ra, 0, payload_len);\n        memset(auth_tag, 0, AES_GCM_TAG_LEN);\n        return false;\n    }\n\n    mbedtls_gcm_context gcm;\n    mbedtls_gcm_init(&gcm);\n\n    int rc = mbedtls_gcm_setkey(\n        &gcm,\n        MBEDTLS_CIPHER_ID_AES,\n        khoa,\n        128\n    );\n\n    Xoa_BoNho_NhayCam(khoa, sizeof(khoa));\n\n    if (rc != 0)\n    {\n        mbedtls_gcm_free(&gcm);\n        memset(payload_ra, 0, payload_len);\n        memset(auth_tag, 0, AES_GCM_TAG_LEN);\n        return false;\n    }\n\n    uint8_t iv[AES_GCM_IV_LEN];\n    Tao_IV_GCM(iv, session_id, nonce_seq);\n\n    rc = mbedtls_gcm_crypt_and_tag(\n        &gcm,\n        MBEDTLS_GCM_ENCRYPT,\n        payload_len,\n        iv,\n        AES_GCM_IV_LEN,\n        header,\n        AES_GCM_AAD_LEN,\n        payload_vao,\n        payload_ra,\n        AES_GCM_TAG_LEN,\n        auth_tag\n    );\n\n    mbedtls_gcm_free(&gcm);\n    Xoa_BoNho_NhayCam(iv, sizeof(iv));\n\n    if (rc != 0)\n    {\n        memset(payload_ra, 0, payload_len);\n        memset(auth_tag, 0, AES_GCM_TAG_LEN);\n        return false;\n    }\n\n    return true;\n}\n\ninline bool GiaiMa_GCM_Len(\n    const uint8_t *header,\n    const uint8_t *payload_vao,\n    uint8_t *payload_ra,\n    const uint8_t *auth_tag,\n    size_t payload_len,\n    uint64_t session_id,\n    uint32_t nonce_seq)\n{\n    if (!header || !payload_vao || !payload_ra || !auth_tag || payload_len == 0)\n        return false;\n\n    uint8_t khoa[16] = {0};\n\n    if (!Lay_Khoa_Session_AES128(session_id, khoa))\n    {\n        memset(payload_ra, 0, payload_len);\n        return false;\n    }\n\n    mbedtls_gcm_context gcm;\n    mbedtls_gcm_init(&gcm);\n\n    int rc = mbedtls_gcm_setkey(\n        &gcm,\n        MBEDTLS_CIPHER_ID_AES,\n        khoa,\n        128\n    );\n\n    Xoa_BoNho_NhayCam(khoa, sizeof(khoa));\n\n    if (rc != 0)\n    {\n        mbedtls_gcm_free(&gcm);\n        memset(payload_ra, 0, payload_len);\n        return false;\n    }\n\n    uint8_t iv[AES_GCM_IV_LEN];\n    Tao_IV_GCM(iv, session_id, nonce_seq);\n\n    rc = mbedtls_gcm_auth_decrypt(\n        &gcm,\n        payload_len,\n        iv,\n        AES_GCM_IV_LEN,\n        header,\n        AES_GCM_AAD_LEN,\n        auth_tag,\n        AES_GCM_TAG_LEN,\n        payload_vao,\n        payload_ra\n    );\n\n    mbedtls_gcm_free(&gcm);\n    Xoa_BoNho_NhayCam(iv, sizeof(iv));\n\n    if (rc != 0)\n    {\n        memset(payload_ra, 0, payload_len);\n        return false;\n    }\n\n    return true;\n}\n\ninline bool MaHoa_GCM(\n    const uint8_t *header,\n    const uint8_t *payload_vao,\n    uint8_t *payload_ra,\n    uint8_t *auth_tag,\n    uint64_t session_id,\n    uint32_t nonce_seq)\n{\n    return MaHoa_GCM_Len(\n        header, payload_vao, payload_ra, auth_tag,\n        AES_GCM_VOICE_LEN, session_id, nonce_seq\n    );\n}\n\ninline bool GiaiMa_GCM(\n    const uint8_t *header,\n    const uint8_t *payload_vao,\n    uint8_t *payload_ra,\n    const uint8_t *auth_tag,\n    uint64_t session_id,\n    uint32_t nonce_seq)\n{\n    return GiaiMa_GCM_Len(\n        header, payload_vao, payload_ra, auth_tag,\n        AES_GCM_VOICE_LEN, session_id, nonce_seq\n    );\n}\n\ninline bool MaHoa_GCM_FEC(\n    const uint8_t *header,\n    const uint8_t *parity_vao_168,\n    uint8_t *parity_ma_hoa_168,\n    uint8_t *fec_auth_tag,\n    uint64_t session_id,\n    uint32_t fec_nonce)\n{\n    return MaHoa_GCM_Len(\n        header, parity_vao_168, parity_ma_hoa_168, fec_auth_tag,\n        AES_GCM_FEC_LEN, session_id, fec_nonce\n    );\n}\n\ninline bool GiaiMa_GCM_FEC(\n    const uint8_t *header,\n    const uint8_t *parity_ma_hoa_168,\n    uint8_t *parity_ra_168,\n    const uint8_t *fec_auth_tag,\n    uint64_t session_id,\n    uint32_t fec_nonce)\n{\n    return GiaiMa_GCM_Len(\n        header, parity_ma_hoa_168, parity_ra_168, fec_auth_tag,\n        AES_GCM_FEC_LEN, session_id, fec_nonce\n    );\n}\n\n#endif\n'

SU_CRYPTO.write_text(FINAL_HEADER, encoding="utf-8")
DU_CRYPTO.write_text(FINAL_HEADER, encoding="utf-8")
print("[PATCH] SU/DU ma_hoa.h -> final fixed AES-128-GCM")

for p in (PHAT, DU_MAIN, NHAN_CPP, SU_CRYPTO, DU_CRYPTO):
    txt = p.read_text(encoding="utf-8")
    for marker in (
        "Dat_Profile_BaoMat",
        "Lay_Profile_BaoMat",
        "Profile_BaoMat_HopLe",
        "REKEY_EVERY=",
        "BAOMAT_PROFILE_",
        "Lay_Khoa_Adaptive",
    ):
        if marker in txt:
            raise RuntimeError(f"[STOP] Con adaptive symbol {marker!r} trong {p}")

print()
print("[DONE] FIXED AES-128-GCM STAGE B1 WINDOWS")
print("[NOTE] Security telemetry 0x1A chua xoa o B1; se don o B2.")
print("[NEXT] Build SU + DU. Neu SUCCESS, gui log de patch Pi B1.")
