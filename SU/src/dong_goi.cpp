#include "dong_goi.h"
#include "ma_hoa.h"
#include "esp_system.h"


// =====================================================
// SESSION + SEQUENCE
// =====================================================

uint32_t so_thu_tu_goi = 0;

uint64_t session_id_hien_tai = 0;


// =====================================================
// TẠO SESSION MỚI
// =====================================================

uint64_t Tao_Session_Moi()
{
    uint64_t phan_cao =
        ((uint64_t)esp_random()) << 32;

    uint64_t phan_thap =
        (uint64_t)esp_random();

    session_id_hien_tai =
        phan_cao | phan_thap;


    if (
        session_id_hien_tai
        == 0
    )
    {
        session_id_hien_tai =
            1;
    }


    so_thu_tu_goi =
        0;


    Serial.printf(
        "[SU] NEW SESSION = %016llX\n",
        (unsigned long long)
        session_id_hien_tai
    );


    return session_id_hien_tai;
}


// =====================================================
// SESSION_START = 12 BYTE
// =====================================================

void Tao_GoiTin_SessionStart(
    uint8_t *goi_tin_ra)
{
    goi_tin_ra[0] =
        ID_TRAM_DU;

    goi_tin_ra[1] =
        ID_TRAM_SU;

    goi_tin_ra[2] =
        0x02;

    goi_tin_ra[3] =
        8;


    goi_tin_ra[4] =
        (session_id_hien_tai >> 56) & 0xFF;

    goi_tin_ra[5] =
        (session_id_hien_tai >> 48) & 0xFF;

    goi_tin_ra[6] =
        (session_id_hien_tai >> 40) & 0xFF;

    goi_tin_ra[7] =
        (session_id_hien_tai >> 32) & 0xFF;

    goi_tin_ra[8] =
        (session_id_hien_tai >> 24) & 0xFF;

    goi_tin_ra[9] =
        (session_id_hien_tai >> 16) & 0xFF;

    goi_tin_ra[10] =
        (session_id_hien_tai >> 8) & 0xFF;

    goi_tin_ra[11] =
        session_id_hien_tai & 0xFF;
}


// =====================================================
// VOICE = 96 BYTE
//
// Byte 0      DST
// Byte 1      SRC
//
// Byte 2:
//   bit 0..5  TYPE_VOICE = 0x01
//   bit 6..7  frame_count - 1
//
// Byte 3      LENGTH = 88
//             = ciphertext 80B + tag 8B
//
// Byte 4..7   SEQ32 big-endian
// Byte 8..87  ciphertext 80B
// Byte 88..95 GCM tag 8B
//
// AAD = byte 0..7
// IV  = SESSION_ID64 || SEQ32
// =====================================================

void Tao_GoiTin_LoRa(
    uint8_t *khung_1,
    uint8_t *khung_2,
    uint8_t *khung_3,
    uint8_t *khung_4,
    uint8_t so_frame,
    uint8_t *goi_tin_ra)
{
    if (
        so_frame < 1
        ||
        so_frame > 4
    )
    {
        so_frame =
            1;
    }


    uint32_t seq =
        so_thu_tu_goi++;


    // =================================================
    // HEADER 8 BYTE
    // =================================================

    goi_tin_ra[0] =
        ID_TRAM_DU;

    goi_tin_ra[1] =
        ID_TRAM_SU;


    // Low 6 bit = TYPE.
    // High 2 bit = frame_count - 1.
    goi_tin_ra[2] =
        0x01
        |
        ((so_frame - 1) << 6);


    // 80B ciphertext + 8B GCM tag
    goi_tin_ra[3] =
        88;


    goi_tin_ra[4] =
        (seq >> 24) & 0xFF;

    goi_tin_ra[5] =
        (seq >> 16) & 0xFF;

    goi_tin_ra[6] =
        (seq >> 8) & 0xFF;

    goi_tin_ra[7] =
        seq & 0xFF;


    // =================================================
    // PLAINTEXT 80 BYTE
    //
    // Packet cuối có thể chỉ có 1/2/3 frame.
    // Phần frame không sử dụng được zero-pad.
    // GCM vẫn mã hóa đủ 80B để packet luôn cố định 96B.
    // =================================================

    uint8_t payload_tho[80];

    memset(
        payload_tho,
        0,
        sizeof(payload_tho)
    );


    uint8_t *cac_khung[4] =
    {
        khung_1,
        khung_2,
        khung_3,
        khung_4
    };


    for (
        uint8_t i = 0;
        i < so_frame;
        i++
    )
    {
        if (
            cac_khung[i]
            != nullptr
        )
        {
            memcpy(
                &payload_tho[i * 20],
                cac_khung[i],
                20
            );
        }
    }


    // =================================================
    // AES-GCM
    // =================================================

    MaHoa_GCM(
        goi_tin_ra,

        payload_tho,

        &goi_tin_ra[8],

        &goi_tin_ra[88],

        session_id_hien_tai,

        seq
    );
}
