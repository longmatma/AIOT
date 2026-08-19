#include "dong_goi.h"
#include "ma_hoa.h"
#include "esp_system.h"
#include <string.h>

// =====================================================
// SESSION + VOICE SEQUENCE
// =====================================================

uint32_t so_thu_tu_goi = 0;
uint64_t session_id_hien_tai = 0;


// =====================================================
// SESSION MỚI
// =====================================================

uint64_t Tao_Session_Moi()
{
    uint64_t phan_cao =
        ((uint64_t)esp_random()) << 32;

    uint64_t phan_thap =
        (uint64_t)esp_random();

    session_id_hien_tai =
        phan_cao | phan_thap;

    if (session_id_hien_tai == 0)
    {
        session_id_hien_tai = 1;
    }

    so_thu_tu_goi = 0;

    Serial.printf(
        "[SU] NEW SESSION = %016llX\n",
        (unsigned long long)session_id_hien_tai
    );

    return session_id_hien_tai;
}


// =====================================================
// SESSION_START = 12 BYTE
// =====================================================

void Tao_GoiTin_SessionStart(
    uint8_t *goi_tin_ra)
{
    goi_tin_ra[0] = ID_TRAM_DU;
    goi_tin_ra[1] = ID_TRAM_SU;
    goi_tin_ra[2] = 0x02;
    goi_tin_ra[3] = 8;

    goi_tin_ra[4]  = (session_id_hien_tai >> 56) & 0xFF;
    goi_tin_ra[5]  = (session_id_hien_tai >> 48) & 0xFF;
    goi_tin_ra[6]  = (session_id_hien_tai >> 40) & 0xFF;
    goi_tin_ra[7]  = (session_id_hien_tai >> 32) & 0xFF;
    goi_tin_ra[8]  = (session_id_hien_tai >> 24) & 0xFF;
    goi_tin_ra[9]  = (session_id_hien_tai >> 16) & 0xFF;
    goi_tin_ra[10] = (session_id_hien_tai >> 8)  & 0xFF;
    goi_tin_ra[11] =  session_id_hien_tai        & 0xFF;
}


// =====================================================
// VOICE = 176 BYTE
//
// Byte 0      DST
// Byte 1      SRC
//
// Byte 2:
//   bit 7..5 = frame_count - 1 (0..7 => 1..8 frame)
//   bit 4    = LAST_AUDIO
//   bit 3..0 = TYPE_VOICE = 0x01
//
// Byte 3      LENGTH = 168
// Byte 4..7   SEQ32
// Byte 8..167 ciphertext 160B
// Byte 168..175 GCM tag 8B
//
// AAD = byte 0..7
// IV  = SESSION_ID64 || SEQ32
// =====================================================

void Tao_GoiTin_Voice(
    const uint8_t payload_160[VOICE_PLAINTEXT_BYTES],
    uint8_t so_frame,
    bool la_packet_cuoi,
    uint8_t *goi_tin_ra)
{
    if (so_frame < 1 || so_frame > MAX_FRAME_PER_PACKET)
    {
        so_frame = 1;
    }

    // High bit của SEQ dành làm domain FEC.
    // Với MAX_KHUNG_THOAI hiện tại (~375 packet/session)
    // sẽ không bao giờ chạm giới hạn này.
    if (so_thu_tu_goi & 0x80000000UL)
    {
        Serial.println(
            "[SU FATAL] VOICE SEQ vuot mien nonce cho phep!"
        );

        so_thu_tu_goi = 0;
    }

    uint32_t seq =
        so_thu_tu_goi++;

    goi_tin_ra[0] =
        ID_TRAM_DU;

    goi_tin_ra[1] =
        ID_TRAM_SU;

    goi_tin_ra[2] =
        TYPE_VOICE_SU
        |
        ((so_frame - 1) << COUNT_SHIFT_SU)
        |
        (la_packet_cuoi ? FLAG_LAST_SU : 0x00);

    goi_tin_ra[3] =
        VOICE_LENGTH_SU;

    goi_tin_ra[4] =
        (seq >> 24) & 0xFF;

    goi_tin_ra[5] =
        (seq >> 16) & 0xFF;

    goi_tin_ra[6] =
        (seq >> 8) & 0xFF;

    goi_tin_ra[7] =
        seq & 0xFF;

    MaHoa_GCM(
        goi_tin_ra,
        payload_160,
        &goi_tin_ra[8],
        &goi_tin_ra[168],
        session_id_hien_tai,
        seq
    );
}


// =====================================================
// FEC = 184 BYTE
//
// KHÔNG XOR PLAINTEXT.
//
// Mỗi VOICE có protected block 168B:
//   byte 8..167   = ciphertext 160B
//   byte 168..175 = ORIGINAL VOICE GCM TAG 8B
//
// parity_168 = XOR protected block của tối đa 8 VOICE.
//
// Sau đó parity_168 tự được AES-GCM bảo vệ:
//
// Byte 0..7    FEC header / AAD
// Byte 8..175  encrypted parity 168B
// Byte 176..183 FEC GCM tag 8B
//
// Khi DU khôi phục 1 VOICE:
//   recover ciphertext + original VOICE tag
//   -> dựng lại VOICE header
//   -> chạy GCM verify của chính VOICE đó
//   -> chỉ PASS mới decode Speex.
//
// IV FEC:
// SESSION_ID64 || (0x80000000 | group_start_seq)
// =====================================================

void Tao_GoiTin_FEC(
    const uint8_t parity_168[FEC_PARITY_BYTES],
    uint32_t group_start_seq,
    uint8_t data_count,
    bool group_has_last,
    uint8_t final_frame_count,
    uint8_t *goi_tin_ra)
{
    if (data_count < 1 || data_count > FEC_DATA_PER_GROUP)
    {
        data_count = 1;
    }

    if (!group_has_last)
    {
        final_frame_count = 0;
    }
    else if (
        final_frame_count < 1
        ||
        final_frame_count > MAX_FRAME_PER_PACKET
    )
    {
        final_frame_count =
            MAX_FRAME_PER_PACKET;
    }

    goi_tin_ra[0] =
        ID_TRAM_DU;

    goi_tin_ra[1] =
        ID_TRAM_SU;

    goi_tin_ra[2] =
        TYPE_FEC_SU
        |
        ((data_count - 1) << COUNT_SHIFT_SU)
        |
        (group_has_last ? FLAG_LAST_SU : 0x00);

    goi_tin_ra[3] =
        final_frame_count;

    goi_tin_ra[4] =
        (group_start_seq >> 24) & 0xFF;

    goi_tin_ra[5] =
        (group_start_seq >> 16) & 0xFF;

    goi_tin_ra[6] =
        (group_start_seq >> 8) & 0xFF;

    goi_tin_ra[7] =
        group_start_seq & 0xFF;

    uint32_t fec_nonce =
        FEC_IV_DOMAIN
        |
        group_start_seq;

    MaHoa_GCM_FEC(
        goi_tin_ra,
        parity_168,
        &goi_tin_ra[8],
        &goi_tin_ra[176],
        session_id_hien_tai,
        fec_nonce
    );
}
