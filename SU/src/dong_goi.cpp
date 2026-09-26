#include "dong_goi.h"
#include "ma_hoa.h"
#include "esp_system.h"
#include <Preferences.h>
#include <string.h>

// =====================================================
// SESSION + VOICE SEQUENCE
// =====================================================

uint32_t so_thu_tu_goi = 0;
uint64_t session_id_hien_tai = 0;


// =====================================================
// CRYPTO FINAL SESSION-ID V1
//
// AES-GCM yeu cau: KHONG DUOC reuse cung KEY + IV.
//
// Session key phu thuoc SESSION_ID, con IV = SESSION_ID || nonce32.
// Vi vay SESSION_ID phai tranh lap qua reboot.
//
// Co che:
//   HIGH32 = persistent monotonic counter trong NVS.
//   LOW32  = esp_random().
//
// De giam wear flash:
//   - moi lan reserve 4096 counter truoc khi dung;
//   - ghi high-water mark vao NVS TRUOC;
//   - neu mat dien, ID chua dung bi bo qua, KHONG bi reuse;
//   - chi ghi NVS 1 lan/boot + moi 4096 session.
// =====================================================

static constexpr uint32_t SESSION_COUNTER_BLOCK = 4096UL;

static uint32_t session_counter_next = 0;
static uint32_t session_counter_end = 0;
static bool session_allocator_ready = false;


static bool Reserve_Session_Counter_Block()
{
    Preferences prefs;

    if (!prefs.begin("vedc_crypto", false))
    {
        Serial.println(
            "[SU CRYPTO FATAL] KHONG MO DUOC NVS namespace vedc_crypto"
        );
        return false;
    }

    uint32_t persisted_end =
        prefs.getUInt("sid_hi", 0UL);

    if (
        persisted_end
        >
        (0xFFFFFFFFUL - SESSION_COUNTER_BLOCK)
    )
    {
        prefs.end();

        Serial.println(
            "[SU CRYPTO FATAL] SESSION COUNTER DA GAN TRAN 32-BIT"
        );
        return false;
    }

    const uint32_t new_end =
        persisted_end + SESSION_COUNTER_BLOCK;

    size_t written =
        prefs.putUInt(
            "sid_hi",
            new_end
        );

    prefs.end();

    if (written != sizeof(uint32_t))
    {
        Serial.println(
            "[SU CRYPTO FATAL] GHI NVS sid_hi THAT BAI"
        );
        return false;
    }

    session_counter_next =
        persisted_end + 1UL;

    session_counter_end =
        new_end;

    session_allocator_ready =
        true;

    Serial.printf(
        "[SU CRYPTO] SESSION-ID BLOCK RESERVED | COUNTER=%u..%u | SIZE=%u\n",
        (unsigned int)session_counter_next,
        (unsigned int)session_counter_end,
        (unsigned int)SESSION_COUNTER_BLOCK
    );

    return true;
}


bool KhoiTao_Session_ID_BenVung()
{
    if (session_allocator_ready)
        return true;

    return Reserve_Session_Counter_Block();
}


// =====================================================
// SESSION MỚI
// =====================================================

uint64_t Tao_Session_Moi()
{
    if (
        !session_allocator_ready
        ||
        session_counter_next == 0
        ||
        session_counter_next > session_counter_end
    )
    {
        if (!Reserve_Session_Counter_Block())
        {
            Serial.println(
                "[SU CRYPTO FATAL] KHONG CAP DUOC SESSION-ID -> REBOOT"
            );
            Serial.flush();
            delay(200);
            ESP.restart();
            return 0;
        }
    }

    const uint32_t counter32 =
        session_counter_next++;

    const uint32_t random32 =
        esp_random();

    session_id_hien_tai =
        (((uint64_t)counter32) << 32)
        |
        (uint64_t)random32;

    so_thu_tu_goi = 0;

    Serial.printf(
        "[SU] NEW SESSION = %016llX | SID_COUNTER=%u\n",
        (unsigned long long)session_id_hien_tai,
        (unsigned int)counter32
    );

    return session_id_hien_tai;
}


uint32_t Lay_Seq_Voice_TiepTheo()
{
    return so_thu_tu_goi;
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
// VOICE = 106 BYTE (toi da 15 frame x 6B @ Speex 2.15 kbps)
//
// Byte 0      DST
// Byte 1      SRC
//
// Byte 2:
//   bit 4    = LAST_AUDIO
//   bit 3..0 = TYPE_VOICE = 0x01
//   bit 7..5 = 0 (V2C0: frame_count chuyen sang byte3)
//
// Byte 3      FRAME_COUNT = 1..15
// Byte 4..7   SEQ32
// Byte 8..97   ciphertext 90B
// Byte 98..105 GCM tag 8B
//
// AAD = byte 0..7
// IV  = SESSION_ID64 || SEQ32
// =====================================================

void Tao_GoiTin_Voice(
    const uint8_t payload_voice[VOICE_PLAINTEXT_BYTES],
    uint8_t so_frame,
    bool la_packet_cuoi,
    bool codec_hq,
    uint8_t *goi_tin_ra)
{
    if (so_frame < 1 || so_frame > MAX_FRAME_PER_PACKET)
    {
        so_frame = 1;
    }

    // High bit của SEQ dành làm domain FEC.
    // Với MAX_KHUNG_THOAI hiện tại (~375 packet/session)
    
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

    // V2C0: VOICE frame_count toi 15 khong con vua 3 bit o byte2.
    // Byte2 chi giu TYPE + LAST; byte3 mang frame_count truc tiep.
    goi_tin_ra[2] =
        TYPE_VOICE_SU
        |
        (la_packet_cuoi ? FLAG_LAST_SU : 0x00)
        |
        (codec_hq ? FLAG_CODEC_HQ_SU : 0x00);

    goi_tin_ra[3] =
        so_frame;

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
        payload_voice,
        &goi_tin_ra[8],
        &goi_tin_ra[8 + VOICE_PLAINTEXT_BYTES],
        session_id_hien_tai,
        seq
    );
}


// =====================================================
// FEC = 114 BYTE
//
// KHÔNG XOR PLAINTEXT.
//
// Mỗi VOICE có protected block 98B:
//   byte 8..97 = ciphertext 90B
//   byte 98..105 = ORIGINAL VOICE GCM TAG 8B
//
// parity = XOR protected block của tối đa 4 VOICE trong STREAM V1.
//
// Sau đó parity tự được AES-GCM bảo vệ:
//
// Byte 0..7    FEC header / AAD
// Byte 8..105  encrypted parity 98B
// Byte 106..113 FEC GCM tag 8B
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
    const uint8_t parity_block[FEC_PARITY_BYTES],
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
        parity_block,
        &goi_tin_ra[8],
        &goi_tin_ra[8 + FEC_PARITY_BYTES],
        session_id_hien_tai,
        fec_nonce
    );
}
