#include "nen_speex.h"
#include <speex/speex.h>

extern "C" {
    __attribute__((weak)) void * _impure_ptr = nullptr;
}

void *trang_thai_speex;
SpeexBits cac_bit_speex;

// =====================================================
// AUDIO QUALITY V1 - ZERO LATENCY
//
// - Speex Narrowband van CBR 8 kbps.
// - Van 160 sample / 20 ms / frame.
// - Van toi da 20 byte/frame.
// - Khong doi packet/AES/FEC/LoRa.
// - Khong them hang doi hay look-ahead.
// =====================================================

static int32_t dc_offset_tich_luy =
    1551 * 2048;

// HPF ~150 Hz @ 8 kHz.
// Tang nhe tu 120 Hz de giam um/om tan so thap.
static float hpf_x_truoc = 0.0f;
static float hpf_y_truoc = 0.0f;

static constexpr float HPF_ALPHA =
    0.894607f;

// Gain vua phai hon baseline x8.
static constexpr float AUDIO_GAIN =
    6.0f;

// Safety limiter sample-level, khong look-ahead -> khong them latency.
static constexpr int32_t AUDIO_LIMIT =
    15500;

void KhoiTao_MayEp_Speex()
{
    speex_bits_init(&cac_bit_speex);

    trang_thai_speex =
        speex_encoder_init(&speex_nb_mode);

    int bitrate = 8000;

    speex_encoder_ctl(
        trang_thai_speex,
        SPEEX_SET_BITRATE,
        &bitrate
    );

    int vad = 0;

    speex_encoder_ctl(
        trang_thai_speex,
        SPEEX_SET_VAD,
        &vad
    );

    hpf_x_truoc = 0.0f;
    hpf_y_truoc = 0.0f;

    Serial.println(
        "Khoi tao Speex CBR 8kbps | HPF150 | GAIN6 | LIMIT15500 | AQ V1"
    );
}

bool Nen_Thanh_KhungThoai(
    uint8_t *pcm_vao,
    uint8_t *khung_thoai_ra)
{
    speex_bits_reset(&cac_bit_speex);

    int16_t *mau_am_thanh =
        (int16_t *)pcm_vao;

    for (int i = 0; i < 160; i++)
    {
        uint16_t mau_goc =
            mau_am_thanh[i] & 0x0FFF;

        dc_offset_tich_luy =
            dc_offset_tich_luy
            - (dc_offset_tich_luy / 2048)
            + mau_goc;

        int16_t dc_offset_hien_tai =
            dc_offset_tich_luy / 2048;

        float x =
            (float)(
                (int32_t)mau_goc
                - (int32_t)dc_offset_hien_tai
            );

        float y =
            HPF_ALPHA
            * (
                hpf_y_truoc
                + x
                - hpf_x_truoc
            );

        hpf_x_truoc = x;
        hpf_y_truoc = y;

        int32_t mau_sau_gain =
            (int32_t)(y * AUDIO_GAIN);

        if (mau_sau_gain > AUDIO_LIMIT)
            mau_sau_gain = AUDIO_LIMIT;
        else if (mau_sau_gain < -AUDIO_LIMIT)
            mau_sau_gain = -AUDIO_LIMIT;

        mau_am_thanh[i] =
            (int16_t)mau_sau_gain;
    }

    speex_encode_int(
        trang_thai_speex,
        mau_am_thanh,
        &cac_bit_speex
    );

    int so_byte_da_nen =
        speex_bits_write(
            &cac_bit_speex,
            (char *)khung_thoai_ra,
            20
        );

    return (
        so_byte_da_nen > 0
        && so_byte_da_nen <= 20
    );
}
