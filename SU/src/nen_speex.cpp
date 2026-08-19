#include "nen_speex.h"
#include <speex/speex.h>

extern "C" {
    __attribute__((weak)) void * _impure_ptr = nullptr;
}

void *trang_thai_speex;
SpeexBits cac_bit_speex;


// =====================================================
// DC OFFSET TRACKING
// =====================================================

static int32_t dc_offset_tich_luy =
    1551 * 2048;


// =====================================================
// HIGH-PASS FILTER ~120 Hz
//
// Fs = 8000 Hz
//
// Công thức:
//   y[n] = alpha * (y[n-1] + x[n] - x[n-1])
//
// Với:
//   fc    ~= 120 Hz
//   Fs    = 8000 Hz
//   alpha ~= 0.91387
//
// Mục đích:
//   - giảm ù / ồm / rung tần số thấp
//   - giữ dải tiếng nói chính
//   - không thay đổi sample rate / Speex / LoRa
// =====================================================

static float hpf_x_truoc =
    0.0f;

static float hpf_y_truoc =
    0.0f;

static const float HPF_ALPHA =
    0.91387f;


// =====================================================
// KHỞI TẠO SPEEX
// =====================================================

void KhoiTao_MayEp_Speex()
{
    speex_bits_init(
        &cac_bit_speex
    );


    trang_thai_speex =
        speex_encoder_init(
            &speex_nb_mode
        );


    int bitrate =
        8000;


    speex_encoder_ctl(
        trang_thai_speex,
        SPEEX_SET_BITRATE,
        &bitrate
    );


    int vad =
        0;


    speex_encoder_ctl(
        trang_thai_speex,
        SPEEX_SET_VAD,
        &vad
    );


    // Reset trạng thái HPF khi khởi động encoder.
    hpf_x_truoc =
        0.0f;

    hpf_y_truoc =
        0.0f;


    Serial.println(
        "Khoi tao May Ep Speex (CBR 8000bps + HPF 120Hz) THANH CONG!"
    );
}


// =====================================================
// NÉN 1 FRAME 20 ms
//
// Input:
//   160 sample @ 8 kHz
//
// Output:
//   tối đa 20 byte Speex
// =====================================================

bool Nen_Thanh_KhungThoai(
    uint8_t* pcm_vao,
    uint8_t* khung_thoai_ra)
{
    speex_bits_reset(
        &cac_bit_speex
    );


    int16_t* mau_am_thanh =
        (int16_t*)pcm_vao;


    for (
        int i = 0;
        i < 160;
        i++
    )
    {
        // =============================================
        // 1. LẤY ADC 12 BIT
        // =============================================

        uint16_t mau_goc =
            mau_am_thanh[i]
            & 0x0FFF;


        // =============================================
        // 2. LOẠI DC OFFSET CHẬM
        // =============================================

        dc_offset_tich_luy =
            dc_offset_tich_luy
            -
            (
                dc_offset_tich_luy
                / 2048
            )
            +
            mau_goc;


        int16_t dc_offset_hien_tai =
            dc_offset_tich_luy
            / 2048;


        float x =
            (float)
            (
                (int32_t)mau_goc
                -
                (int32_t)dc_offset_hien_tai
            );


        // =============================================
        // 3. HIGH-PASS ~120 Hz
        //
        // Giảm phần bass thấp / ù / ồm trước Speex.
        // =============================================

        float y =
            HPF_ALPHA
            *
            (
                hpf_y_truoc
                +
                x
                -
                hpf_x_truoc
            );


        hpf_x_truoc =
            x;

        hpf_y_truoc =
            y;


        // =============================================
        // 4. GAIN GIỮ NGUYÊN x8
        // =============================================

        int32_t mau_sau_gain =
            (int32_t)
            (
                y
                * 8.0f
            );


        // Chống overflow int16.
        if (
            mau_sau_gain
            >
            32767
        )
        {
            mau_sau_gain =
                32767;
        }


        if (
            mau_sau_gain
            <
            -32768
        )
        {
            mau_sau_gain =
                -32768;
        }


        mau_am_thanh[i] =
            (int16_t)
            mau_sau_gain;
    }


    // =================================================
    // SPEEX NB
    // =================================================

    speex_encode_int(
        trang_thai_speex,
        mau_am_thanh,
        &cac_bit_speex
    );


    int so_byte_da_nen =
        speex_bits_write(
            &cac_bit_speex,
            (char*)khung_thoai_ra,
            20
        );


    return (
        so_byte_da_nen > 0
        &&
        so_byte_da_nen <= 20
    );
}