#include "thu_am.h"
#include "driver/adc.h"

// =====================================================
// AUDIO QUALITY V1 - ZERO LATENCY
//
// - Giu output 8 kHz.
// - Khong doi Speex/packet/AES/FEC/LoRa.
// - Khong them buffer.
// - LPF bac 4 Fc~3.6kHz truoc decimate 32kHz -> 8kHz.
// =====================================================

void KhoiTao_ThuAm_DMA()
{
    adc_digi_init_config_t cau_hinh_dma = {};

    cau_hinh_dma.max_store_buf_size = 8192;
    cau_hinh_dma.conv_num_each_intr = 1280;
    cau_hinh_dma.adc1_chan_mask = BIT(0);
    cau_hinh_dma.adc2_chan_mask = 0;

    ESP_ERROR_CHECK(adc_digi_initialize(&cau_hinh_dma));

    adc_digi_pattern_config_t chan_thu_am[1] = {};
    chan_thu_am[0].atten = ADC_ATTEN_DB_12;
    chan_thu_am[0].channel = ADC_CHANNEL_0;
    chan_thu_am[0].unit = 0;
    chan_thu_am[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_digi_configuration_t cau_hinh_adc = {};
    cau_hinh_adc.conv_limit_en = false;
    cau_hinh_adc.conv_limit_num = 250;
    cau_hinh_adc.pattern_num = 1;
    cau_hinh_adc.adc_pattern = chan_thu_am;
    cau_hinh_adc.sample_freq_hz = 32000;
    cau_hinh_adc.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    cau_hinh_adc.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;

    ESP_ERROR_CHECK(adc_digi_controller_configure(&cau_hinh_adc));
    ESP_ERROR_CHECK(adc_digi_start());

    Serial.println(
        "Khoi tao ADC DMA (32kHz) | LPF4 3.6kHz -> 8kHz THANH CONG!"
    );
}

bool LayMau_AmThanh(uint8_t *buffer_dich)
{
    uint32_t tong_byte_da_doc = 0;
    static uint8_t buffer_goc_32k[2560];

    while (tong_byte_da_doc < sizeof(buffer_goc_32k))
    {
        uint32_t byte_doc_duoc = 0;

        esp_err_t ket_qua =
            adc_digi_read_bytes(
                &buffer_goc_32k[tong_byte_da_doc],
                sizeof(buffer_goc_32k) - tong_byte_da_doc,
                &byte_doc_duoc,
                50
            );

        if (ket_qua == ESP_OK)
        {
            tong_byte_da_doc += byte_doc_duoc;
        }
        else if (ket_qua != ESP_ERR_TIMEOUT)
        {
            return false;
        }
    }

    adc_digi_output_data_t *mau_adc_32k =
        (adc_digi_output_data_t *)buffer_goc_32k;

    uint16_t *mau_adc_8k =
        (uint16_t *)buffer_dich;

    // Butterworth LPF bac 4, Fs=32kHz, Fc~3.6kHz.
    // SOS 1
    static float s1_x1 = 0.0f;
    static float s1_x2 = 0.0f;
    static float s1_y1 = 0.0f;
    static float s1_y2 = 0.0f;

    // SOS 2
    static float s2_x1 = 0.0f;
    static float s2_x2 = 0.0f;
    static float s2_y1 = 0.0f;
    static float s2_y2 = 0.0f;

    constexpr float S1_B0 = 0.00718404f;
    constexpr float S1_B1 = 0.01436808f;
    constexpr float S1_B2 = 0.00718404f;
    constexpr float S1_A1 = -0.95050047f;
    constexpr float S1_A2 = 0.24999081f;

    constexpr float S2_B0 = 1.0f;
    constexpr float S2_B1 = 2.0f;
    constexpr float S2_B2 = 1.0f;
    constexpr float S2_A1 = -1.21807907f;
    constexpr float S2_A2 = 0.60187996f;

    for (int i = 0; i < 640; i++)
    {
        float x =
            (float)mau_adc_32k[i].type2.data;

        float y1 =
            S1_B0 * x
            + S1_B1 * s1_x1
            + S1_B2 * s1_x2
            - S1_A1 * s1_y1
            - S1_A2 * s1_y2;

        s1_x2 = s1_x1;
        s1_x1 = x;
        s1_y2 = s1_y1;
        s1_y1 = y1;

        float y2 =
            S2_B0 * y1
            + S2_B1 * s2_x1
            + S2_B2 * s2_x2
            - S2_A1 * s2_y1
            - S2_A2 * s2_y2;

        s2_x2 = s2_x1;
        s2_x1 = y1;
        s2_y2 = s2_y1;
        s2_y1 = y2;

        if ((i & 0x03) == 0)
        {
            if (y2 < 0.0f)
                y2 = 0.0f;

            if (y2 > 4095.0f)
                y2 = 4095.0f;

            mau_adc_8k[i >> 2] =
                (uint16_t)y2;
        }
    }

    return true;
}
