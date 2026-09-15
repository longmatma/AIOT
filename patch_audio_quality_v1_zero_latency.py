from pathlib import Path
from datetime import datetime
import shutil

ROOT = Path(__file__).resolve().parent
THU_AM = ROOT / "SU" / "src" / "thu_am.cpp"
NEN_SPEEX = ROOT / "SU" / "src" / "nen_speex.cpp"

for p in (THU_AM, NEN_SPEEX):
    if not p.exists():
        raise SystemExit(f"[FAIL] Khong tim thay: {p}")

old_thu = THU_AM.read_text(encoding="utf-8")
old_speex = NEN_SPEEX.read_text(encoding="utf-8")

for marker in (
    "cau_hinh_adc.sample_freq_hz = 32000",
    "Fc=4000",
    "const float b0 = 0.097631f",
):
    if marker not in old_thu:
        raise SystemExit(
            f"[FAIL] thu_am.cpp khong dung baseline mong doi: thieu {marker!r}"
        )

for marker in (
    "static const float HPF_ALPHA",
    "0.91387f",
    "* 8.0f",
    "SPEEX_SET_BITRATE",
):
    if marker not in old_speex:
        raise SystemExit(
            f"[FAIL] nen_speex.cpp khong dung baseline mong doi: thieu {marker!r}"
        )

if "AUDIO QUALITY V1 - ZERO LATENCY" in old_thu or "AUDIO QUALITY V1 - ZERO LATENCY" in old_speex:
    raise SystemExit("[FAIL] Audio Quality V1 co ve da patch truoc do.")

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = ROOT / f"backup_before_audio_quality_v1_{stamp}"
backup.mkdir(parents=True, exist_ok=False)

for p in (THU_AM, NEN_SPEEX):
    dst = backup / p.relative_to(ROOT)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dst)

print(f"[BACKUP] {backup}")

NEW_THU = '#include "thu_am.h"\n#include "driver/adc.h"\n\n// =====================================================\n// AUDIO QUALITY V1 - ZERO LATENCY\n//\n// - Giu output 8 kHz.\n// - Khong doi Speex/packet/AES/FEC/LoRa.\n// - Khong them buffer.\n// - LPF bac 4 Fc~3.6kHz truoc decimate 32kHz -> 8kHz.\n// =====================================================\n\nvoid KhoiTao_ThuAm_DMA()\n{\n    adc_digi_init_config_t cau_hinh_dma = {};\n\n    cau_hinh_dma.max_store_buf_size = 8192;\n    cau_hinh_dma.conv_num_each_intr = 1280;\n    cau_hinh_dma.adc1_chan_mask = BIT(0);\n    cau_hinh_dma.adc2_chan_mask = 0;\n\n    ESP_ERROR_CHECK(adc_digi_initialize(&cau_hinh_dma));\n\n    adc_digi_pattern_config_t chan_thu_am[1] = {};\n    chan_thu_am[0].atten = ADC_ATTEN_DB_12;\n    chan_thu_am[0].channel = ADC_CHANNEL_0;\n    chan_thu_am[0].unit = 0;\n    chan_thu_am[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;\n\n    adc_digi_configuration_t cau_hinh_adc = {};\n    cau_hinh_adc.conv_limit_en = false;\n    cau_hinh_adc.conv_limit_num = 250;\n    cau_hinh_adc.pattern_num = 1;\n    cau_hinh_adc.adc_pattern = chan_thu_am;\n    cau_hinh_adc.sample_freq_hz = 32000;\n    cau_hinh_adc.conv_mode = ADC_CONV_SINGLE_UNIT_1;\n    cau_hinh_adc.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;\n\n    ESP_ERROR_CHECK(adc_digi_controller_configure(&cau_hinh_adc));\n    ESP_ERROR_CHECK(adc_digi_start());\n\n    Serial.println(\n        "Khoi tao ADC DMA (32kHz) | LPF4 3.6kHz -> 8kHz THANH CONG!"\n    );\n}\n\nbool LayMau_AmThanh(uint8_t *buffer_dich)\n{\n    uint32_t tong_byte_da_doc = 0;\n    static uint8_t buffer_goc_32k[2560];\n\n    while (tong_byte_da_doc < sizeof(buffer_goc_32k))\n    {\n        uint32_t byte_doc_duoc = 0;\n\n        esp_err_t ket_qua =\n            adc_digi_read_bytes(\n                &buffer_goc_32k[tong_byte_da_doc],\n                sizeof(buffer_goc_32k) - tong_byte_da_doc,\n                &byte_doc_duoc,\n                50\n            );\n\n        if (ket_qua == ESP_OK)\n        {\n            tong_byte_da_doc += byte_doc_duoc;\n        }\n        else if (ket_qua != ESP_ERR_TIMEOUT)\n        {\n            return false;\n        }\n    }\n\n    adc_digi_output_data_t *mau_adc_32k =\n        (adc_digi_output_data_t *)buffer_goc_32k;\n\n    uint16_t *mau_adc_8k =\n        (uint16_t *)buffer_dich;\n\n    // Butterworth LPF bac 4, Fs=32kHz, Fc~3.6kHz.\n    // SOS 1\n    static float s1_x1 = 0.0f;\n    static float s1_x2 = 0.0f;\n    static float s1_y1 = 0.0f;\n    static float s1_y2 = 0.0f;\n\n    // SOS 2\n    static float s2_x1 = 0.0f;\n    static float s2_x2 = 0.0f;\n    static float s2_y1 = 0.0f;\n    static float s2_y2 = 0.0f;\n\n    constexpr float S1_B0 = 0.00718404f;\n    constexpr float S1_B1 = 0.01436808f;\n    constexpr float S1_B2 = 0.00718404f;\n    constexpr float S1_A1 = -0.95050047f;\n    constexpr float S1_A2 = 0.24999081f;\n\n    constexpr float S2_B0 = 1.0f;\n    constexpr float S2_B1 = 2.0f;\n    constexpr float S2_B2 = 1.0f;\n    constexpr float S2_A1 = -1.21807907f;\n    constexpr float S2_A2 = 0.60187996f;\n\n    for (int i = 0; i < 640; i++)\n    {\n        float x =\n            (float)mau_adc_32k[i].type2.data;\n\n        float y1 =\n            S1_B0 * x\n            + S1_B1 * s1_x1\n            + S1_B2 * s1_x2\n            - S1_A1 * s1_y1\n            - S1_A2 * s1_y2;\n\n        s1_x2 = s1_x1;\n        s1_x1 = x;\n        s1_y2 = s1_y1;\n        s1_y1 = y1;\n\n        float y2 =\n            S2_B0 * y1\n            + S2_B1 * s2_x1\n            + S2_B2 * s2_x2\n            - S2_A1 * s2_y1\n            - S2_A2 * s2_y2;\n\n        s2_x2 = s2_x1;\n        s2_x1 = y1;\n        s2_y2 = s2_y1;\n        s2_y1 = y2;\n\n        if ((i & 0x03) == 0)\n        {\n            if (y2 < 0.0f)\n                y2 = 0.0f;\n\n            if (y2 > 4095.0f)\n                y2 = 4095.0f;\n\n            mau_adc_8k[i >> 2] =\n                (uint16_t)y2;\n        }\n    }\n\n    return true;\n}\n'
NEW_SPEEX = '#include "nen_speex.h"\n#include <speex/speex.h>\n\nextern "C" {\n    __attribute__((weak)) void * _impure_ptr = nullptr;\n}\n\nvoid *trang_thai_speex;\nSpeexBits cac_bit_speex;\n\n// =====================================================\n// AUDIO QUALITY V1 - ZERO LATENCY\n//\n// - Speex Narrowband van CBR 8 kbps.\n// - Van 160 sample / 20 ms / frame.\n// - Van toi da 20 byte/frame.\n// - Khong doi packet/AES/FEC/LoRa.\n// - Khong them hang doi hay look-ahead.\n// =====================================================\n\nstatic int32_t dc_offset_tich_luy =\n    1551 * 2048;\n\n// HPF ~150 Hz @ 8 kHz.\n// Tang nhe tu 120 Hz de giam um/om tan so thap.\nstatic float hpf_x_truoc = 0.0f;\nstatic float hpf_y_truoc = 0.0f;\n\nstatic constexpr float HPF_ALPHA =\n    0.894607f;\n\n// Gain vua phai hon baseline x8.\nstatic constexpr float AUDIO_GAIN =\n    6.0f;\n\n// Safety limiter sample-level, khong look-ahead -> khong them latency.\nstatic constexpr int32_t AUDIO_LIMIT =\n    15500;\n\nvoid KhoiTao_MayEp_Speex()\n{\n    speex_bits_init(&cac_bit_speex);\n\n    trang_thai_speex =\n        speex_encoder_init(&speex_nb_mode);\n\n    int bitrate = 8000;\n\n    speex_encoder_ctl(\n        trang_thai_speex,\n        SPEEX_SET_BITRATE,\n        &bitrate\n    );\n\n    int vad = 0;\n\n    speex_encoder_ctl(\n        trang_thai_speex,\n        SPEEX_SET_VAD,\n        &vad\n    );\n\n    hpf_x_truoc = 0.0f;\n    hpf_y_truoc = 0.0f;\n\n    Serial.println(\n        "Khoi tao Speex CBR 8kbps | HPF150 | GAIN6 | LIMIT15500 | AQ V1"\n    );\n}\n\nbool Nen_Thanh_KhungThoai(\n    uint8_t *pcm_vao,\n    uint8_t *khung_thoai_ra)\n{\n    speex_bits_reset(&cac_bit_speex);\n\n    int16_t *mau_am_thanh =\n        (int16_t *)pcm_vao;\n\n    for (int i = 0; i < 160; i++)\n    {\n        uint16_t mau_goc =\n            mau_am_thanh[i] & 0x0FFF;\n\n        dc_offset_tich_luy =\n            dc_offset_tich_luy\n            - (dc_offset_tich_luy / 2048)\n            + mau_goc;\n\n        int16_t dc_offset_hien_tai =\n            dc_offset_tich_luy / 2048;\n\n        float x =\n            (float)(\n                (int32_t)mau_goc\n                - (int32_t)dc_offset_hien_tai\n            );\n\n        float y =\n            HPF_ALPHA\n            * (\n                hpf_y_truoc\n                + x\n                - hpf_x_truoc\n            );\n\n        hpf_x_truoc = x;\n        hpf_y_truoc = y;\n\n        int32_t mau_sau_gain =\n            (int32_t)(y * AUDIO_GAIN);\n\n        if (mau_sau_gain > AUDIO_LIMIT)\n            mau_sau_gain = AUDIO_LIMIT;\n        else if (mau_sau_gain < -AUDIO_LIMIT)\n            mau_sau_gain = -AUDIO_LIMIT;\n\n        mau_am_thanh[i] =\n            (int16_t)mau_sau_gain;\n    }\n\n    speex_encode_int(\n        trang_thai_speex,\n        mau_am_thanh,\n        &cac_bit_speex\n    );\n\n    int so_byte_da_nen =\n        speex_bits_write(\n            &cac_bit_speex,\n            (char *)khung_thoai_ra,\n            20\n        );\n\n    return (\n        so_byte_da_nen > 0\n        && so_byte_da_nen <= 20\n    );\n}\n'

THU_AM.write_text(NEW_THU, encoding="utf-8")
NEN_SPEEX.write_text(NEW_SPEEX, encoding="utf-8")

combined = (
    THU_AM.read_text(encoding="utf-8")
    + "\n"
    + NEN_SPEEX.read_text(encoding="utf-8")
)

for marker in (
    "AUDIO QUALITY V1 - ZERO LATENCY",
    "LPF4 3.6kHz",
    "S1_B0 = 0.00718404f",
    "S2_A2 = 0.60187996f",
    "0.894607f",
    "AUDIO_GAIN",
    "6.0f",
    "AUDIO_LIMIT",
    "15500",
    "SPEEX_SET_BITRATE",
    "8000",
):
    if marker not in combined:
        raise RuntimeError(
            f"[STOP] Thieu marker sau patch: {marker}"
        )

print()
print("[DONE] AUDIO QUALITY V1 - ZERO LATENCY")
print("[SU] thu_am.cpp: LPF bac 4 Fc~3.6kHz truoc decimate 32k->8k")
print("[SU] nen_speex.cpp: HPF 150Hz + gain x6 + limiter +/-15500")
print("[KEEP] Speex NB CBR 8000bps, 20ms/frame, 20B/frame")
print("[KEEP] AES/FEC/LoRa/session/ARQ packet format y nguyen")
print("[KEEP] DU/rBS/STM32 khong bi sua")
print("[LATENCY] Khong them buffer/look-ahead/network wait")
print("[NEXT] Build SU. Neu SUCCESS thi upload CHI SU va test nghe.")
