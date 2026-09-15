#include "thu_am.h"
#include "driver/adc.h"

void KhoiTao_ThuAm_DMA() {
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
    
    Serial.println("Khoi tao ADC DMA (32kHz) Thanh Cong!");
}

bool LayMau_AmThanh(uint8_t *buffer_dich) {
    uint32_t tong_byte_da_doc = 0;
    static uint8_t buffer_goc_32k[2560]; 
    
    // Ép ESP32 phải nhả đủ dữ liệu
    while (tong_byte_da_doc < 2560) {
        uint32_t byte_doc_duoc = 0;
        esp_err_t ket_qua = adc_digi_read_bytes(&buffer_goc_32k[tong_byte_da_doc], 2560 - tong_byte_da_doc, &byte_doc_duoc, 50);
        
        if (ket_qua == ESP_OK) {
            tong_byte_da_doc += byte_doc_duoc;
        } else if (ket_qua != ESP_ERR_TIMEOUT) {
            return false; 
        }
    }
    
    adc_digi_output_data_t *mau_adc_32k = (adc_digi_output_data_t *)buffer_goc_32k;
    uint16_t *mau_adc_8k = (uint16_t *)buffer_dich;
    
    // ==============================================================
    // THUẬT TOÁN IIR LOW-PASS FILTER BẬC 2 (Butterworth)
    // Cắt dứt khoát tại 4kHz để chống hiện tượng đục tiếng Aliasing
    // ==============================================================
    
    // Các biến trạng thái phải được lưu lại giữa các khung thoại (dùng static)
    static float x1 = 0, x2 = 0;
    static float y_1 = 0, y_2 = 0; 
    
    // Hệ số đã được tính sẵn cho Fs=32000, Fc=4000
    const float b0 = 0.097631f;
    const float b1 = 0.195262f;
    const float b2 = 0.097631f;
    const float a1 = -0.942809f;
    const float a2 = 0.333333f;

    // Quét toàn bộ 640 mẫu ở 32kHz
    for (int i = 0; i < 640; i++) {
        // Lấy dữ liệu 12-bit sạch từ struct Type 2
        float x0 = (float)mau_adc_32k[i].type2.data;
        
        // Phương trình vi phân IIR
        float y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y_1 - a2 * y_2;
        
        // Dịch chuyển trạng thái cho vòng lặp sau
        x2 = x1; x1 = x0;
        y_2 = y_1; y_1 = y0;
        
        // DECIMATION: Sau khi lọc sạch, lấy thưa 1/4 (mẫu 0, 4, 8, 12...)
        if (i % 4 == 0) {
            // Giới hạn biên độ an toàn, chống tràn bit
            if (y0 < 0) y0 = 0;
            if (y0 > 4095) y0 = 4095;
            mau_adc_8k[i / 4] = (uint16_t)y0;
        }
    }
    
    return true;
}