#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <LoRa.h> 

#include "nhan_lora.h"
#include "giai_ma_speex.h" 
#include "man_hinh.h" 
#include "ma_hoa.h" 

#define CHAN_AUDIO_OUT 1 
#define PWM_CHANNEL 0
#define PWM_FREQ 8000
#define PWM_RESOLUTION 8

QueueHandle_t HangDoi_GoiTinNhan;
RingbufHandle_t Audio_Buffer; 

// ==========================================
// CÁC TÁC VỤ CỦA DU
// ==========================================
void TacVu_LoRaRX(void *thamSo) {
    
    uint8_t goi_tin[54]; 
    size_t do_dai;
    
    // Cờ báo hiệu chỉ vẽ màn hình 1 lần duy nhất khi bắt đầu có sóng
    bool da_ve_man_hinh = false; 

    while(1) {
        if (Nhan_GoiTin_LoRa(goi_tin, do_dai)) {
            
            uint16_t so_thu_tu = (goi_tin[4] << 8) | goi_tin[5];
            Serial.printf(">> Nhan goi: %d\n", so_thu_tu);

            // CHỐNG RỚT PHẦN MỀM: Kiên nhẫn chờ 10ms nếu hàng đợi bị kẹt, không vứt gói tin đi ngay
            xQueueSend(HangDoi_GoiTinNhan, &goi_tin, pdMS_TO_TICKS(10));

            // CHỐNG MÙ SÓNG: Chỉ vẽ chữ "Dang nhan..." 1 lần duy nhất, tuyệt đối không đếm số liên tục
            if (!da_ve_man_hinh) {
                CapNhat_TrangThai_OLED("Dang Nhan Voice...", LoRa.packetRssi(), so_thu_tu);
                da_ve_man_hinh = true;
            }
        } 
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

void TacVu_GiaiMa(void *thamSo) {
    
    uint8_t goi_tin[54]; 
    int16_t pcm_frame_1[160];
    int16_t pcm_frame_2[160];
    
    while(1) {
        if (xQueueReceive(HangDoi_GoiTinNhan, &goi_tin, portMAX_DELAY) == pdTRUE) {
            
            uint16_t so_thu_tu = (goi_tin[4] << 8) | goi_tin[5];
            uint8_t so_frame =
    (goi_tin[2] & 0x80)
    ? 2
    : 1;
            // CHIA LÔ DỮ LIỆU TỪ GÓI TIN 54 BYTE
            uint8_t* header = &goi_tin[0];           // 6 byte đầu
            uint8_t* payload_ma_hoa = &goi_tin[6];   // 40 byte giữa
            uint8_t* auth_tag = &goi_tin[46];        // 8 byte cuối cùng
            uint8_t payload_sach[40];
            
            // ĐÃ THÊM: KIỂM TRA TEM XÁC THỰC VÀ GIẢI MÃ
            if (GiaiMa_GCM(header, payload_ma_hoa, payload_sach, auth_tag, so_thu_tu)) {
                
                // Giải mã thành công, tem hợp lệ!
                uint8_t voice_frame_1[20];
                uint8_t voice_frame_2[20];
                
                // Copy từ payload SẠCH (đã được GCM giải mã)
                memcpy(voice_frame_1, payload_sach, 20);
                memcpy(voice_frame_2, payload_sach + 20, 20);
                
                // Frame 1 luôn tồn tại
GiaiMa_KhungThoai(
    voice_frame_1,
    (uint8_t*)pcm_frame_1
);

xRingbufferSend(
    Audio_Buffer,
    pcm_frame_1,
    320,
    pdMS_TO_TICKS(10)
);

// Chỉ giải mã frame 2 nếu packet thực sự có frame 2
if (so_frame == 2)
{
    GiaiMa_KhungThoai(
        voice_frame_2,
        (uint8_t*)pcm_frame_2
    );

    xRingbufferSend(
        Audio_Buffer,
        pcm_frame_2,
        320,
        pdMS_TO_TICKS(10)
    );
}
                
            } else {
                // Thùng rác: Tem bị sai lệch do nhiễu môi trường hoặc bị EVE chọc phá
                Serial.printf("❌ LOẠI BỎ: Goi tin %d sai tem xac thuc hoac bi loi!\n", so_thu_tu);
            }
        }
    }
}

void TacVu_PhatAmThanh(void *thamSo) {
    size_t kich_thuoc_lay_duoc;
    bool dang_phat_loa = false; 
    int dem_khung_thoai = 0; 
    
    while(1) {
        uint8_t *pcm_data = (uint8_t *)xRingbufferReceive(Audio_Buffer, &kich_thuoc_lay_duoc, pdMS_TO_TICKS(15));
        
        if (pcm_data != NULL) {
            
            // BƯỚC 1: BẬT ĐIỆN CHO LOA
            if (!dang_phat_loa) {
                pinMode(CHAN_AUDIO_OUT, OUTPUT);
                ledcAttachPin(CHAN_AUDIO_OUT, PWM_CHANNEL);
                dang_phat_loa = true;
            }

            int16_t *pcm16 = (int16_t *)pcm_data; 
            int so_mau = kich_thuoc_lay_duoc / 2;
            uint32_t thoi_gian_mau_tiep_theo = esp_timer_get_time(); 
            
            for (int i = 0; i < so_mau; i++) { 
                int pwm_val = 128 + (pcm16[i] / 128);
                
                if (pwm_val < 0) pwm_val = 0;
                if (pwm_val > 255) pwm_val = 255;
                
                ledcWrite(PWM_CHANNEL, pwm_val);
                
                thoi_gian_mau_tiep_theo += 125; 
                
                while (esp_timer_get_time() < thoi_gian_mau_tiep_theo) {}
            }
            vRingbufferReturnItem(Audio_Buffer, (void *)pcm_data);
            
            // =========================================================
            // CHỐNG RESET: Cứ 50 khung (1 giây) thì nhường CPU 1ms
            // =========================================================
            dem_khung_thoai++;
            if (dem_khung_thoai >= 50) {
                // Lệnh này nhấc bổng PWM_OUT ra khỏi CPU, cho IDLE0 chạy vào cho Watchdog ăn!
                vTaskDelay(pdMS_TO_TICKS(1)); 
                dem_khung_thoai = 0;
            }
            
        } else {
            // BƯỚC 2: RÚT ĐIỆN LOA KHI HẾT SÓNG ÂM
            if (dang_phat_loa) {
                ledcDetachPin(CHAN_AUDIO_OUT);     
                pinMode(CHAN_AUDIO_OUT, INPUT); 
                dang_phat_loa = false;
            }
            // Reset bộ đếm khi nhả PTT
            dem_khung_thoai = 0; 
        }
    }
}

// ==========================================
// KHỞI TẠO VÀ VÒNG LẶP CHÍNH
// ==========================================
void setup() {
    Serial.begin(115200);

    KhoiTao_OLED(); 
    CapNhat_TrangThai_OLED("Dang kiem tra LoRa...", 0, 0);

    KhoiTao_LoRa_RX();
    KhoiTao_GiaiMa_Speex();
    CapNhat_TrangThai_OLED("Dang cho song...", 0, 0);
    
    // Tần số PWM chuẩn 100kHz
    ledcSetup(PWM_CHANNEL, 100000, PWM_RESOLUTION); 
    
    // Ép chân phát về chế độ thả nổi (High-Z) ngay khi có điện
    pinMode(CHAN_AUDIO_OUT, INPUT);
    
    // Hàng đợi đã được khởi tạo chuẩn ở mức 54 byte
    HangDoi_GoiTinNhan = xQueueCreate(50, 54);
    Audio_Buffer = xRingbufferCreate(32000, RINGBUF_TYPE_NOSPLIT);
    
    xTaskCreatePinnedToCore(TacVu_PhatAmThanh, "PWM_OUT", 8192,  NULL, 3, NULL, 0); 
    xTaskCreatePinnedToCore(TacVu_LoRaRX,      "LoRa_RX", 8192,  NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(TacVu_GiaiMa,      "Giai_Ma", 10240, NULL, 2, NULL, 1);
}

void loop() {
    vTaskDelete(NULL); 
}
