#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h> 
#include "driver/adc.h" 

#include "thu_am.h"
#include "nen_speex.h"
#include "dong_goi.h"
#include "phat_lora.h"
#include "hien_thi.h"

extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2; 

#define CHAN_NUT_PTT 6
#define MAX_KHUNG_THOAI 3000 
uint8_t *Kho_Chua_AmThanh;
uint32_t tong_so_khung_da_ghi = 0;

enum TrangThaiHeThong { NGHI_NGOI, DANG_GHI_AM, DANG_PHAT_SONG };
TrangThaiHeThong trang_thai = NGHI_NGOI;

void setup() {
    Serial.begin(115200);
    pinMode(CHAN_NUT_PTT, INPUT_PULLUP);

    Wire.begin(4, 5); 
    KhoiTao_OLED(); 
    Ve_GiaoDien_OLED(0, false); 

    KhoiTao_ThuAm_DMA();
    adc_digi_stop();
    uint8_t rac[1280];
    uint32_t len;
    while(adc_digi_read_bytes(rac, 1280, &len, 0) == ESP_OK && len > 0) {}

    KhoiTao_MayEp_Speex();
    KhoiTao_LoRa();

    Kho_Chua_AmThanh = (uint8_t *)heap_caps_malloc(MAX_KHUNG_THOAI * 20, MALLOC_CAP_8BIT);
    if (Kho_Chua_AmThanh == NULL) {
        Serial.println("LỖI: Chip không đủ RAM để cấp phát!");
        while(1);
    }
}

void loop() {
    bool nut_dang_bam = (digitalRead(CHAN_NUT_PTT) == LOW);
    static uint32_t thoi_gian_ve_oled = 0; 

    // ========================================================
    // TRẠNG THÁI 1: VỪA BẤM NÚT
    // ========================================================
    if (nut_dang_bam && trang_thai == NGHI_NGOI) {
        trang_thai = DANG_GHI_AM;
        tong_so_khung_da_ghi = 0;

        adc_digi_start();
        
        Ve_GiaoDien_OLED(0, true); 
        Serial.println(">> BAT DAU GHI AM VAO RAM...");
    }

    // ========================================================
    // TRẠNG THÁI 2: ĐANG GIỮ NÚT -> GHI VÀO RAM
    // ========================================================
    else if (nut_dang_bam && trang_thai == DANG_GHI_AM) {
        uint8_t mang_tam_PCM[320];
        uint8_t khung_speex_20b[20];

        if (LayMau_AmThanh(mang_tam_PCM)) {
            
            // Vẽ màn hình nhún nhảy
            if (millis() - thoi_gian_ve_oled > 150) {
                int bien_do = Tinh_BienDo_Mic(mang_tam_PCM, 320);
                Ve_GiaoDien_OLED(bien_do, true); 
                thoi_gian_ve_oled = millis();
            }

            // Nén và lưu kho
            if (tong_so_khung_da_ghi < MAX_KHUNG_THOAI) {
                if (Nen_Thanh_KhungThoai(mang_tam_PCM, khung_speex_20b)) {
                    memcpy(&Kho_Chua_AmThanh[tong_so_khung_da_ghi * 20], khung_speex_20b, 20);
                    tong_so_khung_da_ghi++;
                }
            }
        }
    }

    // ========================================================
    // TRẠNG THÁI 3: VỪA NHẢ NÚT -> GỬI LORA
    // ========================================================
    else if (!nut_dang_bam && trang_thai == DANG_GHI_AM) {
        trang_thai = DANG_PHAT_SONG;

        adc_digi_stop();
        uint8_t rac[1280];
        uint32_t len;
        while(adc_digi_read_bytes(rac, 1280, &len, 0) == ESP_OK && len > 0) {}

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(10, 35, "DANG GUI...");
        u8g2.sendBuffer();

        Serial.print(">> DA NHA NUT! Dang ban ");
        Serial.print(tong_so_khung_da_ghi);
        Serial.println(" khung qua LoRa...");

        uint8_t goi_tin_hoan_chinh[54];
        uint8_t khung_trong[20] = {0};

for (uint32_t i = 0; i < tong_so_khung_da_ghi; i += 2)
{
    uint8_t *khung_1 = &Kho_Chua_AmThanh[i * 20];

    uint8_t *khung_2 = nullptr;

    uint8_t so_frame = 1;

    if (i + 1 < tong_so_khung_da_ghi)
    {
        khung_2 = &Kho_Chua_AmThanh[(i + 1) * 20];
        so_frame = 2;
    }

    Tao_GoiTin_LoRa(
        khung_1,
        khung_2,
        so_frame,
        goi_tin_hoan_chinh
    );

    Phat_GoiTin_LoRa(goi_tin_hoan_chinh, 54);

    delay(5);
}

        Serial.println(">> DA GUI XONG!");
        trang_thai = NGHI_NGOI;
        Ve_GiaoDien_OLED(0, false);
    }

    // ========================================================
    // TRẠNG THÁI 0: ĐANG NGHỈ NGƠI
    // ========================================================
    else if (!nut_dang_bam && trang_thai == NGHI_NGOI) {

        delay(10);
    }
}