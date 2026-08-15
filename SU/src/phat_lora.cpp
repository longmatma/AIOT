#include "phat_lora.h"

    // ==========================================
    // ĐỊNH NGHĨA CHÂN LORA THEO SƠ ĐỒ SCHEMATIC
    // ==========================================
    #define LORA_SCK  12
    #define LORA_MISO 13
    #define LORA_MOSI 11
    #define LORA_CS   10
    #define LORA_RST  9
    #define LORA_DIO0 14

    void KhoiTao_LoRa() {
        // Khởi tạo kênh SPI với các chân tùy chỉnh trên ESP32-S3
        SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
        LoRa.setSPI(SPI);
        LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

        // Kích hoạt chip LoRa ở dải tần 433 MHz (phổ biến cho SX1278)
        if (!LoRa.begin(433E6)) {
            Serial.println("LỖI: Không tìm thấy module LoRa!");
            while (1); // Dừng hệ thống nếu lỗi phần cứng
        }
        
        // Cấu hình thông số đài phát ép xung tốc độ cao cho Voice
        LoRa.setSpreadingFactor(7);     // SF thấp nhất để truyền nhanh
        LoRa.setSignalBandwidth(500E3);
        LoRa.setCodingRate4(5);         // Tỉ lệ sửa lỗi tiêu chuẩn (CR 4/5)
        
        // BẬT CHỐT CHẶN CRC (Bit sửa lỗi phần cứng)
        LoRa.enableCrc(); 

        Serial.println("Khoi tao LoRa THANH CONG!");
    }
    void Phat_GoiTin_LoRa(uint8_t* goi_tin, size_t do_dai) {
        LoRa.beginPacket();             // Mở phong bì
        LoRa.write(goi_tin, do_dai);    // Bỏ dữ liệu vào
        LoRa.endPacket();               // Niêm phong và bắn đi!
    }