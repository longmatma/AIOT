#include "nhan_lora.h"
#include <SPI.h>
#include <LoRa.h>

#define LORA_SCK  12
#define LORA_MISO 13
#define LORA_MOSI 11
#define LORA_CS   10
#define LORA_RST  9
#define LORA_DIO0 14

void KhoiTao_LoRa_RX() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setSPI(SPI);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(433E6)) {
        Serial.println("LỖI: Không tìm thấy module LoRa RX!");
        while (1); 
    }
    
    LoRa.setSpreadingFactor(7);     
    LoRa.setSignalBandwidth(500E3); 
    LoRa.setCodingRate4(5);         
    LoRa.enableCrc(); 
    
    Serial.println("Khoi tao LoRa RX THANH CONG!");
}

bool Nhan_GoiTin_LoRa(uint8_t* buffer, size_t &do_dai_nhan) {
    int packetSize = LoRa.parsePacket();
    
    // BỘ LỌC SINH TỬ: Chỉ chấp nhận gói tin nào có độ dài chuẩn xác 46 byte!
    if (packetSize == 54) {
        do_dai_nhan = 0;
        while (LoRa.available() && do_dai_nhan < 54) {
            buffer[do_dai_nhan++] = LoRa.read();
        }
        return true;
    } 
    // Nếu có gói tin lạ bay lạc vào (kích thước khác 46)
    else if (packetSize > 0) { 
        // Phải đọc hết để "xả rác" ra khỏi bộ đệm của chip LoRa, tránh bị kẹt
        while (LoRa.available()) {
            LoRa.read(); 
        }
    }
    
    return false;
}