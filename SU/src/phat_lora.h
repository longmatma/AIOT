#ifndef PHAT_LORA_H
#define PHAT_LORA_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>


// =====================================================
// CHÂN LORA
// =====================================================

#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10
#define LORA_RST   9
#define LORA_DIO0  14


// =====================================================
// KHỞI TẠO LORA
// =====================================================

void KhoiTao_LoRa();


// =====================================================
// PHÁT PACKET
// =====================================================

void Phat_GoiTin_LoRa(
    uint8_t* goi_tin,
    size_t do_dai
);


// =====================================================
// CHỜ READY TỪ rBS
//
// Dùng cho cơ chế:
//
// gửi 4 packet
// ↓
// SU chuyển sang RX
// ↓
// chờ READY từ rBS
// ↓
// gửi burst tiếp theo
// =====================================================

bool Cho_READY_RBS(
    uint32_t timeout_ms
);


#endif