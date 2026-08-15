#ifndef PHAT_LORA_H
#define PHAT_LORA_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// Định nghĩa chân cắm chuẩn theo sơ đồ mạch của ông
#define LORA_SCK  12
#define LORA_MISO 13
#define LORA_MOSI 11
#define LORA_CS   10
#define LORA_RST  9
#define LORA_DIO0 14

void KhoiTao_LoRa();
void Phat_GoiTin_LoRa(uint8_t* goi_tin, size_t do_dai);

#endif