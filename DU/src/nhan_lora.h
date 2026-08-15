#ifndef NHAN_LORA_H
#define NHAN_LORA_H

#include <Arduino.h>

void KhoiTao_LoRa_RX();
bool Nhan_GoiTin_LoRa(uint8_t* buffer, size_t &do_dai_nhan);

#endif