#ifndef DONG_GOI_H
#define DONG_GOI_H

#include <Arduino.h>

// Địa chỉ giả định (Để LoRa phân biệt ai gửi cho ai)
#define ID_TRAM_DU 0x02
#define ID_TRAM_SU 0x01

void Tao_GoiTin_LoRa(
    uint8_t* khung_1,
    uint8_t* khung_2,
    uint8_t so_frame,
    uint8_t* goi_tin_ra
);

#endif