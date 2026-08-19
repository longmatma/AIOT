#ifndef DONG_GOI_H
#define DONG_GOI_H

#include <Arduino.h>

#ifndef ID_TRAM_SU
#define ID_TRAM_SU 0x01
#endif

#ifndef ID_TRAM_DU
#define ID_TRAM_DU 0x02
#endif

#define SIZE_SESSION_PACKET_SU   12
#define SIZE_VOICE_PACKET_SU     96

// 4 frame x 20B = 80B plaintext/ciphertext
#define MAX_FRAME_PER_PACKET      4

uint64_t Tao_Session_Moi();

void Tao_GoiTin_SessionStart(
    uint8_t *goi_tin_ra
);

void Tao_GoiTin_LoRa(
    uint8_t *khung_1,
    uint8_t *khung_2,
    uint8_t *khung_3,
    uint8_t *khung_4,
    uint8_t so_frame,
    uint8_t *goi_tin_ra
);

#endif
