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

// Byte 3 của VOICE:
//   bit 7    = LAST_AUDIO (1 nếu đây là packet cuối của cả câu)
//   bit 0..6 = LENGTH = 88
//
// Byte 3 nằm trong AAD 8 byte nên LAST_AUDIO cũng được AES-GCM xác thực.
#define VOICE_LENGTH_SU       88
#define FLAG_LAST_AUDIO_SU  0x80
#define VOICE_LENGTH_MASK   0x7F

void Tao_GoiTin_LoRa(
    uint8_t *khung_1,
    uint8_t *khung_2,
    uint8_t *khung_3,
    uint8_t *khung_4,
    uint8_t so_frame,
    bool la_packet_cuoi,
    uint8_t *goi_tin_ra
);

#endif
