#ifndef DONG_GOI_H
#define DONG_GOI_H

#include <Arduino.h>

#ifndef ID_TRAM_SU
#define ID_TRAM_SU 0x01
#endif

#ifndef ID_TRAM_DU
#define ID_TRAM_DU 0x02
#endif

// =====================================================
// PROTOCOL 8-FRAME NATIVE
//
// Byte 2 dùng chung:
//   bit 7..5 = count - 1 (1..8)
//   bit 4    = LAST/HAS_LAST
//   bit 3..0 = TYPE
//
// VOICE byte 3 = 168
//   160B ciphertext + 8B GCM tag
//
// FEC packet = 184B:
//   8B header
//   168B encrypted parity(ciphertext+original-tag)
//   8B FEC GCM tag
//
// FEC byte 3:
//   0     nếu group không chứa LAST_AUDIO
//   1..8  số frame của final VOICE nếu group có LAST
// =====================================================

#define TYPE_VOICE_SU 0x01
#define TYPE_FEC_SU   0x05

#define TYPE_MASK_SU          0x0F
#define FLAG_LAST_SU          0x10
#define COUNT_SHIFT_SU        5
#define COUNT_MASK_SU         0xE0

#define SIZE_SESSION_PACKET_SU   12
#define SIZE_VOICE_PACKET_SU    176
#define SIZE_FEC_PACKET_SU      184

#define MAX_FRAME_PER_PACKET      8
#define SPEEX_BYTES_PER_FRAME    20
#define VOICE_PLAINTEXT_BYTES   160
#define VOICE_GCM_TAG_BYTES        8
#define VOICE_PROTECTED_BYTES    168
#define FEC_PARITY_BYTES         168
#define VOICE_LENGTH_SU          168

#define FEC_DATA_PER_GROUP        8

uint64_t Tao_Session_Moi();

void Tao_GoiTin_SessionStart(
    uint8_t *goi_tin_ra
);

void Tao_GoiTin_Voice(
    const uint8_t payload_160[VOICE_PLAINTEXT_BYTES],
    uint8_t so_frame,
    bool la_packet_cuoi,
    uint8_t *goi_tin_ra
);

void Tao_GoiTin_FEC(
    const uint8_t parity_168[FEC_PARITY_BYTES],
    uint32_t group_start_seq,
    uint8_t data_count,
    bool group_has_last,
    uint8_t final_frame_count,
    uint8_t *goi_tin_ra
);

#endif
