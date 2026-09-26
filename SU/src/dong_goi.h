#ifndef DONG_GOI_H
#define DONG_GOI_H

#include <Arduino.h>
#include "node_config.h"

#define ID_TRAM_SU V2C_SU_ID
#define ID_TRAM_DU V2C_DU_ID

// =====================================================
// PROTOCOL 8-FRAME NATIVE
//
// Byte 2 dùng chung:
//   bit 7..5 = count - 1 (1..8)
//   bit 4    = LAST/HAS_LAST
//   bit 3..0 = TYPE
//
// V2C0 CAPACITY PROFILE dung Speex NB CBR 2.15 kbps:
//   6 byte / 20 ms frame.
//   15 frame = 90B plaintext = 300 ms audio.
//
// Ly do: 2 cap SU-DU cung chia 1 E22 rBS khong du airtime o 3.95 kbps.
// Profile nay GIU AES-GCM + packet-FEC, chi doi bitrate/packetization de mo duong V2C.
//
// VOICE byte 3 = 88
//   80B ciphertext + 8B GCM tag
//
// FEC packet = 104B:
//   8B header
//   88B encrypted parity(ciphertext+original-tag)
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
#define FLAG_CODEC_HQ_SU      0x20  // SINGLE HQ=3.95kbps; 0=DUAL LQ 2.15kbps
#define COUNT_SHIFT_SU        5
#define COUNT_MASK_SU         0xE0

#define SIZE_SESSION_PACKET_SU   12
#define SIZE_VOICE_PACKET_SU    106
#define SIZE_FEC_PACKET_SU      114

#define MAX_FRAME_PER_PACKET      15
#define SPEEX_BYTES_PER_FRAME      6
#define VOICE_PLAINTEXT_BYTES     90
#define VOICE_GCM_TAG_BYTES        8
#define VOICE_PROTECTED_BYTES     98
#define FEC_PARITY_BYTES          98
#define VOICE_LENGTH_SU           98

// V2C0: 8 DATA + 1 PARITY de giu packet-FEC nhung giam airtime cho 2 stream.
// Van recover duoc 1 VOICE mat trong moi group 8 DATA.
#define FEC_DATA_PER_GROUP         8

// Khoi tao SESSION_ID ben vung tu NVS tai boot.
bool KhoiTao_Session_ID_BenVung();

uint64_t Tao_Session_Moi();

// V2C4 scheduler dung parity cua SEQ de chuyen DUAL->SINGLE khong tao lo audio.
uint32_t Lay_Seq_Voice_TiepTheo();

void Tao_GoiTin_SessionStart(
    uint8_t *goi_tin_ra
);

void Tao_GoiTin_Voice(
    const uint8_t payload_voice[VOICE_PLAINTEXT_BYTES],
    uint8_t so_frame,
    bool la_packet_cuoi,
    bool codec_hq,
    uint8_t *goi_tin_ra
);

void Tao_GoiTin_FEC(
    const uint8_t parity_block[FEC_PARITY_BYTES],
    uint32_t group_start_seq,
    uint8_t data_count,
    bool group_has_last,
    uint8_t final_frame_count,
    uint8_t *goi_tin_ra
);

#endif
