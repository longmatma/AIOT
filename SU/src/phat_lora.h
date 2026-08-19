#ifndef PHAT_LORA_H
#define PHAT_LORA_H

#include <Arduino.h>

void KhoiTao_LoRa();

void Phat_GoiTin_LoRa(
    uint8_t *goi_tin,
    size_t do_dai
);

// READY/ACK mới:
//   byte0 DST=SU
//   byte1 SRC=rBS
//   byte2 TYPE_READY
//   byte3 ACK_KIND (VOICE/FEC)
//   byte4..7 ACK_SEQ32
bool Cho_READY_RBS(
    uint32_t timeout_ms,
    uint8_t expected_kind,
    uint32_t expected_seq
);

#endif
