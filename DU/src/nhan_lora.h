#ifndef NHAN_LORA_H
#define NHAN_LORA_H

#include <Arduino.h>

void KhoiTao_LoRa_RX();

bool Nhan_GoiTin_LoRa(
    uint8_t *buffer,
    size_t &do_dai_nhan
);

// DU -> rBS: bao thoi diem speaker da bat dau PLAY.
// Packet physical 12B = 4B control header + SESSION_ID64.
bool Gui_PLAY_STARTED_RBS(
    uint64_t session_id
);

#endif
