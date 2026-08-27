#ifndef NHAN_LORA_H
#define NHAN_LORA_H

#include <Arduino.h>
#include "gps_du.h"

void KhoiTao_LoRa_RX();

bool Nhan_GoiTin_LoRa(
    uint8_t *buffer,
    size_t &do_dai_nhan
);

// DU -> rBS: xac nhan da nhan SESSION_START va da vao dung session.
// Packet physical 12B = 4B control header + SESSION_ID64.
bool Gui_SESSION_READY_RBS(
    uint64_t session_id
);

// DU -> rBS: bao thoi diem speaker da bat dau PLAY.
// Packet physical 12B = 4B control header + SESSION_ID64.
bool Gui_PLAY_STARTED_RBS(
    uint64_t session_id
);


// HMI USER ACK/NACK
#define USER_RESPONSE_ACK   0x01
#define USER_RESPONSE_NACK  0x02

bool Gui_USER_RESPONSE_RBS(
    uint64_t session_id,
    uint8_t response_code
);


// GPS REPORT DU -> rBS, 44B packet rieng.
#define TYPE_GPS_REPORT 0x17
#define SIZE_GPS_REPORT 44

bool Gui_GPS_REPORT_DU(
    uint64_t session_id,
    const DuLieuGPS_DU &gps
);

#endif
