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
#define TYPE_VI_TRI_DINH_KY 0x18
#define SIZE_GPS_REPORT 44
#define CONG_SUAT_PHAT_MAC_DINH_DU_DBM 17

// Trạng thái RF hiện tại của DU, phục vụ telemetry và kiểm tra.
int8_t Lay_CongSuat_Phat_DU_dBm();
uint8_t Lay_HeSo_TraiPho_DU();

bool Gui_GPS_REPORT_DU(
    uint64_t ma_phien,
    const DuLieuGPS_DU &du_lieu_gps
);

bool Gui_VI_TRI_DINH_KY_DU(
    uint64_t so_thu_tu_bao_cao,
    const DuLieuGPS_DU &du_lieu_gps
);

#endif
