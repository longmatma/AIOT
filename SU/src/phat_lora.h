#ifndef PHAT_LORA_H
#define PHAT_LORA_H

#include <Arduino.h>
#include "gps_su.h"

void KhoiTao_LoRa();

bool Phat_GoiTin_LoRa(
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


// SESSION control plane
// rBS -> SU physical 12B:
//   TYPE_SESSION_READY 0x15 / TYPE_SESSION_FAIL 0x16
//   payload SESSION_ID64.
// session_fail=true neu rBS da retry SESSION_START toi DU 3 lan ma van khong READY.
bool Cho_SESSION_READY_RBS(
    uint32_t timeout_ms,
    uint64_t expected_session_id,
    bool &session_fail
);

// Cho rBS forward su kien DU da bat dau PLAY.
// Physical packet: 4B RadioHead header + SESSION_ID64 8B = 12B.
bool Cho_PLAY_STARTED_RBS(
    uint32_t timeout_ms,
    uint64_t expected_session_id
);

// =====================================================
// HMI USER ACK/NACK
// DU -> rBS -> SU: TYPE 0x13, flags ACK/NACK, SESSION_ID64
// SU -> rBS -> DU: TYPE 0x14 confirm, same SESSION/code
// =====================================================
#define USER_RESPONSE_ACK   0x01
#define USER_RESPONSE_NACK  0x02

void Bat_CheDo_RX_LoRa();

bool KiemTra_USER_RESPONSE_RBS(
    uint64_t expected_session_id,
    uint8_t &response_code
);

bool Gui_USER_CONFIRM_RBS(
    uint64_t session_id,
    uint8_t response_code
);


// =====================================================
// GPS REPORT SU -> rBS
// Physical 44B, packet rieng, KHONG chen vao SESSION_START/VOICE.
// TYPE_GPS_REPORT = 0x17.
// =====================================================
#define TYPE_GPS_REPORT 0x17
#define TYPE_VI_TRI_DINH_KY 0x18
#define SIZE_GPS_REPORT 44

// Arduino LoRa library khoi tao PA_BOOST mac dinh 17 dBm.
// Dat ro rang de rBS co P_TX xac dinh khi uoc luong kenh.
#define CONG_SUAT_PHAT_SU_DBM 17

bool Gui_GPS_REPORT_SU(
    uint64_t ma_phien,
    const DuLieuGPS_SU &du_lieu_gps
);

// Bao cao vi tri ngoai phien, best-effort, khong ACK.
bool Gui_VI_TRI_DINH_KY_SU(
    uint64_t so_thu_tu_bao_cao,
    const DuLieuGPS_SU &du_lieu_gps
);

#endif
