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
#define CONG_SUAT_PHAT_DU_DBM 20
#define HE_SO_TRAI_PHO_DU 7

// Getter chỉ phục vụ telemetry/GPS; RF cố định, không đổi runtime.
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


// =====================================================
// DO KENH THAT SU -> DU
// DU nghe ke beacon VI_TRI_DINH_KY 0x18 cua SU, do RSSI/SNR,
// sau do gui mot bao cao nho ve rBS. KHONG xu ly data/voice SU truc tiep.
// =====================================================
#define TYPE_BAO_CAO_KENH_SU_DU 0x19
#define SIZE_BAO_CAO_KENH_SU_DU 20

bool Gui_BaoCao_Kenh_SU_DU_DangCho();


// =====================================================
// STAGE 3B1A - SECURITY TELEMETRY DU -> rBS
//
// SHADOW ONLY:
// packet nay CHUA co authentication rieng, nen TUYET DOI
// chua duoc dung de closed-loop doi AES.
// =====================================================
#define TYPE_BAO_CAO_BAO_MAT 0x1A
#define SIZE_BAO_CAO_BAO_MAT 28

void Dat_BaoCao_BaoMat_DangCho(
    uint64_t session_id,
    uint16_t voice_gcm_fail,
    uint16_t fec_gcm_fail,
    uint16_t replay_suspect,
    uint16_t voice_gcm_ok,
    uint16_t fec_gcm_ok,
    uint32_t highest_seq
);

bool Gui_BaoCao_BaoMat_DangCho();

#endif
