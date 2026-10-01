#ifndef PHAT_LORA_H
#define PHAT_LORA_H

#include <Arduino.h>
#include "node_config.h"
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

// V2A.1: block ACK cho cum 1..2 VOICE, co the kem 1 FEC parity.
// Physical 8B: TYPE=0x1A, payload=base_seq32.
// flags:
//   bits5..4 = expected VOICE count (1..2)
//   bit3     = FEC da nhan
//   bit2     = block nay co FEC
//   bits1..0 = VOICE bitmap
bool Cho_BLOCK_ACK_RBS(
    uint32_t timeout_ms,
    uint32_t expected_base_seq,
    uint8_t expected_count,
    bool expected_fec,
    uint8_t &bitmap_out,
    bool &fec_ok_out
);

// V2B.1 SUPERFRAME beacon/ACK VOICE. Physical 13B:
//   byte0 DST=SU, byte1 SRC=rBS, byte2 TYPE=0x1B
//   byte3 SCHEDULE_VERSION
//   byte4..7  FRAME_ID32
//   byte8..11 ACK_BASE_SEQ32 (0xFFFFFFFF neu chua co ACK)
//   byte12    ACK_FLAGS V2B.1:
//             bits7..6 count (1..3), bits2..0 VOICE bitmap.
//             FEC 4+1 la best-effort, khong lam dieu kien ACK/retry VOICE.
// rx_ms_out la millis() ngay khi beacon duoc parse xong; SU dung moc nay de
// can UL slot bang timer cuc bo, khong dung thoi gian Python.
bool Cho_SUPERFRAME_V2B(
    uint32_t timeout_ms,
    uint8_t expected_schedule_version,
    uint32_t &frame_id_out,
    uint32_t &ack_base_seq_out,
    uint8_t &ack_count_out,
    uint8_t &ack_bitmap_out,
    bool &ack_fec_expected_out,
    bool &ack_fec_ok_out,
    uint8_t &schedule_mode_out,
    uint8_t &transition_target_out,
    uint8_t &jam_policy_out,
    uint8_t &fec_grant_pair_out,
    uint32_t &rx_ms_out
);

// V2C2: beacon broadcast cũng được dùng để khóa telemetry ngoài slot.
bool V2C1_NetworkBusy();
void V2C1_Poll_Beacon_Idle();


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

// RF cố định cho bản dựng lại đường thoại.
// Không còn nhận lệnh thay đổi công suất/SF từ rBS.
#define CONG_SUAT_PHAT_SU_DBM 20
#define HE_SO_TRAI_PHO_SU 7

int8_t Lay_CongSuat_Phat_SU_dBm();
uint8_t Lay_HeSo_TraiPho_SU();

bool Gui_GPS_REPORT_SU(
    uint64_t ma_phien,
    const DuLieuGPS_SU &du_lieu_gps
);

// Bao cao vi tri ngoai phien, best-effort, khong ACK.
bool Gui_VI_TRI_DINH_KY_SU(
    uint64_t so_thu_tu_bao_cao,
    const DuLieuGPS_SU &du_lieu_gps
);


// V2C5.3B2 downlink permission-window SIM. RF helper remains OFF.
void SU_Jam53B2_Prepare(uint64_t session_id);
void SU_Jam53B2_Stop(const char *reason);

#endif
