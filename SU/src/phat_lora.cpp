#include "phat_lora.h"

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <LoRa.h>

// =====================================================
// CHÂN LORA
// =====================================================

#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10
#define LORA_RST   9
#define LORA_DIO0  14

// =====================================================
// ID / TYPE ACK
// =====================================================

static const uint8_t ID_SU_READY  = 0x01;
static const uint8_t ID_RBS_READY = 0x03;
static const uint8_t TYPE_READY   = 0x03;
static const uint8_t TYPE_PLAY_STARTED = 0x12;
static const uint8_t TYPE_USER_RESPONSE = 0x13;
static const uint8_t TYPE_USER_CONFIRM  = 0x14;
static const uint8_t TYPE_SESSION_READY  = 0x15;
static const uint8_t TYPE_SESSION_FAIL   = 0x16;
static const uint8_t ID_DU_CTRL = 0x02;

// Giữ fix timing đã ổn định.
static const uint32_t POST_READY_TX_GUARD_MS = 8;


// =====================================================
// ĐIỀU KHIỂN RF V11 TỪ rBS
//
// Packet physical 12B (RadioHead-compatible):
// [0] DST = SU hoặc 0xFF broadcast
// [1] SRC = rBS
// [2] TYPE_DIEU_KHIEN_RF = 0x19
// [3] LENH: 1=CHUAN_BI, 2=AP_DUNG, 4=KHOI_PHUC
// [4] PHIEN_BAN = 1
// [5..6] MA_LENH16
// [7] P_TX_SU int8
// [8] P_TX_DU int8
// [9] SF chung
// [10] do tre ap dung, don vi 100 ms
// [11] reserved
//
// SU ACK CHUAN_BI bằng packet 8B ở SF hiện tại rồi mới chờ COMMIT.
// Đây là cơ chế 2 pha để tránh rBS/SU/DU đổi SF lệch nhau.
// =====================================================
static const uint8_t TYPE_DIEU_KHIEN_RF = 0x19;
static const uint8_t TYPE_XAC_NHAN_RF = 0x1A;
static const uint8_t LENH_RF_CHUAN_BI = 0x01;
static const uint8_t LENH_RF_AP_DUNG = 0x02;
static const uint8_t LENH_RF_KHOI_PHUC = 0x04;
static const uint8_t PHIEN_BAN_LENH_RF = 0x01;
static const uint8_t ID_QUANG_BA_RF = 0xFF;

static int8_t cong_suat_phat_su_hien_tai_dbm = CONG_SUAT_PHAT_MAC_DINH_SU_DBM;
static uint8_t he_so_trai_pho_su_hien_tai = 7;

static uint16_t ma_lenh_rf_cho_ap_dung = 0;
static int8_t cong_suat_su_cho_ap_dung_dbm = CONG_SUAT_PHAT_MAC_DINH_SU_DBM;
static uint8_t sf_cho_ap_dung = 7;


// =====================================================
// KHỞI TẠO
// =====================================================

void KhoiTao_LoRa()
{
    SPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_CS
    );

    LoRa.setSPI(
        SPI
    );

    LoRa.setPins(
        LORA_CS,
        LORA_RST,
        LORA_DIO0
    );

    if (!LoRa.begin(433E6))
    {
        Serial.println(
            "LOI: Khong tim thay module LoRa!"
        );

        while (1)
        {
            delay(1000);
        }
    }

    LoRa.setSpreadingFactor(he_so_trai_pho_su_hien_tai);
    LoRa.setSignalBandwidth(500E3);

    // GIỮ CR 4/5.
    // Packet-level FEC xử lý mất packet ở tầng trên;
    // không tăng airtime toàn bộ link bằng CR4/6-4/8 mặc định.
    LoRa.setCodingRate4(5);

    LoRa.enableCrc();

    // Dat ro cong suat de tinh he so kenh tai rBS.
    LoRa.setTxPower(cong_suat_phat_su_hien_tai_dbm);

    LoRa.idle();

    Serial.printf(
        "Khoi tao LoRa THANH CONG! | SF=%u | BW500 | CR4/5 | P_TX=%d dBm\n",
        he_so_trai_pho_su_hien_tai,
        cong_suat_phat_su_hien_tai_dbm
    );
}


// =====================================================
// TX BLOCKING
// =====================================================

bool Phat_GoiTin_LoRa(
    uint8_t *goi_tin,
    size_t do_dai)
{
    // TX blocking. Tra ve ket qua de beacon dinh ky chi tang STT
    // khi packet da duoc driver phat thanh cong.
    LoRa.idle();

    LoRa.beginPacket();

    LoRa.write(
        goi_tin,
        do_dai
    );

    int ket_qua_tx = LoRa.endPacket();
    return ket_qua_tx == 1;
}


// =====================================================
// CHỜ READY/ACK CÓ ĐỊNH DANH
//
// rBS dùng RadioHead header:
// byte0 DST = SU
// byte1 SRC = rBS
// byte2 identifier = TYPE_READY
// byte3 flags = ACK_KIND (TYPE_VOICE hoặc TYPE_FEC)
//
// Payload 4B:
// byte4..7 ACK_SEQ32
//
// Tổng physical packet = 8B.
//
// ACK có KIND + SEQ giúp SU không nhận nhầm READY cũ.
// =====================================================

bool Cho_READY_RBS(
    uint32_t timeout_ms,
    uint8_t expected_kind,
    uint32_t expected_seq)
{
    uint32_t bat_dau =
        millis();

    Serial.printf(
        "[SU] CHUYEN TX -> RX | CHO ACK KIND=0x%02X SEQ=%u...\n",
        expected_kind,
        expected_seq
    );

    LoRa.receive();

    while (
        millis() - bat_dau
        <
        timeout_ms
    )
    {
        int packetSize =
            LoRa.parsePacket();

        if (packetSize > 0)
        {
            // Relay mới có thể dài 180B.
            // Buffer 256B để xả trọn packet rBS->DU nghe ké.
            uint8_t buffer[256];

            size_t so_byte_da_doc =
                0;

            while (
                LoRa.available()
                &&
                so_byte_da_doc < sizeof(buffer)
            )
            {
                buffer[so_byte_da_doc++] =
                    (uint8_t)LoRa.read();
            }

            while (LoRa.available())
            {
                LoRa.read();
            }

            if (
                packetSize == 8
                &&
                so_byte_da_doc == 8
            )
            {
                uint8_t dst =
                    buffer[0];

                uint8_t src =
                    buffer[1];

                uint8_t packet_type =
                    buffer[2];

                uint8_t ack_kind =
                    buffer[3];

                uint32_t ack_seq =
                    ((uint32_t)buffer[4] << 24)
                    |
                    ((uint32_t)buffer[5] << 16)
                    |
                    ((uint32_t)buffer[6] << 8)
                    |
                    ((uint32_t)buffer[7]);

                if (
                    dst == ID_SU_READY
                    &&
                    src == ID_RBS_READY
                    &&
                    packet_type == TYPE_READY
                    &&
                    ack_kind == expected_kind
                    &&
                    ack_seq == expected_seq
                )
                {
                    Serial.printf(
                        "[SU RX] ACK OK | KIND=0x%02X | SEQ=%u\n",
                        ack_kind,
                        ack_seq
                    );

                    LoRa.idle();

                    delay(
                        POST_READY_TX_GUARD_MS
                    );

                    return true;
                }
            }

            // parsePacket() có thể làm radio rời RX continuous.
            // Packet không phải ACK đúng => re-arm ngay.
            LoRa.receive();

            Serial.printf(
                "[SU DROP RX] Khong phai ACK dang cho | SIZE=%d\n",
                packetSize
            );
        }

        delay(1);
    }

    LoRa.idle();

    Serial.printf(
        "[SU TIMEOUT] ACK KIND=0x%02X SEQ=%u\n",
        expected_kind,
        expected_seq
    );

    return false;
}


// =====================================================
// CHO SESSION_READY / SESSION_FAIL TU rBS
//
// Physical 12B RadioHead-compatible:
// byte0 DST=SU, byte1 SRC=rBS,
// byte2 TYPE_SESSION_READY(0x15) hoac TYPE_SESSION_FAIL(0x16),
// byte3 flags=0, byte4..11 SESSION_ID64.
//
// Trong luc cho, SU co the nghe ke:
// - SESSION_START rBS->DU (16B)
// - SESSION_READY DU->rBS (12B, DST=rBS)
// Nen packet khong dung phai xa het + LoRa.receive() lai ngay.
// =====================================================
bool Cho_SESSION_READY_RBS(
    uint32_t timeout_ms,
    uint64_t expected_session_id,
    bool &session_fail)
{
    session_fail = false;

    uint32_t bat_dau = millis();

    Serial.printf(
        "[SU SESSION] CHO SESSION_READY | SESSION=%016llX...\n",
        (unsigned long long)expected_session_id
    );

    LoRa.receive();

    while (millis() - bat_dau < timeout_ms)
    {
        int packetSize = LoRa.parsePacket();

        if (packetSize > 0)
        {
            uint8_t buffer[256];
            size_t n = 0;

            while (LoRa.available() && n < sizeof(buffer))
            {
                buffer[n++] = (uint8_t)LoRa.read();
            }

            while (LoRa.available())
            {
                LoRa.read();
            }

            if (packetSize == 12 && n == 12)
            {
                uint64_t session_id =
                    ((uint64_t)buffer[4] << 56) |
                    ((uint64_t)buffer[5] << 48) |
                    ((uint64_t)buffer[6] << 40) |
                    ((uint64_t)buffer[7] << 32) |
                    ((uint64_t)buffer[8] << 24) |
                    ((uint64_t)buffer[9] << 16) |
                    ((uint64_t)buffer[10] << 8) |
                    ((uint64_t)buffer[11]);

                if (
                    buffer[0] == ID_SU_READY
                    && buffer[1] == ID_RBS_READY
                    && session_id == expected_session_id
                )
                {
                    if (buffer[2] == TYPE_SESSION_READY)
                    {
                        Serial.printf(
                            "[SU SESSION] SESSION_READY OK | SESSION=%016llX\n",
                            (unsigned long long)session_id
                        );

                        LoRa.idle();

                        // Giu guard da duoc chung minh: rBS can quay lai RX
                        // truoc khi SU phat VOICE dau tien.
                        delay(POST_READY_TX_GUARD_MS);

                        return true;
                    }

                    if (buffer[2] == TYPE_SESSION_FAIL)
                    {
                        session_fail = true;

                        Serial.printf(
                            "[SU SESSION] SESSION_FAIL FROM rBS | SESSION=%016llX\n",
                            (unsigned long long)session_id
                        );

                        LoRa.idle();
                        return false;
                    }
                }
            }

            LoRa.receive();
        }

        delay(1);
    }

    LoRa.idle();

    Serial.printf(
        "[SU SESSION] TIMEOUT SESSION_READY | SESSION=%016llX\n",
        (unsigned long long)expected_session_id
    );

    return false;
}


// =====================================================
// CHO PLAY_STARTED TU rBS
//
// rBS RadioHead wrapper:
// byte0 DST = SU
// byte1 SRC = rBS
// byte2 identifier = TYPE_PLAY_STARTED (0x12)
// byte3 flags = 0
// byte4..11 SESSION_ID64
// =====================================================

bool Cho_PLAY_STARTED_RBS(
    uint32_t timeout_ms,
    uint64_t expected_session_id)
{
    uint32_t bat_dau =
        millis();

    Serial.printf(
        "[SU] CHO PLAY_STARTED | SESSION=%016llX...\n",
        (unsigned long long)expected_session_id
    );

    LoRa.receive();

    while (
        millis() - bat_dau
        < timeout_ms
    )
    {
        int packetSize =
            LoRa.parsePacket();

        if (packetSize > 0)
        {
            uint8_t buffer[256];
            size_t n = 0;

            while (
                LoRa.available()
                && n < sizeof(buffer)
            )
            {
                buffer[n++] =
                    (uint8_t)LoRa.read();
            }

            while (LoRa.available())
            {
                LoRa.read();
            }

            if (
                packetSize == 12
                && n == 12
            )
            {
                uint8_t dst = buffer[0];
                uint8_t src = buffer[1];
                uint8_t type = buffer[2];

                uint64_t session_id =
                    ((uint64_t)buffer[4] << 56)
                    |
                    ((uint64_t)buffer[5] << 48)
                    |
                    ((uint64_t)buffer[6] << 40)
                    |
                    ((uint64_t)buffer[7] << 32)
                    |
                    ((uint64_t)buffer[8] << 24)
                    |
                    ((uint64_t)buffer[9] << 16)
                    |
                    ((uint64_t)buffer[10] << 8)
                    |
                    ((uint64_t)buffer[11]);

                if (
                    dst == ID_SU_READY
                    && src == ID_RBS_READY
                    && type == TYPE_PLAY_STARTED
                    && session_id == expected_session_id
                )
                {
                    Serial.printf(
                        "[SU RX] PLAY_STARTED OK | SESSION=%016llX\n",
                        (unsigned long long)session_id
                    );

                    LoRa.idle();
                    return true;
                }
            }

            // SU co the nghe ke END rBS->DU hoac packet khac.
            // Re-arm RX continuous ngay.
            LoRa.receive();
        }

        delay(1);
    }

    LoRa.idle();

    Serial.printf(
        "[SU TIMEOUT] PLAY_STARTED | SESSION=%016llX\n",
        (unsigned long long)expected_session_id
    );

    return false;
}


// =====================================================
// RX CONTINUOUS KHI SU IDLE
// =====================================================
void Bat_CheDo_RX_LoRa()
{
    LoRa.receive();
}


// =====================================================
// Poll USER_RESPONSE rBS -> SU
// Physical 12B:
// [DST=SU][SRC=rBS][TYPE=0x13][flags ACK/NACK][SESSION64]
// =====================================================
bool KiemTra_USER_RESPONSE_RBS(
    uint64_t expected_session_id,
    uint8_t &response_code)
{
    response_code = 0;

    if (expected_session_id == 0)
    {
        return false;
    }

    int packetSize = LoRa.parsePacket();

    if (packetSize <= 0)
    {
        return false;
    }

    uint8_t buffer[256];
    size_t n = 0;

    while (LoRa.available() && n < sizeof(buffer))
    {
        buffer[n++] = (uint8_t)LoRa.read();
    }

    while (LoRa.available())
    {
        LoRa.read();
    }

    LoRa.receive();

    if (packetSize != 12 || n != 12)
    {
        return false;
    }

    if (
        buffer[0] != ID_SU_READY
        || buffer[1] != ID_RBS_READY
        || buffer[2] != TYPE_USER_RESPONSE
    )
    {
        return false;
    }

    uint8_t code = buffer[3];

    if (code != USER_RESPONSE_ACK && code != USER_RESPONSE_NACK)
    {
        return false;
    }

    uint64_t session_id =
        ((uint64_t)buffer[4] << 56) |
        ((uint64_t)buffer[5] << 48) |
        ((uint64_t)buffer[6] << 40) |
        ((uint64_t)buffer[7] << 32) |
        ((uint64_t)buffer[8] << 24) |
        ((uint64_t)buffer[9] << 16) |
        ((uint64_t)buffer[10] << 8) |
        ((uint64_t)buffer[11]);

    if (session_id != expected_session_id)
    {
        Serial.printf(
            "[SU HMI DROP] USER_RESPONSE SESSION CU | GOT=%016llX | EXPECT=%016llX\n",
            (unsigned long long)session_id,
            (unsigned long long)expected_session_id
        );
        return false;
    }

    response_code = code;

    Serial.printf(
        "[SU HMI] USER_%s FROM DU | SESSION=%016llX\n",
        code == USER_RESPONSE_ACK ? "ACK" : "NACK",
        (unsigned long long)session_id
    );

    return true;
}


// =====================================================
// SU -> rBS: confirm da nhan ACK/NACK cua nguoi dung
// =====================================================
bool Gui_USER_CONFIRM_RBS(
    uint64_t session_id,
    uint8_t response_code)
{
    if (
        session_id == 0
        || (response_code != USER_RESPONSE_ACK && response_code != USER_RESPONSE_NACK)
    )
    {
        return false;
    }

    uint8_t packet[12];
    packet[0] = ID_RBS_READY;
    packet[1] = ID_SU_READY;
    packet[2] = TYPE_USER_CONFIRM;
    packet[3] = response_code;
    packet[4]  = (session_id >> 56) & 0xFF;
    packet[5]  = (session_id >> 48) & 0xFF;
    packet[6]  = (session_id >> 40) & 0xFF;
    packet[7]  = (session_id >> 32) & 0xFF;
    packet[8]  = (session_id >> 24) & 0xFF;
    packet[9]  = (session_id >> 16) & 0xFF;
    packet[10] = (session_id >> 8)  & 0xFF;
    packet[11] =  session_id        & 0xFF;

    Phat_GoiTin_LoRa(packet, sizeof(packet));
    LoRa.receive();

    Serial.printf(
        "[SU HMI] USER_CONFIRM -> rBS | CODE=%s | SESSION=%016llX\n",
        response_code == USER_RESPONSE_ACK ? "ACK" : "NACK",
        (unsigned long long)session_id
    );

    return true;
}


// =====================================================
// API TRẠNG THÁI RF HIỆN TẠI
// =====================================================
int8_t Lay_CongSuat_Phat_SU_dBm()
{
    return cong_suat_phat_su_hien_tai_dbm;
}

uint8_t Lay_HeSo_TraiPho_SU()
{
    return he_so_trai_pho_su_hien_tai;
}


// =====================================================
// ÁP DỤNG RF TẠI SU
// Chỉ chấp nhận miền đã quy hoạch cho V11: P_TX 8..20 dBm, SF7..SF9.
// =====================================================
static bool SU_ApDung_CauHinh_RF(int8_t cong_suat_dbm, uint8_t sf)
{
    if (cong_suat_dbm < 8 || cong_suat_dbm > 20)
        return false;

    if (sf < 7 || sf > 9)
        return false;

    LoRa.idle();
    LoRa.setTxPower(cong_suat_dbm);
    LoRa.setSpreadingFactor(sf);
    LoRa.receive();

    cong_suat_phat_su_hien_tai_dbm = cong_suat_dbm;
    he_so_trai_pho_su_hien_tai = sf;

    Serial.printf(
        "[SU RF] DA AP DUNG | P_TX=%d dBm | SF=%u\n",
        cong_suat_phat_su_hien_tai_dbm,
        he_so_trai_pho_su_hien_tai
    );

    return true;
}


static void SU_Gui_XacNhan_RF(uint16_t ma_lenh)
{
    // Physical 8B:
    // DST rBS | SRC SU | TYPE 0x1A | KQ=1 | MA_LENH16 | P_TX | SF.
    uint8_t goi_xac_nhan[8] = {
        ID_RBS_READY,
        ID_SU_READY,
        TYPE_XAC_NHAN_RF,
        0x01,
        (uint8_t)((ma_lenh >> 8) & 0xFF),
        (uint8_t)(ma_lenh & 0xFF),
        (uint8_t)cong_suat_phat_su_hien_tai_dbm,
        he_so_trai_pho_su_hien_tai
    };

    Phat_GoiTin_LoRa(goi_xac_nhan, sizeof(goi_xac_nhan));
    LoRa.receive();
}


// =====================================================
// XỬ LÝ LỆNH RF KHI SU ĐANG IDLE
// Chỉ chạy khi SU idle; COMMIT có thể giữ SU tối đa ~0.8 s để đổi SF đồng bộ.
// =====================================================
void XuLy_DieuKhien_RF_SU()
{
    int packet_size = LoRa.parsePacket();
    if (packet_size <= 0)
        return;

    uint8_t goi_tin[64] = {};
    size_t n = 0;
    while (LoRa.available() && n < sizeof(goi_tin))
        goi_tin[n++] = (uint8_t)LoRa.read();
    while (LoRa.available())
        LoRa.read();

    LoRa.receive();

    if (packet_size != 12 || n != 12)
        return;

    bool dung_dich = goi_tin[0] == ID_SU_READY || goi_tin[0] == ID_QUANG_BA_RF;
    if (
        !dung_dich
        || goi_tin[1] != ID_RBS_READY
        || goi_tin[2] != TYPE_DIEU_KHIEN_RF
        || goi_tin[4] != PHIEN_BAN_LENH_RF
    )
    {
        return;
    }

    uint8_t lenh = goi_tin[3];
    uint16_t ma_lenh = ((uint16_t)goi_tin[5] << 8) | goi_tin[6];
    int8_t cong_suat_su_moi = (int8_t)goi_tin[7];
    uint8_t sf_moi = goi_tin[9];
    uint32_t do_tre_ms = (uint32_t)goi_tin[10] * 100UL;

    if (
        ma_lenh == 0
        || cong_suat_su_moi < 8 || cong_suat_su_moi > 20
        || sf_moi < 7 || sf_moi > 9
    )
    {
        return;
    }

    if (lenh == LENH_RF_CHUAN_BI)
    {
        ma_lenh_rf_cho_ap_dung = ma_lenh;
        cong_suat_su_cho_ap_dung_dbm = cong_suat_su_moi;
        sf_cho_ap_dung = sf_moi;

        // ACK tại cấu hình CŨ; rBS chỉ COMMIT nếu cả SU và DU đều ACK.
        SU_Gui_XacNhan_RF(ma_lenh);
        return;
    }

    if (
        lenh == LENH_RF_AP_DUNG
        && ma_lenh_rf_cho_ap_dung == ma_lenh
    )
    {
        // SU không có RX task riêng. Vì vậy sau khi nhận COMMIT, giữ main ở trạng
        // thái idle trong đúng khoảng trì hoãn rồi áp dụng ngay. Cách này tránh
        // trường hợp người dùng bấm PTT giữa COMMIT và mốc đổi SF làm SU đổi muộn.
        delay(max((uint32_t)200UL, do_tre_ms));
        SU_ApDung_CauHinh_RF(
            cong_suat_su_cho_ap_dung_dbm,
            sf_cho_ap_dung
        );
        return;
    }

    // Lệnh khôi phục được phép tự chứa cấu hình, kể cả khi PREPARE trước đó mất.
    if (lenh == LENH_RF_KHOI_PHUC)
    {
        ma_lenh_rf_cho_ap_dung = ma_lenh;
        cong_suat_su_cho_ap_dung_dbm = CONG_SUAT_PHAT_MAC_DINH_SU_DBM;
        sf_cho_ap_dung = 7;
        delay(max((uint32_t)200UL, do_tre_ms));
        SU_ApDung_CauHinh_RF(
            cong_suat_su_cho_ap_dung_dbm,
            sf_cho_ap_dung
        );
    }
}


// =====================================================
// BAO CAO GPS / VI TRI SU -> rBS, 44B
//
// [0]     DICH=rBS
// [1]     NGUON=SU
// [2]     0x17 = GPS gan voi SESSION, 0x18 = VI TRI DINH KY
// [3]     FLAGS: bit0 GPS, bit1 DO_CAO, bit2 TOC_DO, bit3 HDOP
// [4..11] 0x17: MA_PHIEN64 | 0x18: SO_THU_TU_BAO_CAO64
// [12..15] VI_DO_E7 int32
// [16..19] KINH_DO_E7 int32
// [20..23] DO_CAO_CM int32
// [24..27] TOC_DO_CM_S uint32
// [28] SO_VE_TINH
// [29] CONG_SUAT_PHAT_DBM int8
// [30..31] HDOP_X100 uint16
// [32..35] TUOI_FIX_MS uint32
// [36..39] NGAY_UTC_YYYYMMDD uint32
// [40..43] GIO_UTC_MS_TRONG_NGAY uint32
// =====================================================
static void Ghi_U32_BE(uint8_t *con_tro, uint32_t gia_tri)
{
    con_tro[0] = (gia_tri >> 24) & 0xFF;
    con_tro[1] = (gia_tri >> 16) & 0xFF;
    con_tro[2] = (gia_tri >> 8) & 0xFF;
    con_tro[3] = gia_tri & 0xFF;
}

static void Ghi_U64_BE(uint8_t *con_tro, uint64_t gia_tri)
{
    for (int i = 0; i < 8; ++i)
        con_tro[i] = (uint8_t)((gia_tri >> (56 - 8 * i)) & 0xFF);
}

static bool Gui_Goi_ViTri_SU(
    uint8_t loai_bao_cao,
    uint64_t ma_tham_chieu,
    const DuLieuGPS_SU &du_lieu_gps,
    bool bat_lai_che_do_thu)
{
    if (ma_tham_chieu == 0) return false;
    if (loai_bao_cao != TYPE_GPS_REPORT && loai_bao_cao != TYPE_VI_TRI_DINH_KY)
        return false;

    uint8_t goi_tin[SIZE_GPS_REPORT] = {};
    goi_tin[0] = ID_RBS_READY;
    goi_tin[1] = ID_SU_READY;
    goi_tin[2] = loai_bao_cao;

    uint8_t co_hieu = 0;
    if (du_lieu_gps.gps_hop_le) co_hieu |= 0x01;
    if (du_lieu_gps.do_cao_hop_le) co_hieu |= 0x02;
    if (du_lieu_gps.toc_do_hop_le) co_hieu |= 0x04;
    if (du_lieu_gps.hdop_hop_le) co_hieu |= 0x08;

    // 4 bit cao mang SF hien tai; SF7->1, SF8->2, SF9->3.
    co_hieu |= (uint8_t)(((Lay_HeSo_TraiPho_SU() - 6) & 0x0F) << 4);
    goi_tin[3] = co_hieu;

    Ghi_U64_BE(&goi_tin[4], ma_tham_chieu);

    int32_t vi_do_e7 = du_lieu_gps.gps_hop_le
        ? (int32_t)llround(du_lieu_gps.vi_do * 10000000.0) : 0;
    int32_t kinh_do_e7 = du_lieu_gps.gps_hop_le
        ? (int32_t)llround(du_lieu_gps.kinh_do * 10000000.0) : 0;
    int32_t do_cao_cm = du_lieu_gps.do_cao_hop_le
        ? (int32_t)llround(du_lieu_gps.do_cao_m * 100.0) : 0;
    uint32_t toc_do_cm_s = du_lieu_gps.toc_do_hop_le && du_lieu_gps.toc_do_m_s > 0.0
        ? (uint32_t)llround(du_lieu_gps.toc_do_m_s * 100.0) : 0;
    uint16_t hdop_x100 = du_lieu_gps.hdop_hop_le && du_lieu_gps.hdop > 0.0
        ? (uint16_t)min(65535L, (long)llround(du_lieu_gps.hdop * 100.0)) : 0;

    Ghi_U32_BE(&goi_tin[12], (uint32_t)vi_do_e7);
    Ghi_U32_BE(&goi_tin[16], (uint32_t)kinh_do_e7);
    Ghi_U32_BE(&goi_tin[20], (uint32_t)do_cao_cm);
    Ghi_U32_BE(&goi_tin[24], toc_do_cm_s);
    goi_tin[28] = du_lieu_gps.so_ve_tinh;
    goi_tin[29] = (uint8_t)Lay_CongSuat_Phat_SU_dBm();
    goi_tin[30] = (hdop_x100 >> 8) & 0xFF;
    goi_tin[31] = hdop_x100 & 0xFF;
    Ghi_U32_BE(&goi_tin[32], du_lieu_gps.tuoi_fix_ms);
    Ghi_U32_BE(&goi_tin[36], du_lieu_gps.ngay_utc_yyyymmdd);
    Ghi_U32_BE(&goi_tin[40], du_lieu_gps.gio_utc_ms_trong_ngay);

    bool tx_ok = Phat_GoiTin_LoRa(goi_tin, sizeof(goi_tin));

    if (bat_lai_che_do_thu)
        LoRa.receive();

    const char *ten_loai = loai_bao_cao == TYPE_VI_TRI_DINH_KY
        ? "VI_TRI_DINH_KY" : "GPS_PHIEN";

    Serial.printf(
        "[SU GPS TX] %s -> rBS | MA=%016llX | HOP_LE=%u | VI_DO=%.7f | KINH_DO=%.7f | DO_CAO=%.1fm | TOC_DO=%.2fm/s | VE_TINH=%u | HDOP=%.2f | P_TX=%d dBm | SF=%u | SIZE=%uB\n",
        ten_loai,
        (unsigned long long)ma_tham_chieu,
        du_lieu_gps.gps_hop_le ? 1 : 0,
        du_lieu_gps.vi_do,
        du_lieu_gps.kinh_do,
        du_lieu_gps.do_cao_m,
        du_lieu_gps.toc_do_m_s,
        du_lieu_gps.so_ve_tinh,
        du_lieu_gps.hdop,
        Lay_CongSuat_Phat_SU_dBm(),
        Lay_HeSo_TraiPho_SU(),
        (unsigned int)sizeof(goi_tin)
    );

    return tx_ok;
}

bool Gui_GPS_REPORT_SU(
    uint64_t ma_phien,
    const DuLieuGPS_SU &du_lieu_gps)
{
    return Gui_Goi_ViTri_SU(TYPE_GPS_REPORT, ma_phien, du_lieu_gps, false);
}

bool Gui_VI_TRI_DINH_KY_SU(
    uint64_t so_thu_tu_bao_cao,
    const DuLieuGPS_SU &du_lieu_gps)
{
    return Gui_Goi_ViTri_SU(TYPE_VI_TRI_DINH_KY, so_thu_tu_bao_cao, du_lieu_gps, true);
}
