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

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(500E3);

    // GIỮ CR 4/5.
    // Packet-level FEC xử lý mất packet ở tầng trên;
    // không tăng airtime toàn bộ link bằng CR4/6-4/8 mặc định.
    LoRa.setCodingRate4(5);

    LoRa.enableCrc();

    LoRa.idle();

    Serial.println(
        "Khoi tao LoRa THANH CONG! | SF7 BW500 CR4/5"
    );
}


// =====================================================
// TX BLOCKING
// =====================================================

void Phat_GoiTin_LoRa(
    uint8_t *goi_tin,
    size_t do_dai)
{
    LoRa.idle();

    LoRa.beginPacket();

    LoRa.write(
        goi_tin,
        do_dai
    );

    LoRa.endPacket();
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
// GPS REPORT SU -> rBS, 44B
//
// [0]     DST=rBS
// [1]     SRC=SU
// [2]     TYPE_GPS_REPORT=0x17
// [3]     FLAGS: bit0 GPS_VALID, bit1 ALT, bit2 SPEED, bit3 HDOP
// [4..11] SESSION_ID64
// [12..15] LAT_E7 int32
// [16..19] LON_E7 int32
// [20..23] ALT_CM int32
// [24..27] SPEED_CM_S uint32
// [28] SAT
// [29] reserved
// [30..31] HDOP_X100 uint16
// [32..35] FIX_AGE_MS uint32
// [36..39] UTC_DATE_YYYYMMDD uint32
// [40..43] UTC_TIME_MS_OF_DAY uint32
// =====================================================
static void Ghi_U32_BE(uint8_t *p, uint32_t v)
{
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;
    p[3] = v & 0xFF;
}

static void Ghi_U64_BE(uint8_t *p, uint64_t v)
{
    for (int i = 0; i < 8; ++i)
        p[i] = (uint8_t)((v >> (56 - 8 * i)) & 0xFF);
}

bool Gui_GPS_REPORT_SU(
    uint64_t session_id,
    const DuLieuGPS_SU &gps)
{
    if (session_id == 0) return false;

    uint8_t packet[SIZE_GPS_REPORT] = {};
    packet[0] = ID_RBS_READY;
    packet[1] = ID_SU_READY;
    packet[2] = TYPE_GPS_REPORT;

    uint8_t flags = 0;
    if (gps.gps_valid) flags |= 0x01;
    if (gps.altitude_valid) flags |= 0x02;
    if (gps.speed_valid) flags |= 0x04;
    if (gps.hdop_valid) flags |= 0x08;
    packet[3] = flags;

    Ghi_U64_BE(&packet[4], session_id);

    int32_t lat_e7 = gps.gps_valid ? (int32_t)llround(gps.latitude * 10000000.0) : 0;
    int32_t lon_e7 = gps.gps_valid ? (int32_t)llround(gps.longitude * 10000000.0) : 0;
    int32_t alt_cm = gps.altitude_valid ? (int32_t)llround(gps.altitude_m * 100.0) : 0;
    uint32_t speed_cms = gps.speed_valid && gps.speed_mps > 0.0
        ? (uint32_t)llround(gps.speed_mps * 100.0)
        : 0;
    uint16_t hdop_x100 = gps.hdop_valid && gps.hdop > 0.0
        ? (uint16_t)min(65535L, (long)llround(gps.hdop * 100.0))
        : 0;

    Ghi_U32_BE(&packet[12], (uint32_t)lat_e7);
    Ghi_U32_BE(&packet[16], (uint32_t)lon_e7);
    Ghi_U32_BE(&packet[20], (uint32_t)alt_cm);
    Ghi_U32_BE(&packet[24], speed_cms);
    packet[28] = gps.satellites;
    packet[29] = 0;
    packet[30] = (hdop_x100 >> 8) & 0xFF;
    packet[31] = hdop_x100 & 0xFF;
    Ghi_U32_BE(&packet[32], gps.fix_age_ms);
    Ghi_U32_BE(&packet[36], gps.utc_date_yyyymmdd);
    Ghi_U32_BE(&packet[40], gps.utc_time_ms_of_day);

    Phat_GoiTin_LoRa(packet, sizeof(packet));

    Serial.printf(
        "[SU GPS TX] GPS_REPORT -> rBS | SESSION=%016llX | VALID=%u | LAT=%.7f | LON=%.7f | SAT=%u | HDOP=%.2f | AGE=%u ms | SIZE=%uB\n",
        (unsigned long long)session_id,
        gps.gps_valid ? 1 : 0,
        gps.latitude,
        gps.longitude,
        gps.satellites,
        gps.hdop,
        (unsigned int)gps.fix_age_ms,
        (unsigned int)sizeof(packet)
    );

    return true;
}
