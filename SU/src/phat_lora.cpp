#include "phat_lora.h"

#include <Arduino.h>
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
