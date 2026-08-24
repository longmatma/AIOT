#include "nhan_lora.h"

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


// =====================================================
// CHAN LORA
// =====================================================

#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10
#define LORA_RST   9
#define LORA_DIO0  14


// =====================================================
// ID
// =====================================================

#define ID_TRAM_SU   0x01
#define ID_TRAM_DU   0x02
#define ID_TRAM_RBS  0x03


// =====================================================
// TYPE
// =====================================================

#define TYPE_RELAY          0x10
#define TYPE_RELAY_END      0x11
#define TYPE_PLAY_STARTED   0x12

// TYPE noi bo dua sang main.cpp.
#define TYPE_AUDIO_END      0x04


// =====================================================
// KICH THUOC
// =====================================================

#define SIZE_SESSION_INNER  12
#define SIZE_VOICE_INNER   176
#define SIZE_FEC_INNER     184

#define SIZE_SESSION_RELAY  16
#define SIZE_VOICE_RELAY   180
#define SIZE_FEC_RELAY     188
#define SIZE_END_RELAY       5


// RX task va PLAY_REPORT task cung dung mot SX1278.
// Mutex ngan hai task cham SPI/radio cung luc.
static SemaphoreHandle_t LoRa_Mutex = nullptr;


// =====================================================
// KHOI TAO LORA RX
// =====================================================

void KhoiTao_LoRa_RX()
{
    LoRa_Mutex =
        xSemaphoreCreateMutex();

    if (LoRa_Mutex == nullptr)
    {
        Serial.println(
            "[DU ERROR] Khong tao duoc LoRa mutex!"
        );

        while (1)
        {
            delay(1000);
        }
    }

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
            "LOI: Khong tim thay module LoRa RX!"
        );

        while (1)
        {
            delay(1000);
        }
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(500E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();

    pinMode(
        LORA_DIO0,
        INPUT
    );

    LoRa.receive();

    Serial.println(
        "Khoi tao LoRa RX THANH CONG!"
    );
}


// =====================================================
// NHAN PACKET rBS -> DU
//
// GIU FIX IDLE:
// - DIO0 INPUT
// - chi parsePacket khi RX_DONE HIGH
// - sau packet / parse error quay lai LoRa.receive()
// =====================================================

bool Nhan_GoiTin_LoRa(
    uint8_t* buffer,
    size_t &do_dai_nhan)
{
    do_dai_nhan = 0;

    if (LoRa_Mutex == nullptr)
    {
        return false;
    }

    if (
        xSemaphoreTake(
            LoRa_Mutex,
            pdMS_TO_TICKS(2)
        )
        != pdTRUE
    )
    {
        return false;
    }

    if (
        digitalRead(LORA_DIO0)
        == LOW
    )
    {
        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    int packetSize =
        LoRa.parsePacket();

    if (packetSize <= 0)
    {
        LoRa.receive();

        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    uint8_t raw_packet[256];
    size_t raw_len = 0;

    while (
        LoRa.available()
        && raw_len < sizeof(raw_packet)
    )
    {
        raw_packet[raw_len++] =
            (uint8_t)LoRa.read();
    }

    while (LoRa.available())
    {
        LoRa.read();
    }

    // Re-arm RX continuous NGAY sau khi doc packet.
    LoRa.receive();

    if (
        raw_len
        != (size_t)packetSize
    )
    {
        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    // Bo packet SU truc tiep; DU chi nhan qua rBS.
    if (
        packetSize == SIZE_SESSION_INNER
        || packetSize == SIZE_VOICE_INNER
        || packetSize == SIZE_FEC_INNER
    )
    {
        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    // END AUDIO 5B outer -> packet noi bo 4B.
    if (packetSize == SIZE_END_RELAY)
    {
        if (
            raw_packet[0] == ID_TRAM_DU
            && raw_packet[1] == ID_TRAM_RBS
            && raw_packet[2] == TYPE_RELAY_END
        )
        {
            memset(
                buffer,
                0,
                SIZE_FEC_INNER
            );

            buffer[0] = ID_TRAM_DU;
            buffer[1] = ID_TRAM_RBS;
            buffer[2] = TYPE_AUDIO_END;
            buffer[3] = 0;

            do_dai_nhan = 4;

            xSemaphoreGive(
                LoRa_Mutex
            );

            return true;
        }

        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    if (
        packetSize != SIZE_SESSION_RELAY
        && packetSize != SIZE_VOICE_RELAY
        && packetSize != SIZE_FEC_RELAY
    )
    {
        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    uint8_t outer_dst = raw_packet[0];
    uint8_t outer_src = raw_packet[1];
    uint8_t outer_type = raw_packet[2];
    uint8_t inner_len = raw_packet[3];

    if (
        outer_dst != ID_TRAM_DU
        || outer_src != ID_TRAM_RBS
        || outer_type != TYPE_RELAY
    )
    {
        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    if (
        inner_len != SIZE_SESSION_INNER
        && inner_len != SIZE_VOICE_INNER
        && inner_len != SIZE_FEC_INNER
    )
    {
        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    if (
        packetSize
        != (4 + inner_len)
    )
    {
        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    uint8_t *inner =
        &raw_packet[4];

    if (
        inner[0] != ID_TRAM_DU
        || inner[1] != ID_TRAM_SU
    )
    {
        xSemaphoreGive(
            LoRa_Mutex
        );

        return false;
    }

    memset(
        buffer,
        0,
        SIZE_FEC_INNER
    );

    memcpy(
        buffer,
        inner,
        inner_len
    );

    do_dai_nhan =
        inner_len;

    xSemaphoreGive(
        LoRa_Mutex
    );

    return true;
}


// =====================================================
// DU -> rBS: PLAY_STARTED
//
// Goi SAU khi first PWM sample da duoc ghi ra GPIO audio.
// Physical 12B:
//   [DST=rBS][SRC=DU][TYPE=0x12][flags=0][SESSION_ID64]
// =====================================================

bool Gui_PLAY_STARTED_RBS(
    uint64_t session_id)
{
    if (
        session_id == 0
        || LoRa_Mutex == nullptr
    )
    {
        return false;
    }

    if (
        xSemaphoreTake(
            LoRa_Mutex,
            pdMS_TO_TICKS(50)
        )
        != pdTRUE
    )
    {
        Serial.println(
            "[DU PLAY REPORT] Khong lay duoc LoRa mutex"
        );

        return false;
    }

    uint8_t report[12];

    report[0] = ID_TRAM_RBS;
    report[1] = ID_TRAM_DU;
    report[2] = TYPE_PLAY_STARTED;
    report[3] = 0x00;

    report[4]  = (session_id >> 56) & 0xFF;
    report[5]  = (session_id >> 48) & 0xFF;
    report[6]  = (session_id >> 40) & 0xFF;
    report[7]  = (session_id >> 32) & 0xFF;
    report[8]  = (session_id >> 24) & 0xFF;
    report[9]  = (session_id >> 16) & 0xFF;
    report[10] = (session_id >> 8)  & 0xFF;
    report[11] =  session_id        & 0xFF;

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write(
        report,
        sizeof(report)
    );

    int ok =
        LoRa.endPacket();

    // Bao xong phai quay lai RX continuous.
    LoRa.receive();

    xSemaphoreGive(
        LoRa_Mutex
    );

    if (ok == 1)
    {
        Serial.printf(
            "[DU CTRL] PLAY_STARTED -> rBS | SESSION=%016llX\n",
            (unsigned long long)session_id
        );

        return true;
    }

    Serial.println(
        "[DU PLAY REPORT] TX FAIL"
    );

    return false;
}
