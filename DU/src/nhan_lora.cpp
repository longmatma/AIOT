#include "nhan_lora.h"

#include <Arduino.h>
#include <math.h>
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
#define TYPE_USER_RESPONSE  0x13
#define TYPE_USER_CONFIRM   0x14
#define TYPE_SESSION_READY  0x15

// TYPE noi bo dua sang main.cpp.
#define TYPE_AUDIO_END           0x04
#define TYPE_USER_CONFIRM_LOCAL  0x06


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

    // USER_CONFIRM rBS -> DU, physical 12B.
    // Chuyen type 0x14 -> type noi bo 0x06 de khong va cham TYPE_MASK.
    if (
        packetSize == 12
        && raw_packet[0] == ID_TRAM_DU
        && raw_packet[1] == ID_TRAM_RBS
        && raw_packet[2] == TYPE_USER_CONFIRM
    )
    {
        uint8_t code = raw_packet[3];

        if (code == USER_RESPONSE_ACK || code == USER_RESPONSE_NACK)
        {
            memset(buffer, 0, SIZE_FEC_INNER);
            buffer[0] = ID_TRAM_DU;
            buffer[1] = ID_TRAM_RBS;
            buffer[2] = TYPE_USER_CONFIRM_LOCAL;
            buffer[3] = code;
            memcpy(&buffer[4], &raw_packet[4], 8);
            do_dai_nhan = 12;
            xSemaphoreGive(LoRa_Mutex);
            return true;
        }

        xSemaphoreGive(LoRa_Mutex);
        return false;
    }

    // Bo packet SU truc tiep; DU chi nhan DATA qua rBS.
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
// DU -> rBS: SESSION_READY
//
// DU tu dong gui ngay sau khi da nhan/chap nhan SESSION_START.
// Physical 12B:
//   [DST=rBS][SRC=DU][TYPE=0x15][flags=0][SESSION_ID64]
// =====================================================
bool Gui_SESSION_READY_RBS(
    uint64_t session_id)
{
    if (
        session_id == 0
        || LoRa_Mutex == nullptr
    )
    {
        return false;
    }

    // Cho rBS ket thuc TX SESSION_START va quay lai RX.
    delay(8);

    if (
        xSemaphoreTake(
            LoRa_Mutex,
            pdMS_TO_TICKS(80)
        )
        != pdTRUE
    )
    {
        Serial.println("[DU SESSION] Khong lay duoc LoRa mutex");
        return false;
    }

    uint8_t packet[12];
    packet[0] = ID_TRAM_RBS;
    packet[1] = ID_TRAM_DU;
    packet[2] = TYPE_SESSION_READY;
    packet[3] = 0x00;
    packet[4]  = (session_id >> 56) & 0xFF;
    packet[5]  = (session_id >> 48) & 0xFF;
    packet[6]  = (session_id >> 40) & 0xFF;
    packet[7]  = (session_id >> 32) & 0xFF;
    packet[8]  = (session_id >> 24) & 0xFF;
    packet[9]  = (session_id >> 16) & 0xFF;
    packet[10] = (session_id >> 8)  & 0xFF;
    packet[11] =  session_id        & 0xFF;

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write(packet, sizeof(packet));
    int ok = LoRa.endPacket();
    LoRa.receive();

    xSemaphoreGive(LoRa_Mutex);

    Serial.printf(
        "[DU SESSION] SESSION_READY -> rBS | SESSION=%016llX | TX=%s\n",
        (unsigned long long)session_id,
        ok == 1 ? "OK" : "FAIL"
    );

    return ok == 1;
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


// =====================================================
// DU -> rBS: USER ACK/NACK
// Physical 12B: [DST=rBS][SRC=DU][TYPE=0x13][flags][SESSION64]
// =====================================================
bool Gui_USER_RESPONSE_RBS(
    uint64_t session_id,
    uint8_t response_code)
{
    if (
        session_id == 0
        || LoRa_Mutex == nullptr
        || (response_code != USER_RESPONSE_ACK && response_code != USER_RESPONSE_NACK)
    )
    {
        return false;
    }

    if (xSemaphoreTake(LoRa_Mutex, pdMS_TO_TICKS(80)) != pdTRUE)
    {
        Serial.println("[DU HMI] Khong lay duoc LoRa mutex");
        return false;
    }

    uint8_t packet[12];
    packet[0] = ID_TRAM_RBS;
    packet[1] = ID_TRAM_DU;
    packet[2] = TYPE_USER_RESPONSE;
    packet[3] = response_code;
    packet[4]  = (session_id >> 56) & 0xFF;
    packet[5]  = (session_id >> 48) & 0xFF;
    packet[6]  = (session_id >> 40) & 0xFF;
    packet[7]  = (session_id >> 32) & 0xFF;
    packet[8]  = (session_id >> 24) & 0xFF;
    packet[9]  = (session_id >> 16) & 0xFF;
    packet[10] = (session_id >> 8)  & 0xFF;
    packet[11] =  session_id        & 0xFF;

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write(packet, sizeof(packet));
    int ok = LoRa.endPacket();
    LoRa.receive();
    xSemaphoreGive(LoRa_Mutex);

    Serial.printf(
        "[DU HMI] USER_%s -> rBS | SESSION=%016llX | TX=%s\n",
        response_code == USER_RESPONSE_ACK ? "ACK" : "NACK",
        (unsigned long long)session_id,
        ok == 1 ? "OK" : "FAIL"
    );

    return ok == 1;
}


// =====================================================
// GPS REPORT DU -> rBS, 44B. Dung cung LoRa_Mutex voi RX/PLAY_REPORT.
// DU gui GPS snapshot truoc SESSION_READY de rBS co the thu no ngay
// trong cua so bat tay, nhung GPS KHONG quyet dinh session thanh/bai.
// =====================================================
static void DU_Ghi_U32_BE(uint8_t *p, uint32_t v)
{
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;
    p[3] = v & 0xFF;
}

static void DU_Ghi_U64_BE(uint8_t *p, uint64_t v)
{
    for (int i = 0; i < 8; ++i)
        p[i] = (uint8_t)((v >> (56 - 8 * i)) & 0xFF);
}

bool Gui_GPS_REPORT_DU(
    uint64_t session_id,
    const DuLieuGPS_DU &gps)
{
    if (session_id == 0 || LoRa_Mutex == nullptr) return false;

    // rBS vua ket thuc TX SESSION_START; cho no quay lai RX.
    delay(8);

    if (xSemaphoreTake(LoRa_Mutex, pdMS_TO_TICKS(80)) != pdTRUE)
    {
        Serial.println("[DU GPS] Khong lay duoc LoRa mutex");
        return false;
    }

    uint8_t packet[SIZE_GPS_REPORT] = {};
    packet[0] = ID_TRAM_RBS;
    packet[1] = ID_TRAM_DU;
    packet[2] = TYPE_GPS_REPORT;

    uint8_t flags = 0;
    if (gps.gps_valid) flags |= 0x01;
    if (gps.altitude_valid) flags |= 0x02;
    if (gps.speed_valid) flags |= 0x04;
    if (gps.hdop_valid) flags |= 0x08;
    packet[3] = flags;

    DU_Ghi_U64_BE(&packet[4], session_id);

    int32_t lat_e7 = gps.gps_valid ? (int32_t)llround(gps.latitude * 10000000.0) : 0;
    int32_t lon_e7 = gps.gps_valid ? (int32_t)llround(gps.longitude * 10000000.0) : 0;
    int32_t alt_cm = gps.altitude_valid ? (int32_t)llround(gps.altitude_m * 100.0) : 0;
    uint32_t speed_cms = gps.speed_valid && gps.speed_mps > 0.0
        ? (uint32_t)llround(gps.speed_mps * 100.0)
        : 0;
    uint16_t hdop_x100 = gps.hdop_valid && gps.hdop > 0.0
        ? (uint16_t)min(65535L, (long)llround(gps.hdop * 100.0))
        : 0;

    DU_Ghi_U32_BE(&packet[12], (uint32_t)lat_e7);
    DU_Ghi_U32_BE(&packet[16], (uint32_t)lon_e7);
    DU_Ghi_U32_BE(&packet[20], (uint32_t)alt_cm);
    DU_Ghi_U32_BE(&packet[24], speed_cms);
    packet[28] = gps.satellites;
    packet[29] = 0;
    packet[30] = (hdop_x100 >> 8) & 0xFF;
    packet[31] = hdop_x100 & 0xFF;
    DU_Ghi_U32_BE(&packet[32], gps.fix_age_ms);
    DU_Ghi_U32_BE(&packet[36], gps.utc_date_yyyymmdd);
    DU_Ghi_U32_BE(&packet[40], gps.utc_time_ms_of_day);

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write(packet, sizeof(packet));
    int ok = LoRa.endPacket();
    LoRa.receive();

    xSemaphoreGive(LoRa_Mutex);

    Serial.printf(
        "[DU GPS TX] GPS_REPORT -> rBS | SESSION=%016llX | VALID=%u | LAT=%.7f | LON=%.7f | SAT=%u | HDOP=%.2f | AGE=%u ms | TX=%s\n",
        (unsigned long long)session_id,
        gps.gps_valid ? 1 : 0,
        gps.latitude,
        gps.longitude,
        gps.satellites,
        gps.hdop,
        (unsigned int)gps.fix_age_ms,
        ok == 1 ? "OK" : "FAIL"
    );

    return ok == 1;
}
