#include "nhan_lora.h"

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <LoRa.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
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

    // Dat ro cong suat de rBS tinh he so kenh tu RSSI.
    LoRa.setTxPower(CONG_SUAT_PHAT_DU_DBM);

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

    // Luu RSSI/SNR cua packet vua nhan TRUOC khi re-arm RX.
    // Day la thong tin quan trong de tach loi rBS->DU khoi loi DU->rBS.
    int rssi_packet_dbm = LoRa.packetRssi();
    float snr_packet_db = LoRa.packetSnr();

    // GIU FIX IDLE: quay lai RX continuous ngay sau khi doc packet.
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

            Serial.printf(
                "[DU PHY RX] USER_CONFIRM rBS->DU | LEN=12 | RSSI=%d dBm | SNR=%.1f dB\n",
                rssi_packet_dbm,
                snr_packet_db
            );

            xSemaphoreGive(LoRa_Mutex);
            return true;
        }

        xSemaphoreGive(LoRa_Mutex);
        return false;
    }

    // QUAN TRONG: bo packet SU truc tiep.
    // Kien truc he thong la SU -> rBS -> DU, KHONG phai SU -> DU truc tiep.
    // Vi vay viec DU khong xu ly packet raw cua SU la CHU DICH, khong phai loi.
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

            Serial.printf(
                "[DU PHY RX] END rBS->DU | LEN=%d | RSSI=%d dBm | SNR=%.1f dB\n",
                packetSize,
                rssi_packet_dbm,
                snr_packet_db
            );

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

    uint8_t loai_inner = inner[2] & 0x0F;
    Serial.printf(
        "[DU PHY RX] RELAY rBS->DU | INNER_TYPE=0x%02X | LEN=%d | RSSI=%d dBm | SNR=%.1f dB\n",
        loai_inner,
        packetSize,
        rssi_packet_dbm,
        snr_packet_db
    );

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
// KIEM TRA KENH IM LANG TRUOC BEACON DINH KY
//
// Beacon la telemetry best-effort, nen KHONG duoc tranh quyen voi RX.
// DIO0 chi len HIGH khi RxDone; vi vay V10 cho them mot khoang nghe ngan
// truoc khi chuyen radio sang TX. SESSION_START 16B co airtime ngan hon
// khoang guard nay o SF7/BW500, nen neu rBS dang gui START thi DU co co hoi
// nhan xong va DIO0 len HIGH -> beacon se tu hoan.
// =====================================================
static bool DU_Kenh_Im_Lang_Cho_Beacon(uint32_t guard_ms)
{
    uint32_t bat_dau = millis();

    while (millis() - bat_dau < guard_ms)
    {
        if (digitalRead(LORA_DIO0) == HIGH)
            return false;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return true;
}


// =====================================================
// BAO CAO GPS / VI TRI DU -> rBS, 44B
// Dung chung LoRa_Mutex voi RX / PLAY_REPORT / HMI.
// =====================================================
static void DU_Ghi_U32_BE(uint8_t *con_tro, uint32_t gia_tri)
{
    con_tro[0] = (gia_tri >> 24) & 0xFF;
    con_tro[1] = (gia_tri >> 16) & 0xFF;
    con_tro[2] = (gia_tri >> 8) & 0xFF;
    con_tro[3] = gia_tri & 0xFF;
}

static void DU_Ghi_U64_BE(uint8_t *con_tro, uint64_t gia_tri)
{
    for (int i = 0; i < 8; ++i)
        con_tro[i] = (uint8_t)((gia_tri >> (56 - 8 * i)) & 0xFF);
}

static bool Gui_Goi_ViTri_DU(
    uint8_t loai_bao_cao,
    uint64_t ma_tham_chieu,
    const DuLieuGPS_DU &du_lieu_gps,
    bool la_bao_cao_dinh_ky)
{
    if (ma_tham_chieu == 0 || LoRa_Mutex == nullptr) return false;
    if (loai_bao_cao != TYPE_GPS_REPORT && loai_bao_cao != TYPE_VI_TRI_DINH_KY)
        return false;

    if (la_bao_cao_dinh_ky)
    {
        // V10: beacon best-effort KHONG cho mutex toi 80 ms nua.
        // Neu radio dang co packet RxDone hoac khong im lang -> bo qua/thu lai sau.
        if (digitalRead(LORA_DIO0) == HIGH)
            return false;

        if (!DU_Kenh_Im_Lang_Cho_Beacon(30UL))
            return false;

        // Try-lock = 0 tick: telemetry khong duoc xep hang truoc RX/control.
        if (xSemaphoreTake(LoRa_Mutex, 0) != pdTRUE)
            return false;

        // Re-check sau khi chiem mutex de dong cua race cuoi cung.
        if (digitalRead(LORA_DIO0) == HIGH)
        {
            xSemaphoreGive(LoRa_Mutex);
            return false;
        }
    }
    else
    {
        // GPS gan SESSION la control telemetry, duoc gui sau START.
        delay(8);

        if (xSemaphoreTake(LoRa_Mutex, pdMS_TO_TICKS(80)) != pdTRUE)
        {
            Serial.println("[DU GPS] Khong lay duoc LoRa mutex");
            return false;
        }
    }

    uint8_t goi_tin[SIZE_GPS_REPORT] = {};
    goi_tin[0] = ID_TRAM_RBS;
    goi_tin[1] = ID_TRAM_DU;
    goi_tin[2] = loai_bao_cao;

    uint8_t co_hieu = 0;
    if (du_lieu_gps.gps_hop_le) co_hieu |= 0x01;
    if (du_lieu_gps.do_cao_hop_le) co_hieu |= 0x02;
    if (du_lieu_gps.toc_do_hop_le) co_hieu |= 0x04;
    if (du_lieu_gps.hdop_hop_le) co_hieu |= 0x08;
    goi_tin[3] = co_hieu;

    DU_Ghi_U64_BE(&goi_tin[4], ma_tham_chieu);

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

    DU_Ghi_U32_BE(&goi_tin[12], (uint32_t)vi_do_e7);
    DU_Ghi_U32_BE(&goi_tin[16], (uint32_t)kinh_do_e7);
    DU_Ghi_U32_BE(&goi_tin[20], (uint32_t)do_cao_cm);
    DU_Ghi_U32_BE(&goi_tin[24], toc_do_cm_s);
    goi_tin[28] = du_lieu_gps.so_ve_tinh;
    goi_tin[29] = (uint8_t)(int8_t)CONG_SUAT_PHAT_DU_DBM;
    goi_tin[30] = (hdop_x100 >> 8) & 0xFF;
    goi_tin[31] = hdop_x100 & 0xFF;
    DU_Ghi_U32_BE(&goi_tin[32], du_lieu_gps.tuoi_fix_ms);
    DU_Ghi_U32_BE(&goi_tin[36], du_lieu_gps.ngay_utc_yyyymmdd);
    DU_Ghi_U32_BE(&goi_tin[40], du_lieu_gps.gio_utc_ms_trong_ngay);

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write(goi_tin, sizeof(goi_tin));
    int ket_qua_tx = LoRa.endPacket();
    LoRa.receive();

    xSemaphoreGive(LoRa_Mutex);

    const char *ten_loai = loai_bao_cao == TYPE_VI_TRI_DINH_KY
        ? "VI_TRI_DINH_KY" : "GPS_PHIEN";

    Serial.printf(
        "[DU GPS TX] %s -> rBS | MA=%016llX | HOP_LE=%u | VI_DO=%.7f | KINH_DO=%.7f | DO_CAO=%.1fm | TOC_DO=%.2fm/s | VE_TINH=%u | HDOP=%.2f | P_TX=%d dBm | TX=%s\n",
        ten_loai,
        (unsigned long long)ma_tham_chieu,
        du_lieu_gps.gps_hop_le ? 1 : 0,
        du_lieu_gps.vi_do,
        du_lieu_gps.kinh_do,
        du_lieu_gps.do_cao_m,
        du_lieu_gps.toc_do_m_s,
        du_lieu_gps.so_ve_tinh,
        du_lieu_gps.hdop,
        CONG_SUAT_PHAT_DU_DBM,
        ket_qua_tx == 1 ? "OK" : "FAIL"
    );

    return ket_qua_tx == 1;
}

bool Gui_GPS_REPORT_DU(
    uint64_t ma_phien,
    const DuLieuGPS_DU &du_lieu_gps)
{
    return Gui_Goi_ViTri_DU(TYPE_GPS_REPORT, ma_phien, du_lieu_gps, false);
}

bool Gui_VI_TRI_DINH_KY_DU(
    uint64_t so_thu_tu_bao_cao,
    const DuLieuGPS_DU &du_lieu_gps)
{
    return Gui_Goi_ViTri_DU(TYPE_VI_TRI_DINH_KY, so_thu_tu_bao_cao, du_lieu_gps, true);
}
