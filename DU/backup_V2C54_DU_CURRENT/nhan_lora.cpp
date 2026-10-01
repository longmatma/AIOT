#include "nhan_lora.h"
#include "node_config.h"

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

#define ID_TRAM_SU   V2C_SU_ID
#define ID_TRAM_DU   V2C_DU_ID
#define ID_TRAM_RBS  V2C_RBS_ID


// =====================================================
// TYPE
// =====================================================

#define TYPE_RELAY          0x10
#define TYPE_RELAY_NEXT_P1  0x2E  // R2B1: current payload unchanged; next burst VOICE is Pair1
#define TYPE_RELAY_NEXT_P2  0x2F  // R2B1: current payload unchanged; next burst VOICE is Pair2
// V2C5.3B2R2B1_DU_RELAY_MARKER_ALIAS
#define TYPE_RELAY_END      0x11
#define TYPE_PLAY_STARTED   0x12
#define TYPE_USER_RESPONSE  0x13
#define TYPE_USER_CONFIRM   0x14
#define TYPE_SESSION_READY  0x15
#define TYPE_VI_TRI_DINH_KY_SU 0x18

// TYPE noi bo dua sang main.cpp.
#define TYPE_AUDIO_END           0x04
#define TYPE_USER_CONFIRM_LOCAL  0x06


// =====================================================
// KICH THUOC
// =====================================================

#define SIZE_SESSION_INNER  12
#define SIZE_VOICE_INNER   106
#define SIZE_FEC_INNER     114

#define SIZE_SESSION_RELAY  16
#define SIZE_VOICE_RELAY   110
#define SIZE_FEC_RELAY     118
#define SIZE_END_RELAY       5
#define SIZE_GPS_REPORT_SU   44


// RX task va PLAY_REPORT task cung dung mot SX1278.
// Mutex ngan hai task cham SPI/radio cung luc.
static SemaphoreHandle_t LoRa_Mutex = nullptr;

static volatile uint32_t v2c1_network_busy_until_ms = 0;

static bool DU_V2C1_NetworkBusy()
{
    return (int32_t)(v2c1_network_busy_until_ms - millis()) > 0;
}


// =====================================================
// V2C5.3A - FRIENDLY-JAM UL WINDOW SIM (DU HELPER ONLY)
//
// Muc tieu:
// - Giu nguyen lease V2C5.2 tu beacon.
// - DU mo phong START_SIM / STOP_SIM dung khe uplink cua pair minh.
// - KHONG goi LoRa TX, KHONG doi cong suat, RF_JAM luon OFF.
// - PREPARE/COMMIT, mat lease, mat beacon, session end -> fail-safe OFF.
//
// Moc thoi gian deu tinh tu luc DU nhan xong beacon (millis()).
// SINGLE: SU active bat dau +12 ms, 2 VOICE ket thuc xap xi +105 ms.
// DUAL  : P1 +12 ms, P2 +80 ms; moi pair 1 VOICE ~45 ms, cua so SIM 50 ms.
// Day CHI la mo phong timing. KHONG duoc dung cac hang so nay de bat RF that.
// =====================================================
enum DUJam53State : uint8_t
{
    DU_JAM53_IDLE = 0,
    DU_JAM53_PREPARED = 1,
    DU_JAM53_ARMED = 2,
    DU_JAM53_WINDOW_ACTIVE = 3
};

static constexpr uint8_t DU_JAM53_MODE_SINGLE1 = 1U;
static constexpr uint8_t DU_JAM53_MODE_SINGLE2 = 2U;
static constexpr uint8_t DU_JAM53_MODE_DUAL = 3U;

static constexpr uint32_t DU_JAM53_SINGLE_UL_START_MS = 12U;
static constexpr uint32_t DU_JAM53_SINGLE_UL_STOP_MS = 105U;
static constexpr uint32_t DU_JAM53_DUAL_P1_START_MS = 12U;
static constexpr uint32_t DU_JAM53_DUAL_P2_START_MS = 80U;
static constexpr uint32_t DU_JAM53_DUAL_WINDOW_MS = 50U;
static constexpr uint32_t DU_JAM53_BEACON_FAILSAFE_MS = 700U;

// HARD LOCK: V2C5.3A chi log START/STOP_SIM, khong co duong phat RF.
static constexpr bool DU_JAM53_RF_ENABLE = false;

static portMUX_TYPE DU_Jam53_Mux = portMUX_INITIALIZER_UNLOCKED;
static volatile DUJam53State du_jam53_state = DU_JAM53_IDLE;
static volatile uint64_t du_jam53_session_id = 0;
static volatile uint32_t du_jam53_last_beacon_ms = 0;
static volatile uint32_t du_jam53_last_frame = 0;
static volatile uint8_t du_jam53_last_mask = 0;
static volatile uint8_t du_jam53_last_mode = 0;
static volatile bool du_jam53_window_valid = false;
static volatile uint32_t du_jam53_window_start_ms = 0;
static volatile uint32_t du_jam53_window_stop_ms = 0;

// V2C5.4_DU_RF_SHADOW_GATE

// ============================================================
// V2C5.4 - DU RF SHADOW GATE
// Software-only shadow for the existing V2C5.3A UL window.
// NO GPIO, NO LoRa TX, NO RF helper is called here.
// ============================================================
static constexpr bool DU_RF_SHADOW_ENABLE = true;
static bool du_rf_shadow_gate = false;
static uint32_t du_rf_shadow_on_ms = 0;
static uint32_t du_rf_shadow_on_count = 0;
static uint32_t du_rf_shadow_off_count = 0;
static uint32_t du_rf_shadow_max_window_ms = 0;

static void DU_RFShadow_Reset()
{
    du_rf_shadow_gate = false;
    du_rf_shadow_on_ms = 0;
    du_rf_shadow_on_count = 0;
    du_rf_shadow_off_count = 0;
    du_rf_shadow_max_window_ms = 0;
}

static void DU_RFShadow_Set(bool on, const char *reason)
{
    if (!DU_RF_SHADOW_ENABLE)
        return;

    if (du_rf_shadow_gate == on)
        return;

    const uint32_t now = millis();
    du_rf_shadow_gate = on;

    if (on)
    {
        du_rf_shadow_on_ms = now;
        du_rf_shadow_on_count++;

        Serial.printf(
            "[DU V2C5.4 RF_SHADOW] GATE=ON | PAIR=%u | FRAME=%u | MODE=%u | REASON=%s | RF_TX=0 | RF_JAM=OFF\\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)du_jam53_last_frame,
            (unsigned int)du_jam53_last_mode,
            reason != nullptr ? reason : "UNKNOWN"
        );
    }
    else
    {
        const uint32_t dur =
            du_rf_shadow_on_ms != 0U
                ? (uint32_t)(now - du_rf_shadow_on_ms)
                : 0U;

        if (dur > du_rf_shadow_max_window_ms)
            du_rf_shadow_max_window_ms = dur;

        du_rf_shadow_off_count++;

        Serial.printf(
            "[DU V2C5.4 RF_SHADOW] GATE=OFF | PAIR=%u | FRAME=%u | MODE=%u | DURATION=%ums | REASON=%s | RF_TX=0 | RF_JAM=OFF\\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)du_jam53_last_frame,
            (unsigned int)du_jam53_last_mode,
            (unsigned int)dur,
            reason != nullptr ? reason : "UNKNOWN"
        );

        du_rf_shadow_on_ms = 0;
    }
}

static void DU_RFShadow_Summary(const char *reason)
{
    Serial.printf(
        "[DU V2C5.4 RF_SHADOW SUMMARY] PAIR=%u | ON=%u | OFF=%u | ACTIVE=%u | MAX_WINDOW=%ums | REASON=%s | RF_TX=0 | RF_JAM=OFF\\n",
        (unsigned int)V2C_PAIR_INDEX,
        (unsigned int)du_rf_shadow_on_count,
        (unsigned int)du_rf_shadow_off_count,
        du_rf_shadow_gate ? 1U : 0U,
        (unsigned int)du_rf_shadow_max_window_ms,
        reason != nullptr ? reason : "UNKNOWN"
    );
}

static uint8_t DU_Jam53_MyMaskBit()
{
    return (uint8_t)(1U << (V2C_PAIR_INDEX - 1U));
}

static bool DU_Jam53_TimeReached(uint32_t now, uint32_t due)
{
    return (int32_t)(now - due) >= 0;
}

static bool DU_Jam53_BuildWindow(
    uint8_t mode,
    uint32_t beacon_rx_ms,
    uint32_t &start_ms,
    uint32_t &stop_ms)
{
    if (mode == DU_JAM53_MODE_SINGLE1 || mode == DU_JAM53_MODE_SINGLE2)
    {
        const uint8_t active_pair = (mode == DU_JAM53_MODE_SINGLE1) ? 1U : 2U;
        if (active_pair != (uint8_t)V2C_PAIR_INDEX)
            return false;

        start_ms = beacon_rx_ms + DU_JAM53_SINGLE_UL_START_MS;
        stop_ms = beacon_rx_ms + DU_JAM53_SINGLE_UL_STOP_MS;
        return true;
    }

    if (mode == DU_JAM53_MODE_DUAL)
    {
        const uint32_t offset_ms =
            (V2C_PAIR_INDEX == 1U)
                ? DU_JAM53_DUAL_P1_START_MS
                : DU_JAM53_DUAL_P2_START_MS;

        start_ms = beacon_rx_ms + offset_ms;
        stop_ms = start_ms + DU_JAM53_DUAL_WINDOW_MS;
        return true;
    }

    return false;
}

void DU_Jam53_Prepare(uint64_t session_id)
{
    DU_RFShadow_Reset();
    portENTER_CRITICAL(&DU_Jam53_Mux);
    du_jam53_session_id = session_id;
    du_jam53_state = DU_JAM53_PREPARED;
    du_jam53_last_mask = 0;
    du_jam53_window_valid = false;
    portEXIT_CRITICAL(&DU_Jam53_Mux);

    Serial.printf(
        "[DU JAM V2C5.3A PREPARED] PAIR=%u | SESSION=%016llX | WAIT_BEACON_LEASE=1 | RF_JAM=OFF\n",
        (unsigned int)V2C_PAIR_INDEX,
        (unsigned long long)session_id
    );
}

void DU_Jam53_Stop(const char *reason)
{
    uint64_t sid = 0;
    uint32_t frame_id = 0;
    bool was_active = false;
    bool had_state = false;

    portENTER_CRITICAL(&DU_Jam53_Mux);
    sid = du_jam53_session_id;
    frame_id = du_jam53_last_frame;
    was_active = du_jam53_state == DU_JAM53_WINDOW_ACTIVE;
    had_state = du_jam53_state != DU_JAM53_IDLE || sid != 0;
    du_jam53_state = DU_JAM53_IDLE;
    du_jam53_session_id = 0;
    du_jam53_last_mask = 0;
    du_jam53_window_valid = false;
    portEXIT_CRITICAL(&DU_Jam53_Mux);

    DU_RFShadow_Set(false, reason != nullptr ? reason : "STOP");
    if (had_state)
        DU_RFShadow_Summary(reason != nullptr ? reason : "STOP");

    if (was_active)
    {
        Serial.printf(
            "[DU JAM V2C5.3A STOP_SIM] PAIR=%u | FRAME=%u | REASON=%s | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)frame_id,
            reason != nullptr ? reason : "UNKNOWN"
        );
    }

    if (had_state)
    {
        Serial.printf(
            "[DU JAM V2C5.3A STOP] PAIR=%u | SESSION=%016llX | REASON=%s | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned long long)sid,
            reason != nullptr ? reason : "UNKNOWN"
        );
    }
}

static void DU_Jam53_OnBeacon(
    uint32_t frame_id,
    uint8_t mode,
    uint8_t target,
    uint8_t lease_mask)
{
    if (DU_JAM53_RF_ENABLE)
    {
        DU_Jam53_Stop("RF_ENABLE_FORBIDDEN");
        return;
    }

    const uint32_t now = millis();
    const uint8_t my_bit = DU_Jam53_MyMaskBit();
    const bool lease_allowed = target == 0U && ((lease_mask & my_bit) != 0U);

    uint32_t plan_start = 0;
    uint32_t plan_stop = 0;
    const bool timing_ok = lease_allowed && DU_Jam53_BuildWindow(
        mode, now, plan_start, plan_stop
    );

    DUJam53State old_state;
    DUJam53State new_state;
    uint64_t sid;
    bool abort_active = false;
    const char *revoke_reason = nullptr;

    portENTER_CRITICAL(&DU_Jam53_Mux);
    old_state = du_jam53_state;
    sid = du_jam53_session_id;

    du_jam53_last_beacon_ms = now;
    du_jam53_last_frame = frame_id;
    du_jam53_last_mask = lease_mask;
    du_jam53_last_mode = mode;

    if (sid != 0 && old_state != DU_JAM53_IDLE)
    {
        if (timing_ok)
        {
            // Moi beacon lap ke hoach cho dung MOT UL window cua frame nay.
            du_jam53_state = DU_JAM53_ARMED;
            du_jam53_window_start_ms = plan_start;
            du_jam53_window_stop_ms = plan_stop;
            du_jam53_window_valid = true;
        }
        else
        {
            abort_active = old_state == DU_JAM53_WINDOW_ACTIVE;
            du_jam53_state = DU_JAM53_PREPARED;
            du_jam53_window_valid = false;
            revoke_reason = target != 0U
                ? "PREPARE_TRANSITION"
                : (lease_allowed ? "BAD_MODE_TIMING" : "LEASE_REVOKED");
        }
    }

    new_state = du_jam53_state;
    portEXIT_CRITICAL(&DU_Jam53_Mux);

    if (sid == 0 || old_state == DU_JAM53_IDLE)
        return;

    if (timing_ok)
    {
        if (old_state != DU_JAM53_ARMED && old_state != DU_JAM53_WINDOW_ACTIVE)
        {
            Serial.printf(
                "[DU JAM V2C5.3A ARMED] PAIR=%u | FRAME=%u | MODE=%u | LEASE_MASK=0x%X | SESSION=%016llX | RF_JAM=OFF\n",
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned int)frame_id,
                (unsigned int)mode,
                (unsigned int)lease_mask,
                (unsigned long long)sid
            );
        }
        return;
    }

    if (abort_active)
    {
        DU_RFShadow_Set(false, revoke_reason != nullptr ? revoke_reason : "ABORT_SIM");
        Serial.printf(
            "[DU JAM V2C5.3A ABORT_SIM] PAIR=%u | FRAME=%u | REASON=%s | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)frame_id,
            revoke_reason != nullptr ? revoke_reason : "UNKNOWN"
        );
    }

    if (old_state == DU_JAM53_ARMED || old_state == DU_JAM53_WINDOW_ACTIVE)
    {
        Serial.printf(
            "[DU JAM V2C5.3A REVOKE] PAIR=%u | FRAME=%u | MODE=%u | LEASE_MASK=0x%X | REASON=%s | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)frame_id,
            (unsigned int)mode,
            (unsigned int)lease_mask,
            revoke_reason != nullptr ? revoke_reason : "UNKNOWN"
        );
    }
}

static void DU_Jam53_Service()
{
    if (DU_JAM53_RF_ENABLE)
    {
        DU_Jam53_Stop("RF_ENABLE_FORBIDDEN");
        return;
    }

    const uint32_t now = millis();

    enum Action : uint8_t
    {
        ACTION_NONE = 0,
        ACTION_START,
        ACTION_STOP,
        ACTION_SKIP,
        ACTION_TIMEOUT_REVOKE,
        ACTION_TIMEOUT_ABORT
    };

    Action action = ACTION_NONE;
    uint64_t sid = 0;
    uint32_t frame_id = 0;
    uint8_t mode = 0;
    uint32_t start_ms = 0;
    uint32_t stop_ms = 0;
    uint32_t last_beacon_ms = 0;

    portENTER_CRITICAL(&DU_Jam53_Mux);
    sid = du_jam53_session_id;
    frame_id = du_jam53_last_frame;
    mode = du_jam53_last_mode;
    start_ms = du_jam53_window_start_ms;
    stop_ms = du_jam53_window_stop_ms;
    last_beacon_ms = du_jam53_last_beacon_ms;

    if (
        sid != 0
        && (du_jam53_state == DU_JAM53_ARMED || du_jam53_state == DU_JAM53_WINDOW_ACTIVE)
        && last_beacon_ms != 0U
        && (uint32_t)(now - last_beacon_ms) > DU_JAM53_BEACON_FAILSAFE_MS
    )
    {
        action = (du_jam53_state == DU_JAM53_WINDOW_ACTIVE)
            ? ACTION_TIMEOUT_ABORT
            : ACTION_TIMEOUT_REVOKE;
        du_jam53_state = DU_JAM53_PREPARED;
        du_jam53_window_valid = false;
    }
    else if (sid != 0 && du_jam53_state == DU_JAM53_ARMED && du_jam53_window_valid)
    {
        if (DU_Jam53_TimeReached(now, start_ms))
        {
            if (!DU_Jam53_TimeReached(now, stop_ms))
            {
                du_jam53_state = DU_JAM53_WINDOW_ACTIVE;
                action = ACTION_START;
            }
            else
            {
                du_jam53_window_valid = false;
                action = ACTION_SKIP;
            }
        }
    }
    else if (sid != 0 && du_jam53_state == DU_JAM53_WINDOW_ACTIVE)
    {
        if (DU_Jam53_TimeReached(now, stop_ms))
        {
            du_jam53_state = DU_JAM53_ARMED;
            du_jam53_window_valid = false;
            action = ACTION_STOP;
        }
    }

    portEXIT_CRITICAL(&DU_Jam53_Mux);

    if (action == ACTION_START)
    {
        DU_RFShadow_Set(true, "START_SIM");
        Serial.printf(
            "[DU JAM V2C5.3A START_SIM] PAIR=%u | FRAME=%u | MODE=%u | AGE=%ums | WINDOW=%ums | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)frame_id,
            (unsigned int)mode,
            (unsigned int)(now - last_beacon_ms),
            (unsigned int)(stop_ms - start_ms)
        );
    }
    else if (action == ACTION_STOP)
    {
        DU_RFShadow_Set(false, "WINDOW_END");
        Serial.printf(
            "[DU JAM V2C5.3A STOP_SIM] PAIR=%u | FRAME=%u | MODE=%u | AGE=%ums | REASON=WINDOW_END | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)frame_id,
            (unsigned int)mode,
            (unsigned int)(now - last_beacon_ms)
        );
    }
    else if (action == ACTION_SKIP)
    {
        DU_RFShadow_Set(false, "WINDOW_MISSED");
        Serial.printf(
            "[DU JAM V2C5.3A SKIP_SIM] PAIR=%u | FRAME=%u | MODE=%u | REASON=WINDOW_MISSED | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)frame_id,
            (unsigned int)mode
        );
    }
    else if (action == ACTION_TIMEOUT_ABORT)
    {
        DU_RFShadow_Set(false, "BEACON_TIMEOUT_ABORT");
        Serial.printf(
            "[DU JAM V2C5.3A ABORT_SIM] PAIR=%u | FRAME=%u | REASON=BEACON_TIMEOUT_%ums | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)frame_id,
            (unsigned int)DU_JAM53_BEACON_FAILSAFE_MS
        );
    }
    else if (action == ACTION_TIMEOUT_REVOKE)
    {
        DU_RFShadow_Set(false, "BEACON_TIMEOUT_REVOKE");
        Serial.printf(
            "[DU JAM V2C5.3A REVOKE] PAIR=%u | FRAME=%u | REASON=BEACON_TIMEOUT_%ums | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)frame_id,
            (unsigned int)DU_JAM53_BEACON_FAILSAFE_MS
        );
    }
}

// =====================================================
// V2C4.7A - TX BOUNDED + RADIO SELF-RECOVERY
//
// LoRa.endPacket() cua thu vien Sandeep Mistry la BLOCKING khong timeout.
// Neu SX1278 mat TX_DONE, task dang gui se giu LoRa_Mutex vo han:
//   - RX task van song nhung khong lay duoc mutex,
//   - DU khong nhan SESSION_START moi,
//   - HMI task co the dung ngay luc LED dang HIGH -> nhin nhu treo mach.
//
// Doi sang async TX + poll co timeout. Neu qua timeout, reset/re-config SX1278
// ngay khi van dang giu mutex, sau do quay ve RX continuous.
// =====================================================

static constexpr uint32_t DU_LORA_TX_TIMEOUT_MS = 180UL;
static volatile uint32_t du_lora_tx_timeout_count = 0;
static volatile bool du_lora_tx_done_flag = false;

// Sandeep Mistry LoRa 0.8.0 de LoRaClass::isTransmitting() o private.
// Dung public API onTxDone() + endPacket(true) de co TX timeout ma khong
// sua thu vien LoRa. Callback chi duoc gan trong luc TX; truoc khi quay
// lai RX se detach de khong lam mat RX_DONE cua parsePacket().
static void DU_LoRa_OnTxDone()
{
    du_lora_tx_done_flag = true;
}

static bool DU_LoRa_Recover_RX_Unsafe(const char *reason)
{
    Serial.printf(
        "[DU RADIO RECOVER] REASON=%s | HARD_RESET_SX1278\n",
        reason != nullptr ? reason : "UNKNOWN"
    );

    // Khoi dong lai SPI theo DUNG pin custom cua board truoc khi LoRa.begin().
    SPI.end();
    delay(2);
    SPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_CS
    );

    LoRa.setSPI(SPI);
    LoRa.setPins(
        LORA_CS,
        LORA_RST,
        LORA_DIO0
    );

    if (!LoRa.begin(433E6))
    {
        Serial.println(
            "[DU RADIO RECOVER] FAIL: Khong khoi tao lai duoc SX1278"
        );
        return false;
    }

    LoRa.setSpreadingFactor(HE_SO_TRAI_PHO_DU);
    LoRa.setSignalBandwidth(500E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
    LoRa.setTxPower(CONG_SUAT_PHAT_DU_DBM);
    pinMode(LORA_DIO0, INPUT);
    LoRa.receive();

    Serial.println(
        "[DU RADIO RECOVER] OK -> RX_CONTINUOUS"
    );

    return true;
}


static int DU_LoRa_EndPacket_CoTimeout(const char *tag)
{
    du_lora_tx_done_flag = false;

    // Chi bat IRQ DIO0/TX_DONE trong luc TX. Neu de callback ton tai khi RX,
    // ISR cua thu vien se clear RX_DONE truoc parsePacket() va lam mat goi.
    LoRa.onTxDone(DU_LoRa_OnTxDone);

    const int start_ok = LoRa.endPacket(true);

    if (start_ok != 1)
    {
        LoRa.onTxDone(nullptr);
        Serial.printf(
            "[DU RADIO TX] START FAIL | TAG=%s\n",
            tag != nullptr ? tag : "UNKNOWN"
        );
        LoRa.receive();
        return 0;
    }

    const uint32_t t0 = millis();

    while (!du_lora_tx_done_flag)
    {
        if ((uint32_t)(millis() - t0) > DU_LORA_TX_TIMEOUT_MS)
        {
            du_lora_tx_timeout_count++;

            Serial.printf(
                "[DU RADIO TX TIMEOUT] TAG=%s | AGE=%u ms | COUNT=%u -> RECOVER\n",
                tag != nullptr ? tag : "UNKNOWN",
                (unsigned int)(millis() - t0),
                (unsigned int)du_lora_tx_timeout_count
            );

            // Tat callback truoc khi re-init de DIO0 khong con ISR cu.
            LoRa.onTxDone(nullptr);
            LoRa.idle();
            (void)DU_LoRa_Recover_RX_Unsafe(tag);
            return 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // TX_DONE da duoc ISR cua thu vien clear. Detach callback truoc RX de
    // parsePacket() tu quan ly RX_DONE binh thuong.
    LoRa.onTxDone(nullptr);
    LoRa.receive();
    return 1;
}


// =====================================================
// SNAPSHOT KENH THAT SU -> DU
// DU chi "nghe ke" beacon 0x18 SU->rBS. Khong doi kien truc voice relay.
// =====================================================
struct MauKenhSUDU
{
    bool pending;
    uint64_t stt_su;
    int16_t rssi_x10;
    int16_t snr_x10;
    int8_t pt_su_dbm;
    uint8_t sf_su;
};

static MauKenhSUDU mau_kenh_su_du = {};
static portMUX_TYPE KenhSUDU_Mux = portMUX_INITIALIZER_UNLOCKED;

static uint64_t DU_Doc_U64_BE(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
        v = (v << 8) | (uint64_t)p[i];
    return v;
}

static void DU_Ghi_I16_BE(uint8_t *p, int16_t v)
{
    uint16_t u = (uint16_t)v;
    p[0] = (uint8_t)((u >> 8) & 0xFF);
    p[1] = (uint8_t)(u & 0xFF);
}

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

    LoRa.setSpreadingFactor(HE_SO_TRAI_PHO_DU);
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

    Serial.printf(
        "Khoi tao LoRa RX THANH CONG! | SF=%u | P_TX=%d dBm\n",
        HE_SO_TRAI_PHO_DU,
        CONG_SUAT_PHAT_DU_DBM
    );
}


// =====================================================
// RF CỐ ĐỊNH - getter chỉ phục vụ telemetry/GPS.
// =====================================================
int8_t Lay_CongSuat_Phat_DU_dBm()
{
    return CONG_SUAT_PHAT_DU_DBM;
}

uint8_t Lay_HeSo_TraiPho_DU()
{
    return HE_SO_TRAI_PHO_DU;
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
    DU_Jam53_Service();

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

    // =====================================================
    // DO KENH THAT SU -> DU BANG BEACON DINH KY CUA SU
    // Packet 44B nay van dich rBS, DU chi nghe ke o PHY de lay RSSI/SNR.
    // Sau khi ghi snapshot, packet bi bo tai DU nhu cu; voice/session khong doi.
    // =====================================================
    if (
        packetSize == SIZE_GPS_REPORT_SU
        && raw_packet[0] == ID_TRAM_RBS
        && raw_packet[1] == ID_TRAM_SU
        && raw_packet[2] == TYPE_VI_TRI_DINH_KY_SU
    )
    {
        uint64_t stt_su = DU_Doc_U64_BE(&raw_packet[4]);
        int8_t pt_su_dbm = (int8_t)raw_packet[29];
        uint8_t sf_su = (uint8_t)(((raw_packet[3] >> 4) & 0x0F) + 6);

        portENTER_CRITICAL(&KenhSUDU_Mux);
        mau_kenh_su_du.pending = true;
        mau_kenh_su_du.stt_su = stt_su;
        mau_kenh_su_du.rssi_x10 = (int16_t)(rssi_packet_dbm * 10);
        mau_kenh_su_du.snr_x10 = (int16_t)lroundf(snr_packet_db * 10.0f);
        mau_kenh_su_du.pt_su_dbm = pt_su_dbm;
        mau_kenh_su_du.sf_su = sf_su;
        portEXIT_CRITICAL(&KenhSUDU_Mux);

        Serial.printf(
            "[DU KENH THAT] NGHE SU 0x18 | STT=%llu | RSSI=%d dBm | SNR=%.1f dB | P_TX_SU=%d dBm | SF=%u\n",
            (unsigned long long)stt_su,
            rssi_packet_dbm,
            snr_packet_db,
            pt_su_dbm,
            sf_su
        );

        xSemaphoreGive(LoRa_Mutex);
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

    // V2C5.3A broadcast beacon: DU doc JAM LEASE MASK va lap UL WINDOW SIM.
    // Chi log START_SIM/STOP_SIM; KHONG phat RF.
    if (
        packetSize == 19
        && raw_packet[0] == V2C_BROADCAST_ID
        && raw_packet[1] == ID_TRAM_RBS
        && raw_packet[2] == 0x1C
        && raw_packet[3] >= 4
    )
    {
        v2c1_network_busy_until_ms = millis() + 900U;
        const uint32_t frame_id =
            ((uint32_t)raw_packet[4] << 24) | ((uint32_t)raw_packet[5] << 16)
            | ((uint32_t)raw_packet[6] << 8) | (uint32_t)raw_packet[7];
        const uint8_t schedule_ctl = raw_packet[18];
        const uint8_t mode = (schedule_ctl >> 6) & 0x03U;
        const uint8_t target = (schedule_ctl >> 4) & 0x03U;
        const uint8_t lease_mask = (schedule_ctl >> 2) & 0x03U;
        DU_Jam53_OnBeacon(frame_id, mode, target, lease_mask);
        xSemaphoreGive(LoRa_Mutex);
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
        || (
            outer_type != TYPE_RELAY
            && outer_type != TYPE_RELAY_NEXT_P1
            && outer_type != TYPE_RELAY_NEXT_P2
        )
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
    const int next_pair_marker =
        (outer_type == TYPE_RELAY_NEXT_P1) ? 1 :
        ((outer_type == TYPE_RELAY_NEXT_P2) ? 2 : 0);
    Serial.printf(
        "[DU PHY RX] RELAY rBS->DU | INNER_TYPE=0x%02X | NEXT_PAIR_MARK=%d | LEN=%d | RSSI=%d dBm | SNR=%.1f dB\n",
        loai_inner,
        next_pair_marker,
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
    int ok = DU_LoRa_EndPacket_CoTimeout("SESSION_READY");

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
        DU_LoRa_EndPacket_CoTimeout("PLAY_STARTED");

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
    int ok = DU_LoRa_EndPacket_CoTimeout("USER_RESPONSE");
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
// TELEMETRY PRE/POST V1
// XOA MAU KENH PRE CON PENDING KHI SESSION BAT DAU.
//
// Khong TX gi ca. Chi xoa snapshot RAM cu de sau session
// DU cho beacon SU moi va bao cao dung mau POST.
// =====================================================
void Xoa_BaoCao_Kenh_SU_DU_DangCho()
{
    portENTER_CRITICAL(&KenhSUDU_Mux);
    mau_kenh_su_du.pending = false;
    portEXIT_CRITICAL(&KenhSUDU_Mux);
}


// =====================================================
// GUI SNAPSHOT KENH SU -> DU VE rBS
// Physical 20B:
// [0] DST=rBS, [1] SRC=DU, [2] TYPE=0x19, [3] VER=1
// [4..11] STT beacon SU64
// [12..13] RSSI_x10 int16
// [14..15] SNR_x10 int16
// [16] P_TX_SU_dBm int8
// [17] SF_SU uint8
// [18..19] reserved
// =====================================================
bool Gui_BaoCao_Kenh_SU_DU_DangCho()
{
    if (DU_V2C1_NetworkBusy())
    {
        return false;
    }

    if (LoRa_Mutex == nullptr)
        return false;

    MauKenhSUDU mau = {};
    portENTER_CRITICAL(&KenhSUDU_Mux);
    if (!mau_kenh_su_du.pending)
    {
        portEXIT_CRITICAL(&KenhSUDU_Mux);
        return false;
    }
    mau = mau_kenh_su_du;
    portEXIT_CRITICAL(&KenhSUDU_Mux);

    // Measurement la best-effort, khong duoc cuop radio neu dang co packet den.
    if (digitalRead(LORA_DIO0) == HIGH)
        return false;
    if (xSemaphoreTake(LoRa_Mutex, 0) != pdTRUE)
        return false;
    if (digitalRead(LORA_DIO0) == HIGH)
    {
        xSemaphoreGive(LoRa_Mutex);
        return false;
    }

    uint8_t p[SIZE_BAO_CAO_KENH_SU_DU] = {};
    p[0] = ID_TRAM_RBS;
    p[1] = ID_TRAM_DU;
    p[2] = TYPE_BAO_CAO_KENH_SU_DU;
    p[3] = 1;
    for (int i = 0; i < 8; ++i)
        p[4 + i] = (uint8_t)((mau.stt_su >> (56 - 8 * i)) & 0xFF);
    DU_Ghi_I16_BE(&p[12], mau.rssi_x10);
    DU_Ghi_I16_BE(&p[14], mau.snr_x10);
    p[16] = (uint8_t)mau.pt_su_dbm;
    p[17] = mau.sf_su;

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write(p, sizeof(p));
    int ok = DU_LoRa_EndPacket_CoTimeout("CHANNEL_REPORT");
    xSemaphoreGive(LoRa_Mutex);

    if (ok == 1)
    {
        portENTER_CRITICAL(&KenhSUDU_Mux);
        // Chi xoa neu pending van la cung snapshot; neu RX da cap nhat mau moi thi giu lai.
        if (mau_kenh_su_du.pending && mau_kenh_su_du.stt_su == mau.stt_su)
            mau_kenh_su_du.pending = false;
        portEXIT_CRITICAL(&KenhSUDU_Mux);

        Serial.printf(
            "[DU KENH THAT] REPORT -> rBS | STT_SU=%llu | RSSI_SU_DU=%.1f dBm | P_TX_SU=%d dBm\n",
            (unsigned long long)mau.stt_su,
            ((float)mau.rssi_x10) / 10.0f,
            mau.pt_su_dbm
        );
        return true;
    }
    return false;
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

    // 4 bit cao mang SF hien tai: SF7->1, SF8->2, SF9->3.
    co_hieu |= (uint8_t)(((Lay_HeSo_TraiPho_DU() - 6) & 0x0F) << 4);
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
    goi_tin[29] = (uint8_t)Lay_CongSuat_Phat_DU_dBm();
    goi_tin[30] = (hdop_x100 >> 8) & 0xFF;
    goi_tin[31] = hdop_x100 & 0xFF;
    DU_Ghi_U32_BE(&goi_tin[32], du_lieu_gps.tuoi_fix_ms);
    DU_Ghi_U32_BE(&goi_tin[36], du_lieu_gps.ngay_utc_yyyymmdd);
    DU_Ghi_U32_BE(&goi_tin[40], du_lieu_gps.gio_utc_ms_trong_ngay);

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write(goi_tin, sizeof(goi_tin));
    int ket_qua_tx = DU_LoRa_EndPacket_CoTimeout(
        loai_bao_cao == TYPE_VI_TRI_DINH_KY ? "GPS_PERIODIC" : "GPS_SESSION"
    );

    xSemaphoreGive(LoRa_Mutex);

    const char *ten_loai = loai_bao_cao == TYPE_VI_TRI_DINH_KY
        ? "VI_TRI_DINH_KY" : "GPS_PHIEN";

    Serial.printf(
        "[DU GPS TX] %s -> rBS | MA=%016llX | HOP_LE=%u | VI_DO=%.7f | KINH_DO=%.7f | DO_CAO=%.1fm | TOC_DO=%.2fm/s | VE_TINH=%u | HDOP=%.2f | P_TX=%d dBm | SF=%u | TX=%s\n",
        ten_loai,
        (unsigned long long)ma_tham_chieu,
        du_lieu_gps.gps_hop_le ? 1 : 0,
        du_lieu_gps.vi_do,
        du_lieu_gps.kinh_do,
        du_lieu_gps.do_cao_m,
        du_lieu_gps.toc_do_m_s,
        du_lieu_gps.so_ve_tinh,
        du_lieu_gps.hdop,
        Lay_CongSuat_Phat_DU_dBm(),
        Lay_HeSo_TraiPho_DU(),
        ket_qua_tx == 1 ? "OK" : "FAIL"
    );

    return ket_qua_tx == 1;
}

bool Gui_GPS_REPORT_DU(
    uint64_t ma_phien,
    const DuLieuGPS_DU &du_lieu_gps)
{
    if (DU_V2C1_NetworkBusy())
    {
        return false;
    }

    return Gui_Goi_ViTri_DU(TYPE_GPS_REPORT, ma_phien, du_lieu_gps, false);
}

bool Gui_VI_TRI_DINH_KY_DU(
    uint64_t so_thu_tu_bao_cao,
    const DuLieuGPS_DU &du_lieu_gps)
{
    if (DU_V2C1_NetworkBusy())
    {
        return false;
    }

    return Gui_Goi_ViTri_DU(TYPE_VI_TRI_DINH_KY, so_thu_tu_bao_cao, du_lieu_gps, true);
}
