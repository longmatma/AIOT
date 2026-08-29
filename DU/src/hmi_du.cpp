#include "hmi_du.h"
#include "nhan_lora.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ============================================================
// PIN HMI DU
// ============================================================
static constexpr uint8_t CHAN_NUT_NACK_DU = 6;
static constexpr uint8_t CHAN_LED_DU = 16;

// LED pulse theo packet that.
static constexpr uint32_t DU_PACKET_PULSE_MS = 70UL;
static constexpr uint32_t DU_DATA_GAP_TIMEOUT_MS = 300UL;

// Trang thai HMI dung chung giua cac task/core.
static portMUX_TYPE hmi_du_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool cho_phep_nack = false;
static volatile bool yeu_cau_auto_ack = false;
static volatile uint64_t phien_auto_ack = 0;
static volatile uint64_t phien_vua_phat_xong = 0;

static volatile bool dang_cho_confirm = false;
static volatile bool da_nhan_confirm = false;
static volatile uint8_t ma_phan_hoi_dang_cho = 0;
static volatile uint64_t phien_phan_hoi_dang_cho = 0;
static volatile uint32_t moc_tat_led_confirm_ms = 0;

static volatile bool led_session_active = false;
static volatile bool da_bat_dau_nhan_data = false;
static volatile uint32_t moc_data_gan_nhat_ms = 0;
static volatile uint32_t moc_tat_pulse_packet_ms = 0;

// Khoa beacon ngan han ngay khi RX task vua nhan packet.
// Muc dich: tranh khe thoi gian giua RX task va decoder/HMI.
static volatile uint32_t khoa_beacon_den_ms = 0;


static void CapNhat_LED_DU()
{
    uint32_t now = millis();

    // Snapshot cac bien can doc de thoi gian giu critical section rat ngan.
    bool pending;
    bool confirmed;
    uint32_t confirm_until;
    bool session_active;
    bool data_started;
    uint32_t last_data;
    uint32_t pulse_until;

    portENTER_CRITICAL(&hmi_du_mux);
    pending = dang_cho_confirm;
    confirmed = da_nhan_confirm;
    confirm_until = moc_tat_led_confirm_ms;
    session_active = led_session_active;
    data_started = da_bat_dau_nhan_data;
    last_data = moc_data_gan_nhat_ms;
    pulse_until = moc_tat_pulse_packet_ms;
    portEXIT_CRITICAL(&hmi_du_mux);

    // Uu tien cao nhat: dang gui ACK/NACK va cho SU confirm.
    if (pending && !confirmed)
    {
        bool sang = (((now / 220UL) % 2UL) == 0);
        digitalWrite(CHAN_LED_DU, sang ? HIGH : LOW);
        return;
    }

    // SU da confirm -> LED sang lien tuc 2 giay.
    if (confirmed)
    {
        if ((int32_t)(confirm_until - now) > 0)
        {
            digitalWrite(CHAN_LED_DU, HIGH);
            return;
        }

        portENTER_CRITICAL(&hmi_du_mux);
        da_nhan_confirm = false;
        ma_phan_hoi_dang_cho = 0;
        phien_phan_hoi_dang_cho = 0;
        portEXIT_CRITICAL(&hmi_du_mux);
    }

    if (session_active)
    {
        // Da co SESSION nhung chua co DATA -> LED sang dung.
        if (!data_started)
        {
            digitalWrite(CHAN_LED_DU, HIGH);
            return;
        }

        // DATA tung toi nhung hien tai ngung > timeout -> LED sang dung.
        if (now - last_data >= DU_DATA_GAP_TIMEOUT_MS)
        {
            digitalWrite(CHAN_LED_DU, HIGH);
            return;
        }

        // DATA dang toi -> chop theo packet that.
        bool pulse = ((int32_t)(pulse_until - now) > 0);
        digitalWrite(CHAN_LED_DU, pulse ? HIGH : LOW);
        return;
    }

    digitalWrite(CHAN_LED_DU, LOW);
}


void HMI_DU_TamDung_Beacon(uint32_t thoi_gian_ms)
{
    uint32_t de_xuat = millis() + thoi_gian_ms;

    portENTER_CRITICAL(&hmi_du_mux);
    // So sanh signed de van dung khi millis() wrap.
    if ((int32_t)(de_xuat - khoa_beacon_den_ms) > 0)
        khoa_beacon_den_ms = de_xuat;
    portEXIT_CRITICAL(&hmi_du_mux);
}


void HMI_DU_Reset_Cho_Session_Moi()
{
    uint32_t now = millis();

    portENTER_CRITICAL(&hmi_du_mux);
    cho_phep_nack = false;
    yeu_cau_auto_ack = false;
    phien_auto_ack = 0;
    phien_vua_phat_xong = 0;

    dang_cho_confirm = false;
    da_nhan_confirm = false;
    ma_phan_hoi_dang_cho = 0;
    phien_phan_hoi_dang_cho = 0;
    moc_tat_led_confirm_ms = 0;

    led_session_active = true;
    da_bat_dau_nhan_data = false;
    moc_data_gan_nhat_ms = now;
    moc_tat_pulse_packet_ms = 0;

    khoa_beacon_den_ms = now + 1500UL;
    portEXIT_CRITICAL(&hmi_du_mux);
}


void HMI_DU_Bao_Nhan_Data()
{
    uint32_t now = millis();

    portENTER_CRITICAL(&hmi_du_mux);
    led_session_active = true;
    da_bat_dau_nhan_data = true;
    moc_data_gan_nhat_ms = now;
    moc_tat_pulse_packet_ms = now + DU_PACKET_PULSE_MS;
    khoa_beacon_den_ms = now + 1000UL;
    portEXIT_CRITICAL(&hmi_du_mux);
}


void HMI_DU_Bao_END_Audio()
{
    uint32_t now = millis();

    portENTER_CRITICAL(&hmi_du_mux);
    led_session_active = false;
    da_bat_dau_nhan_data = false;
    moc_tat_pulse_packet_ms = 0;
    khoa_beacon_den_ms = now + 1000UL;
    portEXIT_CRITICAL(&hmi_du_mux);
}


void HMI_DU_Bao_Phat_Xong(uint64_t session_id)
{
    if (session_id == 0)
        return;

    portENTER_CRITICAL(&hmi_du_mux);
    phien_vua_phat_xong = session_id;
    cho_phep_nack = true;
    phien_auto_ack = session_id;
    yeu_cau_auto_ack = true;
    khoa_beacon_den_ms = millis() + 1500UL;
    portEXIT_CRITICAL(&hmi_du_mux);

    Serial.printf(
        "[DU HMI] AUTO_ACK REQUEST | SESSION=%016llX | GPIO6=NACK neu nghe khong ro\n",
        (unsigned long long)session_id
    );
}


bool HMI_DU_XuLy_User_Confirm(uint64_t session_id, uint8_t code)
{
    bool khop = false;

    portENTER_CRITICAL(&hmi_du_mux);
    if (
        dang_cho_confirm
        && session_id == phien_phan_hoi_dang_cho
        && code == ma_phan_hoi_dang_cho
    )
    {
        da_nhan_confirm = true;
        dang_cho_confirm = false;
        moc_tat_led_confirm_ms = millis() + 2000UL;
        cho_phep_nack = true;
        khoa_beacon_den_ms = millis() + 1000UL;
        khop = true;
    }
    portEXIT_CRITICAL(&hmi_du_mux);

    if (khop)
    {
        Serial.printf(
            "[DU HMI] SU CONFIRMED USER_%s | SESSION=%016llX\n",
            code == USER_RESPONSE_ACK ? "ACK" : "NACK",
            (unsigned long long)session_id
        );
    }
    else
    {
        Serial.printf(
            "[DU HMI DROP] USER_CONFIRM khong khop pending | SESSION=%016llX\n",
            (unsigned long long)session_id
        );
    }

    return khop;
}


bool HMI_DU_Radio_Dang_Ban()
{
    uint32_t now = millis();
    bool busy;

    portENTER_CRITICAL(&hmi_du_mux);
    busy =
        led_session_active
        || yeu_cau_auto_ack
        || dang_cho_confirm
        || ((int32_t)(khoa_beacon_den_ms - now) > 0);
    portEXIT_CRITICAL(&hmi_du_mux);

    return busy;
}


static void TacVu_HMI_NguoiDung(void *tham_so)
{
    (void)tham_so;
    bool nack_truoc = HIGH;

    while (true)
    {
        CapNhat_LED_DU();

        uint8_t code_can_gui = 0;
        uint64_t session_can_gui = 0;

        // ----------------------------------------------------
        // AUTO ACK: tao dung 1 transaction sau khi PLAY xong.
        // ----------------------------------------------------
        portENTER_CRITICAL(&hmi_du_mux);
        if (yeu_cau_auto_ack && !dang_cho_confirm)
        {
            code_can_gui = USER_RESPONSE_ACK;
            session_can_gui = phien_auto_ack;
            yeu_cau_auto_ack = false;
        }
        portEXIT_CRITICAL(&hmi_du_mux);

        // ----------------------------------------------------
        // MANUAL NACK: nut GPIO6 active LOW, debounce 25 ms.
        // ----------------------------------------------------
        bool nack_hien_tai = digitalRead(CHAN_NUT_NACK_DU);
        bool co_the_nack;
        bool pending;
        uint64_t session_vua_phat;

        portENTER_CRITICAL(&hmi_du_mux);
        co_the_nack = cho_phep_nack;
        pending = dang_cho_confirm;
        session_vua_phat = phien_vua_phat_xong;
        portEXIT_CRITICAL(&hmi_du_mux);

        if (
            code_can_gui == 0
            && co_the_nack
            && !pending
            && nack_truoc == HIGH
            && nack_hien_tai == LOW
        )
        {
            vTaskDelay(pdMS_TO_TICKS(25));

            if (digitalRead(CHAN_NUT_NACK_DU) == LOW)
            {
                code_can_gui = USER_RESPONSE_NACK;
                session_can_gui = session_vua_phat;
            }
        }

        nack_truoc = nack_hien_tai;

        // ----------------------------------------------------
        // GUI RESPONSE + CHO SU CONFIRM, toi da 3 lan.
        // ----------------------------------------------------
        if (code_can_gui != 0 && session_can_gui != 0)
        {
            portENTER_CRITICAL(&hmi_du_mux);
            dang_cho_confirm = true;
            da_nhan_confirm = false;
            moc_tat_led_confirm_ms = 0;
            ma_phan_hoi_dang_cho = code_can_gui;
            phien_phan_hoi_dang_cho = session_can_gui;
            khoa_beacon_den_ms = millis() + 1500UL;
            portEXIT_CRITICAL(&hmi_du_mux);

            Serial.printf(
                "[DU HMI] %s -> BAT DAU GUI | SESSION=%016llX\n",
                code_can_gui == USER_RESPONSE_ACK ? "AUTO_ACK" : "USER_NACK",
                (unsigned long long)session_can_gui
            );

            bool da_confirm = false;

            for (uint8_t lan = 0; lan < 3 && !da_confirm; lan++)
            {
                Gui_USER_RESPONSE_RBS(session_can_gui, code_can_gui);
                HMI_DU_TamDung_Beacon(500UL);

                uint32_t t0 = millis();
                while (millis() - t0 < 350UL)
                {
                    CapNhat_LED_DU();

                    portENTER_CRITICAL(&hmi_du_mux);
                    da_confirm = da_nhan_confirm;
                    portEXIT_CRITICAL(&hmi_du_mux);

                    if (da_confirm)
                        break;

                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

            if (!da_confirm)
            {
                Serial.printf(
                    "[DU HMI] %s TIMEOUT -> CHUA CO CONFIRM TU SU\n",
                    code_can_gui == USER_RESPONSE_ACK ? "AUTO_ACK" : "USER_NACK"
                );

                portENTER_CRITICAL(&hmi_du_mux);
                dang_cho_confirm = false;
                da_nhan_confirm = false;
                ma_phan_hoi_dang_cho = 0;
                phien_phan_hoi_dang_cho = 0;
                moc_tat_led_confirm_ms = 0;
                portEXIT_CRITICAL(&hmi_du_mux);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void KhoiTao_HMI_DU()
{
    pinMode(CHAN_NUT_NACK_DU, INPUT_PULLUP);
    pinMode(CHAN_LED_DU, OUTPUT);
    digitalWrite(CHAN_LED_DU, LOW);

    xTaskCreatePinnedToCore(
        TacVu_HMI_NguoiDung,
        "HMI_USER",
        4096,
        nullptr,
        2,
        nullptr,
        1
    );

    Serial.println("[DU HMI] Khoi tao GPIO6=NACK, GPIO16=LED");
}
