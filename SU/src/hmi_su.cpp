#include "hmi_su.h"
#include "phat_lora.h"

// ============================================================
// PIN HMI SU
// ============================================================
static constexpr uint8_t CHAN_NUT_PTT_SU = 7;
static constexpr uint8_t CHAN_LED_SU = 16;

// Session gan nhat va feedback cua DU.
static uint64_t ma_phien_su_gan_nhat = 0;
static uint8_t phan_hoi_nguoi_dung_su = 0;
static uint32_t moc_tat_led_ack_ms = 0;


void KhoiTao_HMI_SU()
{
    pinMode(CHAN_NUT_PTT_SU, INPUT_PULLUP);
    pinMode(CHAN_LED_SU, OUTPUT);
    digitalWrite(CHAN_LED_SU, LOW);

    ma_phien_su_gan_nhat = 0;
    phan_hoi_nguoi_dung_su = 0;
    moc_tat_led_ack_ms = 0;

    Serial.println("[SU HMI] Khoi tao GPIO6=PTT, GPIO16=LED");
}


bool Nut_PTT_Dang_Bam_SU()
{
    // Nut active LOW vi dung INPUT_PULLUP.
    return digitalRead(CHAN_NUT_PTT_SU) == LOW;
}


void Dat_LED_SU(bool sang)
{
    digitalWrite(CHAN_LED_SU, sang ? HIGH : LOW);
}


void HMI_SU_BatDau_CauMoi()
{
    // Cau moi khong duoc ke thua ACK/NACK cua cau truoc.
    ma_phien_su_gan_nhat = 0;
    phan_hoi_nguoi_dung_su = 0;
    moc_tat_led_ack_ms = 0;
    Dat_LED_SU(false);
}


void HMI_SU_Dat_Session(uint64_t session_id)
{
    ma_phien_su_gan_nhat = session_id;
}


uint64_t HMI_SU_Lay_Session()
{
    return ma_phien_su_gan_nhat;
}


bool HMI_SU_Dang_Cho_PhanHoi()
{
    return ma_phien_su_gan_nhat != 0 && phan_hoi_nguoi_dung_su == 0;
}


void Dat_PhanHoi_SU(uint8_t code)
{
    phan_hoi_nguoi_dung_su = code;

    if (code == USER_RESPONSE_ACK)
    {
        // DU da phat xong va ACK -> LED sang 2 giay.
        moc_tat_led_ack_ms = millis() + 2000UL;
        Dat_LED_SU(true);
    }
    else if (code == USER_RESPONSE_NACK)
    {
        // NACK se dung mau chop nhanh/cham cho toi cau PTT moi.
        moc_tat_led_ack_ms = 0;
    }
    else
    {
        moc_tat_led_ack_ms = 0;
        Dat_LED_SU(false);
    }
}


void Bao_Loi_Session_SU()
{
    // 3 chop cham, khac ro chop packet khi dang TX.
    for (uint8_t i = 0; i < 3; i++)
    {
        Dat_LED_SU(true);
        delay(320);
        Dat_LED_SU(false);
        delay(320);
    }
}


void CapNhat_LED_PhanHoi_SU()
{
    if (phan_hoi_nguoi_dung_su == USER_RESPONSE_ACK)
    {
        if ((int32_t)(moc_tat_led_ack_ms - millis()) > 0)
        {
            Dat_LED_SU(true);
        }
        else
        {
            phan_hoi_nguoi_dung_su = 0;
            moc_tat_led_ack_ms = 0;
            Dat_LED_SU(false);
        }
        return;
    }

    if (phan_hoi_nguoi_dung_su == USER_RESPONSE_NACK)
    {
        // Chu ky 2.4 s:
        //   0..0.8 s  : chop nhanh, doi trang thai moi 100 ms.
        //   0.8..2.4 s: chop cham, doi trang thai moi 400 ms.
        uint32_t pha = millis() % 2400UL;
        bool sang = false;

        if (pha < 800UL)
            sang = ((pha / 100UL) % 2UL) == 0;
        else
            sang = (((pha - 800UL) / 400UL) % 2UL) == 0;

        Dat_LED_SU(sang);
        return;
    }

    Dat_LED_SU(false);
}
