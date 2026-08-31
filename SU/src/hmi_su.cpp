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

// Bao ve khoi truong hop mat ACK/NACK lam SU cho phan hoi mai mai.
// MAX_KHUNG_THOAI hien tai cho phep cau noi toi da xap xi 60 giay,
// nen 75 giay chi la watchdog du phong; khong tac dong khi ACK den binh thuong.
static constexpr uint32_t THOI_GIAN_CHO_PHAN_HOI_TOI_DA_MS = 75000UL;
static uint32_t moc_bat_dau_cho_phan_hoi_ms = 0;


void KhoiTao_HMI_SU()
{
    pinMode(CHAN_NUT_PTT_SU, INPUT_PULLUP);
    pinMode(CHAN_LED_SU, OUTPUT);
    digitalWrite(CHAN_LED_SU, LOW);

    ma_phien_su_gan_nhat = 0;
    phan_hoi_nguoi_dung_su = 0;
    moc_tat_led_ack_ms = 0;
    moc_bat_dau_cho_phan_hoi_ms = 0;

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
    moc_bat_dau_cho_phan_hoi_ms = 0;
    Dat_LED_SU(false);
}


void HMI_SU_Dat_Session(uint64_t session_id)
{
    ma_phien_su_gan_nhat = session_id;
    phan_hoi_nguoi_dung_su = 0;
    moc_bat_dau_cho_phan_hoi_ms = 0;
}


uint64_t HMI_SU_Lay_Session()
{
    return ma_phien_su_gan_nhat;
}


bool HMI_SU_Dang_Cho_PhanHoi()
{
    if (ma_phien_su_gan_nhat == 0 || phan_hoi_nguoi_dung_su != 0)
        return false;

    // Moc timeout chi bat dau khi SU da quay ve trang thai nghi va
    // ham nay duoc goi lan dau. Vi vay thoi gian ghi/gui cau noi dai
    // khong bi tinh nham vao watchdog cho phan hoi.
    uint32_t bay_gio_ms = millis();
    if (moc_bat_dau_cho_phan_hoi_ms == 0)
    {
        moc_bat_dau_cho_phan_hoi_ms = bay_gio_ms == 0 ? 1UL : bay_gio_ms;
        return true;
    }

    if ((uint32_t)(bay_gio_ms - moc_bat_dau_cho_phan_hoi_ms) >= THOI_GIAN_CHO_PHAN_HOI_TOI_DA_MS)
    {
        // Mat phan hoi khong duoc phep khoa beacon/RF vo thoi han.
        ma_phien_su_gan_nhat = 0;
        phan_hoi_nguoi_dung_su = 0;
        moc_bat_dau_cho_phan_hoi_ms = 0;
        Dat_LED_SU(false);
        return false;
    }

    return true;
}


void Dat_PhanHoi_SU(uint8_t code)
{
    phan_hoi_nguoi_dung_su = code;
    moc_bat_dau_cho_phan_hoi_ms = 0;

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

    // Session bat tay that bai thi khong co phan hoi DU de cho nua.
    // Giai phong ngay trang thai HMI de beacon va dieu khien RF tiep tuc.
    ma_phien_su_gan_nhat = 0;
    phan_hoi_nguoi_dung_su = 0;
    moc_bat_dau_cho_phan_hoi_ms = 0;
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
            // ACK da duoc hien thi du 2 giay -> ket thuc hoan toan
            // trang thai HMI cua phien cu.
            //
            // Neu chi xoa phan_hoi_nguoi_dung_su ma van giu ma phien,
            // HMI_SU_Dang_Cho_PhanHoi() se lai tra ve true va lam SU
            // ngung beacon dinh ky / ngung nhan dieu khien RF sau khi noi xong.
            phan_hoi_nguoi_dung_su = 0;
            ma_phien_su_gan_nhat = 0;
            moc_bat_dau_cho_phan_hoi_ms = 0;
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
