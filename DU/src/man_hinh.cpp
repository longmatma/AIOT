#include "man_hinh.h"
#include <U8g2lib.h>
#include <Wire.h>

#define OLED_SDA 4
#define OLED_SCL 5

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    /* reset=*/ U8X8_PIN_NONE
);


void KhoiTao_OLED()
{
    Wire.begin(
        OLED_SDA,
        OLED_SCL
    );

    u8g2.begin();
    u8g2.enableUTF8Print();

    HienThi_DU_KhoiDong(
        "Dang khoi dong..."
    );

    Serial.println(
        "Khoi tao OLED THANH CONG!"
    );
}


void HienThi_DU_KhoiDong(
    const char *trang_thai)
{
    u8g2.clearBuffer();

    u8g2.setFont(
        u8g2_font_ncenB08_tr
    );

    u8g2.drawStr(
        15,
        26,
        "TRAM NHAN (DU)"
    );

    u8g2.setFont(
        u8g2_font_6x10_tr
    );

    u8g2.setCursor(
        10,
        47
    );

    u8g2.print(
        trang_thai
    );

    u8g2.sendBuffer();
}


void HienThi_DU_ChoNhan()
{
    u8g2.clearBuffer();
    u8g2.drawFrame(
        0,
        0,
        128,
        64
    );

    u8g2.setFont(
        u8g2_font_6x10_tr
    );

    u8g2.setCursor(
        5,
        15
    );

    u8g2.print(
        "DU - SAN SANG"
    );

    u8g2.setCursor(
        5,
        34
    );

    u8g2.print(
        "CHO NHAN AUDIO..."
    );

    u8g2.setCursor(
        5,
        54
    );

    u8g2.print(
        "HOP2: READY"
    );

    u8g2.sendBuffer();
}


void HienThi_DU_DangNhan(
    uint32_t so_voice_expected,
    uint32_t so_goi_mat_raw,
    uint32_t so_goi_fec_cuu)
{
    u8g2.clearBuffer();
    u8g2.drawFrame(
        0,
        0,
        128,
        64
    );

    u8g2.setFont(
        u8g2_font_6x10_tr
    );

    u8g2.setCursor(
        5,
        11
    );

    u8g2.print(
        "DU - DANG NHAN"
    );

    u8g2.setCursor(
        5,
        27
    );

    u8g2.print(
        "DATA: "
    );

    u8g2.print(
        so_voice_expected
    );

    u8g2.setCursor(
        5,
        42
    );

    u8g2.print(
        "LOST: "
    );

    u8g2.print(
        so_goi_mat_raw
    );

    u8g2.setCursor(
        5,
        57
    );

    u8g2.print(
        "FEC : "
    );

    u8g2.print(
        so_goi_fec_cuu
    );

    u8g2.sendBuffer();
}


void HienThi_DU_KetQua(
    uint32_t so_voice_expected,
    uint32_t so_goi_mat_raw,
    uint32_t so_goi_fec_cuu,
    uint32_t so_goi_con_mat)
{
    const char *chat_luong;

    if (so_goi_con_mat > 0)
    {
        chat_luong =
            "BAD";
    }
    else if (so_goi_mat_raw > 0)
    {
        chat_luong =
            "WARN";
    }
    else
    {
        chat_luong =
            "GOOD";
    }

    u8g2.clearBuffer();
    u8g2.drawFrame(
        0,
        0,
        128,
        64
    );

    u8g2.setFont(
        u8g2_font_5x8_tr
    );

    u8g2.setCursor(
        4,
        9
    );

    u8g2.print(
        "DU - KET QUA"
    );

    u8g2.setCursor(
        4,
        20
    );

    u8g2.print(
        "DATA : "
    );

    u8g2.print(
        so_voice_expected
    );

    u8g2.setCursor(
        4,
        31
    );

    u8g2.print(
        "LOST : "
    );

    u8g2.print(
        so_goi_mat_raw
    );

    u8g2.setCursor(
        4,
        42
    );

    u8g2.print(
        "FEC  : "
    );

    u8g2.print(
        so_goi_fec_cuu
    );

    u8g2.setCursor(
        4,
        53
    );

    u8g2.print(
        "FINAL: "
    );

    u8g2.print(
        so_goi_con_mat
    );

    u8g2.setCursor(
        67,
        53
    );

    u8g2.print(
        "H2:"
    );

    u8g2.print(
        chat_luong
    );

    u8g2.sendBuffer();
}
