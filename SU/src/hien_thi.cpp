#include "hien_thi.h"

// OLED 1.3 inch SH1106, 128x64.
// Wire.begin(4, 5) duoc goi tu SU/main.cpp.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    /* reset=*/ U8X8_PIN_NONE
);

void KhoiTao_OLED()
{
    u8g2.begin();
}


int Tinh_BienDo_Mic(
    uint8_t *buffer_am_thanh_8bit,
    int so_byte)
{
    int max_val = -32768;
    int min_val = 32767;

    int16_t *buffer_16bit =
        (int16_t *)buffer_am_thanh_8bit;

    int so_mau =
        so_byte / 2;

    for (int i = 0; i < so_mau; i++)
    {
        if (buffer_16bit[i] > max_val)
        {
            max_val = buffer_16bit[i];
        }

        if (buffer_16bit[i] < min_val)
        {
            min_val = buffer_16bit[i];
        }
    }

    int bien_do =
        max_val - min_val;

    if (bien_do < 50)
    {
        bien_do = 0;
    }

    return bien_do;
}


void Ve_GiaoDien_OLED(
    int bien_do_mic,
    bool dang_phat)
{
    u8g2.clearBuffer();

    u8g2.setFont(
        u8g2_font_ncenB08_tr
    );

    u8g2.drawStr(
        0,
        10,
        "LoRa:"
    );

    u8g2.setDrawColor(1);
    u8g2.drawBox(
        35,
        1,
        24,
        11
    );

    u8g2.setDrawColor(0);
    u8g2.drawStr(
        37,
        10,
        " OK "
    );

    u8g2.setDrawColor(1);

    u8g2.setFont(
        u8g2_font_ncenB14_tr
    );

    if (dang_phat)
    {
        u8g2.drawStr(
            12,
            35,
            "PHAT AM"
        );
    }
    else
    {
        u8g2.drawStr(
            10,
            35,
            "SAN SANG"
        );
    }

    u8g2.setFont(
        u8g2_font_ncenB08_tr
    );

    u8g2.setCursor(
        0,
        52
    );

    if (dang_phat)
    {
        u8g2.print(
            "ADC: "
        );

        u8g2.print(
            bien_do_mic
        );

        int chieu_dai_thanh =
            map(
                bien_do_mic,
                0,
                4096,
                0,
                124
            );

        if (chieu_dai_thanh > 124)
        {
            chieu_dai_thanh = 124;
        }

        if (chieu_dai_thanh < 0)
        {
            chieu_dai_thanh = 0;
        }

        u8g2.drawFrame(
            0,
            55,
            128,
            9
        );

        if (chieu_dai_thanh > 0)
        {
            u8g2.drawBox(
                2,
                57,
                chieu_dai_thanh,
                5
            );
        }
    }
    else
    {
        u8g2.print(
            "Mic: Tam dung"
        );

        u8g2.drawFrame(
            0,
            55,
            128,
            9
        );
    }

    u8g2.sendBuffer();
}


static void Ve_ThongKe_SU(
    const char *tieu_de,
    uint32_t so_voice_packet,
    uint32_t so_lan_retry,
    uint32_t so_packet_fail,
    bool hien_hop)
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
        tieu_de
    );

    u8g2.setCursor(
        5,
        24
    );

    u8g2.print(
        "DATA : "
    );

    u8g2.print(
        so_voice_packet
    );

    u8g2.setCursor(
        5,
        36
    );

    u8g2.print(
        "RETRY: "
    );

    u8g2.print(
        so_lan_retry
    );

    u8g2.setCursor(
        5,
        48
    );

    u8g2.print(
        "FAIL : "
    );

    u8g2.print(
        so_packet_fail
    );

    if (hien_hop)
    {
        const char *chat_luong;

        if (so_packet_fail > 0)
        {
            chat_luong =
                "BAD";
        }
        else if (so_lan_retry > 0)
        {
            chat_luong =
                "WARN";
        }
        else
        {
            chat_luong =
                "GOOD";
        }

        u8g2.setCursor(
            5,
            60
        );

        u8g2.print(
            "HOP1 : "
        );

        u8g2.print(
            chat_luong
        );
    }

    u8g2.sendBuffer();
}


void HienThi_SU_DangGui(
    uint32_t so_voice_packet,
    uint32_t so_lan_retry,
    uint32_t so_packet_fail)
{
    Ve_ThongKe_SU(
        "SU - DANG GUI",
        so_voice_packet,
        so_lan_retry,
        so_packet_fail,
        false
    );
}


void HienThi_SU_KetQua(
    uint32_t so_voice_packet,
    uint32_t so_lan_retry,
    uint32_t so_packet_fail)
{
    Ve_ThongKe_SU(
        "SU - KET QUA",
        so_voice_packet,
        so_lan_retry,
        so_packet_fail,
        true
    );
}
