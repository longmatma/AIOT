#include "nhan_lora.h"

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
// ID
// =====================================================

#define ID_TRAM_SU   0x01
#define ID_TRAM_DU   0x02
#define ID_TRAM_RBS  0x03


// =====================================================
// TYPE WRAPPER rBS -> DU
// =====================================================

#define TYPE_RELAY       0x10
#define TYPE_RELAY_END   0x11

// TYPE nội bộ đưa sang main.cpp.
// Gói này không tồn tại trong ciphertext SU.
#define TYPE_AUDIO_END   0x04


// =====================================================
// KÍCH THƯỚC
// =====================================================

#define SIZE_SESSION_INNER  12
#define SIZE_VOICE_INNER    96

#define SIZE_SESSION_RELAY  16
#define SIZE_VOICE_RELAY   100
#define SIZE_END_RELAY       5


// =====================================================
// KHỞI TẠO LORA RX
// =====================================================

void KhoiTao_LoRa_RX()
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
            "LOI: Khong tim thay module LoRa RX!"
        );


        while (1)
        {
            delay(1000);
        }
    }


    LoRa.setSpreadingFactor(
        7
    );


    LoRa.setSignalBandwidth(
        500E3
    );


    LoRa.setCodingRate4(
        5
    );


    LoRa.enableCrc();


    // DIO0 dùng làm RX_DONE trong chế độ receive().
    pinMode(
        LORA_DIO0,
        INPUT
    );


    // RX continuous ngay sau khi khởi tạo.
    LoRa.receive();


    Serial.println(
        "Khoi tao LoRa RX THANH CONG!"
    );
}


// =====================================================
// NHẬN PACKET
//
// DU TỪ GIỜ CHỈ NHẬN PACKET DO rBS BỌC.
//
// rBS SESSION:
//   4B wrapper + 12B packet gốc = 16B
//
// rBS VOICE:
//   4B wrapper + 96B packet gốc = 100B
//
// rBS END_AUDIO:
//   4B header + 1B payload = 5B
//
// Packet SU trực tiếp 12B / 96B sẽ bị bỏ.
// =====================================================

bool Nhan_GoiTin_LoRa(
    uint8_t* buffer,
    size_t &do_dai_nhan)
{
    // =================================================
    // GIỮ SX1278 Ở RX CONTINUOUS KHI IDLE
    //
    // LoRa.receive() đã map DIO0 = RX_DONE và đưa radio
    // vào MODE_RX_CONTINUOUS.
    //
    // QUAN TRỌNG:
    // Không gọi LoRa.parsePacket() liên tục khi chưa có
    // RX_DONE, vì parsePacket() có thể chuyển radio sang
    // RX_SINGLE. Sau thời gian idle dài, việc trộn hai
    // cơ chế này có thể làm trạng thái RX không ổn định.
    //
    // Chỉ parse khi DIO0 báo đã nhận xong packet.
    // =================================================

    if (
        digitalRead(LORA_DIO0)
        == LOW
    )
    {
        return false;
    }


    int packetSize =
        LoRa.parsePacket();


    if (packetSize <= 0)
    {
        // Có thể là RX_DONE kèm CRC error hoặc IRQ bất thường.
        // Khôi phục RX continuous ngay.
        LoRa.receive();

        return false;
    }


    // =================================================
    // ĐỌC TRỌN PACKET RA BUFFER TẠM
    // =================================================

    uint8_t raw_packet[128];

    size_t raw_len =
        0;


    while (
        LoRa.available()
        &&
        raw_len < sizeof(raw_packet)
    )
    {
        raw_packet[raw_len++] =
            (uint8_t)LoRa.read();
    }


    // Nếu packet lớn hơn buffer tạm thì xả nốt.
    while (LoRa.available())
    {
        LoRa.read();
    }


    // QUAY LẠI RX NGAY.
    LoRa.receive();


    if (
        raw_len
        != (size_t)packetSize
    )
    {
        return false;
    }


    // =================================================
    // BỎ PACKET SU TRỰC TIẾP
    //
    // Đây là điều quan trọng để DU chỉ nghe qua rBS.
    // =================================================

    if (
        packetSize == SIZE_SESSION_INNER
        ||
        packetSize == SIZE_VOICE_INNER
    )
    {
        return false;
    }


    // =================================================
    // END AUDIO: 5 BYTE
    // =================================================

    if (
        packetSize == SIZE_END_RELAY
    )
    {
        uint8_t dst =
            raw_packet[0];

        uint8_t src =
            raw_packet[1];

        uint8_t type =
            raw_packet[2];


        if (
            dst == ID_TRAM_DU
            &&
            src == ID_TRAM_RBS
            &&
            type == TYPE_RELAY_END
        )
        {
            memset(
                buffer,
                0,
                SIZE_VOICE_INNER
            );


            // Tạo control packet nội bộ 4B cho main.cpp.
            buffer[0] =
                ID_TRAM_DU;

            buffer[1] =
                ID_TRAM_RBS;

            buffer[2] =
                TYPE_AUDIO_END;

            buffer[3] =
                0;


            do_dai_nhan =
                4;


            return true;
        }


        return false;
    }


    // =================================================
    // CHỈ CHẤP NHẬN WRAPPER 16B / 100B
    // =================================================

    if (
        packetSize != SIZE_SESSION_RELAY
        &&
        packetSize != SIZE_VOICE_RELAY
    )
    {
        return false;
    }


    // =================================================
    // KIỂM TRA OUTER HEADER
    // =================================================

    uint8_t outer_dst =
        raw_packet[0];

    uint8_t outer_src =
        raw_packet[1];

    uint8_t outer_type =
        raw_packet[2];

    uint8_t inner_len =
        raw_packet[3];


    if (
        outer_dst != ID_TRAM_DU
        ||
        outer_src != ID_TRAM_RBS
        ||
        outer_type != TYPE_RELAY
    )
    {
        return false;
    }


    if (
        inner_len != SIZE_SESSION_INNER
        &&
        inner_len != SIZE_VOICE_INNER
    )
    {
        return false;
    }


    if (
        packetSize
        != (4 + inner_len)
    )
    {
        return false;
    }


    // =================================================
    // KIỂM TRA PACKET GỐC BÊN TRONG
    //
    // Packet gốc vẫn phải là SU -> DU.
    // =================================================

    uint8_t *inner =
        &raw_packet[4];


    if (
        inner[0] != ID_TRAM_DU
        ||
        inner[1] != ID_TRAM_SU
    )
    {
        return false;
    }


    // =================================================
    // UNWRAP
    //
    // Copy packet SU nguyên vẹn về buffer main.cpp.
    // AES-GCM nhìn nguyên packet 96B bên trong wrapper.
    // AAD vẫn là 8 byte header gốc của SU.
    // =================================================

    memset(
        buffer,
        0,
        SIZE_VOICE_INNER
    );


    memcpy(
        buffer,
        inner,
        inner_len
    );


    do_dai_nhan =
        inner_len;


    return true;
}