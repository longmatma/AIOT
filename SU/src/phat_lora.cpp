#include "phat_lora.h"

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
// ID DÙNG CHO PACKET READY
// =====================================================

static const uint8_t ID_SU_READY  = 0x01;
static const uint8_t ID_RBS_READY = 0x03;


// =====================================================
// TYPE PACKET ĐIỀU KHIỂN
// =====================================================

static const uint8_t TYPE_READY = 0x03;


// =====================================================
// KHỞI TẠO LORA
// =====================================================

void KhoiTao_LoRa()
{
    // =================================================
    // SPI
    // =================================================

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


    // =================================================
    // KHỞI TẠO SX1278 433 MHz
    // =================================================

    if (!LoRa.begin(433E6))
    {
        Serial.println(
            "LOI: Khong tim thay module LoRa!"
        );


        while (1)
        {
            delay(1000);
        }
    }


    // =================================================
    // CẤU HÌNH GIỐNG rBS / DU
    // =================================================

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


    // Sau khởi tạo để radio ở standby
    LoRa.idle();


    Serial.println(
        "Khoi tao LoRa THANH CONG!"
    );
}


// =====================================================
// PHÁT PACKET LORA
// =====================================================

void Phat_GoiTin_LoRa(
    uint8_t *goi_tin,
    size_t do_dai)
{
    // Radio về standby trước khi TX
    LoRa.idle();


    // =================================================
    // TẠO PACKET
    // =================================================

    LoRa.beginPacket();


    LoRa.write(
        goi_tin,
        do_dai
    );


    // =================================================
    // PHÁT BLOCKING
    //
    // Hàm chỉ return khi packet đã TX xong.
    // =================================================

    LoRa.endPacket();
}


// =====================================================
// CHỜ READY TỪ rBS
//
// rBS gửi packet 4 BYTE:
//
// Byte 0 = DST  = SU  = 0x01
// Byte 1 = SRC  = rBS = 0x03
// Byte 2 = TYPE = READY = 0x03
// Byte 3 = 0
//
// Khi nhận READY:
// SU được phép gửi burst tiếp theo.
// =====================================================

bool Cho_READY_RBS(
    uint32_t timeout_ms)
{
    uint32_t bat_dau =
        millis();


    Serial.println(
        "[SU] CHUYEN TX -> RX | CHO READY..."
    );


    // =================================================
    // CHUYỂN SX1278 SANG RECEIVE
    // =================================================

    LoRa.receive();


    while (
        millis() - bat_dau
        <
        timeout_ms
    )
    {
        // =============================================
        // KIỂM TRA PACKET MỚI
        // =============================================

        int packetSize =
            LoRa.parsePacket();


        if (packetSize > 0)
        {
            // rBS bản 4-frame có thể relay packet 100B tới DU.
            // SU phải đọc/xả TRỌN packet đó trước khi chờ READY.
            uint8_t buffer[128];

            size_t so_byte_da_doc =
                0;


            // =========================================
            // ĐỌC PACKET
            // =========================================

            while (
                LoRa.available()
                &&
                so_byte_da_doc
                    <
                sizeof(buffer)
            )
            {
                buffer[
                    so_byte_da_doc
                ] =
                    (uint8_t)
                    LoRa.read();


                so_byte_da_doc++;
            }


            // =========================================
            // READY hiện là 5 BYTE:
            // 4 byte RadioHead header + 1 byte payload 0x00
            // =========================================

            if (
                packetSize == 5
                &&
                so_byte_da_doc == 5
            )
            {
                uint8_t dst =
                    buffer[0];


                uint8_t src =
                    buffer[1];


                uint8_t packet_type =
                    buffer[2]
                    & 0x7F;


                // =====================================
                // KIỂM TRA READY
                // =====================================

                if (
                    dst
                        == ID_SU_READY

                    &&
                    src
                        == ID_RBS_READY

                    &&
                    packet_type
                        == TYPE_READY
                )
                {
                    Serial.println(
                        "[SU RX] READY FROM rBS"
                    );


                    // =============================
                    // Rời RX
                    // Chuẩn bị cho lần TX tiếp theo
                    // =============================

                    LoRa.idle();


                    return true;
                }
            }


            // =========================================
            // PACKET KHÁC -> BỎ QUA
            //
            // Có thể SU nghe thấy packet rBS -> DU.
            // SU chỉ quan tâm TYPE_READY gửi cho SU.
            // =========================================

            Serial.print(
                "[SU DROP RX] Packet khong phai READY | SIZE = "
            );


            Serial.println(
                packetSize
            );
        }


        // Nhường CPU một chút
        delay(1);
    }


    // =================================================
    // TIMEOUT
    // =================================================

    LoRa.idle();


    Serial.println(
        "[SU TIMEOUT] Khong nhan READY tu rBS!"
    );


    return false;
}