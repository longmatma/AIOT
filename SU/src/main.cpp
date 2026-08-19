#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "driver/adc.h"

#include "thu_am.h"
#include "nen_speex.h"
#include "dong_goi.h"
#include "phat_lora.h"
#include "hien_thi.h"


extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;


// ========================================================
// CẤU HÌNH
// ========================================================

#define CHAN_NUT_PTT 6

#define MAX_KHUNG_THOAI 3000


// ========================================================
// BURST TDD
//
// 1 packet = tối đa 4 frame = 96 byte
//            = tối đa 80 ms audio.
//
// Dùng 2 packet / burst:
// 2 x 4 frame = 8 frame = khoảng 160 ms audio.
//
// Như vậy lượng audio mỗi burst gần như giữ bằng trước,
// nhưng số packet / burst giảm từ 4 xuống 2.
// ========================================================

#define BURST_SIZE 2

#define READY_TIMEOUT_MS 3000

// ========================================================
// CONTROL SU -> rBS
//
// Không còn gửi BURST_END riêng.
// Packet VOICE cuối mang cờ LAST_AUDIO ngay trong byte 3/AAD.
//
// TYPE_AUDIO_END_SU vẫn được giữ:
//   - gửi sau READY của burst cuối;
//   - đóng vai trò fallback/confirmation cho END_AUDIO sớm
//     mà rBS đã gửi tới DU.
// ========================================================

#define TYPE_AUDIO_END_SU 0x04

#define ID_TRAM_SU_CTRL   0x01
#define ID_TRAM_DU_CTRL   0x02


// ========================================================
// BỘ NHỚ AUDIO
// ========================================================

uint8_t *Kho_Chua_AmThanh;

uint32_t tong_so_khung_da_ghi = 0;

// Đo thời gian giữ PTT thực tế cho từng câu.
uint32_t thoi_diem_bat_dau_ghi_ms = 0;


// ========================================================
// TRẠNG THÁI HỆ THỐNG
// ========================================================

enum TrangThaiHeThong
{
    NGHI_NGOI,

    DANG_GHI_AM,

    DANG_PHAT_SONG
};


TrangThaiHeThong trang_thai =
    NGHI_NGOI;


// ========================================================
// SETUP
// ========================================================

void setup()
{
    Serial.begin(
        115200
    );


    pinMode(
        CHAN_NUT_PTT,
        INPUT_PULLUP
    );


    // ====================================================
    // OLED
    // ====================================================

    Wire.begin(
        4,
        5
    );


    KhoiTao_OLED();


    Ve_GiaoDien_OLED(
        0,
        false
    );


    // ====================================================
    // AUDIO ADC
    // ====================================================

    KhoiTao_ThuAm_DMA();


    adc_digi_stop();


    uint8_t rac[1280];

    uint32_t len;


    while (
        adc_digi_read_bytes(
            rac,
            1280,
            &len,
            0
        )
        == ESP_OK
        &&
        len > 0
    )
    {
    }


    // ====================================================
    // SPEEX
    // ====================================================

    KhoiTao_MayEp_Speex();


    // ====================================================
    // LORA
    // ====================================================

    KhoiTao_LoRa();


    // ====================================================
    // CẤP PHÁT RAM
    // ====================================================

    Kho_Chua_AmThanh =
        (uint8_t *)
        heap_caps_malloc(
            MAX_KHUNG_THOAI * 20,
            MALLOC_CAP_8BIT
        );


    if (
        Kho_Chua_AmThanh
        == NULL
    )
    {
        Serial.println(
            "LOI: Chip khong du RAM!"
        );


        while (1)
        {
        }
    }


    Serial.println(
        "[SU] Khoi tao thanh cong!"
    );
}


// ========================================================
// LOOP
// ========================================================

void loop()
{
    bool nut_dang_bam =
        (
            digitalRead(
                CHAN_NUT_PTT
            )
            == LOW
        );


    static uint32_t thoi_gian_ve_oled =
        0;


    // ====================================================
    // TRẠNG THÁI 1
    // VỪA BẤM PTT
    // ====================================================

    if (
        nut_dang_bam
        &&
        trang_thai == NGHI_NGOI
    )
    {
        trang_thai =
            DANG_GHI_AM;


        tong_so_khung_da_ghi =
            0;


        // Ghi lại thời điểm PTT thực sự chuyển sang trạng thái RECORD.
        thoi_diem_bat_dau_ghi_ms =
            millis();


        adc_digi_start();


        Ve_GiaoDien_OLED(
            0,
            true
        );


        Serial.println(
            ">> BAT DAU GHI AM VAO RAM..."
        );
    }


    // ====================================================
    // TRẠNG THÁI 2
    // ĐANG GHI ÂM
    // ====================================================

    else if (
        nut_dang_bam
        &&
        trang_thai == DANG_GHI_AM
    )
    {
        uint8_t mang_tam_PCM[320];

        uint8_t khung_speex_20b[20];


        if (
            LayMau_AmThanh(
                mang_tam_PCM
            )
        )
        {
            // ============================================
            // OLED
            // ============================================

            if (
                millis()
                -
                thoi_gian_ve_oled
                > 150
            )
            {
                int bien_do =
                    Tinh_BienDo_Mic(
                        mang_tam_PCM,
                        320
                    );


                Ve_GiaoDien_OLED(
                    bien_do,
                    true
                );


                thoi_gian_ve_oled =
                    millis();
            }


            // ============================================
            // SPEEX
            // ============================================

            if (
                tong_so_khung_da_ghi
                <
                MAX_KHUNG_THOAI
            )
            {
                if (
                    Nen_Thanh_KhungThoai(
                        mang_tam_PCM,
                        khung_speex_20b
                    )
                )
                {
                    memcpy(
                        &Kho_Chua_AmThanh[
                            tong_so_khung_da_ghi
                            *
                            20
                        ],

                        khung_speex_20b,

                        20
                    );


                    tong_so_khung_da_ghi++;
                }
            }
        }
    }


    // ====================================================
    // TRẠNG THÁI 3
    // NHẢ PTT -> BẮT ĐẦU GỬI
    // ====================================================

    else if (
        !nut_dang_bam
        &&
        trang_thai == DANG_GHI_AM
    )
    {
        trang_thai =
            DANG_PHAT_SONG;


        // =================================================
        // ĐO THỜI GIAN GHI THỰC TẾ
        //
        // 1 frame Speex = 20 ms audio.
        // Nếu HOLD_MS lớn nhưng FRAME rất nhỏ -> lỗi ADC/record.
        // Nếu HOLD_MS cũng chỉ ~160 ms -> PTT thực sự bị nhả sớm.
        // =================================================

        uint32_t thoi_gian_giu_ptt_ms =
            millis() - thoi_diem_bat_dau_ghi_ms;


        Serial.printf(
            "[SU RECORD] HOLD_MS=%u | FRAME=%u | AUDIO_MS=%u\n",
            thoi_gian_giu_ptt_ms,
            (unsigned int)tong_so_khung_da_ghi,
            (unsigned int)(tong_so_khung_da_ghi * 20)
        );


        // =================================================
        // DỪNG ADC
        // =================================================

        adc_digi_stop();


        uint8_t rac[1280];

        uint32_t len;


        while (
            adc_digi_read_bytes(
                rac,
                1280,
                &len,
                0
            )
            == ESP_OK
            &&
            len > 0
        )
        {
        }


        // =================================================
        // OLED
        // =================================================

        u8g2.clearBuffer();


        u8g2.setFont(
            u8g2_font_ncenB14_tr
        );


        u8g2.drawStr(
            10,
            35,
            "DANG GUI..."
        );


        u8g2.sendBuffer();


        Serial.print(
            ">> DA NHA NUT! Dang gui "
        );


        Serial.print(
            tong_so_khung_da_ghi
        );


        Serial.println(
            " khung qua LoRa..."
        );


        // =================================================
        // BUFFER PACKET
        // =================================================

        uint8_t goi_tin_hoan_chinh[96];


        // =================================================
        // TẠO SESSION MỚI
        // =================================================

        Tao_Session_Moi();


        // =================================================
        // SESSION_START
        // =================================================

        uint8_t goi_session[12];


        Tao_GoiTin_SessionStart(
            goi_session
        );


        // =================================================
        // SESSION_START LOW-LATENCY
        //
        // Trước đây:
        //   gửi 3 lần + delay 100 ms sau mỗi lần
        //   -> mất hơn 300 ms trước VOICE đầu tiên.
        //
        // Bây giờ:
        //   gửi 2 lần, chỉ nghỉ 5 ms GIỮA hai packet.
        //   Không delay sau packet thứ hai.
        //
        // Vẫn giữ 2 bản để có dự phòng nếu một SESSION bị mất.
        // =================================================

        for (
            int lan = 0;
            lan < 2;
            lan++
        )
        {
            Phat_GoiTin_LoRa(
                goi_session,
                12
            );


            if (lan == 0)
            {
                delay(
                    5
                );
            }
        }


        // =================================================
        // BẮT ĐẦU GỬI VOICE THEO BURST
        // =================================================

        uint8_t dem_packet_trong_burst =
            0;


        bool gui_that_bai =
            false;


        // =================================================
        // MỖI PACKET = TỐI ĐA 4 FRAME
        //
        // 1 frame  = 20 ms
        // 4 frame  = 80 ms audio
        // VOICE    = 96 byte cố định
        // =================================================

        for (
            uint32_t i = 0;

            i < tong_so_khung_da_ghi;

            i += 4
        )
        {
            uint8_t *khung_1 =
                &Kho_Chua_AmThanh[
                    i * 20
                ];


            uint8_t *khung_2 =
                nullptr;

            uint8_t *khung_3 =
                nullptr;

            uint8_t *khung_4 =
                nullptr;


            uint8_t so_frame =
                1;


            if (
                i + 1
                <
                tong_so_khung_da_ghi
            )
            {
                khung_2 =
                    &Kho_Chua_AmThanh[
                        (i + 1) * 20
                    ];

                so_frame =
                    2;
            }


            if (
                i + 2
                <
                tong_so_khung_da_ghi
            )
            {
                khung_3 =
                    &Kho_Chua_AmThanh[
                        (i + 2) * 20
                    ];

                so_frame =
                    3;
            }


            if (
                i + 3
                <
                tong_so_khung_da_ghi
            )
            {
                khung_4 =
                    &Kho_Chua_AmThanh[
                        (i + 3) * 20
                    ];

                so_frame =
                    4;
            }


            // =============================================
            // PACKET CUỐI CÙNG CỦA TOÀN BỘ AUDIO?
            //
            // Cần xác định TRƯỚC khi AES-GCM để cờ LAST_AUDIO
            // được đặt vào byte 3 và nằm trong AAD.
            // =============================================

            bool la_packet_cuoi =
                (
                    i + 4
                    >=
                    tong_so_khung_da_ghi
                );


            // =============================================
            // ĐÓNG GÓI VOICE 96 BYTE
            //
            // 0-7     HEADER / AAD
            // byte 3  bit7 = LAST_AUDIO
            // 8-87    CIPHERTEXT 80B
            // 88-95   GCM TAG 8B
            // =============================================

            Tao_GoiTin_LoRa(
                khung_1,
                khung_2,
                khung_3,
                khung_4,
                so_frame,
                la_packet_cuoi,
                goi_tin_hoan_chinh
            );


            // =============================================
            // ĐỌC SEQ32
            // =============================================

            uint32_t seq_num_tx =
                ((uint32_t)
                    goi_tin_hoan_chinh[4]
                    << 24)
                |
                ((uint32_t)
                    goi_tin_hoan_chinh[5]
                    << 16)
                |
                ((uint32_t)
                    goi_tin_hoan_chinh[6]
                    << 8)
                |
                ((uint32_t)
                    goi_tin_hoan_chinh[7]);


            Serial.print(
                "[SU TX] SEQ = "
            );


            Serial.println(
                seq_num_tx
            );


            // =============================================
            // PHÁT 1 PACKET 96 BYTE
            // =============================================

            Phat_GoiTin_LoRa(
                goi_tin_hoan_chinh,
                96
            );


            dem_packet_trong_burst++;


            // =============================================
            // CHƯA ĐỦ BURST
            //
            // Cho rBS một khoảng rất nhỏ để quay lại
            // vòng receive Python.
            //
            // KHÔNG dùng delay(40).
            // =============================================

            if (
                dem_packet_trong_burst
                    < BURST_SIZE
                &&
                !la_packet_cuoi
            )
            {
                delay(
                    5
                );
            }


            // =============================================
            // ĐỦ 2 PACKET
            //
            // HOẶC BURST CUỐI CHỈ CÓ 1 PACKET
            //
            // -> SU DỪNG PHÁT
            // -> chuyển sang RX
            // -> chờ READY từ rBS
            // =============================================

            if (
                dem_packet_trong_burst
                    >= BURST_SIZE

                ||

                la_packet_cuoi
            )
            {
                Serial.print(
                    "[SU] BURST XONG | "
                    "SO PACKET = "
                );


                Serial.println(
                    dem_packet_trong_burst
                );


                Serial.println(
                    "[SU] CHO READY TU rBS..."
                );


                // =========================================
                // Hàm này nằm trong phat_lora.cpp
                //
                // SU chuyển sang RX và chờ:
                //
                // DST  = SU
                // SRC  = rBS
                // TYPE = READY = 0x03
                // =========================================

                bool co_ready =
                    Cho_READY_RBS(
                        READY_TIMEOUT_MS
                    );


                if (!co_ready)
                {
                    Serial.println(
                        "[SU ERROR] "
                        "KHONG NHAN DUOC READY!"
                    );


                    gui_that_bai =
                        true;


                    break;
                }


                Serial.println(
                    "[SU] READY OK -> "
                    "GUI BURST TIEP"
                );


                // Burst mới
                dem_packet_trong_burst =
                    0;
            }
        }


        // =================================================
        // KẾT THÚC
        // =================================================

        if (!gui_that_bai)
        {
            // =================================================
            // AUDIO_END FALLBACK / CONFIRMATION
            //
            // rBS đã biết packet cuối qua LAST_AUDIO và đã gửi
            // END_AUDIO sớm cho DU trước READY.
            //
            // Control này vẫn gửi sau READY để:
            //   1) xác nhận SU đã hoàn tất phiên;
            //   2) cho rBS gửi thêm một END dự phòng nếu END sớm mất.
            // =================================================

            // =================================================
            // AUDIO_END = 5 BYTE
            //
            // QUAN TRỌNG:
            // rBS dùng thư viện Adafruit RFM9x.
            // Driver này loại packet vật lý < 5 byte vì mặc định
            // coi 4 byte đầu là RadioHead header.
            //
            // Vì vậy AUDIO_END không thể chỉ có 4 byte.
            //
            // Byte 0 = DST  = DU
            // Byte 1 = SRC  = SU
            // Byte 2 = TYPE = AUDIO_END
            // Byte 3 = 0
            // Byte 4 = dummy 0x00
            //
            // rBS chỉ dùng byte 0..3 để parse control.
            // =================================================

            uint8_t goi_audio_end[5] =
            {
                ID_TRAM_DU_CTRL,
                ID_TRAM_SU_CTRL,
                TYPE_AUDIO_END_SU,
                0x00,
                0x00
            };


            Phat_GoiTin_LoRa(
                goi_audio_end,
                sizeof(goi_audio_end)
            );


            Serial.println(
                "[SU CTRL] AUDIO_END 5B -> rBS"
            );


            Serial.println(
                ">> DA GUI XONG!"
            );
        }
        else
        {
            Serial.println(
                ">> GUI BI DUNG DO TIMEOUT READY!"
            );
        }


        // =================================================
        // VỀ TRẠNG THÁI NGHỈ
        // =================================================

        trang_thai =
            NGHI_NGOI;


        Ve_GiaoDien_OLED(
            0,
            false
        );
    }


    // ====================================================
    // TRẠNG THÁI NGHỈ
    // ====================================================

    else if (
        !nut_dang_bam
        &&
        trang_thai == NGHI_NGOI
    )
    {
        delay(
            10
        );
    }
}