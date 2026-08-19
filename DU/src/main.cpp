#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <LoRa.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "nhan_lora.h"
#include "giai_ma_speex.h"
#include "man_hinh.h"
#include "ma_hoa.h"


// =====================================================
// CẤU HÌNH AUDIO
// =====================================================

#define CHAN_AUDIO_OUT      1
#define PWM_CHANNEL         0

// Tần số lấy mẫu âm thanh
#define AUDIO_SAMPLE_RATE   8000

// PWM carrier để phát audio
#define PWM_CARRIER_FREQ    100000

#define PWM_RESOLUTION      8


// =====================================================
// QUEUE / RING BUFFER
// =====================================================

QueueHandle_t HangDoi_GoiTinNhan;
RingbufHandle_t Audio_Buffer;


// =====================================================
// AUDIO BUFFER TRONG PSRAM
//
// ESP32-S3 N16R8 có 8 MB PSRAM.
// Dùng 2 MB cho PCM toàn bộ câu.
//
// PCM hiện tại:
// 8 kHz * 16 bit = 16000 byte/giây.
//
// 2 MB đủ dư cho câu rất dài, trong khi vẫn chừa nhiều
// PSRAM cho các phần khác của hệ thống.
// =====================================================

#define AUDIO_BUFFER_SIZE_PSRAM (2 * 1024 * 1024)

StaticRingbuffer_t *Audio_Buffer_Struct =
    nullptr;

uint8_t *Audio_Buffer_Storage =
    nullptr;


// DU chỉ bắt đầu lấy PCM ra khỏi Audio_Buffer sau khi
// nhận END_AUDIO từ rBS.
volatile bool cho_phep_phat_audio = false;


// =====================================================
// PHIÊN BẢO MẬT HIỆN TẠI
// =====================================================

// SESSION_ID nhận từ SU
uint64_t session_id_hien_tai = 0;

// DU chỉ giải mã VOICE khi đã nhận SESSION_START
bool da_co_session = false;


// =====================================================
// CHỐNG PACKET TRÙNG TRONG CÙNG SESSION
// =====================================================

// Sequence cuối cùng DU đã chấp nhận xử lý
uint32_t seq_cuoi_da_xu_ly = 0;

// Đánh dấu đã có sequence hợp lệ trong session hiện tại hay chưa
bool da_co_seq = false;


// =====================================================
// CÁC LOẠI PACKET
// =====================================================

#define TYPE_VOICE          0x01
#define TYPE_SESSION_START  0x02

// Control nội bộ do nhan_lora.cpp tạo khi nhận
// TYPE_RELAY_END từ rBS.
#define TYPE_AUDIO_END      0x04

#define SIZE_VOICE_PACKET   96
#define SIZE_SESSION_PACKET 12
#define SIZE_AUDIO_END_PACKET 4

// Byte 3 của VOICE:
//   bit 7    = LAST_AUDIO
//   bit 0..6 = LENGTH = 88
//
// Chỉ mask khi KIỂM TRA LENGTH.
// Khi GCM authenticate, header vẫn giữ nguyên byte3 để
// LAST_AUDIO được xác thực như một phần của AAD.
#define VOICE_LENGTH_DU       88
#define FLAG_LAST_AUDIO_DU  0x80
#define VOICE_LENGTH_MASK   0x7F


// =====================================================
// TASK 1
// NHẬN PACKET LORA
// =====================================================

void TacVu_LoRaRX(void *thamSo)
{
    uint8_t goi_tin[SIZE_VOICE_PACKET];

    size_t do_dai = 0;

    while (1)
    {
        // Xóa buffer để packet SESSION 12 byte
        // không còn dữ liệu dư của packet trước
        memset(
            goi_tin,
            0,
            sizeof(goi_tin)
        );


        if (Nhan_GoiTin_LoRa(
                goi_tin,
                do_dai))
        {
            // =================================================
            // XÁC ĐỊNH LOẠI PACKET
            // =================================================

            uint8_t packet_type =
                goi_tin[2] & 0x3F;


            // =================================================
            // SESSION_START
            // =================================================

            if (packet_type == TYPE_SESSION_START)
            {
                if (do_dai != SIZE_SESSION_PACKET)
                {
                    Serial.println(
                        "[DU DROP] SESSION sai kich thuoc!"
                    );

                    continue;
                }


                Serial.println(
                    "[DU RX] SESSION_START"
                );
            }


            // =================================================
            // VOICE
            // =================================================

            else if (packet_type == TYPE_VOICE)
            {
                if (do_dai != SIZE_VOICE_PACKET)
                {
                    Serial.println(
                        "[DU DROP] VOICE sai kich thuoc!"
                    );

                    continue;
                }


                // =============================================
                // SEQ32
                // =============================================

                uint32_t seq_num_rx =
                    ((uint32_t)goi_tin[4] << 24)
                    |
                    ((uint32_t)goi_tin[5] << 16)
                    |
                    ((uint32_t)goi_tin[6] << 8)
                    |
                    ((uint32_t)goi_tin[7]);


                Serial.print(
                    "[DU RX] VOICE SEQ = "
                );

                Serial.println(
                    seq_num_rx
                );


                // =============================================
                // KHÔNG cập nhật OLED trong đường RX thời gian thực.
                // Việc cập nhật OLED/I2C giữa các packet có thể
                // làm DU quay lại RX chậm và mất packet kế tiếp.
                // =============================================
            }


            // =================================================
            // END AUDIO TỪ rBS
            // =================================================

            else if (packet_type == TYPE_AUDIO_END)
            {
                if (do_dai != SIZE_AUDIO_END_PACKET)
                {
                    Serial.println(
                        "[DU DROP] END_AUDIO sai kich thuoc!"
                    );

                    continue;
                }


                Serial.println(
                    "[DU RX] END_AUDIO FROM rBS"
                );
            }


            // =================================================
            // PACKET LẠ
            // =================================================

            else
            {
                Serial.printf(
                    "[DU DROP] TYPE khong hop le: 0x%02X\n",
                    packet_type
                );

                continue;
            }


            // =================================================
            // ĐƯA PACKET VÀO QUEUE
            //
            // Queue luôn có item 96 byte.
            // SESSION chỉ dùng 12 byte đầu.
            // Phần còn lại đã memset = 0.
            // =================================================

            if (
                xQueueSend(
                    HangDoi_GoiTinNhan,
                    goi_tin,
                    pdMS_TO_TICKS(10)
                )
                == pdTRUE
            )
            {
                // Không cần in QUEUE OK từng packet
                // vì sẽ làm Serial rất nhiều
            }
            else
            {
                Serial.println(
                    "[DU ERROR] QUEUE FULL -> MAT PACKET!"
                );
            }
        }


        vTaskDelay(
            pdMS_TO_TICKS(1)
        );
    }
}


// =====================================================
// TASK 2
// SESSION + AES + SPEEX
// =====================================================

void TacVu_GiaiMa(void *thamSo)
{
    uint8_t goi_tin[SIZE_VOICE_PACKET];

    int16_t pcm_frames[4][160];


    while (1)
    {
        if (
            xQueueReceive(
                HangDoi_GoiTinNhan,
                goi_tin,
                portMAX_DELAY
            )
            == pdTRUE
        )
        {
            // =================================================
            // 1. XÁC ĐỊNH LOẠI PACKET
            // =================================================

            uint8_t packet_type =
                goi_tin[2] & 0x3F;


            // =================================================
            // END AUDIO
            //
            // END nằm sau toàn bộ VOICE trong cùng Queue.
            // Vì vậy khi tới đây, PCM của câu hiện tại đã được
            // giải mã và nằm trong Audio_Buffer.
            // =================================================

            if (
                packet_type
                == TYPE_AUDIO_END
            )
            {
                // rBS có thể gửi END_AUDIO lặp để tăng độ tin cậy.
                // Chỉ END đầu tiên mới được phép mở cổng phát.
                if (!cho_phep_phat_audio)
                {
                    Serial.println(
                        "[DU AUDIO] DA NHAN DU CAU -> BAT DAU PHAT"
                    );

                    cho_phep_phat_audio =
                        true;
                }
                else
                {
                    Serial.println(
                        "[DU AUDIO] END_AUDIO LAP LAI -> BO QUA"
                    );
                }


                continue;
            }


            // =================================================
            // 2. SESSION_START
            //
            // Byte:
            //
            // 0      DST
            // 1      SRC
            // 2      TYPE = 0x02
            // 3      LENGTH = 8
            // 4-11   SESSION_ID 64-bit
            // =================================================

            if (
                packet_type
                == TYPE_SESSION_START
            )
            {
                uint64_t session_moi =
                    ((uint64_t)goi_tin[4] << 56)
                    |
                    ((uint64_t)goi_tin[5] << 48)
                    |
                    ((uint64_t)goi_tin[6] << 40)
                    |
                    ((uint64_t)goi_tin[7] << 32)
                    |
                    ((uint64_t)goi_tin[8] << 24)
                    |
                    ((uint64_t)goi_tin[9] << 16)
                    |
                    ((uint64_t)goi_tin[10] << 8)
                    |
                    ((uint64_t)goi_tin[11]);

                if (session_moi == 0)
                {
                    Serial.println(
                        "[DU DROP] SESSION_ID = 0!"
                    );
                    continue;
                }

                if (
                    da_co_session
                    &&
                    session_moi == session_id_hien_tai
                )
                {
                    Serial.printf(
                        "[DU] SESSION LAP LAI = %016llX\n",
                        (unsigned long long)session_moi
                    );
                    continue;
                }

                session_id_hien_tai =
                    session_moi;

                da_co_session =
                    true;


                // Session mới: chỉ buffer, chưa phát.
                cho_phep_phat_audio =
                    false;


                da_co_seq =
                    false;

                seq_cuoi_da_xu_ly =
                    0;

                Serial.printf(
                    "[DU] NEW SESSION = %016llX\n",
                    (unsigned long long)session_id_hien_tai
                );

                continue;
            }


            // =================================================
            // 3. CHỈ CHO VOICE ĐI TIẾP
            // =================================================

            if (
                packet_type
                != TYPE_VOICE
            )
            {
                Serial.println(
                    "[DU DROP] Khong phai VOICE!"
                );

                continue;
            }


            // =================================================
            // 4. CHƯA CÓ SESSION THÌ KHÔNG GIẢI MÃ VOICE
            // =================================================

            if (!da_co_session)
            {
                Serial.println(
                    "[DU DROP] VOICE chua co SESSION_ID!"
                );

                continue;
            }


            // =================================================
            // 5. ĐỌC SEQ32
            //
            // Byte 4-7
            // =================================================

            uint32_t so_thu_tu =
                ((uint32_t)goi_tin[4] << 24)
                |
                ((uint32_t)goi_tin[5] << 16)
                |
                ((uint32_t)goi_tin[6] << 8)
                |
                ((uint32_t)goi_tin[7]);


            // =================================================
            // 6. LOẠI PACKET TRÙNG
            //
            // Nếu cùng SESSION_ID + cùng SEQ thì chỉ xử lý 1 lần.
            // =================================================

            if (
                da_co_seq
                &&
                so_thu_tu <= seq_cuoi_da_xu_ly
            )
            {
                Serial.printf(
                    "[DU DROP DUP/OLD] SEQ = %u | LAST = %u\n",
                    so_thu_tu,
                    seq_cuoi_da_xu_ly
                );

                continue;
            }


            // =================================================
            // CHƯA CẬP NHẬT LAST_SEQ Ở ĐÂY
            //
            // Chỉ cập nhật sau khi AES-GCM xác thực thành công.
            // Nếu packet bị lỗi AES, DU vẫn cho phép một bản
            // cùng SEQ hợp lệ đến sau được xử lý.
            // =================================================


            // =================================================
            // 7. XÁC ĐỊNH SỐ FRAME SPEEX
            //
            // Byte 2:
            //   bit 0..5 = TYPE
            //   bit 6..7 = frame_count - 1
            //
            // => hỗ trợ 1..4 frame.
            // =================================================

            uint8_t so_frame =
                ((goi_tin[2] >> 6) & 0x03)
                + 1;


            // =================================================
            // 8. FORMAT VOICE PACKET 96 BYTE
            //
            // Byte 0-7    HEADER / AAD
            // Byte 8-87   Ciphertext 80B
            // Byte 88-95  GCM Tag 8B
            // =================================================

            uint8_t voice_length =
                goi_tin[3]
                & VOICE_LENGTH_MASK;

            if (
                voice_length
                != VOICE_LENGTH_DU
            )
            {
                Serial.printf(
                    "[DU DROP] LENGTH VOICE sai: %u\n",
                    voice_length
                );

                continue;
            }


            uint8_t *header =
                &goi_tin[0];

            uint8_t *payload_ma_hoa =
                &goi_tin[8];

            uint8_t *auth_tag =
                &goi_tin[88];

            uint8_t payload_sach[80];


            // =================================================
            // 9. AES-GCM AUTHENTICATE + DECRYPT
            //
            // IV:
            // SESSION_ID 64-bit || SEQ32
            //
            // AAD vẫn giữ 8 byte header.
            // =================================================

            if (
                GiaiMa_GCM(
                    header,
                    payload_ma_hoa,
                    payload_sach,
                    auth_tag,
                    session_id_hien_tai,
                    so_thu_tu
                )
            )
            {
                // =============================================
                // AES THÀNH CÔNG
                //
                // CHỈ cập nhật LAST_SEQ sau auth thành công.
                // =============================================

                seq_cuoi_da_xu_ly =
                    so_thu_tu;

                da_co_seq =
                    true;


                // =============================================
                // GIẢI MÃ LẦN LƯỢT 1..4 FRAME
                //
                // Speex decoder state được giữ đúng thứ tự.
                // Mỗi frame PCM = 160 sample = 320 byte.
                // =============================================

                for (
                    uint8_t frame_index = 0;
                    frame_index < so_frame;
                    frame_index++
                )
                {
                    uint8_t voice_frame[20];


                    memcpy(
                        voice_frame,
                        &payload_sach[
                            frame_index * 20
                        ],
                        20
                    );


                    GiaiMa_KhungThoai(
                        voice_frame,
                        (uint8_t *)
                        pcm_frames[
                            frame_index
                        ]
                    );


                    if (
                        xRingbufferSend(
                            Audio_Buffer,
                            pcm_frames[
                                frame_index
                            ],
                            320,
                            pdMS_TO_TICKS(10)
                        )
                        != pdTRUE
                    )
                    {
                        Serial.printf(
                            "[DU ERROR] Audio Buffer day - FRAME %u!\n",
                            frame_index + 1
                        );
                    }
                }
            }


            // =================================================
            // AES-GCM AUTH FAIL
            // =================================================

            else
            {
                Serial.printf(
                    "[DU AES FAIL] SEQ = %u\n",
                    so_thu_tu
                );
            }
        }
    }
}


// =====================================================
// TASK 3
// PHÁT ÂM THANH
// =====================================================

void TacVu_PhatAmThanh(void *thamSo)
{
    size_t kich_thuoc_lay_duoc;

    bool dang_phat_loa =
        false;

    int dem_khung_thoai =
        0;


    // 8 kHz -> 125 us / sample
    const uint32_t CHU_KY_MAU_US =
        1000000UL / AUDIO_SAMPLE_RATE;


    while (1)
    {
        // =================================================
        // CHƯA CÓ END_AUDIO
        //
        // Không lấy dữ liệu ra khỏi RingBuffer.
        // Nhờ vậy toàn bộ câu được tích lại trước khi phát.
        // =================================================

        if (!cho_phep_phat_audio)
        {
            vTaskDelay(
                pdMS_TO_TICKS(5)
            );

            continue;
        }


        // =================================================
        // ĐÃ CÓ END_AUDIO -> PHÁT LIÊN TỤC
        // =================================================

        uint8_t *pcm_data =
            (uint8_t *)
            xRingbufferReceive(
                Audio_Buffer,
                &kich_thuoc_lay_duoc,
                pdMS_TO_TICKS(50)
            );


        if (pcm_data != NULL)
        {
            // =============================================
            // BẬT AUDIO OUTPUT
            // =============================================

            if (!dang_phat_loa)
            {
                pinMode(
                    CHAN_AUDIO_OUT,
                    OUTPUT
                );


                ledcAttachPin(
                    CHAN_AUDIO_OUT,
                    PWM_CHANNEL
                );


                dang_phat_loa =
                    true;


                Serial.println(
                    "[DU AUDIO] PLAY"
                );
            }


            int16_t *pcm16 =
                (int16_t *)pcm_data;


            int so_mau =
                kich_thuoc_lay_duoc / 2;


            uint32_t thoi_gian_mau_tiep_theo =
                esp_timer_get_time();


            for (
                int i = 0;
                i < so_mau;
                i++
            )
            {
                int pwm_val =
                    128
                    +
                    (pcm16[i] / 128);


                if (pwm_val < 0)
                {
                    pwm_val = 0;
                }


                if (pwm_val > 255)
                {
                    pwm_val = 255;
                }


                ledcWrite(
                    PWM_CHANNEL,
                    pwm_val
                );


                thoi_gian_mau_tiep_theo +=
                    CHU_KY_MAU_US;


                while (
                    esp_timer_get_time()
                    <
                    thoi_gian_mau_tiep_theo
                )
                {
                    // busy wait
                }
            }


            vRingbufferReturnItem(
                Audio_Buffer,
                (void *)pcm_data
            );


            dem_khung_thoai++;


            if (
                dem_khung_thoai
                >= 50
            )
            {
                vTaskDelay(
                    pdMS_TO_TICKS(1)
                );

                dem_khung_thoai =
                    0;
            }
        }


        // =================================================
        // BUFFER ĐÃ PHÁT HẾT
        // =================================================

        else
        {
            if (dang_phat_loa)
            {
                ledcDetachPin(
                    CHAN_AUDIO_OUT
                );


                pinMode(
                    CHAN_AUDIO_OUT,
                    INPUT
                );


                dang_phat_loa =
                    false;
            }


            dem_khung_thoai =
                0;


            // Quay về chế độ buffer cho câu tiếp theo.
            cho_phep_phat_audio =
                false;


            Serial.println(
                "[DU AUDIO] PHAT XONG -> CHO CAU TIEP"
            );
        }
    }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);


    // =================================================
    // OLED
    // =================================================

    KhoiTao_OLED();


    CapNhat_TrangThai_OLED(
        "Dang kiem tra LoRa...",
        0,
        0
    );


    // =================================================
    // LORA
    // =================================================

    KhoiTao_LoRa_RX();


    // =================================================
    // SPEEX
    // =================================================

    KhoiTao_GiaiMa_Speex();


    CapNhat_TrangThai_OLED(
        "Dang cho song...",
        0,
        0
    );


    // =================================================
    // PWM CARRIER 100 kHz
    // =================================================

    ledcSetup(
        PWM_CHANNEL,
        PWM_CARRIER_FREQ,
        PWM_RESOLUTION
    );


    // Audio GPIO ban đầu High-Z
    pinMode(
        CHAN_AUDIO_OUT,
        INPUT
    );


    // =================================================
    // QUEUE
    //
    // Mỗi item = 96 byte
    //
    // SESSION chỉ dùng 12 byte đầu.
    // =================================================

    HangDoi_GoiTinNhan =
        xQueueCreate(
            50,
            SIZE_VOICE_PACKET
        );


    // =================================================
    // AUDIO RING BUFFER TRONG PSRAM
    // =================================================

    size_t psram_total =
        heap_caps_get_total_size(
            MALLOC_CAP_SPIRAM
        );


    size_t psram_free_truoc =
        heap_caps_get_free_size(
            MALLOC_CAP_SPIRAM
        );


    Serial.printf(
        "[DU PSRAM] TOTAL = %u bytes | FREE BEFORE = %u bytes\n",
        (unsigned int)psram_total,
        (unsigned int)psram_free_truoc
    );


    // Nếu PSRAM chưa được bật trong cấu hình board,
    // tổng dung lượng MALLOC_CAP_SPIRAM sẽ bằng 0.
    if (
        psram_total
        == 0
    )
    {
        Serial.println(
            "[DU ERROR] KHONG TIM THAY PSRAM!"
        );


        Serial.println(
            "[DU ERROR] Kiem tra cau hinh PSRAM/OPI cua ESP32-S3 N16R8."
        );


        while (1)
        {
            delay(1000);
        }
    }


    // Cấp phát control block của RingBuffer trong PSRAM.
    Audio_Buffer_Struct =
        (StaticRingbuffer_t *)
        heap_caps_malloc(
            sizeof(
                StaticRingbuffer_t
            ),
            MALLOC_CAP_SPIRAM
        );


    // Cấp phát 2 MB storage thật của Audio Buffer trong PSRAM.
    Audio_Buffer_Storage =
        (uint8_t *)
        heap_caps_malloc(
            AUDIO_BUFFER_SIZE_PSRAM,
            MALLOC_CAP_SPIRAM
        );


    if (
        Audio_Buffer_Struct
            == nullptr

        ||

        Audio_Buffer_Storage
            == nullptr
    )
    {
        Serial.println(
            "[DU ERROR] CAP PHAT PSRAM CHO AUDIO BUFFER THAT BAI!"
        );


        Serial.printf(
            "[DU PSRAM] FREE NOW = %u bytes\n",
            (unsigned int)
            heap_caps_get_free_size(
                MALLOC_CAP_SPIRAM
            )
        );


        while (1)
        {
            delay(1000);
        }
    }


    // Tạo RingBuffer từ chính vùng nhớ PSRAM đã cấp phát.
    Audio_Buffer =
        xRingbufferCreateStatic(
            AUDIO_BUFFER_SIZE_PSRAM,
            RINGBUF_TYPE_NOSPLIT,
            Audio_Buffer_Storage,
            Audio_Buffer_Struct
        );


    Serial.printf(
        "[DU PSRAM] AUDIO BUFFER = %u bytes (%.2f MB)\n",
        (unsigned int)AUDIO_BUFFER_SIZE_PSRAM,
        AUDIO_BUFFER_SIZE_PSRAM
            / 1024.0
            / 1024.0
    );


    Serial.printf(
        "[DU PSRAM] FREE AFTER = %u bytes\n",
        (unsigned int)
        heap_caps_get_free_size(
            MALLOC_CAP_SPIRAM
        )
    );


    // =================================================
    // KIỂM TRA QUEUE
    // =================================================

    if (
        HangDoi_GoiTinNhan
        == NULL
    )
    {
        Serial.println(
            "[DU ERROR] Tao Queue that bai!"
        );


        while (1)
        {
            delay(1000);
        }
    }


    // =================================================
    // KIỂM TRA AUDIO BUFFER
    // =================================================

    if (
        Audio_Buffer
        == NULL
    )
    {
        Serial.println(
            "[DU ERROR] Tao Audio Buffer that bai!"
        );


        while (1)
        {
            delay(1000);
        }
    }


    // =================================================
    // TASK AUDIO
    // =================================================

    xTaskCreatePinnedToCore(
        TacVu_PhatAmThanh,
        "PWM_OUT",
        8192,
        NULL,
        3,
        NULL,
        0
    );


    // =================================================
    // TASK LORA RX
    // =================================================

    xTaskCreatePinnedToCore(
        TacVu_LoRaRX,
        "LoRa_RX",
        8192,
        NULL,
        4,
        NULL,
        1
    );


    // =================================================
    // TASK AES + SPEEX
    // =================================================

    xTaskCreatePinnedToCore(
        TacVu_GiaiMa,
        "Giai_Ma",
        10240,
        NULL,
        2,
        NULL,
        1
    );


    Serial.println(
        "[DU] Khoi tao thanh cong!"
    );
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
    vTaskDelete(NULL);
}