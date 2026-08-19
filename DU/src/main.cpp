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
#define TYPE_AUDIO_END      0x04
#define TYPE_FEC            0x05

#define TYPE_MASK           0x0F
#define FLAG_LAST_AUDIO     0x10
#define COUNT_SHIFT         5

#define SIZE_VOICE_PACKET   176
#define SIZE_FEC_PACKET     184
#define SIZE_MAX_PACKET     184
#define SIZE_SESSION_PACKET 12
#define SIZE_AUDIO_END_PACKET 4

#define VOICE_LENGTH        168
#define VOICE_PAYLOAD_BYTES   160
#define VOICE_GCM_TAG_BYTES      8
#define VOICE_PROTECTED_BYTES  168
#define MAX_FRAME_PACKET      8
#define FEC_DATA_PER_GROUP    8

#define ID_TRAM_SU_PROTO      0x01
#define ID_TRAM_DU_PROTO      0x02

// Hàm PLC mới được thêm trong giai_ma_speex.cpp.
extern void GiaiMa_KhungMat(
    uint8_t *pcm_ra_320b
);


// =====================================================
// FEC GROUP STATE
//
// DU không decode Speex ngay khi VOICE đến.
// Nó giữ plaintext 160B trong group tối đa 8 packet.
// Khi FEC tới:
//   - 0 packet mất: decode bình thường.
//   - đúng 1 packet mất: XOR parity để khôi phục.
//   - >1 packet mất: dùng Speex PLC cho phần không cứu được.
//
// Vì toàn hệ thống đang theo kiến trúc whole-utterance,
// việc chờ parity không làm thay đổi nguyên tắc phát audio:
// END_AUDIO -> toàn bộ PCM đã sẵn sàng -> PLAY.
// =====================================================

struct FECGroupState
{
    bool active;
    uint32_t start_seq;

    bool present[FEC_DATA_PER_GROUP];

    // Ciphertext160 + original VOICE tag8.
    // Cần giữ để XOR recover packet bị mất.
    uint8_t protected_block[
        FEC_DATA_PER_GROUP
    ][VOICE_PROTECTED_BYTES];

    // Plaintext chỉ được lưu sau khi VOICE GCM PASS
    // hoặc recovered VOICE GCM PASS.
    uint8_t payload[
        FEC_DATA_PER_GROUP
    ][VOICE_PAYLOAD_BYTES];

    uint8_t frame_count[
        FEC_DATA_PER_GROUP
    ];

    uint8_t expected_count;

    bool has_last;
    uint8_t final_frames;
};


static FECGroupState fec_group;


static void Reset_FEC_Group()
{
    memset(
        &fec_group,
        0,
        sizeof(fec_group)
    );
}


static void Start_FEC_Group(
    uint32_t start_seq)
{
    Reset_FEC_Group();

    fec_group.active =
        true;

    fec_group.start_seq =
        start_seq;
}


// =====================================================
// ĐẨY 1 FRAME PCM VÀO AUDIO BUFFER
// =====================================================

static void Day_PCM_Vao_Buffer(
    const int16_t pcm[160])
{
    if (
        xRingbufferSend(
            Audio_Buffer,
            (void *)pcm,
            320,
            pdMS_TO_TICKS(10)
        )
        != pdTRUE
    )
    {
        Serial.println(
            "[DU ERROR] Audio Buffer day!"
        );
    }
}


// =====================================================
// DECODE 1 DATA PACKET HOẶC PLC CHO PACKET MẤT
// =====================================================

static void Decode_Data_Slot(
    uint8_t slot,
    uint8_t so_frame)
{
    int16_t pcm[160];

    if (
        so_frame < 1
        ||
        so_frame > MAX_FRAME_PACKET
    )
    {
        so_frame =
            MAX_FRAME_PACKET;
    }

    if (fec_group.present[slot])
    {
        for (
            uint8_t f = 0;
            f < so_frame;
            f++
        )
        {
            uint8_t voice_frame[20];

            memcpy(
                voice_frame,
                &fec_group.payload[slot][f * 20],
                20
            );

            GiaiMa_KhungThoai(
                voice_frame,
                (uint8_t *)pcm
            );

            Day_PCM_Vao_Buffer(
                pcm
            );
        }
    }
    else
    {
        // FEC không cứu được -> Speex PLC 20ms/frame.
        for (
            uint8_t f = 0;
            f < so_frame;
            f++
        )
        {
            GiaiMa_KhungMat(
                (uint8_t *)pcm
            );

            Day_PCM_Vao_Buffer(
                pcm
            );
        }
    }
}


// =====================================================
// FLUSH GROUP THEO THỨ TỰ SEQ
// =====================================================

static void Flush_FEC_Group(
    const char *ly_do)
{
    if (!fec_group.active)
    {
        return;
    }

    uint8_t count =
        fec_group.expected_count;

    if (
        count < 1
        ||
        count > FEC_DATA_PER_GROUP
    )
    {
        count =
            FEC_DATA_PER_GROUP;
    }

    uint8_t missing =
        0;

    for (
        uint8_t i = 0;
        i < count;
        i++
    )
    {
        if (!fec_group.present[i])
        {
            missing++;
        }
    }

    Serial.printf(
        "[DU FEC] FLUSH GROUP=%u | DATA=%u | MISSING=%u | REASON=%s\n",
        fec_group.start_seq,
        count,
        missing,
        ly_do
    );

    for (
        uint8_t i = 0;
        i < count;
        i++
    )
    {
        uint8_t frames =
            MAX_FRAME_PACKET;

        if (
            fec_group.has_last
            &&
            i == count - 1
        )
        {
            frames =
                fec_group.final_frames;
        }
        else if (
            fec_group.present[i]
            &&
            fec_group.frame_count[i] >= 1
            &&
            fec_group.frame_count[i] <= MAX_FRAME_PACKET
        )
        {
            frames =
                fec_group.frame_count[i];
        }

        Decode_Data_Slot(
            i,
            frames
        );
    }

    Reset_FEC_Group();
}


// =====================================================
// TASK 1
// NHẬN PACKET LORA
// =====================================================

void TacVu_LoRaRX(void *thamSo)
{
    uint8_t goi_tin[SIZE_MAX_PACKET];

    size_t do_dai = 0;

    while (1)
    {
        memset(
            goi_tin,
            0,
            sizeof(goi_tin)
        );

        if (
            Nhan_GoiTin_LoRa(
                goi_tin,
                do_dai
            )
        )
        {
            uint8_t packet_type =
                goi_tin[2]
                & TYPE_MASK;

            if (
                packet_type
                == TYPE_SESSION_START
            )
            {
                if (
                    do_dai
                    != SIZE_SESSION_PACKET
                )
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
            else if (
                packet_type
                    == TYPE_VOICE
                ||
                packet_type
                    == TYPE_FEC
            )
            {
                if (
                    (
                        packet_type == TYPE_VOICE
                        &&
                        do_dai != SIZE_VOICE_PACKET
                    )
                    ||
                    (
                        packet_type == TYPE_FEC
                        &&
                        do_dai != SIZE_FEC_PACKET
                    )
                )
                {
                    Serial.printf(
                        "[DU DROP] SIZE DATA sai | TYPE=0x%02X | LEN=%u\n",
                        packet_type,
                        (unsigned int)do_dai
                    );

                    continue;
                }

                uint32_t seq_or_group =
                    ((uint32_t)goi_tin[4] << 24)
                    |
                    ((uint32_t)goi_tin[5] << 16)
                    |
                    ((uint32_t)goi_tin[6] << 8)
                    |
                    ((uint32_t)goi_tin[7]);

                if (packet_type == TYPE_VOICE)
                {
                    uint8_t frames =
                        ((goi_tin[2] >> COUNT_SHIFT) & 0x07)
                        + 1;

                    bool last_audio =
                        (
                            goi_tin[2]
                            & FLAG_LAST_AUDIO
                        )
                        != 0;

                    Serial.printf(
                        "[DU RX] VOICE | SEQ=%u | FRAME=%u | LAST=%u\n",
                        seq_or_group,
                        frames,
                        last_audio ? 1 : 0
                    );
                }
                else
                {
                    uint8_t data_count =
                        ((goi_tin[2] >> COUNT_SHIFT) & 0x07)
                        + 1;

                    bool has_last =
                        (
                            goi_tin[2]
                            & FLAG_LAST_AUDIO
                        )
                        != 0;

                    Serial.printf(
                        "[DU RX] FEC | GROUP_START=%u | DATA=%u | HAS_LAST=%u\n",
                        seq_or_group,
                        data_count,
                        has_last ? 1 : 0
                    );
                }
            }
            else if (
                packet_type
                == TYPE_AUDIO_END
            )
            {
                if (
                    do_dai
                    != SIZE_AUDIO_END_PACKET
                )
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
            else
            {
                Serial.printf(
                    "[DU DROP] TYPE khong hop le: 0x%02X\n",
                    packet_type
                );

                continue;
            }

            if (
                xQueueSend(
                    HangDoi_GoiTinNhan,
                    goi_tin,
                    pdMS_TO_TICKS(10)
                )
                != pdTRUE
            )
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
    uint8_t goi_tin[SIZE_MAX_PACKET];

    Reset_FEC_Group();

    while (1)
    {
        if (
            xQueueReceive(
                HangDoi_GoiTinNhan,
                goi_tin,
                portMAX_DELAY
            )
            != pdTRUE
        )
        {
            continue;
        }

        uint8_t packet_type =
            goi_tin[2]
            & TYPE_MASK;


        // =================================================
        // END AUDIO
        // =================================================

        if (
            packet_type
            == TYPE_AUDIO_END
        )
        {
            if (!cho_phep_phat_audio)
            {
                // Nếu final parity bị mất, vẫn flush group.
                // Missing slot không cứu được sẽ dùng Speex PLC.
                Flush_FEC_Group(
                    "END_AUDIO"
                );

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
        // SESSION_START
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
                session_moi
                    == session_id_hien_tai
            )
            {
                Serial.printf(
                    "[DU] SESSION LAP LAI = %016llX\n",
                    (unsigned long long)session_moi
                );

                continue;
            }

            // Nếu vì lỗi control session cũ còn group,
            // flush bằng PLC trước khi reset.
            Flush_FEC_Group(
                "NEW_SESSION"
            );

            session_id_hien_tai =
                session_moi;

            da_co_session =
                true;

            cho_phep_phat_audio =
                false;

            da_co_seq =
                false;

            seq_cuoi_da_xu_ly =
                0;

            Reset_FEC_Group();

            Serial.printf(
                "[DU] NEW SESSION = %016llX\n",
                (unsigned long long)session_id_hien_tai
            );

            continue;
        }


        if (!da_co_session)
        {
            Serial.println(
                "[DU DROP] DATA chua co SESSION_ID!"
            );

            continue;
        }


        // =================================================
        // FEC PACKET — PARITY TRÊN CIPHERTEXT + ORIGINAL TAG
        // =================================================

        if (
            packet_type
            == TYPE_FEC
        )
        {
            uint8_t data_count =
                ((goi_tin[2] >> COUNT_SHIFT) & 0x07)
                + 1;

            bool has_last =
                (
                    goi_tin[2]
                    & FLAG_LAST_AUDIO
                )
                != 0;

            uint8_t final_frames =
                goi_tin[3];

            uint32_t group_start =
                ((uint32_t)goi_tin[4] << 24)
                |
                ((uint32_t)goi_tin[5] << 16)
                |
                ((uint32_t)goi_tin[6] << 8)
                |
                ((uint32_t)goi_tin[7]);

            if (
                data_count < 1
                ||
                data_count > FEC_DATA_PER_GROUP
            )
            {
                Serial.println(
                    "[DU DROP] FEC data_count sai!"
                );

                continue;
            }

            if (
                has_last
                &&
                (
                    final_frames < 1
                    ||
                    final_frames > MAX_FRAME_PACKET
                )
            )
            {
                Serial.println(
                    "[DU DROP] FEC final_frame sai!"
                );

                continue;
            }

            // =============================================
            // FEC packet tự có GCM riêng:
            // encrypted parity = 168B
            // FEC tag          = 8B tại byte176..183
            // =============================================

            uint8_t parity_protected[
                VOICE_PROTECTED_BYTES
            ];

            uint32_t fec_nonce =
                FEC_IV_DOMAIN
                |
                group_start;

            if (
                !GiaiMa_GCM_FEC(
                    &goi_tin[0],
                    &goi_tin[8],
                    parity_protected,
                    &goi_tin[176],
                    session_id_hien_tai,
                    fec_nonce
                )
            )
            {
                Serial.printf(
                    "[DU FEC GCM FAIL] GROUP=%u\n",
                    group_start
                );

                // Không dùng parity không xác thực.
                continue;
            }

            if (
                !fec_group.active
            )
            {
                Start_FEC_Group(
                    group_start
                );
            }
            else if (
                fec_group.start_seq
                != group_start
            )
            {
                Flush_FEC_Group(
                    "FEC_GROUP_SWITCH"
                );

                Start_FEC_Group(
                    group_start
                );
            }

            fec_group.expected_count =
                data_count;

            fec_group.has_last =
                has_last;

            fec_group.final_frames =
                has_last
                ? final_frames
                : 0;

            uint8_t missing_count =
                0;

            int missing_slot =
                -1;

            for (
                uint8_t i = 0;
                i < data_count;
                i++
            )
            {
                if (!fec_group.present[i])
                {
                    missing_count++;

                    missing_slot =
                        i;
                }
            }

            if (missing_count == 1)
            {
                // =========================================
                // 1) KHÔI PHỤC 168B:
                //    ciphertext160 + original VOICE tag8
                // =========================================

                uint8_t recovered_protected[
                    VOICE_PROTECTED_BYTES
                ];

                memcpy(
                    recovered_protected,
                    parity_protected,
                    sizeof(recovered_protected)
                );

                for (
                    uint8_t i = 0;
                    i < data_count;
                    i++
                )
                {
                    if (
                        i
                        ==
                        (uint8_t)missing_slot
                    )
                    {
                        continue;
                    }

                    if (!fec_group.present[i])
                    {
                        continue;
                    }

                    for (
                        size_t b = 0;
                        b < VOICE_PROTECTED_BYTES;
                        b++
                    )
                    {
                        recovered_protected[b] ^=
                            fec_group.protected_block[i][b];
                    }
                }

                // =========================================
                // 2) DỰNG LẠI HEADER VOICE 8B
                //
                // Packet bình thường luôn 8 frame.
                // Chỉ packet cuối có thể 1..8 frame.
                // FEC header mang has_last + final_frames.
                // =========================================

                uint32_t recovered_seq =
                    group_start
                    +
                    (uint32_t)missing_slot;

                bool recovered_last =
                    has_last
                    &&
                    missing_slot
                        == data_count - 1;

                uint8_t recovered_frames =
                    recovered_last
                    ? final_frames
                    : MAX_FRAME_PACKET;

                uint8_t recovered_header[8];

                recovered_header[0] =
                    ID_TRAM_DU_PROTO;

                recovered_header[1] =
                    ID_TRAM_SU_PROTO;

                recovered_header[2] =
                    TYPE_VOICE
                    |
                    ((recovered_frames - 1) << COUNT_SHIFT)
                    |
                    (recovered_last ? FLAG_LAST_AUDIO : 0x00);

                recovered_header[3] =
                    VOICE_LENGTH;

                recovered_header[4] =
                    (recovered_seq >> 24) & 0xFF;

                recovered_header[5] =
                    (recovered_seq >> 16) & 0xFF;

                recovered_header[6] =
                    (recovered_seq >> 8) & 0xFF;

                recovered_header[7] =
                    recovered_seq & 0xFF;

                // =========================================
                // 3) VERIFY ORIGINAL VOICE GCM TAG
                //
                // recovered_protected[0..159] = ciphertext
                // recovered_protected[160..167] = original tag
                //
                // Đây là bước bắt buộc:
                // FEC recover xong chưa được tin ngay.
                // =========================================

                uint8_t recovered_plain[
                    VOICE_PAYLOAD_BYTES
                ];

                bool recovered_gcm_ok =
                    GiaiMa_GCM(
                        recovered_header,
                        &recovered_protected[0],
                        recovered_plain,
                        &recovered_protected[VOICE_PAYLOAD_BYTES],
                        session_id_hien_tai,
                        recovered_seq
                    );

                if (recovered_gcm_ok)
                {
                    memcpy(
                        fec_group.protected_block[
                            missing_slot
                        ],
                        recovered_protected,
                        sizeof(recovered_protected)
                    );

                    memcpy(
                        fec_group.payload[
                            missing_slot
                        ],
                        recovered_plain,
                        sizeof(recovered_plain)
                    );

                    fec_group.present[
                        missing_slot
                    ] =
                        true;

                    fec_group.frame_count[
                        missing_slot
                    ] =
                        recovered_frames;

                    Serial.printf(
                        "[DU FEC RECOVER + VOICE GCM OK] SEQ=%u | SLOT=%d\n",
                        recovered_seq,
                        missing_slot
                    );
                }
                else
                {
                    Serial.printf(
                        "[DU FEC RECOVER BUT VOICE GCM FAIL] SEQ=%u | SLOT=%d\n",
                        recovered_seq,
                        missing_slot
                    );
                }
            }
            else if (
                missing_count > 1
            )
            {
                Serial.printf(
                    "[DU FEC] KHONG CUU DUOC | GROUP=%u | MISSING=%u\n",
                    group_start,
                    missing_count
                );
            }
            else
            {
                Serial.printf(
                    "[DU FEC] GROUP=%u | KHONG MAT DATA\n",
                    group_start
                );
            }

            Flush_FEC_Group(
                "FEC_READY"
            );

            continue;
        }


        // =================================================
        // VOICE PACKET
        // =================================================

        if (
            packet_type
            != TYPE_VOICE
        )
        {
            Serial.println(
                "[DU DROP] Khong phai VOICE/FEC!"
            );

            continue;
        }

        uint8_t so_frame =
            ((goi_tin[2] >> COUNT_SHIFT) & 0x07)
            + 1;

        bool last_audio =
            (
                goi_tin[2]
                & FLAG_LAST_AUDIO
            )
            != 0;

        if (
            so_frame < 1
            ||
            so_frame > MAX_FRAME_PACKET
        )
        {
            Serial.println(
                "[DU DROP] FRAME_COUNT sai!"
            );

            continue;
        }

        if (
            goi_tin[3]
            != VOICE_LENGTH
        )
        {
            Serial.printf(
                "[DU DROP] LENGTH VOICE sai: %u\n",
                goi_tin[3]
            );

            continue;
        }

        uint32_t seq =
            ((uint32_t)goi_tin[4] << 24)
            |
            ((uint32_t)goi_tin[5] << 16)
            |
            ((uint32_t)goi_tin[6] << 8)
            |
            ((uint32_t)goi_tin[7]);

        if (seq & 0x80000000UL)
        {
            Serial.println(
                "[DU DROP] VOICE SEQ vao FEC nonce domain!"
            );

            continue;
        }

        // =================================================
        // AUTHENTICATE TRƯỚC KHI TIN HEADER/GROUP STATE
        // =================================================

        uint8_t payload_sach[
            VOICE_PAYLOAD_BYTES
        ];

        if (
            !GiaiMa_GCM(
                &goi_tin[0],
                &goi_tin[8],
                payload_sach,
                &goi_tin[168],
                session_id_hien_tai,
                seq
            )
        )
        {
            Serial.printf(
                "[DU VOICE GCM FAIL -> XEM NHU MISSING] SEQ=%u\n",
                seq
            );

            // Không dùng LAST/frame/group metadata từ packet GCM fail.
            // FEC packet đã authenticated sẽ cung cấp metadata group/final.
            continue;
        }

        uint32_t group_start =
            seq
            -
            (
                seq
                % FEC_DATA_PER_GROUP
            );

        uint8_t slot =
            (uint8_t)(
                seq - group_start
            );

        if (!fec_group.active)
        {
            Start_FEC_Group(
                group_start
            );
        }
        else if (
            fec_group.start_seq
            != group_start
        )
        {
            // Parity group trước bị mất.
            // Nếu đủ data vẫn decode; thiếu thì PLC.
            Flush_FEC_Group(
                "NEW_DATA_GROUP"
            );

            Start_FEC_Group(
                group_start
            );
        }

        if (fec_group.present[slot])
        {
            Serial.printf(
                "[DU DROP DUP] VOICE SEQ=%u\n",
                seq
            );

            continue;
        }

        // Lưu protected block đúng như trên sóng:
        // ciphertext160 + original GCM tag8.
        memcpy(
            fec_group.protected_block[slot],
            &goi_tin[8],
            VOICE_PROTECTED_BYTES
        );

        memcpy(
            fec_group.payload[slot],
            payload_sach,
            sizeof(payload_sach)
        );

        fec_group.present[slot] =
            true;

        fec_group.frame_count[slot] =
            so_frame;

        if (last_audio)
        {
            fec_group.has_last =
                true;

            fec_group.expected_count =
                slot + 1;

            fec_group.final_frames =
                so_frame;
        }

        Serial.printf(
            "[DU VOICE GCM OK] SEQ=%u | SLOT=%u\n",
            seq,
            slot
        );
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
    // Mỗi item = 184 byte (max VOICE/FEC)
    //
    // SESSION chỉ dùng 12 byte đầu.
    // =================================================

    HangDoi_GoiTinNhan =
        xQueueCreate(
            50,
            SIZE_MAX_PACKET
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