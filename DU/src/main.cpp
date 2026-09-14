#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <LoRa.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_idf_version.h"

#include "nhan_lora.h"
#include "giai_ma_speex.h"
#include "man_hinh.h"
#include "ma_hoa.h"
#include "gps_du.h"
#include "hmi_du.h"


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
// BAO CAO VI TRI DINH KY DU -> rBS
// DU khoi dong lech SU 2.5 giay de giam kha nang hai beacon trung nhau.
// =====================================================
#define CHU_KY_BAO_CAO_VI_TRI_DU_MS 5000UL
#define TRE_BAO_CAO_VI_TRI_DU_LUC_KHOI_DONG_MS 3500UL

volatile uint64_t so_thu_tu_bao_cao_vi_tri_du = 0;


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

// Task bao PLAY_STARTED ve rBS, tach khoi core phat audio.
// Audio task chi notify SAU khi first PWM sample da duoc ghi.
TaskHandle_t Task_PlayReport_Handle = nullptr;
volatile uint64_t play_report_session_id = 0;

// =====================================================
// AUDIO PLAYBACK V2 - HIGH RESOLUTION TIMER / DOUBLE BUFFER
//
// Khong con busy-wait 125 us trong PWM_OUT.
// esp_timer danh nhip 8 kHz; callback chi:
//   - lay 1 sample tu double buffer noi bo,
//   - cap nhat duty PWM,
//   - doi slot sau 160 sample,
//   - notify audio task khi 1 slot duoc giai phong.
//
// Audio task block cho scheduler/IDLE0 chay binh thuong.
// Hai slot 160 sample giup lien tuc giua cac frame 20 ms.
// =====================================================

TaskHandle_t Task_Audio_Handle = nullptr;
esp_timer_handle_t Audio_Sample_Timer = nullptr;

static constexpr uint16_t AUDIO_TIMER_FRAME_SAMPLES = 160;
static constexpr uint64_t AUDIO_TIMER_PERIOD_US =
    1000000ULL / AUDIO_SAMPLE_RATE;

static int16_t Audio_Timer_Frame[2][AUDIO_TIMER_FRAME_SAMPLES];

static volatile uint16_t Audio_Timer_Count[2] = {0, 0};
static volatile bool Audio_Timer_Ready[2] = {false, false};
static volatile uint8_t Audio_Timer_Active_Slot = 0;
static volatile uint16_t Audio_Timer_Sample_Index = 0;
static volatile bool Audio_Timer_Playing = false;
static volatile bool Audio_Timer_First_Sample = false;

static portMUX_TYPE Audio_Timer_Mux =
    portMUX_INITIALIZER_UNLOCKED;

// Session cua cau vua phat xong, dung cho AUTO_ACK/NACK.
// Tach khoi session_id_hien_tai de khong nham neu session moi den som.
volatile uint64_t du_last_played_session_id = 0;

// =====================================================
// HMI DU DA TACH SANG hmi_du.cpp / hmi_du.h
// main.cpp khong con quan ly GPIO nut/LED hay transaction ACK/NACK.
// =====================================================


// =====================================================
// SELF-HEALING V1 - DU ESP32-S3
//
// DU có nhiều task FreeRTOS nên KHÔNG đăng ký trực tiếp từng task
// vào Task Watchdog. Một task SUPERVISOR sẽ:
//   1) nhận heartbeat từ LoRa_RX / Giai_Ma / PWM_OUT,
//   2) theo dõi timer audio khi đang phát,
//   3) tự đăng ký chính nó vào ESP Task Watchdog.
//
// Nếu một task quan trọng mất heartbeat:
//   -> Supervisor ghi log lỗi,
//   -> gọi esp_restart() để khởi tạo lại toàn bộ DU.
//
// Nếu bản thân Supervisor / scheduler bị treo:
//   -> Supervisor không feed Task Watchdog,
//   -> watchdog phần cứng của ESP32 tự reset.
//
// DU_WDT_SELF_TEST chỉ dùng để kiểm thử.
// Production phải để = 0.
// =====================================================

#define DU_WDT_TIMEOUT_S                 12U
#define DU_WDT_SELF_TEST                  0U
#define DU_WDT_SELF_TEST_DELAY_MS      10000U

#define DU_SUPERVISOR_PERIOD_MS          500U
#define DU_STARTUP_GRACE_MS             8000U

#define DU_HB_LORA_TIMEOUT_MS           5000U
#define DU_HB_DECODE_TIMEOUT_MS         5000U
#define DU_HB_AUDIO_TASK_TIMEOUT_MS     5000U

// Khi Audio_Timer_Playing=true, callback 8 kHz phải còn tiến triển.
// Cho dư 1 giây để tránh false-positive khi scheduler bận.
#define DU_AUDIO_TIMER_STALL_MS         1000U

static volatile uint32_t du_hb_lora_ms = 0;
static volatile uint32_t du_hb_decode_ms = 0;
static volatile uint32_t du_hb_audio_task_ms = 0;

// Callback timer chỉ tăng counter; không gọi millis()/Serial trong callback 8 kHz.
static volatile uint32_t du_audio_timer_pulse_count = 0;

static TaskHandle_t Task_Supervisor_Handle = nullptr;
static uint32_t du_supervisor_boot_ms = 0;
static bool du_wdt_self_test_arm = false;


static inline void DU_Heartbeat_LoRa()
{
    du_hb_lora_ms = millis();
}


static inline void DU_Heartbeat_Decode()
{
    du_hb_decode_ms = millis();
}


static inline void DU_Heartbeat_AudioTask()
{
    du_hb_audio_task_ms = millis();
}


static void DU_Fatal_Reboot(const char *ly_do)
{
    Serial.printf(
        "[DU SELF-HEAL FATAL] %s -> REBOOT SAU 1 GIAY\n",
        ly_do != nullptr ? ly_do : "UNKNOWN"
    );

    Serial.flush();
    delay(1000);
    ESP.restart();

    // Chỉ là fallback nếu reset chưa xảy ra ngay.
    while (true)
    {
        delay(1000);
    }
}


static void DU_WDT_Init_For_Supervisor()
{
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms = DU_WDT_TIMEOUT_S * 1000U;
    cfg.idle_core_mask = 0;
    cfg.trigger_panic = true;

    esp_err_t err =
        esp_task_wdt_init(&cfg);

    if (err == ESP_ERR_INVALID_STATE)
    {
        // Arduino core có thể đã init TWDT trước đó.
        (void)esp_task_wdt_reconfigure(&cfg);
    }
#else
    (void)esp_task_wdt_init(
        DU_WDT_TIMEOUT_S,
        true
    );
#endif

    if (esp_task_wdt_status(NULL) != ESP_OK)
    {
        (void)esp_task_wdt_add(NULL);
    }

    du_supervisor_boot_ms = millis();

#if DU_WDT_SELF_TEST
    const esp_reset_reason_t reset_reason =
        esp_reset_reason();

    if (
        reset_reason == ESP_RST_TASK_WDT
        || reset_reason == ESP_RST_WDT
        || reset_reason == ESP_RST_INT_WDT
    )
    {
        du_wdt_self_test_arm = false;

        Serial.println(
            "[DU WDT] SELF_TEST PASS: reboot do watchdog, KHONG lap lai."
        );
    }
    else
    {
        du_wdt_self_test_arm = true;

        Serial.println(
            "[DU WDT] SELF_TEST armed: se gia lap treo Supervisor sau 10s."
        );
    }
#else
    du_wdt_self_test_arm = false;
#endif

    (void)esp_task_wdt_reset();

    Serial.printf(
        "[DU WDT] BAT | TIMEOUT=%us | SELF_TEST=%u | RESET_REASON=%d\n",
        (unsigned int)DU_WDT_TIMEOUT_S,
        (unsigned int)DU_WDT_SELF_TEST,
        (int)esp_reset_reason()
    );
}


static bool DU_Heartbeat_QuaHan(
    uint32_t bay_gio,
    uint32_t heartbeat,
    uint32_t timeout_ms)
{
    return (
        heartbeat != 0U
        &&
        (uint32_t)(bay_gio - heartbeat) > timeout_ms
    );
}


static void TacVu_DU_Supervisor(void *tham_so)
{
    (void)tham_so;

    DU_WDT_Init_For_Supervisor();

    uint32_t audio_pulse_truoc =
        du_audio_timer_pulse_count;

    uint32_t audio_pulse_doi_lan_cuoi_ms =
        millis();

    while (1)
    {
        uint32_t bay_gio =
            millis();

#if DU_WDT_SELF_TEST
        if (
            du_wdt_self_test_arm
            &&
            (uint32_t)(
                bay_gio
                -
                du_supervisor_boot_ms
            )
                >= DU_WDT_SELF_TEST_DELAY_MS
        )
        {
            du_wdt_self_test_arm = false;

            Serial.println(
                "[DU WDT TEST] GIA LAP TREO SUPERVISOR: dung feed watchdog..."
            );
            Serial.flush();

            // Supervisor đã subscribe TWDT.
            // Cố tình không feed để xác minh watchdog reset toàn DU.
            while (true)
            {
                vTaskDelay(
                    pdMS_TO_TICKS(1000)
                );
            }
        }
#endif

        // Cho các task đủ thời gian khởi động trước khi bắt lỗi heartbeat.
        if (
            (uint32_t)(
                bay_gio
                -
                du_supervisor_boot_ms
            )
                >= DU_STARTUP_GRACE_MS
        )
        {
            if (
                DU_Heartbeat_QuaHan(
                    bay_gio,
                    du_hb_lora_ms,
                    DU_HB_LORA_TIMEOUT_MS
                )
            )
            {
                DU_Fatal_Reboot(
                    "LoRa_RX mat heartbeat"
                );
            }

            if (
                DU_Heartbeat_QuaHan(
                    bay_gio,
                    du_hb_decode_ms,
                    DU_HB_DECODE_TIMEOUT_MS
                )
            )
            {
                DU_Fatal_Reboot(
                    "Giai_Ma mat heartbeat"
                );
            }

            if (
                DU_Heartbeat_QuaHan(
                    bay_gio,
                    du_hb_audio_task_ms,
                    DU_HB_AUDIO_TASK_TIMEOUT_MS
                )
            )
            {
                DU_Fatal_Reboot(
                    "PWM_OUT mat heartbeat"
                );
            }

            // Theo dõi riêng callback timer khi audio thực sự đang phát.
            // Nếu timer còn chạy thì pulse counter phải thay đổi rất nhanh.
            if (Audio_Timer_Playing)
            {
                uint32_t pulse_hien_tai =
                    du_audio_timer_pulse_count;

                if (
                    pulse_hien_tai
                    != audio_pulse_truoc
                )
                {
                    audio_pulse_truoc =
                        pulse_hien_tai;

                    audio_pulse_doi_lan_cuoi_ms =
                        bay_gio;
                }
                else if (
                    (uint32_t)(
                        bay_gio
                        -
                        audio_pulse_doi_lan_cuoi_ms
                    )
                        > DU_AUDIO_TIMER_STALL_MS
                )
                {
                    DU_Fatal_Reboot(
                        "Audio timer 8kHz dung tien trinh"
                    );
                }
            }
            else
            {
                audio_pulse_truoc =
                    du_audio_timer_pulse_count;

                audio_pulse_doi_lan_cuoi_ms =
                    bay_gio;
            }
        }

        // Supervisor khỏe -> feed watchdog.
        (void)esp_task_wdt_reset();

        vTaskDelay(
            pdMS_TO_TICKS(
                DU_SUPERVISOR_PERIOD_MS
            )
        );
    }
}


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
#define TYPE_USER_CONFIRM_LOCAL 0x06

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
// THONG KE LINK LOCAL TAI DU - HIEN THI OLED
//
// DATA  = so VOICE packet mong doi theo cac FEC group.
// LOST  = so VOICE packet mat TRUOC khi packet-FEC sua.
// FEC   = so VOICE packet packet-FEC khoi phuc thanh cong.
// FINAL = so VOICE packet con mat SAU packet-FEC.
//
// DAY LA PACKET-FEC CUA PHAN MEM, KHONG PHAI FEC PHY CR 4/5.
// =====================================================

struct DULinkOLEDStats
{
    uint32_t expected_voice;
    uint32_t raw_missing_voice;
    uint32_t fec_recovered_voice;
    uint32_t final_missing_voice;
};

static DULinkOLEDStats du_oled_stats;

static void Reset_ThongKe_OLED_DU()
{
    memset(
        &du_oled_stats,
        0,
        sizeof(du_oled_stats)
    );
}


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

    // Dung de ghi lai so packet mat TRUOC khi FEC recovery
    // thay doi present[].
    bool fec_received;
    uint8_t raw_missing_before_recovery;
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

    uint8_t raw_missing =
        fec_group.fec_received
        ? fec_group.raw_missing_before_recovery
        : missing;

    uint8_t recovered =
        raw_missing >= missing
        ? (raw_missing - missing)
        : 0;

    du_oled_stats.expected_voice +=
        count;

    du_oled_stats.raw_missing_voice +=
        raw_missing;

    du_oled_stats.fec_recovered_voice +=
        recovered;

    du_oled_stats.final_missing_voice +=
        missing;

    Serial.printf(
        "[DU FEC] FLUSH GROUP=%u | DATA=%u | RAW_MISSING=%u | FEC_RECOVERED=%u | FINAL_MISSING=%u | REASON=%s\n",
        fec_group.start_seq,
        count,
        raw_missing,
        recovered,
        missing,
        ly_do
    );

    // Khong refresh OLED theo tung FEC group de tranh chen I2C
    // vao duong transport. Counter van duoc cap nhat trong RAM.

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
        DU_Heartbeat_LoRa();

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
            // V10: khoa beacon ngay tai RX task, truoc khi packet vao queue.
            // Tranh khe race: packet da nhan xong nhung decoder chua kip dat session active.
            HMI_DU_TamDung_Beacon(1200UL);

            uint8_t packet_type =
                goi_tin[2]
                & TYPE_MASK;

            if (packet_type == TYPE_USER_CONFIRM_LOCAL)
            {
                if (do_dai != 12)
                {
                    Serial.println("[DU HMI DROP] USER_CONFIRM sai size");
                    continue;
                }

                uint8_t code = goi_tin[3];
                uint64_t sid =
                    ((uint64_t)goi_tin[4] << 56) |
                    ((uint64_t)goi_tin[5] << 48) |
                    ((uint64_t)goi_tin[6] << 40) |
                    ((uint64_t)goi_tin[7] << 32) |
                    ((uint64_t)goi_tin[8] << 24) |
                    ((uint64_t)goi_tin[9] << 16) |
                    ((uint64_t)goi_tin[10] << 8) |
                    ((uint64_t)goi_tin[11]);

                HMI_DU_XuLy_User_Confirm(sid, code);
                continue;
            }

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

            // Chi VOICE/FEC moi lam LED chop theo DATA that.
            if (packet_type == TYPE_VOICE || packet_type == TYPE_FEC)
            {
                HMI_DU_Bao_Nhan_Data();
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

        DU_Heartbeat_LoRa();

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
        DU_Heartbeat_Decode();

        if (
            xQueueReceive(
                HangDoi_GoiTinNhan,
                goi_tin,
                pdMS_TO_TICKS(250)
            )
            != pdTRUE
        )
        {
            DU_Heartbeat_Decode();
            continue;
        }

        DU_Heartbeat_Decode();

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

                // END da toi -> module HMI ket thuc trang thai RX session.
                HMI_DU_Bao_END_Audio();

                // Mo gate TRUOC khi ve OLED. Audio task o core0 co the
                // bat dau ngay, OLED I2C o core1 khong chen them delay vao PLAY.
                cho_phep_phat_audio =
                    true;

                HienThi_DU_KetQua(
                    du_oled_stats.expected_voice,
                    du_oled_stats.raw_missing_voice,
                    du_oled_stats.fec_recovered_voice,
                    du_oled_stats.final_missing_voice
                );
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
                HMI_DU_TamDung_Beacon(1500UL);

                Serial.printf(
                    "[DU] SESSION LAP LAI = %016llX -> GUI LAI SESSION_READY\n",
                    (unsigned long long)session_moi
                );

                DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
                Gui_GPS_REPORT_DU(session_moi, du_lieu_gps_du);
                Gui_SESSION_READY_RBS(session_moi);
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

            HMI_DU_Reset_Cho_Session_Moi();

            cho_phep_phat_audio =
                false;

            da_co_seq =
                false;

            seq_cuoi_da_xu_ly =
                0;

            Reset_FEC_Group();
            Reset_ThongKe_OLED_DU();

            HienThi_DU_DangNhan(
                0,
                0,
                0
            );

            Serial.printf(
                "[DU] NEW SESSION = %016llX\n",
                (unsigned long long)session_id_hien_tai
            );

            // GPS packet rieng: gui snapshot truoc READY de rBS thu duoc
            // trong cua so bat tay. GPS khong gate session.
            DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
            In_TrangThai_GPS_DU(du_lieu_gps_du);
            Gui_GPS_REPORT_DU(session_id_hien_tai, du_lieu_gps_du);

            // May DU tu dong xac nhan, nguoi dung KHONG can bam nut.
            Gui_SESSION_READY_RBS(session_id_hien_tai);

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

            fec_group.fec_received =
                true;

            fec_group.raw_missing_before_recovery =
                missing_count;

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
// TASK BAO DU DA BAT DAU PLAY VE rBS
//
// Khong TX LoRa truc tiep trong audio task, vi endPacket() blocking
// co the lam tre sample audio dau tien. Audio task chi notify task nay
// SAU khi first PWM sample da duoc ghi ra loa.
// =====================================================

void TacVu_BaoPlay(void *thamSo)
{
    while (1)
    {
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        uint64_t session_can_bao =
            play_report_session_id;

        if (session_can_bao != 0)
        {
            Gui_PLAY_STARTED_RBS(
                session_can_bao
            );
        }
    }
}


// =====================================================
// AUDIO TIMER V2 - CALLBACK 8 kHz
//
// Callback cua esp_timer chay trong ESP_TIMER task, KHONG phai ISR.
// Vi vay co the goi ledcWrite() va xTaskNotify().
// Callback phai rat ngan: khong malloc, khong Serial, khong block.
// =====================================================

static void Audio_Sample_Timer_Callback(void *arg)
{
    (void)arg;

    uint8_t slot;
    uint16_t index;
    uint16_t count;
    int16_t sample = 0;

    bool co_sample = false;
    bool la_mau_dau = false;
    bool frame_vua_xong = false;
    uint8_t slot_vua_xong = 0;

    portENTER_CRITICAL(&Audio_Timer_Mux);

    if (Audio_Timer_Playing)
    {
        slot =
            Audio_Timer_Active_Slot;

        index =
            Audio_Timer_Sample_Index;

        count =
            Audio_Timer_Count[slot];

        if (
            Audio_Timer_Ready[slot]
            &&
            index < count
        )
        {
            sample =
                Audio_Timer_Frame[slot][index];

            co_sample =
                true;

            Audio_Timer_Sample_Index =
                index + 1;

            if (Audio_Timer_First_Sample)
            {
                Audio_Timer_First_Sample =
                    false;

                la_mau_dau =
                    true;
            }

            if (
                Audio_Timer_Sample_Index
                >=
                count
            )
            {
                slot_vua_xong =
                    slot;

                frame_vua_xong =
                    true;

                Audio_Timer_Ready[slot] =
                    false;

                uint8_t slot_ke =
                    slot ^ 1U;

                if (Audio_Timer_Ready[slot_ke])
                {
                    Audio_Timer_Active_Slot =
                        slot_ke;

                    Audio_Timer_Sample_Index =
                        0;
                }
                else
                {
                    // Het du lieu san sang.
                    // Timer van ton tai, nhung callback se khong lay sample nua.
                    // Audio task se nap slot moi hoac dung timer khi het cau.
                    Audio_Timer_Playing =
                        false;

                    Audio_Timer_Sample_Index =
                        0;
                }
            }
        }
    }

    portEXIT_CRITICAL(&Audio_Timer_Mux);


    if (co_sample)
    {
        // Heartbeat cực nhẹ cho đường timer 8 kHz.
        du_audio_timer_pulse_count++;

        int pwm_val =
            128
            +
            (sample / 128);

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
    }


    // Moc PLAY that: sample dau tien da duoc day vao PWM.
    if (la_mau_dau)
    {
        play_report_session_id =
            session_id_hien_tai;

        du_last_played_session_id =
            session_id_hien_tai;

        if (Task_PlayReport_Handle != nullptr)
        {
            xTaskNotifyGive(
                Task_PlayReport_Handle
            );
        }
    }


    // Bao audio task slot nao vua duoc phat xong.
    // Callback esp_timer la task context nen dung xTaskNotify binh thuong.
    if (
        frame_vua_xong
        &&
        Task_Audio_Handle != nullptr
    )
    {
        xTaskNotify(
            Task_Audio_Handle,
            (1UL << slot_vua_xong),
            eSetBits
        );
    }
}


// =====================================================
// COPY 1 PCM FRAME TU RINGBUFFER -> TIMER SLOT
// =====================================================

static bool Nap_Audio_Timer_Slot(
    uint8_t slot,
    TickType_t timeout_ticks)
{
    if (slot > 1)
    {
        return false;
    }

    size_t kich_thuoc =
        0;

    uint8_t *pcm_data =
        (uint8_t *)
        xRingbufferReceive(
            Audio_Buffer,
            &kich_thuoc,
            timeout_ticks
        );

    if (pcm_data == nullptr)
    {
        return false;
    }

    size_t so_mau =
        kich_thuoc / sizeof(int16_t);

    if (
        so_mau
        >
        AUDIO_TIMER_FRAME_SAMPLES
    )
    {
        so_mau =
            AUDIO_TIMER_FRAME_SAMPLES;
    }

    // Copy vao RAM noi bo nho gon (320B/slot).
    // Sau copy co the tra ngay item cho RingBuffer.
    memcpy(
        Audio_Timer_Frame[slot],
        pcm_data,
        so_mau * sizeof(int16_t)
    );

    if (
        so_mau
        <
        AUDIO_TIMER_FRAME_SAMPLES
    )
    {
        memset(
            &Audio_Timer_Frame[slot][so_mau],
            0,
            (
                AUDIO_TIMER_FRAME_SAMPLES
                -
                so_mau
            )
            *
            sizeof(int16_t)
        );
    }

    vRingbufferReturnItem(
        Audio_Buffer,
        (void *)pcm_data
    );


    // Publish metadata sau khi data da copy xong.
    portENTER_CRITICAL(&Audio_Timer_Mux);

    Audio_Timer_Count[slot] =
        (uint16_t)so_mau;

    Audio_Timer_Ready[slot] =
        so_mau > 0;

    portEXIT_CRITICAL(&Audio_Timer_Mux);

    return so_mau > 0;
}


// =====================================================
// KHOI DONG LAI TIMER NEU CALLBACK TAM DUNG DO CHUA CO SLOT KE
// =====================================================

static void Audio_Timer_Resume_If_Needed()
{
    portENTER_CRITICAL(&Audio_Timer_Mux);

    if (!Audio_Timer_Playing)
    {
        if (Audio_Timer_Ready[0])
        {
            Audio_Timer_Active_Slot =
                0;

            Audio_Timer_Sample_Index =
                0;

            Audio_Timer_Playing =
                true;
        }
        else if (Audio_Timer_Ready[1])
        {
            Audio_Timer_Active_Slot =
                1;

            Audio_Timer_Sample_Index =
                0;

            Audio_Timer_Playing =
                true;
        }
    }

    portEXIT_CRITICAL(&Audio_Timer_Mux);
}


// =====================================================
// KIEM TRA DOUBLE BUFFER DA PHAT HET
// =====================================================

static bool Audio_Timer_All_Empty()
{
    bool empty;

    portENTER_CRITICAL(&Audio_Timer_Mux);

    empty =
        !Audio_Timer_Ready[0]
        &&
        !Audio_Timer_Ready[1]
        &&
        !Audio_Timer_Playing;

    portEXIT_CRITICAL(&Audio_Timer_Mux);

    return empty;
}


// =====================================================
// TASK 3
// PHAT AM THANH V2 - HIGH RESOLUTION TIMER 8 kHz
//
// Khong busy-wait.
// PWM_OUT chu yeu block cho notification, nen IDLE0 co thoi gian chay.
// =====================================================

void TacVu_PhatAmThanh(void *thamSo)
{
    (void)thamSo;

    // Tao high-resolution periodic timer dung 1 lan.
    esp_timer_create_args_t timer_args = {};
    timer_args.callback =
        &Audio_Sample_Timer_Callback;

    timer_args.arg =
        nullptr;

    timer_args.dispatch_method =
        ESP_TIMER_TASK;

    timer_args.name =
        "du_audio_8k";

    esp_err_t timer_err =
        esp_timer_create(
            &timer_args,
            &Audio_Sample_Timer
        );

    if (
        timer_err != ESP_OK
        ||
        Audio_Sample_Timer == nullptr
    )
    {
        Serial.printf(
            "[DU AUDIO ERROR] Khong tao duoc esp_timer | ERR=%d\n",
            (int)timer_err
        );

        DU_Fatal_Reboot(
            "Khong tao duoc esp_timer audio"
        );
        return;
    }

    bool dang_phat_loa =
        false;


    while (1)
    {
        DU_Heartbeat_AudioTask();

        // =================================================
        // CHUA CO END_AUDIO
        // =================================================

        if (!cho_phep_phat_audio)
        {
            vTaskDelay(
                pdMS_TO_TICKS(5)
            );

            continue;
        }


        // =================================================
        // CHUAN BI DOUBLE BUFFER
        // =================================================

        portENTER_CRITICAL(&Audio_Timer_Mux);

        Audio_Timer_Ready[0] =
            false;

        Audio_Timer_Ready[1] =
            false;

        Audio_Timer_Count[0] =
            0;

        Audio_Timer_Count[1] =
            0;

        Audio_Timer_Active_Slot =
            0;

        Audio_Timer_Sample_Index =
            0;

        Audio_Timer_Playing =
            false;

        Audio_Timer_First_Sample =
            true;

        portEXIT_CRITICAL(&Audio_Timer_Mux);


        // END_AUDIO chi mo gate sau khi toan bo cau da buffer,
        // nen slot dau tien phai co san neu cau co audio.
        bool co_slot_0 =
            Nap_Audio_Timer_Slot(
                0,
                pdMS_TO_TICKS(50)
            );

        if (!co_slot_0)
        {
            // Khong co PCM de phat.
            cho_phep_phat_audio =
                false;

            Serial.println(
                "[DU AUDIO] BUFFER RONG SAU END_AUDIO"
            );

            continue;
        }


        // Prefill slot 1 neu con frame.
        // Khong block: toan bo cau da nam trong RingBuffer.
        bool con_du_lieu_vao =
            Nap_Audio_Timer_Slot(
                1,
                0
            );


        // =================================================
        // BAT AUDIO OUTPUT
        // =================================================

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

            // Midpoint PWM truoc sample dau de tranh click DC lon.
            ledcWrite(
                PWM_CHANNEL,
                128
            );

            dang_phat_loa =
                true;
        }


        portENTER_CRITICAL(&Audio_Timer_Mux);

        Audio_Timer_Active_Slot =
            0;

        Audio_Timer_Sample_Index =
            0;

        Audio_Timer_Playing =
            true;

        Audio_Timer_First_Sample =
            true;

        portEXIT_CRITICAL(&Audio_Timer_Mux);


        Serial.println(
            "[DU AUDIO] PLAY V2 TIMER 8KHZ"
        );


        // Xoa notification cu neu co.
        uint32_t notify_bits =
            0;

        xTaskNotifyWait(
            0,
            0xFFFFFFFFUL,
            &notify_bits,
            0
        );


        timer_err =
            esp_timer_start_periodic(
                Audio_Sample_Timer,
                AUDIO_TIMER_PERIOD_US
            );

        if (timer_err != ESP_OK)
        {
            Serial.printf(
                "[DU AUDIO ERROR] esp_timer_start_periodic FAIL | ERR=%d\n",
                (int)timer_err
            );

            portENTER_CRITICAL(&Audio_Timer_Mux);
            Audio_Timer_Playing = false;
            portEXIT_CRITICAL(&Audio_Timer_Mux);

            cho_phep_phat_audio =
                false;

            continue;
        }


        // =================================================
        // REFILL SLOT TRONG KHI TIMER DANG PHAT
        // =================================================
        //
        // Moi slot = 20 ms audio. Audio task co gan 20 ms de copy
        // frame tiep theo vao slot vua duoc giai phong.
        //
        // Het RingBuffer => con_du_lieu_vao = false.
        // Cho 2 slot cuoi phat het roi dung timer.
        // =================================================

        bool input_exhausted =
            !con_du_lieu_vao;

        while (1)
        {
            DU_Heartbeat_AudioTask();

            notify_bits =
                0;

            xTaskNotifyWait(
                0,
                0xFFFFFFFFUL,
                &notify_bits,
                pdMS_TO_TICKS(100)
            );

            DU_Heartbeat_AudioTask();


            // Slot 0 vua phat xong.
            if (
                notify_bits & 0x01UL
            )
            {
                if (!input_exhausted)
                {
                    if (
                        !Nap_Audio_Timer_Slot(
                            0,
                            0
                        )
                    )
                    {
                        input_exhausted =
                            true;
                    }
                }
            }


            // Slot 1 vua phat xong.
            if (
                notify_bits & 0x02UL
            )
            {
                if (!input_exhausted)
                {
                    if (
                        !Nap_Audio_Timer_Slot(
                            1,
                            0
                        )
                    )
                    {
                        input_exhausted =
                            true;
                    }
                }
            }


            // Neu callback dung tam thoi do slot ke chua san sang,
            // sau khi refill thi cho phep chay lai.
            Audio_Timer_Resume_If_Needed();


            if (
                input_exhausted
                &&
                Audio_Timer_All_Empty()
            )
            {
                break;
            }
        }


        // =================================================
        // DUNG TIMER SAU KHI PHAT HET CAU
        // =================================================

        esp_timer_stop(
            Audio_Sample_Timer
        );

        portENTER_CRITICAL(&Audio_Timer_Mux);
        Audio_Timer_Playing = false;
        portEXIT_CRITICAL(&Audio_Timer_Mux);


        // Dua PWM ve midpoint truoc khi detach.
        ledcWrite(
            PWM_CHANNEL,
            128
        );

        ledcDetachPin(
            CHAN_AUDIO_OUT
        );

        pinMode(
            CHAN_AUDIO_OUT,
            INPUT
        );

        dang_phat_loa =
            false;


        // Quay ve che do buffer cho cau tiep theo.
        cho_phep_phat_audio =
            false;


        Serial.println(
            "[DU AUDIO] PHAT XONG V2 -> CHO CAU TIEP"
        );


        // HMI module tu tao AUTO_ACK va mo quyen NACK
        // cho session vua phat xong.
        HMI_DU_Bao_Phat_Xong(
            du_last_played_session_id
        );
    }
}


// =====================================================
// TASK BAO CAO KENH THAT SU -> DU
// Nhan_LoRa chi luu snapshot RSSI cua beacon SU. Task nay gui snapshot ve rBS
// voi uu tien thap, khong chen vao voice/audio/HMI.
// =====================================================
void TacVu_BaoCao_Kenh_SU_DU(void *tham_so)
{
    (void)tham_so;
    vTaskDelay(pdMS_TO_TICKS(1500));

    while (1)
    {
        bool radio_dang_ban = HMI_DU_Radio_Dang_Ban() || cho_phep_phat_audio;
        if (!radio_dang_ban)
            Gui_BaoCao_Kenh_SU_DU_DangCho();

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}


// =====================================================
// TASK BAO CAO VI TRI DINH KY DU -> rBS
// - Chi gui khi session/data/audio/HMI khong ban radio.
// - Gui best-effort, khong ACK.
// - Ham TX con kiem tra DIO0 + LoRa mutex de khong giat radio khoi RX.
// =====================================================
void TacVu_BaoCao_ViTri_DinhKy_DU(void *tham_so)
{
    (void)tham_so;

    vTaskDelay(pdMS_TO_TICKS(TRE_BAO_CAO_VI_TRI_DU_LUC_KHOI_DONG_MS));

    uint32_t moc_gui_tiep_theo_ms = millis();

    while (1)
    {
        uint32_t bay_gio_ms = millis();

        bool radio_dang_ban =
            HMI_DU_Radio_Dang_Ban()
            || cho_phep_phat_audio;

        if (!radio_dang_ban &&
            (int32_t)(bay_gio_ms - moc_gui_tiep_theo_ms) >= 0)
        {
            DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
            In_TrangThai_GPS_DU(du_lieu_gps_du);

            // V10: chi tang STT SAU KHI TX thanh cong.
            // Beacon bi hoan do radio ban se thu lai cung STT, khong tao
            // "lo hong STT" gia lam rBS hieu nham la mat packet tren khong trung.
            uint64_t stt_du_kien = so_thu_tu_bao_cao_vi_tri_du + 1;
            if (stt_du_kien == 0)
                stt_du_kien = 1;

            bool gui_thanh_cong = Gui_VI_TRI_DINH_KY_DU(
                stt_du_kien,
                du_lieu_gps_du
            );

            if (gui_thanh_cong)
                so_thu_tu_bao_cao_vi_tri_du = stt_du_kien;

            moc_gui_tiep_theo_ms = bay_gio_ms +
                (gui_thanh_cong ? CHU_KY_BAO_CAO_VI_TRI_DU_MS : 250UL);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    // GPS NEO-6M doc lien tuc tren task rieng, khong block LoRa/audio.
    KhoiTao_GPS_DU();

    Reset_ThongKe_OLED_DU();


    // =================================================
    // OLED
    // =================================================

    KhoiTao_OLED();


    HienThi_DU_KhoiDong(
        "Kiem tra LoRa..."
    );


    // =================================================
    // LORA
    // =================================================

    KhoiTao_LoRa_RX();

    // Nut NACK + LED va task HMI nam hoan toan trong module rieng.
    KhoiTao_HMI_DU();

    // =================================================
    // SPEEX
    // =================================================

    KhoiTao_GiaiMa_Speex();


    HienThi_DU_ChoNhan();


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


        DU_Fatal_Reboot(
            "Khong tim thay PSRAM"
        );
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


        DU_Fatal_Reboot(
            "Cap phat PSRAM audio that bai"
        );
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


        DU_Fatal_Reboot(
            "Tao Queue that bai"
        );
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


        DU_Fatal_Reboot(
            "Tao Audio Buffer that bai"
        );
    }


    // =================================================
    // TASK PLAY REPORT
    // =================================================

    BaseType_t ok_play_report =
        xTaskCreatePinnedToCore(
            TacVu_BaoPlay,
            "PLAY_REPORT",
            4096,
            NULL,
            3,
            &Task_PlayReport_Handle,
            1
        );

    if (ok_play_report != pdPASS)
    {
        DU_Fatal_Reboot(
            "Tao task PLAY_REPORT that bai"
        );
    }


    // =================================================
    // TASK AUDIO
    // =================================================

    BaseType_t ok_audio =
        xTaskCreatePinnedToCore(
            TacVu_PhatAmThanh,
            "PWM_OUT",
            8192,
            NULL,
            3,
            &Task_Audio_Handle,
            0
        );

    if (ok_audio != pdPASS)
    {
        DU_Fatal_Reboot(
            "Tao task PWM_OUT that bai"
        );
    }


    // =================================================
    // TASK LORA RX
    // =================================================

    BaseType_t ok_lora =
        xTaskCreatePinnedToCore(
            TacVu_LoRaRX,
            "LoRa_RX",
            8192,
            NULL,
            4,
            NULL,
            1
        );

    if (ok_lora != pdPASS)
    {
        DU_Fatal_Reboot(
            "Tao task LoRa_RX that bai"
        );
    }


    // =================================================
    // TASK AES + SPEEX
    // =================================================

    BaseType_t ok_decode =
        xTaskCreatePinnedToCore(
            TacVu_GiaiMa,
            "Giai_Ma",
            10240,
            NULL,
            2,
            NULL,
            1
        );

    if (ok_decode != pdPASS)
    {
        DU_Fatal_Reboot(
            "Tao task Giai_Ma that bai"
        );
    }


    // Bao cao kenh SU->DU: uu tien thap, chi gui snapshot da nghe ke.
    BaseType_t ok_kenh =
        xTaskCreatePinnedToCore(
            TacVu_BaoCao_Kenh_SU_DU,
            "KENH_SU_DU",
            4096,
            NULL,
            1,
            NULL,
            0
        );

    if (ok_kenh != pdPASS)
    {
        DU_Fatal_Reboot(
            "Tao task KENH_SU_DU that bai"
        );
    }


    // GPS beacon khoi tao SAU CUNG.
    // Luc nay queue/audio/RX/decode/HMI deu da san sang; telemetry chi la best-effort.
    BaseType_t ok_gps =
        xTaskCreatePinnedToCore(
            TacVu_BaoCao_ViTri_DinhKy_DU,
            "GPS_DINH_KY",
            4096,
            NULL,
            1,
            NULL,
            0
        );

    if (ok_gps != pdPASS)
    {
        DU_Fatal_Reboot(
            "Tao task GPS_DINH_KY that bai"
        );
    }


    // Khởi tạo heartbeat sau khi toàn bộ task đã được tạo.
    // Mỗi task sẽ cập nhật ngay khi scheduler chạy.
    uint32_t hb_ban_dau =
        millis();

    du_hb_lora_ms =
        hb_ban_dau;

    du_hb_decode_ms =
        hb_ban_dau;

    du_hb_audio_task_ms =
        hb_ban_dau;


    BaseType_t ok_supervisor =
        xTaskCreatePinnedToCore(
            TacVu_DU_Supervisor,
            "DU_SUPERVISOR",
            4096,
            NULL,
            5,
            &Task_Supervisor_Handle,
            0
        );

    if (ok_supervisor != pdPASS)
    {
        DU_Fatal_Reboot(
            "Tao task DU_SUPERVISOR that bai"
        );
    }


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