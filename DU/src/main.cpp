#include <Arduino.h>
#include "node_config.h"
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
// Vung PSRAM nay nay dong vai tro jitter/ring buffer cho stream PCM.
//
// PCM hiện tại:
// 8 kHz * 16 bit = 16000 byte/giây.
//
// Dung luong lon giup chong burst/jitter, nhung audio task phat som
// va khong cho day ca cau.
// =====================================================

#define AUDIO_BUFFER_SIZE_PSRAM (6 * 1024 * 1024)

StaticRingbuffer_t *Audio_Buffer_Struct =
    nullptr;

uint8_t *Audio_Buffer_Storage =
    nullptr;


// STREAMING AUDIO V1:
// DU bat dau phat sau khi co mot luong PCM prefill nho, KHONG cho END_AUDIO.
// END_AUDIO chi danh dau "khong con PCM moi" de audio task dung dung luc.
volatile bool cho_phep_phat_audio = false;

#define DU_STREAMING_V1 1
#define DU_STREAMING_V11_EARLY_CONTIGUOUS 1
#define DU_STREAMING_V12_AUDIO_CHUNK 1

// V1.2: mo loa sau tron 1 VOICE packet = 8 frame = 160 ms.
// V1.1 target 120 ms mo gate khi decoder moi day 6/8 frame, trong khi
// double-buffer cu chi giu 2 x 20 ms. Log thuc te cho thay underflow
// du toan bo VOICE GCM/FEC deu tot. V1.2 can bang lai playout local.
#define DU_STREAM_PREFILL_FRAMES 10U

// V2C4.1: 300 ms audio duoc dua xuong theo burst moi superframe 300 ms.
// Neu bat loa ngay khi burst 15 frame vua den, jitter vai ms cua burst ke tiep
// se lam buffer cham 0 dung bien 300 ms -> MID_UNDERFLOW/mat chu.
// Giu them 60 ms pha playout ban dau de burst N+1 den khi van con backlog.
// Chi cong mot lan luc bat dau session, khong cong moi superframe.
#define DU_STREAM_PLAYOUT_GUARD_MS 60U

static volatile bool du_stream_end_received = false;
static volatile uint32_t du_stream_pcm_frames_total = 0;
static volatile uint32_t du_stream_pcm_frames_played = 0;
// V2C2.2 tach ro gap GIUA CAU (anh huong nghe) va cho END o DUOI CAU.
// du_stream_underflow_count chi dem MID-stream underflow that su.
static volatile uint32_t du_stream_underflow_count = 0;
static volatile uint32_t du_stream_tail_wait_count = 0;
static volatile bool du_stream_playout_guard_done = false;
static volatile bool du_stream_last_audio_decoded = false;

// PLAY_STARTED van duoc tao tai sample PWM dau tien, nhung khong TX ngay.
// DU giu report den khi END_AUDIO den de khong chen mot TX control vao
// cua so VOICE/FEC dang stream. rBS do do khong can doi protocol o buoc nay.
static volatile bool du_stream_play_report_pending = false;

// =====================================================
// V2C1-LATENCY-BASELINE V1
// Chi do delta tren cung DU. Khong thay playback/scheduler/Speex.
// =====================================================
static volatile uint32_t du_diag_session_start_ms = 0;
static volatile uint32_t du_diag_first_voice_ms = 0;
static volatile uint32_t du_diag_first_pcm_ms = 0;
static volatile int64_t du_diag_first_play_us = 0;
static volatile uint32_t du_diag_pcm_peak_abs = 0;
static volatile uint32_t du_diag_pwm_clip_count = 0;

static uint32_t DU_DeltaMs(uint32_t now_ms, uint32_t start_ms)
{
    return (start_ms == 0 || now_ms == 0) ? 0U : (uint32_t)(now_ms - start_ms);
}


// =====================================================
// LATENCY OPTIMIZATION V1.1
//
// 1) SESSION_READY duoc uu tien TX ngay khi DU nhan SESSION_START.
// 2) GPS_PHIEN khong chen vao cua so handshake/VOICE/FEC.
//    GPS_PHIEN van duoc GIU, nhung dua sang pending va gui khi radio ranh.
// 3) GPS dinh ky + report kenh SU-DU bi tam khoa trong luc DU dang
//    nhan mot session thoai.
//
// V2C4.7: KHONG duoc mo telemetry chi vi PTT dai qua 30 giay.
// Session hop le co the keo dai hang phut. Timeout phai dua tren THOI GIAN IM LANG,
// khong dua tren tong tuoi session; neu khong, DU co the tu chuyen sang TX telemetry
// giua mot cau noi dai va mat RX.
// =====================================================
static portMUX_TYPE DU_Latency_Mux =
    portMUX_INITIALIZER_UNLOCKED;

static volatile bool DU_Latency_SessionBusy = false;
static volatile uint32_t DU_Latency_SessionStartMs = 0;
static volatile uint32_t DU_Latency_SessionLastActivityMs = 0;

#define DU_LATENCY_SESSION_INACTIVITY_TIMEOUT_MS 5000UL

static bool DU_Latency_GPSPhienPending = false;
static uint64_t DU_Latency_GPSPhienId = 0;

// Sau session, cho mot beacon SU moi de tao report kenh POST.
static bool DU_Latency_PostChannelPending = false;

static void DU_Latency_BatDauSession(uint64_t session_id)
{
    const uint32_t now_ms = millis();

    portENTER_CRITICAL(&DU_Latency_Mux);

    DU_Latency_SessionBusy = true;
    DU_Latency_SessionStartMs = now_ms;
    DU_Latency_SessionLastActivityMs = now_ms;

    // GPS_PHIEN nay se duoc gui SAU session, khong gui trong handshake.
    DU_Latency_GPSPhienPending = true;
    DU_Latency_GPSPhienId = session_id;

    // Channel POST phai den tu beacon SU moi sau session.
    DU_Latency_PostChannelPending = true;

    portEXIT_CRITICAL(&DU_Latency_Mux);

    // Bo mau PRE con pending trong RAM, KHONG TX.
    Xoa_BaoCao_Kenh_SU_DU_DangCho();

    Serial.println(
        "[DU TELEMETRY] LOCK TRONG PHIEN | PRE GIU O rBS | POST PENDING"
    );
}

static void DU_Latency_DanhDauHoatDongSession()
{
    const uint32_t now_ms = millis();

    portENTER_CRITICAL(&DU_Latency_Mux);
    if (DU_Latency_SessionBusy)
        DU_Latency_SessionLastActivityMs = now_ms;
    portEXIT_CRITICAL(&DU_Latency_Mux);
}


static void DU_Latency_KetThucNhanSession()
{
    portENTER_CRITICAL(&DU_Latency_Mux);
    DU_Latency_SessionBusy = false;
    DU_Latency_SessionStartMs = 0;
    DU_Latency_SessionLastActivityMs = 0;
    portEXIT_CRITICAL(&DU_Latency_Mux);
}


static bool DU_Latency_DangNhanSession()
{
    bool busy = false;
    uint32_t last_activity_ms = 0;

    portENTER_CRITICAL(&DU_Latency_Mux);
    busy = DU_Latency_SessionBusy;
    last_activity_ms = DU_Latency_SessionLastActivityMs;
    portEXIT_CRITICAL(&DU_Latency_Mux);

    if (
        busy
        &&
        last_activity_ms != 0
        &&
        (uint32_t)(millis() - last_activity_ms)
            > DU_LATENCY_SESSION_INACTIVITY_TIMEOUT_MS
    )
    {
        const uint32_t idle_ms =
            (uint32_t)(millis() - last_activity_ms);

        DU_Latency_KetThucNhanSession();

        Serial.printf(
            "[DU V2C4.7 LATENCY FAILSAFE] SESSION IM LANG %u ms -> MO LAI TELEMETRY\n",
            (unsigned int)idle_ms
        );

        return false;
    }

    return busy;
}

static bool DU_Latency_LayGPSPhienPending(uint64_t &session_id)
{
    bool pending = false;

    portENTER_CRITICAL(&DU_Latency_Mux);

    pending = DU_Latency_GPSPhienPending;

    if (pending)
        session_id = DU_Latency_GPSPhienId;

    portEXIT_CRITICAL(&DU_Latency_Mux);

    return pending;
}

static void DU_Latency_XoaGPSPhienPending(uint64_t session_id)
{
    portENTER_CRITICAL(&DU_Latency_Mux);

    if (
        DU_Latency_GPSPhienPending
        &&
        DU_Latency_GPSPhienId == session_id
    )
    {
        DU_Latency_GPSPhienPending = false;
    }

    portEXIT_CRITICAL(&DU_Latency_Mux);
}

static bool DU_Latency_PostChannelDangCho()
{
    bool pending = false;

    portENTER_CRITICAL(&DU_Latency_Mux);
    pending = DU_Latency_PostChannelPending;
    portEXIT_CRITICAL(&DU_Latency_Mux);

    return pending;
}

static void DU_Latency_DanhDauPostChannelDaGui()
{
    portENTER_CRITICAL(&DU_Latency_Mux);
    DU_Latency_PostChannelPending = false;
    portEXIT_CRITICAL(&DU_Latency_Mux);
}

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

// V1.2 AUDIO CHUNK:
// Moi timer slot gom toi da 4 Speex/PCM frame = 80 ms thay vi 20 ms.
// Hai slot tao 160 ms scheduling cushion ma KHONG tang startup latency
// qua 1 VOICE packet (8 frame). Day chi la playout local, khong doi RF packet.
static constexpr uint16_t AUDIO_PCM_FRAME_SAMPLES = 160;
static constexpr uint8_t AUDIO_TIMER_CHUNK_FRAMES = 4;
static constexpr uint16_t AUDIO_TIMER_FRAME_SAMPLES =
    AUDIO_PCM_FRAME_SAMPLES * AUDIO_TIMER_CHUNK_FRAMES;
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

// V2C4.2: BUSY la mot state rieng, khong duoc suy ra tu end_received.
// V2C4.1 reset end_received=false sau khi phat xong nhung quen ha da_co_session,
// lam moi session sau do bi BUSY DROP vinh vien.
static volatile bool du_stream_session_busy = false;
static volatile uint32_t du_stream_session_last_activity_ms = 0;
#define DU_STREAM_STALE_SESSION_MS 3500U
#define DU_STREAM_STALE_POLL_MS    250U

// V2C4.8: neu mot SESSION moi hop le den ma session cu da im qua 1.2 s,
// khong duoc de mot co PLAY/backlog bi ket chan SESSION moi vo han.
// Decode task chi REQUEST abort; Audio task tu dung timer/loa va don buffer.
#define DU_STREAM_NEW_SESSION_PREEMPT_MS 1200U
static volatile bool du_stream_force_abort_requested = false;
static volatile uint64_t du_stream_force_abort_new_session = 0;


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
#define FLAG_CODEC_HQ       0x20
#define COUNT_SHIFT         5

#define SIZE_VOICE_PACKET   106
#define SIZE_FEC_PACKET     114
#define SIZE_MAX_PACKET     114
#define SIZE_SESSION_PACKET 12
#define SIZE_AUDIO_END_PACKET 4

#define VOICE_LENGTH         98
#define VOICE_PAYLOAD_BYTES    90
#define VOICE_GCM_TAG_BYTES     8
#define VOICE_PROTECTED_BYTES  98
#define SPEEX_HQ_BYTES          10
#define SPEEX_LQ_BYTES           6
#define MAX_FRAME_PACKET        15
#define FEC_DATA_PER_GROUP     8

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
// STREAM V1.1 tach 2 vai tro:
//   - FEC buffer van giu ciphertext/tag + plaintext toi da 4 packet de recover.
//   - Audio path decode SOM cac VOICE da GCM PASS theo prefix lien tuc.
//
// Neu gap packet:
//   - khong nhay qua gap (giu dung state Speex);
//   - parity den: recover 1 packet roi decode tiep;
//   - >1 packet mat: Flush dung Speex PLC cho phan khong cuu duoc.
// =====================================================

struct FECGroupState
{
    bool active;
    uint32_t start_seq;

    bool present[FEC_DATA_PER_GROUP];

    // Ciphertext80 + original VOICE tag8.
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

    bool codec_hq[
        FEC_DATA_PER_GROUP
    ];

    uint8_t expected_count;

    bool has_last;
    uint8_t final_frames;

    // Dung de ghi lai so packet mat TRUOC khi FEC recovery
    // thay doi present[].
    bool fec_received;
    uint8_t raw_missing_before_recovery;

    // STREAM V1.1: cac slot [0 .. next_decode_slot-1] da duoc
    // decode theo dung thu tu Speex va day vao PCM ring buffer.
    // Slot hop le den som duoc phat som, nhung neu co mot "lo hong"
    // thi cac slot sau no van cho FEC/PLC de giu dung state decoder.
    uint8_t next_decode_slot;
};


static FECGroupState fec_group;

// Neu mot group da nhan DU 4 DATA hop le, DU decode som truoc khi parity den.
// Ghi lai GROUP_START de bo parity den muon, tranh decode/PLC trung lap.
static bool du_fec_last_flushed_valid = false;
static uint32_t du_fec_last_flushed_start = 0;


static void Reset_FEC_Group()
{
    memset(
        &fec_group,
        0,
        sizeof(fec_group)
    );
}


// V2C4.6 DU FAILSAFE:
// V2C4.2 chi stale-release khi MOT SESSION_START MOI den. Neu END_AUDIO bi mat
// va khong co session moi, HMI co the giu led_session_active -> LED do sang dung
// vo han. Service nay chay dinh ky ngay ca khi RX queue im lang.
static bool DU_V2C46_AudioBusyNow()
{
    return
        Audio_Timer_Playing
        || cho_phep_phat_audio
        || (du_stream_pcm_frames_total > du_stream_pcm_frames_played);
}


static void DU_V2C48_RequestAudioAbort(const char *reason, uint64_t new_session)
{
    if (!du_stream_force_abort_requested)
    {
        Serial.printf(
            "[DU V2C4.8 ABORT REQUEST] REASON=%s | ACTIVE=%016llX | NEW=%016llX | AGE=%u ms | PLAY=%u\n",
            reason != nullptr ? reason : "UNKNOWN",
            (unsigned long long)session_id_hien_tai,
            (unsigned long long)new_session,
            (unsigned int)(millis() - du_stream_session_last_activity_ms),
            DU_V2C46_AudioBusyNow() ? 1U : 0U
        );
    }

    du_stream_force_abort_new_session = new_session;
    du_stream_force_abort_requested = true;
}


static void DU_V2C48_DrainAudioRingBuffer()
{
    if (Audio_Buffer == nullptr)
        return;

    while (1)
    {
        size_t item_size = 0;
        void *item = xRingbufferReceive(Audio_Buffer, &item_size, 0);
        if (item == nullptr)
            break;
        vRingbufferReturnItem(Audio_Buffer, item);
    }
}


static void DU_V2C48_ClearStaleStreamState(const char *reason)
{
    const uint64_t old_session = session_id_hien_tai;
    const uint64_t requested_new = du_stream_force_abort_new_session;

    DU_Jam53_Stop("FORCE_RELEASE");
    DU_V2C48_DrainAudioRingBuffer();

    du_stream_session_busy = false;
    da_co_session = false;
    session_id_hien_tai = 0;
    du_stream_session_last_activity_ms = millis();
    du_stream_end_received = false;
    cho_phep_phat_audio = false;
    du_stream_pcm_frames_total = 0;
    du_stream_pcm_frames_played = 0;
    du_stream_underflow_count = 0;
    du_stream_tail_wait_count = 0;
    du_stream_playout_guard_done = false;
    du_stream_last_audio_decoded = false;
    du_stream_play_report_pending = false;
    da_co_seq = false;
    seq_cuoi_da_xu_ly = 0;
    Reset_FEC_Group();

    HMI_DU_Bao_END_Audio();

    du_stream_force_abort_requested = false;
    du_stream_force_abort_new_session = 0;

    Serial.printf(
        "[DU V2C4.8 FORCE RELEASE] REASON=%s | OLD=%016llX | NEW_WAITING=%016llX | LED_OFF/RX_READY\n",
        reason != nullptr ? reason : "UNKNOWN",
        (unsigned long long)old_session,
        (unsigned long long)requested_new
    );
}


static bool DU_V2C46_ReleaseStaleSessionIfSafe(const char *reason)
{
#if !DU_STREAMING_V1
    (void)reason;
    return false;
#else
    if (!du_stream_session_busy)
        return false;

    const uint32_t age = millis() - du_stream_session_last_activity_ms;
    if (age <= DU_STREAM_STALE_SESSION_MS)
        return false;

    const bool audio_busy_now = DU_V2C46_AudioBusyNow();
    if (audio_busy_now)
    {
        // V2C4.8: day la truong hop log thuc te PLAY=1 hang chuc giay.
        // Khong release truc tiep tu decode task; yeu cau audio task tu don sach.
        DU_V2C48_RequestAudioAbort(reason, 0);
        return false;
    }

    const uint64_t stale_session = session_id_hien_tai;

    Serial.printf(
        "[DU V2C4.6 FAILSAFE RELEASE] REASON=%s | SESSION=%016llX | AGE=%u ms | AUDIO_BUSY=0 -> LED_OFF/RX_READY\n",
        reason != nullptr ? reason : "UNKNOWN",
        (unsigned long long)stale_session,
        (unsigned int)age
    );

    DU_Jam53_Stop("STALE_RELEASE");
    du_stream_session_busy = false;
    da_co_session = false;
    session_id_hien_tai = 0;
    du_stream_session_last_activity_ms = millis();
    du_stream_end_received = false;
    cho_phep_phat_audio = false;
    du_stream_pcm_frames_total = 0;
    du_stream_pcm_frames_played = 0;
    du_stream_tail_wait_count = 0;
    du_stream_last_audio_decoded = false;
    du_stream_play_report_pending = false;
    da_co_seq = false;
    seq_cuoi_da_xu_ly = 0;
    Reset_FEC_Group();

    // Quan trong: ha state LED/HMI session. Ham nay khong tao ACK; chi ket thuc
    // trang thai "dang nhan audio" bi treo do mat END_AUDIO/control.
    HMI_DU_Bao_END_Audio();

    return true;
#endif
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
        return;
    }

#if DU_STREAMING_V1
    uint32_t produced = ++du_stream_pcm_frames_total;

    if (du_diag_first_pcm_ms == 0)
    {
        du_diag_first_pcm_ms = millis();
        Serial.printf(
            "[DU LAT] FIRST_PCM_READY | T=%u ms | FROM_SESSION=%u ms | PCM_FRAME=%u\n",
            (unsigned int)du_diag_first_pcm_ms,
            (unsigned int)DU_DeltaMs(du_diag_first_pcm_ms, du_diag_session_start_ms),
            (unsigned int)produced
        );
    }

    // Chi mo gate mot lan khi da co prefill toi thieu. STREAM V1.1
    // decode packet tot ngay khi den; packet thuong co 8 frame = 160 ms,
    // nen target 120 ms se mo sau packet VOICE dau tien hop le.
    if (
        !cho_phep_phat_audio
        && produced >= DU_STREAM_PREFILL_FRAMES
    )
    {
        cho_phep_phat_audio = true;

        Serial.printf(
            "[DU STREAM] MO PLAY GATE SOM | PREFILL=%u frames (%u ms)\n",
            (unsigned int)produced,
            (unsigned int)(produced * 20U)
        );
    }
#endif
}


// =====================================================
// DECODE 1 DATA PACKET HOẶC PLC CHO PACKET MẤT
// =====================================================

static void Decode_Data_Slot(uint8_t slot, uint8_t so_frame)
{
    int16_t pcm[160];
    const bool codec_hq = fec_group.codec_hq[slot];
    const uint8_t frame_bytes = codec_hq ? SPEEX_HQ_BYTES : SPEEX_LQ_BYTES;
    const uint8_t max_frames = codec_hq ? 8U : 15U;
    if (so_frame < 1U || so_frame > max_frames) so_frame = max_frames;

    if (fec_group.present[slot])
    {
        for (uint8_t f=0; f<so_frame; ++f)
        {
            GiaiMa_KhungThoai_Adaptive(
                &fec_group.payload[slot][f * frame_bytes],
                frame_bytes,
                (uint8_t*)pcm
            );
            Day_PCM_Vao_Buffer(pcm);
        }
    }
    else
    {
        for (uint8_t f=0; f<so_frame; ++f)
        {
            GiaiMa_KhungMat((uint8_t*)pcm);
            Day_PCM_Vao_Buffer(pcm);
        }
    }
}


// =====================================================
// STREAM V1.1 - DECODE SOM PHAN LIEN TUC DA XAC THUC
//
// Chi decode cac slot lien tuc tinh tu next_decode_slot.
// Vi Speex decoder co state, TUYET DOI khong nhay qua mot slot bi mat.
// Neu slot i mat ma i+1 da den, i+1 se duoc giu trong FEC buffer;
// khi parity den, FEC recover i roi Flush_FEC_Group() decode tiep.
// =====================================================

static void Decode_Contiguous_Ready_FEC()
{
#if DU_STREAMING_V11_EARLY_CONTIGUOUS
    if (!fec_group.active)
    {
        return;
    }

    uint8_t limit = FEC_DATA_PER_GROUP;
    if (
        fec_group.has_last
        && fec_group.expected_count >= 1
        && fec_group.expected_count <= FEC_DATA_PER_GROUP
    )
    {
        limit = fec_group.expected_count;
    }

    while (fec_group.next_decode_slot < limit)
    {
        uint8_t slot = fec_group.next_decode_slot;

        // Gap: dung lai de cho parity/FEC. Khong decode slot sau gap
        // vi lam nhu vay se pha state cua Speex decoder.
        if (!fec_group.present[slot])
        {
            break;
        }

        uint8_t frames = fec_group.frame_count[slot];
        if (frames < 1 || frames > MAX_FRAME_PACKET)
        {
            frames = MAX_FRAME_PACKET;
        }

        Decode_Data_Slot(slot, frames);
        fec_group.next_decode_slot++;

        Serial.printf(
            "[DU STREAM V1.1] EARLY DECODE | GROUP=%u | SLOT=%u | FRAME=%u | NEXT=%u\n",
            (unsigned int)fec_group.start_seq,
            (unsigned int)slot,
            (unsigned int)frames,
            (unsigned int)fec_group.next_decode_slot
        );
    }

    // Neu day la final group va prefix lien tuc da decode het den LAST_AUDIO,
    // danh dau tail NGAY tai fast path. Khong can doi parity FEC den sau.
    // Nhu vay luc PCM cuoi vua phat het ma END_AUDIO control chua toi, DU se
    // ghi TAIL_WAIT thay vi dem nham thanh MID_UNDERFLOW.
    if (
        fec_group.has_last
        && fec_group.next_decode_slot >= limit
    )
    {
        du_stream_last_audio_decoded = true;
    }
#endif
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

    // STREAM V1.1: cac slot lien tuc tot da duoc decode ngay khi den.
    // Flush chi xu ly phan CON LAI: packet recovered, packet sau gap,
    // hoac PLC cho packet that su khong cuu duoc.
    uint8_t decode_start = fec_group.next_decode_slot;
    if (decode_start > count)
    {
        decode_start = count;
    }

    bool group_hq_hint = false;
    for (uint8_t h = 0; h < count; ++h)
    {
        if (fec_group.present[h] && fec_group.codec_hq[h]) { group_hq_hint = true; break; }
    }

    for (
        uint8_t i = decode_start;
        i < count;
        i++
    )
    {
        if (!fec_group.present[i]) fec_group.codec_hq[i] = group_hq_hint;
        uint8_t frames = group_hq_hint ? 8U : MAX_FRAME_PACKET;

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

        fec_group.next_decode_slot = i + 1U;
    }

    // Chi khi final group da duoc decode/PLC het moi danh dau tail. Neu RingBuffer
    // rong sau moc nay nhung END_AUDIO control chua toi thi do la TAIL_WAIT,
    // khong phai jitter underflow giua cau.
    if (fec_group.has_last)
    {
        du_stream_last_audio_decoded = true;
    }

    du_fec_last_flushed_start = fec_group.start_seq;
    du_fec_last_flushed_valid = true;

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
                        goi_tin[3];

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

                // V2C4.7: moi DATA hop le tren song gia han khoa telemetry.
                // PTT co the dai >30s, vi vay khong duoc timeout theo tong tuoi session.
                DU_Latency_DanhDauHoatDongSession();
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
                pdMS_TO_TICKS(DU_STREAM_STALE_POLL_MS)
            )
            != pdTRUE
        )
        {
            DU_Heartbeat_Decode();
            (void)DU_V2C46_ReleaseStaleSessionIfSafe("RX_IDLE_TIMEOUT");
            continue;
        }

        // Ca khi van co packet telemetry/control, service stale session de LED
        // khong bi giu sang chi vi END_AUDIO cua session cu bi mat.
        (void)DU_V2C46_ReleaseStaleSessionIfSafe("RX_ACTIVITY");

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
            DU_Jam53_Stop("END_AUDIO");
#if DU_STREAMING_V1
            if (!du_stream_end_received)
            {
                // Final parity co the bi mat. Flush group con lai bang data/PLC
                // TRUOC khi danh dau end; sau dong nay khong con PCM moi.
                Flush_FEC_Group(
                    "END_AUDIO"
                );

                du_stream_end_received = true;

                Serial.printf(
                    "[DU STREAM] END_AUDIO | PCM_TOTAL=%u | PLAYED=%u | BACKLOG=%u | MID_UNDERFLOW=%u | TAIL_WAIT=%u\n",
                    (unsigned int)du_stream_pcm_frames_total,
                    (unsigned int)du_stream_pcm_frames_played,
                    (unsigned int)(
                        du_stream_pcm_frames_total >= du_stream_pcm_frames_played
                            ? (du_stream_pcm_frames_total - du_stream_pcm_frames_played)
                            : 0U
                    ),
                    (unsigned int)du_stream_underflow_count,
                    (unsigned int)du_stream_tail_wait_count
                );

                uint32_t first_play_ms =
                    du_diag_first_play_us > 0 ? (uint32_t)(du_diag_first_play_us / 1000LL) : 0U;
                Serial.printf(
                    "[DU LAT SUMMARY] SESSION_TO_FIRST_VOICE=%u ms | SESSION_TO_FIRST_PCM=%u ms | SESSION_TO_FIRST_PLAY=%u ms | MID_UNDERFLOW=%u | TAIL_WAIT=%u\n",
                    (unsigned int)DU_DeltaMs(du_diag_first_voice_ms, du_diag_session_start_ms),
                    (unsigned int)DU_DeltaMs(du_diag_first_pcm_ms, du_diag_session_start_ms),
                    (unsigned int)DU_DeltaMs(first_play_ms, du_diag_session_start_ms),
                    (unsigned int)du_stream_underflow_count,
                    (unsigned int)du_stream_tail_wait_count
                );
                Serial.printf(
                    "[DU AUDIO SUMMARY] PROFILE=ADAPTIVE | PCM_PEAK=%u | PWM_CLIP=%u | ENH=OFF | POST_PRESENCE=OFF | PCM_LIMIT=15000 | SAMPLE_RATE=%u | PWM=%uHz/8bit\n",
                    (unsigned int)du_diag_pcm_peak_abs,
                    (unsigned int)du_diag_pwm_clip_count,
                    (unsigned int)AUDIO_SAMPLE_RATE,
                    (unsigned int)PWM_CARRIER_FREQ
                );

                DU_Latency_KetThucNhanSession();
                HMI_DU_Bao_END_Audio();

                // Cau rat ngan co the chua dat prefill. END cho phep bat play
                // voi bat ky PCM nao da decode duoc.
                if (du_stream_pcm_frames_total > 0)
                {
                    cho_phep_phat_audio = true;
                }

                // Neu loa da phat sample dau tien tu truoc, bay gio moi TX
                // PLAY_STARTED de rBS nhan trong cua so sau END #1.
                if (
                    du_stream_play_report_pending
                    && Task_PlayReport_Handle != nullptr
                )
                {
                    xTaskNotifyGive(Task_PlayReport_Handle);
                }

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
                    "[DU STREAM] END_AUDIO LAP LAI -> BO QUA"
                );
            }
#else
            if (!cho_phep_phat_audio)
            {
                Flush_FEC_Group(
                    "END_AUDIO"
                );

                DU_Latency_KetThucNhanSession();
                HMI_DU_Bao_END_Audio();
                cho_phep_phat_audio = true;

                HienThi_DU_KetQua(
                    du_oled_stats.expected_voice,
                    du_oled_stats.raw_missing_voice,
                    du_oled_stats.fec_recovered_voice,
                    du_oled_stats.final_missing_voice
                );
            }
#endif

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

            if (goi_tin[3] != 8U)
            {
                Serial.printf(
                    "[DU DROP] SESSION_START META sai | META=0x%02X\n",
                    (unsigned int)goi_tin[3]
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
                du_stream_session_last_activity_ms = millis();

                DU_Latency_BatDauSession(session_moi);

                Serial.printf(
                    "[DU] SESSION LAP LAI = %016llX -> GUI NGAY SESSION_READY\n",
                    (unsigned long long)session_moi
                );

                // V2C5.2: session da biet -> PREPARED; beacon moi cap ARMED.
                DU_Jam53_Prepare(session_moi);

                // Latency V1.1: READY la control gate cua voice, nen uu tien.
                // GPS_PHIEN da duoc dua vao pending trong DU_Latency_BatDauSession().
                Gui_SESSION_READY_RBS(session_moi);
                continue;
            }

#if DU_STREAMING_V1
            // V2C4.2: BUSY ton tai den khi AUDIO TASK phat xong that su.
            // Co stale timeout de tu phuc hoi neu rBS/reset lam mat END_AUDIO.
            if (du_stream_session_busy && session_moi != session_id_hien_tai)
            {
                const uint32_t busy_age = millis() - du_stream_session_last_activity_ms;

                // V2C4.2A: TacVu_GiaiMa khong duoc tham chieu bien cuc bo
                // dang_phat_loa cua TacVu_PhatAmThanh. Dung cac state global
                // de biet audio cu con dang phat/cho phat/con backlog hay khong.
                const bool audio_busy_now =
                    Audio_Timer_Playing
                    || cho_phep_phat_audio
                    || (du_stream_pcm_frames_total > du_stream_pcm_frames_played);

                if (busy_age > DU_STREAM_STALE_SESSION_MS && !audio_busy_now)
                {
                    Serial.printf(
                        "[DU V2C4.8 STALE NEW SESSION] ACTIVE=%016llX | AGE=%u ms -> ACCEPT NEW=%016llX\n",
                        (unsigned long long)session_id_hien_tai,
                        (unsigned int)busy_age,
                        (unsigned long long)session_moi
                    );
                    (void)DU_V2C46_ReleaseStaleSessionIfSafe("NEW_SESSION");
                }
                else if (
                    busy_age > DU_STREAM_NEW_SESSION_PREEMPT_MS
                    && audio_busy_now
                )
                {
                    // Session moi la bang chung manh rang session cu da het quyen.
                    // Request audio task abort stream cu; bo request nay de tranh
                    // doi state giua decode/play. rBS se retry SESSION_START.
                    DU_V2C48_RequestAudioAbort("NEW_SESSION_PREEMPT", session_moi);
                    Serial.printf(
                        "[DU V2C4.8 BUSY PREEMPT] NEW=%016llX | ACTIVE=%016llX | AGE=%u ms | PLAY=1 -> ABORT_OLD, WAIT_RETRY\n",
                        (unsigned long long)session_moi,
                        (unsigned long long)session_id_hien_tai,
                        (unsigned int)busy_age
                    );
                    continue;
                }
                else
                {
                    Serial.printf(
                        "[DU V2C4.8 BUSY DROP] NEW_SESSION=%016llX | ACTIVE=%016llX | AGE=%u ms | PLAY=%u | ABORT_REQ=%u\n",
                        (unsigned long long)session_moi,
                        (unsigned long long)session_id_hien_tai,
                        (unsigned int)busy_age,
                        audio_busy_now ? 1U : 0U,
                        du_stream_force_abort_requested ? 1U : 0U
                    );
                    continue;
                }
            }
#endif

            // V2C2.1 FAST_SETUP: khoa telemetry NGAY khi da chap nhan SESSION moi.
            // Muc tieu: khong de GPS/channel report chen truoc SESSION_READY.
            HMI_DU_TamDung_Beacon(1500UL);
            DU_Latency_BatDauSession(session_moi);

            // Nếu vì lỗi control session cũ còn group,
            // flush bằng PLC trước khi reset.
            Flush_FEC_Group(
                "NEW_SESSION"
            );

            Serial.println(
                "[DU CRYPTO] FIXED AES-128-GCM | SESSION KEY THEO SESSION_ID"
            );

            session_id_hien_tai =
                session_moi;

            DU_Jam53_Prepare(session_id_hien_tai);

            da_co_session =
                true;
            du_stream_session_busy = true;
            du_stream_session_last_activity_ms = millis();

            HMI_DU_Reset_Cho_Session_Moi();

            cho_phep_phat_audio =
                false;

#if DU_STREAMING_V1
            du_stream_end_received = false;
            du_stream_pcm_frames_total = 0;
            du_stream_pcm_frames_played = 0;
            du_stream_underflow_count = 0;
            du_stream_tail_wait_count = 0;
            du_stream_playout_guard_done = false;
            du_stream_last_audio_decoded = false;
            du_stream_play_report_pending = false;
            du_diag_session_start_ms = millis();
            du_diag_first_voice_ms = 0;
            du_diag_first_pcm_ms = 0;
            du_diag_first_play_us = 0;
            du_diag_pcm_peak_abs = 0;
            du_diag_pwm_clip_count = 0;
            GiaiMa_ResetAudioProfile();
            Serial.printf(
                "[DU LAT] SESSION_START | T=%u ms | PAIR=%u | SESSION=%016llX\n",
                (unsigned int)du_diag_session_start_ms,
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned long long)session_id_hien_tai
            );
#endif

            da_co_seq =
                false;

            seq_cuoi_da_xu_ly =
                0;

            Reset_FEC_Group();
            du_fec_last_flushed_valid = false;
            du_fec_last_flushed_start = 0;
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

            // V2C2.1 FAST_SETUP: telemetry da bi khoa o NGAY dau nhanh SESSION moi.
            // SESSION_READY la control gate cho voice -> gui NGAY.
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
            // encrypted parity = 98B
            // FEC tag          = 8B tai byte96..103
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
                    &goi_tin[8 + VOICE_PROTECTED_BYTES],
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
                du_fec_last_flushed_valid
                && group_start == du_fec_last_flushed_start
            )
            {
                Serial.printf(
                    "[DU FEC] PARITY DEN SAU KHI GROUP DA FLUSH | GROUP=%u -> BO QUA\n",
                    (unsigned int)group_start
                );
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
                // 1) KHOI PHUC 88B:
                //    ciphertext80 + original VOICE tag8
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
                    (recovered_last ? FLAG_LAST_AUDIO : 0x00);

                // V2C0: frame_count 1..15 nam truc tiep o byte3.
                recovered_header[3] =
                    recovered_frames;

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
                // recovered_protected[0..79] = ciphertext
                // recovered_protected[80..87] = original tag
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

                    fec_group.frame_count[missing_slot] = recovered_frames;
                    fec_group.codec_hq[missing_slot] = false; // FEC chi duoc phat trong DUAL/LQ

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
            goi_tin[3];

        bool last_audio = (goi_tin[2] & FLAG_LAST_AUDIO) != 0;
        bool codec_hq = (goi_tin[2] & FLAG_CODEC_HQ) != 0;
        const uint8_t max_frame_profile = codec_hq ? 8U : 15U;

        if (
            so_frame < 1
            ||
            so_frame > max_frame_profile
        )
        {
            Serial.println(
                "[DU DROP] FRAME_COUNT sai!"
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
                &goi_tin[8 + VOICE_PAYLOAD_BYTES],
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

        du_stream_session_last_activity_ms = millis();

        if (du_diag_first_voice_ms == 0)
        {
            du_diag_first_voice_ms = millis();
            Serial.printf(
                "[DU LAT] FIRST_VOICE_AUTH_OK | T=%u ms | FROM_SESSION=%u ms | SEQ=%u | FC=%u\n",
                (unsigned int)du_diag_first_voice_ms,
                (unsigned int)DU_DeltaMs(du_diag_first_voice_ms, du_diag_session_start_ms),
                (unsigned int)seq,
                (unsigned int)so_frame
            );
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
        // ciphertext80 + original GCM tag8.
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

        fec_group.frame_count[slot] = so_frame;
        fec_group.codec_hq[slot] = codec_hq;

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
            "[DU VOICE GCM OK] SEQ=%u | SLOT=%u | PROFILE=%s | FC=%u\n",
            seq,
            slot,
            codec_hq ? "HQ" : "LQ",
            (unsigned int)so_frame
        );

#if DU_STREAMING_V11_EARLY_CONTIGUOUS
        // V1.1: packet tot khong con bi nhot den khi du ca FEC group.
        // Chi decode prefix lien tuc de van giu dung thu tu/state Speex.
        Decode_Contiguous_Ready_FEC();
#endif

#if DU_STREAMING_V1
        // Fast path: neu ca 4 DATA cua group deu da nhan/GCM PASS thi khong
        // can cho parity. Decode som de giam ~1 chu ky FEC latency. Parity
        // van duoc SU phat de cuu truong hop co mat packet; neu den sau group
        // da complete thi stale-guard o tren se bo qua no.
        if (slot == (FEC_DATA_PER_GROUP - 1U))
        {
            bool group_du_data = true;
            for (uint8_t i = 0; i < FEC_DATA_PER_GROUP; ++i)
            {
                if (!fec_group.present[i])
                {
                    group_du_data = false;
                    break;
                }
            }

            if (group_du_data)
            {
                fec_group.expected_count = FEC_DATA_PER_GROUP;
                Flush_FEC_Group("ALL_DATA_EARLY");
            }
        }
#endif
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

#if DU_STREAMING_V1
            du_stream_play_report_pending = false;
#endif
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
    uint8_t pcm_frames_vua_xong = 0;

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

                pcm_frames_vua_xong =
                    (uint8_t)(
                        (count + AUDIO_PCM_FRAME_SAMPLES - 1U)
                        / AUDIO_PCM_FRAME_SAMPLES
                    );

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

        int32_t sample_abs = sample >= 0 ? (int32_t)sample : -(int32_t)sample;
        if ((uint32_t)sample_abs > du_diag_pcm_peak_abs)
            du_diag_pcm_peak_abs = (uint32_t)sample_abs;

        // V2C3 AUDIO2: 10-bit PWM de giam luong tu hoa/re.  78.125 kHz =
        // 80MHz/1024 tren LEDC, van cao hon rat xa bang thoai 0..4kHz.
        int pwm_val = 128 + (sample / 128);

        if (pwm_val < 0)
        {
            pwm_val = 0;
            du_diag_pwm_clip_count++;
        }

        if (pwm_val > 255)
        {
            pwm_val = 255;
            du_diag_pwm_clip_count++;
        }

        ledcWrite(
            PWM_CHANNEL,
            pwm_val
        );
    }


    // Moc PLAY that: sample dau tien da duoc day vao PWM.
    if (la_mau_dau)
    {
        if (du_diag_first_play_us == 0)
            du_diag_first_play_us = esp_timer_get_time();

        play_report_session_id =
            session_id_hien_tai;

        du_last_played_session_id =
            session_id_hien_tai;

#if DU_STREAMING_V1
        du_stream_play_report_pending = true;

        // Khong chen TX LoRa vao stream. Neu END da den (cau rat ngan),
        // co the bao ngay; neu chua, END handler se notify sau.
        if (
            du_stream_end_received
            && Task_PlayReport_Handle != nullptr
        )
        {
            xTaskNotifyGive(Task_PlayReport_Handle);
        }
#else
        if (Task_PlayReport_Handle != nullptr)
        {
            xTaskNotifyGive(
                Task_PlayReport_Handle
            );
        }
#endif
    }


    // Bao audio task slot nao vua duoc phat xong.
    // Callback esp_timer la task context nen dung xTaskNotify binh thuong.
    if (frame_vua_xong)
    {
#if DU_STREAMING_V1
        du_stream_pcm_frames_played +=
            pcm_frames_vua_xong;
#endif
    }

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
// COPY 1 PCM CHUNK TU RINGBUFFER -> TIMER SLOT
// =====================================================

static bool Nap_Audio_Timer_Slot(
    uint8_t slot,
    TickType_t timeout_ticks)
{
    if (slot > 1)
    {
        return false;
    }

    uint16_t tong_so_mau = 0;
    uint8_t so_pcm_frame = 0;

    // V1.2: lay toi da 4 item PCM 20 ms lien tiep vao 1 timer slot 80 ms.
    // Item dau duoc phep block theo timeout_ticks; cac item sau chi lay neu
    // da co san, de audio task khong bao gio bi giu 80 ms cho du chunk.
    for (uint8_t f = 0; f < AUDIO_TIMER_CHUNK_FRAMES; ++f)
    {
        size_t kich_thuoc = 0;

        TickType_t wait_ticks =
            (f == 0) ? timeout_ticks : 0;

        uint8_t *pcm_data =
            (uint8_t *)
            xRingbufferReceive(
                Audio_Buffer,
                &kich_thuoc,
                wait_ticks
            );

        if (pcm_data == nullptr)
        {
            break;
        }

        size_t so_mau =
            kich_thuoc / sizeof(int16_t);

        if (so_mau > AUDIO_PCM_FRAME_SAMPLES)
        {
            so_mau = AUDIO_PCM_FRAME_SAMPLES;
        }

        if (
            (uint32_t)tong_so_mau + (uint32_t)so_mau
            > AUDIO_TIMER_FRAME_SAMPLES
        )
        {
            so_mau =
                AUDIO_TIMER_FRAME_SAMPLES - tong_so_mau;
        }

        memcpy(
            &Audio_Timer_Frame[slot][tong_so_mau],
            pcm_data,
            so_mau * sizeof(int16_t)
        );

        tong_so_mau += (uint16_t)so_mau;
        so_pcm_frame++;

        vRingbufferReturnItem(
            Audio_Buffer,
            (void *)pcm_data
        );

        if (tong_so_mau >= AUDIO_TIMER_FRAME_SAMPLES)
        {
            break;
        }
    }

    if (tong_so_mau == 0)
    {
        return false;
    }

    if (tong_so_mau < AUDIO_TIMER_FRAME_SAMPLES)
    {
        memset(
            &Audio_Timer_Frame[slot][tong_so_mau],
            0,
            (AUDIO_TIMER_FRAME_SAMPLES - tong_so_mau)
                * sizeof(int16_t)
        );
    }

    // Publish metadata sau khi data da copy xong.
    portENTER_CRITICAL(&Audio_Timer_Mux);

    Audio_Timer_Count[slot] =
        tong_so_mau;

    Audio_Timer_Ready[slot] =
        true;

    portEXIT_CRITICAL(&Audio_Timer_Mux);

#if DU_STREAMING_V12_AUDIO_CHUNK
    // Khong log moi lan nap slot de tranh Serial tao jitter audio.
    (void)so_pcm_frame;
#endif

    return true;
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


static bool Audio_Timer_Slot_Free(uint8_t slot)
{
    if (slot > 1)
    {
        return false;
    }

    bool free_slot = false;

    portENTER_CRITICAL(&Audio_Timer_Mux);
    free_slot = !Audio_Timer_Ready[slot];
    portEXIT_CRITICAL(&Audio_Timer_Mux);

    return free_slot;
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

#if DU_STREAMING_V1
        // V2C4.8: neu decode task phat hien stale PLAY=1, audio task la noi
        // duy nhat duoc dung timer/loa va don PCM de tranh race.
        if (du_stream_force_abort_requested)
        {
            if (Audio_Sample_Timer != nullptr)
            {
                (void)esp_timer_stop(Audio_Sample_Timer);
            }

            portENTER_CRITICAL(&Audio_Timer_Mux);
            Audio_Timer_Ready[0] = false;
            Audio_Timer_Ready[1] = false;
            Audio_Timer_Count[0] = 0;
            Audio_Timer_Count[1] = 0;
            Audio_Timer_Active_Slot = 0;
            Audio_Timer_Sample_Index = 0;
            Audio_Timer_Playing = false;
            Audio_Timer_First_Sample = true;
            portEXIT_CRITICAL(&Audio_Timer_Mux);

            if (dang_phat_loa)
            {
                ledcWrite(PWM_CHANNEL, 128);
                ledcDetachPin(CHAN_AUDIO_OUT);
                pinMode(CHAN_AUDIO_OUT, INPUT);
                dang_phat_loa = false;
            }

            DU_V2C48_ClearStaleStreamState("AUDIO_TASK_IDLE_ABORT");
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
#endif

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

        // V2C4.1 JITTER GUARD:
        // Scheduler tao trung binh dung 300 ms audio / 300 ms superframe.
        // Neu bat timer dung bien burst, chi can downlink tre vai ms la buffer rong.
        // Cho mot lan 60 ms truoc first-play tao phase cushion, trong khi decoder
        // van tiep tuc nap RingBuffer. Khong lap lai sau underflow/slot refill.
        if (!du_stream_playout_guard_done)
        {
            Serial.printf(
                "[DU STREAM JITTER GUARD] WAIT=%u ms | PRODUCED=%u | PLAYED=%u\n",
                (unsigned int)DU_STREAM_PLAYOUT_GUARD_MS,
                (unsigned int)du_stream_pcm_frames_total,
                (unsigned int)du_stream_pcm_frames_played
            );

            vTaskDelay(pdMS_TO_TICKS(DU_STREAM_PLAYOUT_GUARD_MS));
            du_stream_playout_guard_done = true;
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


        // STREAM V1 mo gate khi da co prefill nho; slot dau tien phai
        // co san neu cau co audio.
        bool co_slot_0 =
            Nap_Audio_Timer_Slot(
                0,
                pdMS_TO_TICKS(100)
            );

        if (!co_slot_0)
        {
#if DU_STREAMING_V1
            if (du_stream_end_received)
            {
                cho_phep_phat_audio = false;
                Serial.println(
                    "[DU STREAM] END NHUNG KHONG CO PCM DE PHAT"
                );
            }
            else
            {
                Serial.println(
                    "[DU STREAM] PLAY GATE MO NHUNG CHUA LAY DUOC PCM -> THU LAI"
                );
            }
#else
            cho_phep_phat_audio = false;
            Serial.println(
                "[DU AUDIO] BUFFER RONG SAU END_AUDIO"
            );
#endif
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }


        // V1.2: slot 0 va slot 1 moi slot co the gom toi da 80 ms.
        // Neu packet dau 8 frame da decode xong, double buffer se nap du 160 ms
        // ngay truoc khi bat timer.
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


        Serial.printf(
            "[DU AUDIO V1.2] PLAY TIMER 8KHZ | CHUNK=%u frames/slot (%u ms) | DOUBLE=%u ms\n",
            (unsigned int)AUDIO_TIMER_CHUNK_FRAMES,
            (unsigned int)(AUDIO_TIMER_CHUNK_FRAMES * 20U),
            (unsigned int)(AUDIO_TIMER_CHUNK_FRAMES * 40U)
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
        // REFILL STREAMING TRONG KHI TIMER DANG PHAT
        // =================================================
#if DU_STREAMING_V1
        bool dang_underflow = false;

        while (1)
        {
            DU_Heartbeat_AudioTask();

            if (du_stream_force_abort_requested)
            {
                Serial.println(
                    "[DU V2C4.8 AUDIO ABORT] stale stream dang PLAY -> DUNG TIMER/DON BUFFER"
                );
                break;
            }

            notify_bits = 0;

            xTaskNotifyWait(
                0,
                0xFFFFFFFFUL,
                &notify_bits,
                pdMS_TO_TICKS(5)
            );

            DU_Heartbeat_AudioTask();

            // Khong chi refill khi co notification. Neu ca hai slot tung bi
            // rong truoc END, task se poll moi 5 ms va tu khoi dong lai timer
            // ngay khi decoder day PCM moi vao RingBuffer.
            if (Audio_Timer_Slot_Free(0))
            {
                (void)Nap_Audio_Timer_Slot(0, 0);
            }

            if (Audio_Timer_Slot_Free(1))
            {
                (void)Nap_Audio_Timer_Slot(1, 0);
            }

            Audio_Timer_Resume_If_Needed();

            bool all_empty = Audio_Timer_All_Empty();

            if (all_empty && !du_stream_end_received)
            {
                // Jitter gap: dua PWM ve midpoint thay vi giu sample cu.
                ledcWrite(PWM_CHANNEL, 128);

                if (!dang_underflow)
                {
                    dang_underflow = true;
                    uint32_t produced = du_stream_pcm_frames_total;
                    uint32_t played = du_stream_pcm_frames_played;
                    uint32_t backlog =
                        produced >= played ? (produced - played) : 0U;

                    if (du_stream_last_audio_decoded)
                    {
                        // Final PCM da phat het, chi con control END_AUDIO dang tren song.
                        // Khong tinh vao mid-stream underflow/quality metric.
                        du_stream_tail_wait_count++;
                        Serial.printf(
                            "[DU STREAM TAIL WAIT] #%u | PRODUCED=%u | PLAYED=%u | BACKLOG=%u -> CHO END_AUDIO\n",
                            (unsigned int)du_stream_tail_wait_count,
                            (unsigned int)produced,
                            (unsigned int)played,
                            (unsigned int)backlog
                        );
                    }
                    else
                    {
                        du_stream_underflow_count++;
                        Serial.printf(
                            "[DU STREAM WARN] MID UNDERFLOW #%u | PRODUCED=%u | PLAYED=%u | BACKLOG=%u -> CHO PCM MOI\n",
                            (unsigned int)du_stream_underflow_count,
                            (unsigned int)produced,
                            (unsigned int)played,
                            (unsigned int)backlog
                        );
                    }
                }

                continue;
            }

            if (!all_empty)
            {
                dang_underflow = false;
            }

            if (du_stream_end_received && all_empty)
            {
                // END handler flush FEC truoc khi set flag, nen khi ca timer
                // slot va RingBuffer deu khong nap duoc nua thi stream da het.
                break;
            }
        }
#else
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

            if (notify_bits & 0x01UL)
            {
                if (!input_exhausted)
                {
                    if (!Nap_Audio_Timer_Slot(0, 0))
                    {
                        input_exhausted = true;
                    }
                }
            }

            if (notify_bits & 0x02UL)
            {
                if (!input_exhausted)
                {
                    if (!Nap_Audio_Timer_Slot(1, 0))
                    {
                        input_exhausted = true;
                    }
                }
            }

            Audio_Timer_Resume_If_Needed();

            if (
                input_exhausted
                && Audio_Timer_All_Empty()
            )
            {
                break;
            }
        }
#endif

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

#if DU_STREAMING_V1
        if (du_stream_force_abort_requested)
        {
            DU_V2C48_ClearStaleStreamState("AUDIO_TASK_PLAY_ABORT");
            continue;
        }

        du_stream_end_received = false;
        du_stream_pcm_frames_total = 0;
        du_stream_pcm_frames_played = 0;
        du_stream_tail_wait_count = 0;
        du_stream_last_audio_decoded = false;
        du_stream_play_report_pending = false;
#endif

        Serial.println(
            "[DU AUDIO] PHAT XONG STREAM V1 -> CHO CAU TIEP"
        );


        // HMI module tu tao AUTO_ACK va mo quyen NACK
        // cho session vua phat xong.
        HMI_DU_Bao_Phat_Xong(
            du_last_played_session_id
        );

#if DU_STREAMING_V1
        // QUAN TRONG: release session sau khi PCM cuoi da phat xong.
        // HMI/USER_CONFIRM dung du_last_played_session_id rieng nen khong can
        // giu session_id_hien_tai de chan session moi.
        Serial.printf(
            "[DU V2C4.8 SESSION RELEASE] SESSION=%016llX | READY_FOR_NEXT=1\n",
            (unsigned long long)du_last_played_session_id
        );
        DU_Jam53_Stop("PLAYBACK_DONE");
        du_stream_session_busy = false;
        da_co_session = false;
        session_id_hien_tai = 0;
        du_stream_session_last_activity_ms = millis();
#endif
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
        bool radio_dang_ban =
            HMI_DU_Radio_Dang_Ban()
            || cho_phep_phat_audio
            || du_stream_session_busy
            || da_co_session
            || DU_Latency_DangNhanSession();

        if (!radio_dang_ban)
        {
            bool gui_kenh_ok =
                Gui_BaoCao_Kenh_SU_DU_DangCho();

            if (
                gui_kenh_ok
                &&
                DU_Latency_PostChannelDangCho()
            )
            {
                DU_Latency_DanhDauPostChannelDaGui();

                Serial.println(
                    "[DU TELEMETRY] POST CHANNEL SU-DU -> rBS"
                );
            }
        }

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

    // V2C5.3A-fix1 GPS_PHIEN_RETRY_THROTTLE_500MS
    // GPS_PHIEN pending van duoc uu tien hon GPS dinh ky, nhung neu LoRa dang
    // ban/network busy thi chi thu lai toi da 1 lan / 500 ms. Tranh spam Serial
    // va tranh task telemetry quay lai ham TX moi 50 ms.
    // V2C5.3A-fix2 GPS_LOG_SUCCESS_ONLY_RETRY_2000MS
    static constexpr uint32_t DU_GPS_PHIEN_RETRY_MS = 2000U;
    static constexpr uint32_t DU_GPS_PERIODIC_RETRY_MS = 2000U;
    uint32_t moc_thu_lai_gps_phien_ms = 0U;

    while (1)
    {
        uint32_t bay_gio_ms = millis();

        bool radio_dang_ban =
            HMI_DU_Radio_Dang_Ban()
            || cho_phep_phat_audio
            || du_stream_session_busy
            || da_co_session
            || DU_Latency_DangNhanSession();

        // TELEMETRY PRE/POST V1:
        // PRE = periodic idle snapshot da co o rBS.
        // TRONG PHIEN = khong TX telemetry.
        // POST = GPS_PHIEN session-bound nay, chi gui khi radio/HMI/playback ranh.
        uint64_t gps_phien_pending_id = 0;
        const bool co_gps_phien_pending =
            DU_Latency_LayGPSPhienPending(gps_phien_pending_id);

        if (
            !radio_dang_ban
            &&
            co_gps_phien_pending
        )
        {
            // V2C5.3A-fix1:
            // Neu lan gui truoc bi tu choi do TDMA/network busy, KHONG lap lai
            // moi 50 ms. Trong thoi gian pending, GPS dinh ky van bi chan nhu cu.
            if ((int32_t)(bay_gio_ms - moc_thu_lai_gps_phien_ms) >= 0)
            {
                DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();

                if (
                    Gui_GPS_REPORT_DU(
                        gps_phien_pending_id,
                        du_lieu_gps_du
                    )
                )
                {
                    // Fix2: chi in full GPS snapshot khi packet da TX thanh cong.
                    // Neu network/TDMA busy, khong spam Serial.
                    In_TrangThai_GPS_DU(du_lieu_gps_du);

                    DU_Latency_XoaGPSPhienPending(
                        gps_phien_pending_id
                    );

                    moc_thu_lai_gps_phien_ms = 0U;

                    // POST packet vua gui xong -> khong gui them GPS dinh ky
                    // ngay lap tuc, tranh 2 telemetry packet lien nhau.
                    moc_gui_tiep_theo_ms =
                        bay_gio_ms
                        +
                        CHU_KY_BAO_CAO_VI_TRI_DU_MS;

                    Serial.printf(
                        "[DU TELEMETRY] POST GPS_PHIEN -> rBS | SESSION=%016llX\n",
                        (unsigned long long)gps_phien_pending_id
                    );
                }
                else
                {
                    moc_thu_lai_gps_phien_ms =
                        bay_gio_ms + DU_GPS_PHIEN_RETRY_MS;
                }
            }
        }
        else if (
            !radio_dang_ban
            &&
            (int32_t)(bay_gio_ms - moc_gui_tiep_theo_ms) >= 0
        )
        {
            DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();

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
            {
                so_thu_tu_bao_cao_vi_tri_du = stt_du_kien;

                // Fix2: log full GPS only for a real successful TX.
                In_TrangThai_GPS_DU(du_lieu_gps_du);
            }

            // Thanh cong: giu nguyen chu ky GPS dinh ky 5 s.
            // That bai/network busy: thu lai cham 2 s, khong spam log.
            moc_gui_tiep_theo_ms = bay_gio_ms +
                (gui_thanh_cong
                    ? CHU_KY_BAO_CAO_VI_TRI_DU_MS
                    : DU_GPS_PERIODIC_RETRY_MS);
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

    Serial.printf(
        "[DU BUILD] TAG=V2C4_8_STALE_AUDIO_PREEMPT | PAIR=%u | SU=0x%02X | DU=0x%02X | SPEEX=ADAPTIVE(HQ3950/LQ2150) | HQ_BYTES=10 | LQ_BYTES=6 | VOICE_MAX_FRAMES=15 | VOICE_BYTES=%u | FEC=%u+1 | PREFILL=%u frames (%u ms) | GUARD=60ms | ENH=OFF\n",
        (unsigned int)V2C_PAIR_INDEX,
        (unsigned int)V2C_SU_ID,
        (unsigned int)V2C_DU_ID,
        (unsigned int)SIZE_VOICE_PACKET,
        (unsigned int)FEC_DATA_PER_GROUP,
        (unsigned int)DU_STREAM_PREFILL_FRAMES,
        (unsigned int)(DU_STREAM_PREFILL_FRAMES * 20U)
    );


    HienThi_DU_ChoNhan();


    // =================================================
    // PWM CARRIER 100 kHz / 8-bit (V2C3.2 RESTORE KNOWN-GOOD OUTPUT)
    // =================================================

    double pwm_actual_hz = ledcSetup(
        PWM_CHANNEL,
        PWM_CARRIER_FREQ,
        PWM_RESOLUTION
    );

    Serial.printf(
        "[DU PWM] REQUEST=%u Hz | RES=%u bit | ACTUAL=%.1f Hz | MID=128\n",
        (unsigned int)PWM_CARRIER_FREQ,
        (unsigned int)PWM_RESOLUTION,
        pwm_actual_hz
    );


    // Audio GPIO ban đầu High-Z
    pinMode(
        CHAN_AUDIO_OUT,
        INPUT
    );


    // =================================================
    // QUEUE
    //
    // Mỗi item = 104 byte (max VOICE/FEC)
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
            4,
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
    Serial.println("[DU JAM V2C5.3A] UL_WINDOW_SIM=ON | DU_HELPER_ONLY | START_STOP_SIM=ON | RF_JAM=OFF");

#if DU_STREAMING_V12_AUDIO_CHUNK
    Serial.printf(
        "[DU STREAM V2C5.2] AUDIO CHUNK BAT | PREFILL=%u ms | TIMER_SLOT=%u ms | DOUBLE=%u ms\n",
        (unsigned int)(DU_STREAM_PREFILL_FRAMES * 20U),
        (unsigned int)(AUDIO_TIMER_CHUNK_FRAMES * 20U),
        (unsigned int)(AUDIO_TIMER_CHUNK_FRAMES * 40U)
    );
#endif
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
    vTaskDelete(NULL);
}