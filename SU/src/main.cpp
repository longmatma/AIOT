#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "esp_heap_caps.h"
#include <Wire.h>
#include <U8g2lib.h>
#include "driver/adc.h"

#include "thu_am.h"
#include "nen_speex.h"
#include "dong_goi.h"
#include "phat_lora.h"
#include "node_config.h"
#include "hien_thi.h"
#include "gps_su.h"
#include "hmi_su.h"


extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;


// ========================================================
// CẤU HÌNH
// ========================================================

#define MAX_KHUNG_THOAI 15000

// V2C3 ADAPTIVE TDMA AUDIO2.
// Packet LoRa van 106B; plaintext AES-GCM van 90B.
// SINGLE: Speex HQ 3.95 kbps = 10B/20ms, 8 frame/VOICE (80B + zero pad).
// DUAL:   Speex LQ 2.15 kbps =  6B/20ms, 15 frame/VOICE (90B day du).
// Profile duoc danh dau trong bit5 cua VOICE header, nen DU khong phu thuoc
// vao viec nhan dung beacon de chon decoder profile.
#define V2C3_HQ_FRAME_BYTES 10U
#define V2C3_LQ_FRAME_BYTES  6U
#define V2C3_HQ_FRAMES_PER_VOICE 8U
#define V2C3_LQ_FRAMES_PER_VOICE 15U
#define V2C3_MAX_FRAMES_PER_VOICE 15U

// ========================================================
// STREAMING AUDIO V1 - GIAI DOAN 1
//
// Muc tieu:
// - Thu + nen Speex tren task rieng trong khi task radio dang TX/RX.
// - Gui moi 8 frame (160 ms) ngay khi co du, KHONG cho nha PTT moi gui.
// - Giu nguyen SESSION/AES-GCM/ARQ/packet-FEC/telemetry/HMI/watchdog.
// - Codec giam tu 8 kbps -> 3.95 kbps; FEC = 4 DATA + 1 PARITY.
//
// Day CHUA phai TDMA 500 ms/friendly-jamming scheduler. Giai doan nay chi
// tach audio real-time khoi radio de co baseline on dinh truoc khi them slot.
// ========================================================
#define SU_STREAMING_V1 1
#define SU_STREAM_FRAME_QUEUE_DEPTH 128

// ========================================================
// STREAMING V2C0 - CAPACITY PROFILE 600 ms (BENCH 1 SU + 1 DU)
//
// rBS phat beacon TYPE=0x1B moi 600 ms. Beacon vua la moc dong bo frame
// vua mang ACK bitmap/FEC cua uplink frame truoc. SU chi duoc TX trong
// uplink slot sau beacon; neu mat beacon thi KHONG TX tu do.
//
// Lich bench V2B (moc t=0 la luc rBS BAT DAU phat beacon):
//   ~0..12 ms   : beacon/sync
//   SU RX_DONE+12 ms: bat dau uplink burst toi da 3 VOICE
//   sau VOICE: gui TOI DA 1 FEC parity theo kieu BEST-EFFORT
//   t=300 ms    : rBS bat dau downlink burst (2 VOICE + toi da 1 FEC)
//   t=600 ms    : beacon ke tiep + ACK VOICE frame truoc
//
// ACK rieng sau burst da duoc bo khoi duong binh thuong. ACK beacon chi quyet
// dinh VOICE; FEC 4+1 van duoc tao/relay nhung KHONG duoc phep lam VOICE retry.
// SESSION, AUDIO_END, telemetry va watchdog GIU NGUYEN.
// ========================================================
#define SU_STREAMING_V2B_SUPERFRAME 1
#define V2B_VOICE_PER_BLOCK 2U   // SINGLE: 2x160ms HQ; DUAL: 2x300ms LQ
#define V2B_SCHEDULE_VERSION 7U
#define V2B_SUPERFRAME_MS 600U   // compatibility: DUAL period
#define V2B_INTER_PACKET_GUARD_MS 3U
#define V2B_BEACON_WAIT_MS 1000U
#define V2B_MAX_CONSECUTIVE_BEACON_MISS 3U

// V2C2.2 beacon schedule_ctl (byte18): bits7..6=current MODE,
// bits5..4=PREPARE target for next beacon, bits3..2=future JAM policy, bits1..0=FEC grant.
#define V2C2_MODE_SINGLE1 1U
#define V2C2_MODE_SINGLE2 2U
#define V2C2_MODE_DUAL    3U
#define V2C2_SINGLE_ACTIVE_UL_OFFSET_MS 22U
#define V2C2_SINGLE_ACTIVE_UL_LATEST_MS 32U
// V2C4.5 JOIN SLOT FIX:
// SINGLE HQ transmits two 106B VOICE packets from +12 ms. At SF7/BW500 their
// measured/theoretical airtime is ~44.9 ms each, so the old JOIN slot at 80 ms
// overlapped the active SU's second HQ packet. The inactive SU therefore could
// not deliver SESSION_START until the active PTT was released.
//
// Reserve a control-only JOIN mini-slot after both HQ packets and before the
// rBS downlink phase. This slot is also reserved JAM=OFF for future friendly
// jamming policy. Offsets are relative to BEACON RX_DONE at the SU.
#define V2C2_SINGLE_JOIN_OFFSET_MS     115U
#define V2C2_SINGLE_JOIN_LATEST_MS     130U

// V2C4.6 admission control:
// - Khi IDLE cache khong chac chan, nghe toi da hon 1 superframe truoc khi bootstrap.
// - Sau khi da bat duoc beacon cua mang dang active, khoa TDMA den het handshake;
//   retry TUYET DOI khong quay lai SESSION_START tu do.
#define V2C46_ADMISSION_LISTEN_MS       340U
#define V2C46_TDMA_BEACON_WAIT_MS       450U
#define V2C2_SINGLE_FEC_OFFSET_MS      105U
#define V2C2_SINGLE_PERIOD_MS          300U
#define V2C2_DUAL_PERIOD_MS            300U
#define V2C4_DUAL_P1_OFFSET_MS          12U
#define V2C4_DUAL_P2_OFFSET_MS          80U
#define V2C4_DUAL_P1_LATEST_MS          40U
#define V2C4_DUAL_P2_LATEST_MS         108U
// Cho beacon that den tre toi da 15 ms quanh moc du doan. Neu van mat, SU chi
// duoc phep dung MOT frame holdover local; frame thu hai lien tiep se fail-safe.
#define V2C2_HOLDOVER_BEACON_GRACE_MS   15U

// Giu ten cu cho mot so helper/fallback chua can doi interface.
#define V2A_VOICE_PER_BLOCK V2B_VOICE_PER_BLOCK
#define V2A_INTER_PACKET_GUARD_MS V2B_INTER_PACKET_GUARD_MS
#define V2A_BLOCK_ACK_TIMEOUT_MS 500U


// ========================================================
// SELF-HEALING V1 - ESP32-S3 TASK WATCHDOG (SU)
//
// - Watchdog thật LUÔN hoạt động.
// - SU_WDT_SELF_TEST chỉ dùng để cố tình ngừng feed sau 10 s
//   nhằm kiểm chứng ESP32 có tự reboot hay không.
// - Production phải để SU_WDT_SELF_TEST = 0.
//
// Timeout 12 s được chọn lớn hơn nhiều so với:
//   * ACK wait <= ~1.05 s / packet
//   * SESSION wait <= ~3.6 s tổng
// và watchdog được feed thêm trong các vòng gửi dài.
// Vì vậy không tăng delay và tránh reset giả khi thoại dài.
// ========================================================
#define SU_WDT_TIMEOUT_S              12U
#define SU_WDT_SELF_TEST               0U
#define SU_WDT_SELF_TEST_DELAY_MS   10000U
static bool su_wdt_self_test_arm = false;
static uint32_t su_wdt_boot_ms = 0;


static void SU_WDT_Feed()
{
    // Nếu task chưa được subscribe thì reset() chỉ trả lỗi; bỏ qua an toàn.
    (void)esp_task_wdt_reset();
}


static void SU_WDT_Init()
{
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms = SU_WDT_TIMEOUT_S * 1000U;
    cfg.idle_core_mask = 0;       // chỉ watchdog task SU, không ép IDLE task
    cfg.trigger_panic = true;

    esp_err_t err = esp_task_wdt_init(&cfg);

    if (err == ESP_ERR_INVALID_STATE)
    {
        // Arduino core đã khởi tạo TWDT -> chỉ đổi timeout/config.
        (void)esp_task_wdt_reconfigure(&cfg);
    }
#else
    // Arduino-ESP32 / ESP-IDF 4.x
    (void)esp_task_wdt_init(SU_WDT_TIMEOUT_S, true);
#endif

    // setup()/loop() chạy trên cùng Arduino loopTask.
    // Chỉ add nếu task hiện tại chưa nằm trong TWDT.
    if (esp_task_wdt_status(NULL) != ESP_OK)
    {
        (void)esp_task_wdt_add(NULL);
    }

    su_wdt_boot_ms = millis();

#if SU_WDT_SELF_TEST
    const esp_reset_reason_t reset_reason = esp_reset_reason();

    // Nếu lần boot hiện tại chính là hậu quả của watchdog thì bài test đã PASS.
    // Không arm lại -> tránh reset-loop.
    if (
        reset_reason == ESP_RST_TASK_WDT
        || reset_reason == ESP_RST_WDT
        || reset_reason == ESP_RST_INT_WDT
    )
    {
        su_wdt_self_test_arm = false;
        Serial.println(
            "[SU WDT] SELF_TEST PASS: reboot do watchdog, KHONG lap lai."
        );
    }
    else
    {
        su_wdt_self_test_arm = true;
        Serial.println(
            "[SU WDT] SELF_TEST armed: se gia lap treo sau 10s."
        );
    }
#else
    su_wdt_self_test_arm = false;
#endif

    SU_WDT_Feed();

    Serial.printf(
        "[SU WDT] BAT | TIMEOUT=%us | SELF_TEST=%u | RESET_REASON=%d\n",
        (unsigned int)SU_WDT_TIMEOUT_S,
        (unsigned int)SU_WDT_SELF_TEST,
        (int)esp_reset_reason()
    );
}


static void SU_WDT_SelfTest_Check()
{
#if SU_WDT_SELF_TEST
    if (
        su_wdt_self_test_arm
        &&
        (millis() - su_wdt_boot_ms) >= SU_WDT_SELF_TEST_DELAY_MS
    )
    {
        su_wdt_self_test_arm = false;

        Serial.println(
            "[SU WDT TEST] GIA LAP TREO: dung feed watchdog, cho ESP32 tu reboot..."
        );
        Serial.flush();

        // Cố tình KHÔNG feed watchdog.
        // delay() vẫn nhường CPU nhưng loopTask đã được TWDT theo dõi,
        // nên sau timeout ESP32 sẽ reset.
        while (true)
        {
            delay(1000);
        }
    }
#endif
}


// ========================================================
// BAO CAO VI TRI DINH KY SU -> rBS
// GPS van cap nhat lien tuc trong task rieng; LoRa chi gui khi SU idle.
// 5 giay la diem khoi dau an toan, co the doi ve sau.
// ========================================================
#define CHU_KY_BAO_CAO_VI_TRI_SU_MS 5000UL
#define TRE_BAO_CAO_VI_TRI_SU_LUC_KHOI_DONG_MS 1000UL

uint32_t moc_gui_vi_tri_su_tiep_theo_ms = 0;
uint64_t so_thu_tu_bao_cao_vi_tri_su = 0;


// ========================================================
// TELEMETRY PRE/POST V1
//
// PRE:
// - Khong phat them packet khi bam PTT.
// - Dung snapshot telemetry IDLE gan nhat da gui truoc do.
//
// TRONG PHIEN:
// - trang_thai != NGHI_NGOI -> GPS telemetry bi khoa.
//
// POST:
// - Khi phan voice da ket thuc, dat pending.
// - Neu HMI van dang cho ACK/NACK/confirm thi tiep tuc cho.
// - Ngay khi HMI ket thuc, gui 1 beacon GPS SU ngay lap tuc.
// - Beacon POST nay cung la mau de DU do RSSI/SNR SU->DU.
//
// Khong thay packet format, khong them delay vao SESSION/VOICE/FEC.
// ========================================================
static uint32_t su_telemetry_last_tx_ms = 0;
static bool su_telemetry_post_pending = false;


// ========================================================
// PROTOCOL NATIVE 8-FRAME + ARQ + PACKET-FEC
//
// 1 VOICE packet = tối đa 8 frame = 160 ms audio.
// Speex 3.95 kbps = 10B/frame -> VOICE inner = 96 byte.
//
// rBS relay NGAY từng packet (BURST_SIZE logic cũ bỏ).
//
// First hop SU->rBS:
//   stop-and-wait ACK có KIND + SEQ.
//   Nếu ACK mất / packet mất: retransmit chính packet đã mã hóa.
//
// End-to-end packet FEC (STREAM V1):
//   4 DATA + 1 PARITY: uu tien du airtime/du phong retry o buoc 1.
//   Parity XOR protected block 88B = ciphertext80 + original GCM tag8.
//   FEC parity packet tu duoc AES-GCM bao ve.
//   DU co the khoi phuc 1 VOICE mat trong moi group va verify GCM goc.
//
// CR LoRa vẫn = 4/5.
// ========================================================

#define MAX_TX_RETRY                2

#define ACK_TIMEOUT_SU_MS 350

// SESSION_START -> rBS -> DU -> SESSION_READY -> rBS -> SU.
// V10: cho rong hon de DU co GPS report + SESSION_READY ma khong bi ep timeout.
// Khi READY den som, SU thoat ngay nen khong lam tang do tre binh thuong.
#define SESSION_REQUEST_RETRY        2

#define SESSION_TIMEOUT_SU_MS 1400

// ========================================================
// CONTROL SU -> rBS
//
// Không còn gửi BURST_END riêng.
// Packet VOICE cuối mang cờ LAST_AUDIO trong byte 2/AAD.
//
// TYPE_AUDIO_END_SU vẫn được giữ:
//   - gửi sau ACK của final FEC;
//   - là tín hiệu explicit để rBS đóng session và gửi END x3 tới DU.
// ========================================================

#define TYPE_AUDIO_END_SU 0x04

#define ID_TRAM_SU_CTRL   V2C_SU_ID
#define ID_TRAM_DU_CTRL   V2C_DU_ID


// ========================================================
// DO E2E THUC DUNG: PTT RELEASE -> DU PLAY
//
// DU gui PLAY_STARTED ve rBS ngay sau khi ghi mau PWM dau tien.
// rBS forward PLAY_STARTED ve SU. SU do elapsed tren CHINH dong ho SU.
//
// Gia tri quan sat co them airtime cua duong phan hoi:
//   DU -> rBS : packet 12B ~= 10.304 ms
//   guard de SU re-arm RX sau khi nghe ke DU report = 8 ms
//   rBS -> SU : packet 12B ~= 10.304 ms
// Tong ~= 28.608 ms, lam tron 29 ms.
//
// OLED hien E2E~ = observed - 21 ms.
// Day la uoc luong rat gan de test/toi uu, chua phai phep do PPS dong bo.
// ========================================================

#define PLAY_REPORT_TIMEOUT_MS       1500

#define THOI_GIAN_PHAN_HOI_E2E_UOC_TINH_MS 29


// ========================================================
// BỘ NHỚ AUDIO
// ========================================================

uint8_t *Kho_Chua_AmThanh;

// ========================================================
// SU AUDIO PSRAM V1
//
// Kho_Chua_AmThanh chua cac frame Speex da nen:
//   MAX_KHUNG_THOAI * 6 byte/frame cho legacy buffer; adaptive streaming queue PCM rieng.
//
// Uu tien cap phat tu PSRAM neu PSRAM dang kha dung.
// Neu board/config khong expose PSRAM, firmware fallback ve
// internal RAM de khong lam brick thiet bi; log se canh bao ro.
//
// Khong thay packet/codec/AES/FEC/ARQ.
// Khong tang MAX_KHUNG_THOAI o patch nay.
// ========================================================
static bool su_audio_buffer_in_psram = false;
static size_t su_audio_buffer_bytes =
    (size_t)MAX_KHUNG_THOAI * (size_t)V2C3_LQ_FRAME_BYTES;

volatile uint32_t tong_so_khung_da_ghi = 0;

struct SUStreamAudioFrame
{
    // V2C4.2: Speex duoc encode NGAY trong capture task, theo thu tu thoi gian.
    // Radio task chi dong packet tu frame DA MA HOA -> khong con mat 100..130 ms
    // encode sau beacon va bo cach mot superframe. 10B du cho HQ, LQ dung 6B.
    uint8_t encoded[10];
    uint8_t frame_bytes;
    bool codec_hq;
};

// Capture profile duoc scheduler cap nhat theo mode da COMMIT.
// SINGLE cua chinh cap = HQ; DUAL / dang JOIN = LQ de san sang chia khe.
static volatile bool su_stream_encode_hq = true;

static QueueHandle_t su_stream_frame_queue = nullptr;
static TaskHandle_t su_stream_capture_task = nullptr;

static volatile bool su_stream_capture_requested = false;
static volatile bool su_stream_capture_running = false;
static volatile bool su_stream_capture_done = true;
static volatile bool su_stream_capture_forced_stop = false;
static volatile uint32_t su_stream_queue_overflow = 0;
static volatile uint32_t su_stream_ptt_release_ms = 0;
static bool su_stream_block_until_ptt_release = false;

// ========================================================
// V2C1-LATENCY-BASELINE V1
// Chi them moc do; khong doi packet/AES/FEC/Speex/scheduler.
// millis() la dong ho cuc bo SU, chi dung tinh DELTA tren cung SU.
// ========================================================
static volatile uint32_t su_diag_first_encoded_ms = 0;
static volatile uint32_t su_diag_queue_max_frames = 0;
static uint32_t su_diag_session_ready_ms = 0;
static uint32_t su_diag_first_voice_build_ms = 0;
static uint32_t su_diag_first_voice_tx_ms = 0;

static uint32_t SU_DeltaMs(uint32_t now_ms, uint32_t start_ms)
{
    return (start_ms == 0 || now_ms == 0) ? 0U : (uint32_t)(now_ms - start_ms);
}

// Đo thời gian giữ PTT thực tế cho từng câu.
uint32_t thoi_diem_bat_dau_ghi_ms = 0;

// Moc bat dau E2E = ngay khi loop phat hien PTT vua duoc nha.
uint32_t e2e_ptt_release_ms = 0;

// ========================================================
// HMI SU DA TACH SANG hmi_su.cpp / hmi_su.h
// main.cpp chi goi API HMI, khong con xu ly GPIO/LED truc tiep.
// ========================================================


// ========================================================
// THONG KE LINK LOCAL TAI SU - HIEN THI OLED
//
// DATA  = so VOICE packet goc cua cau noi.
// RETRY = tong so lan phat lai do khong nhan duoc ACK.
// FAIL  = so packet ARQ (VOICE/FEC) that bai sau khi het retry.
//
// Luu y: RETRY cho biet Hop1 co van de, nhung mot retry co the
// do packet SU->rBS mat HOAC ACK rBS->SU mat.
// ========================================================

uint32_t su_oled_voice_packets = 0;
uint32_t su_oled_retransmissions = 0;
uint32_t su_oled_fail_packets = 0;

static void Reset_ThongKe_OLED_SU()
{
    su_oled_voice_packets = 0;
    su_oled_retransmissions = 0;
    su_oled_fail_packets = 0;
}


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
// GỬI 1 PACKET + CHỜ ACK CÓ KIND/SEQ
//
// MAX_TX_RETRY = số lần phát lại SAU lần đầu.
// Retransmit dùng nguyên packet ciphertext/tag cũ;
// không mã hóa lại với cùng IV.
// ========================================================

static bool Gui_Packet_Co_ACK(
    uint8_t *packet,
    size_t packet_len,
    uint8_t ack_kind,
    uint32_t ack_seq)
{
    for (
        uint8_t lan = 0;
        lan <= MAX_TX_RETRY;
        lan++
    )
    {
        SU_WDT_Feed();
        if (lan > 0)
        {
            su_oled_retransmissions++;

            // Khong refresh OLED o tung retry de tranh I2C lam tang delay.

            Serial.printf(
                "[SU ARQ] RETRY %u/%u | KIND=0x%02X | SEQ=%u\n",
                lan,
                MAX_TX_RETRY,
                ack_kind,
                ack_seq
            );
        }

        // LED duy nhat chop theo moi lan TX DATA that.
        Dat_LED_SU(true);
        Phat_GoiTin_LoRa(
            packet,
            packet_len
        );
        Dat_LED_SU(false);

        bool ack_ok =
            Cho_READY_RBS(
                ACK_TIMEOUT_SU_MS,
                ack_kind,
                ack_seq
            );

        SU_WDT_Feed();

        if (ack_ok)
        {
            return true;
        }
    }

    su_oled_fail_packets++;

    // Khong refresh OLED o day; final screen se cap nhat sau session.

    Serial.printf(
        "[SU ARQ FAIL] KIND=0x%02X | SEQ=%u\n",
        ack_kind,
        ack_seq
    );

    return false;
}


// Ham telemetry duoc dinh nghia sau setup; streaming loop can goi khi idle.
static void XuLy_BaoCao_ViTri_DinhKy_SU();

// ========================================================
// STREAMING V1 - TASK THU/NEN DOC LAP VOI RADIO
// ========================================================

static void SU_Stream_Drain_ADC()
{
    uint8_t rac[1280];
    uint32_t len = 0;

    while (
        adc_digi_read_bytes(
            rac,
            sizeof(rac),
            &len,
            0
        ) == ESP_OK
        && len > 0
    )
    {
    }
}


static void TacVu_ThuAm_Streaming_SU(void *tham_so)
{
    (void)tham_so;

    for (;;)
    {
        if (!su_stream_capture_requested)
        {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        su_stream_capture_done = false;
        su_stream_capture_running = true;
        su_stream_capture_forced_stop = false;
        su_stream_ptt_release_ms = 0;

        adc_digi_start();

        while (
            su_stream_capture_requested
            && Nut_PTT_Dang_Bam_SU()
            && tong_so_khung_da_ghi < MAX_KHUNG_THOAI
        )
        {
            uint8_t pcm_320b[320];
            if (!LayMau_AmThanh(pcm_320b))
            {
                continue;
            }

            SUStreamAudioFrame frame = {};
            const bool codec_hq_capture = su_stream_encode_hq;
            const SpeexProfileSU profile_capture = codec_hq_capture
                ? SPEEX_PROFILE_HQ
                : SPEEX_PROFILE_LQ;
            frame.codec_hq = codec_hq_capture;
            frame.frame_bytes = Speex_SU_FrameBytes(profile_capture);

            if (!Nen_Thanh_KhungThoai_Profile(
                    pcm_320b,
                    frame.encoded,
                    profile_capture))
            {
                Serial.printf(
                    "[SU V2C4.2 PREENC FAIL] PROFILE=%s | QUEUE=%u\n",
                    codec_hq_capture ? "HQ" : "LQ",
                    (unsigned int)uxQueueMessagesWaiting(su_stream_frame_queue)
                );
                continue;
            }

            if (su_diag_first_encoded_ms == 0)
            {
                su_diag_first_encoded_ms = millis();
                Serial.printf(
                    "[SU LAT] FIRST_ENCODE | T=%u ms | FROM_PTT=%u ms | PROFILE=%s | PREENC=1\n",
                    (unsigned int)su_diag_first_encoded_ms,
                    (unsigned int)SU_DeltaMs(su_diag_first_encoded_ms, thoi_diem_bat_dau_ghi_ms),
                    codec_hq_capture ? "HQ" : "LQ"
                );
            }

            if (
                xQueueSend(
                    su_stream_frame_queue,
                    &frame,
                    0
                ) != pdTRUE
            )
            {
                // Khong cho hang doi tang delay vo han. Neu radio tam thoi
                // cham hon audio, bo frame CU NHAT de uu tien audio moi.
                SUStreamAudioFrame bo_frame_cu;
                (void)xQueueReceive(
                    su_stream_frame_queue,
                    &bo_frame_cu,
                    0
                );

                if (
                    xQueueSend(
                        su_stream_frame_queue,
                        &frame,
                        0
                    ) != pdTRUE
                )
                {
                    // Rat hiem: queue thay doi dung luc. Bo frame moi.
                }

                su_stream_queue_overflow++;
            }

            tong_so_khung_da_ghi++;

            const uint32_t q_now = (uint32_t)uxQueueMessagesWaiting(su_stream_frame_queue);
            if (q_now > su_diag_queue_max_frames)
                su_diag_queue_max_frames = q_now;

            if ((tong_so_khung_da_ghi % 50U) == 0U)
            {
                Serial.printf(
                    "[SU STREAM REC] FRAME=%u | AUDIO_MS=%u | QUEUE=%u | DROP_OLD=%u\n",
                    (unsigned int)tong_so_khung_da_ghi,
                    (unsigned int)(tong_so_khung_da_ghi * 20U),
                    (unsigned int)uxQueueMessagesWaiting(su_stream_frame_queue),
                    (unsigned int)su_stream_queue_overflow
                );
            }
        }

        if (
            tong_so_khung_da_ghi >= MAX_KHUNG_THOAI
            && Nut_PTT_Dang_Bam_SU()
        )
        {
            su_stream_capture_forced_stop = true;
            Serial.println(
                "[SU STREAM] DAT GIOI HAN AUDIO -> TU DUNG THU"
            );
        }

        if (!Nut_PTT_Dang_Bam_SU())
        {
            su_stream_ptt_release_ms = millis();
            e2e_ptt_release_ms = su_stream_ptt_release_ms;
        }

        adc_digi_stop();
        SU_Stream_Drain_ADC();

        su_stream_capture_running = false;
        su_stream_capture_requested = false;
        su_stream_capture_done = true;
    }
}


static bool SU_Stream_BatDauThu()
{
    if (
        su_stream_frame_queue == nullptr
        || su_stream_capture_task == nullptr
    )
    {
        return false;
    }

    // Chi reset queue khi task capture dang nghi.
    uint32_t wait_start = millis();
    while (
        su_stream_capture_running
        && (millis() - wait_start) < 200U
    )
    {
        delay(1);
    }

    if (su_stream_capture_running)
    {
        return false;
    }

    xQueueReset(su_stream_frame_queue);

    tong_so_khung_da_ghi = 0;
    su_stream_queue_overflow = 0;
    su_stream_ptt_release_ms = 0;
    su_stream_capture_done = false;
    su_stream_capture_requested = true;

    wait_start = millis();
    while (
        !su_stream_capture_running
        && !su_stream_capture_done
        && (millis() - wait_start) < 250U
    )
    {
        delay(1);
    }

    return su_stream_capture_running || su_stream_capture_done;
}


static void SU_Stream_DungThu()
{
    su_stream_capture_requested = false;

    uint32_t wait_start = millis();
    while (
        su_stream_capture_running
        && (millis() - wait_start) < 300U
    )
    {
        delay(1);
    }
}


struct SUStreamFECState
{
    uint8_t parity[FEC_PARITY_BYTES];
    uint8_t data_count;
    uint32_t group_start_seq;
    bool has_last;
    uint8_t final_frame_count;
};


static void SU_Stream_ResetFEC(SUStreamFECState &st)
{
    memset(&st, 0, sizeof(st));
}


struct SUStreamFECPacket
{
    uint8_t raw[SIZE_FEC_PACKET_SU];
    uint32_t group_start_seq;
    uint8_t data_count;
    bool has_last;
    uint8_t final_frame_count;
    bool valid;
};


static bool SU_Stream_TaoFECPacket(
    const SUStreamFECState &st,
    SUStreamFECPacket &out)
{
    memset(&out, 0, sizeof(out));

    if (st.data_count == 0)
    {
        return false;
    }

    Tao_GoiTin_FEC(
        st.parity,
        st.group_start_seq,
        st.data_count,
        st.has_last,
        st.final_frame_count,
        out.raw
    );

    out.group_start_seq = st.group_start_seq;
    out.data_count = st.data_count;
    out.has_last = st.has_last;
    out.final_frame_count = st.final_frame_count;
    out.valid = true;
    return true;
}


// Chi con la FALLBACK hiem khi capture DONE roi vao dung ranh gioi block,
// khien parity con du khong kip ghep vao burst V2A.1. Duong binh thuong KHONG
// dung ACK rieng cho FEC nua.
static bool SU_Stream_GuiFEC(SUStreamFECState &st)
{
    if (st.data_count == 0)
    {
        return true;
    }

    uint8_t goi_fec[SIZE_FEC_PACKET_SU];

    Tao_GoiTin_FEC(
        st.parity,
        st.group_start_seq,
        st.data_count,
        st.has_last,
        st.final_frame_count,
        goi_fec
    );

    Serial.printf(
        "[SU V2A.1 FALLBACK TX] FEC LEGACY | GROUP_START=%u | DATA=%u | HAS_LAST=%u | FINAL_FRAME=%u\n",
        (unsigned int)st.group_start_seq,
        (unsigned int)st.data_count,
        st.has_last ? 1 : 0,
        (unsigned int)st.final_frame_count
    );

    bool ok = Gui_Packet_Co_ACK(
        goi_fec,
        SIZE_FEC_PACKET_SU,
        TYPE_FEC_SU,
        st.group_start_seq
    );

    SU_Stream_ResetFEC(st);
    return ok;
}


struct SUStreamVoicePacket
{
    uint8_t raw[SIZE_VOICE_PACKET_SU];
    uint32_t seq;
    uint8_t frame_count;
    bool last_audio;
    bool codec_hq;
};


static bool SU_Stream_TaoVoicePacketTuQueue(
    bool codec_hq_hint,
    uint8_t frame_cap_hint,
    SUStreamFECState &fec_st,
    SUStreamVoicePacket &out,
    bool &packet_ready)
{
    (void)codec_hq_hint;
    (void)frame_cap_hint;
    (void)fec_st;
    packet_ready = false;

    // V2C4.2: capture task da Speex-encode tung frame 20 ms lien tuc.
    // O day CHI copy byte vao VOICE packet; tuyet doi khong encode trong radio loop.
    // Nho vay beacon moi co the duoc dung ngay thay vi bo cach mot frame 300 ms.
    SUStreamAudioFrame first = {};
    if (xQueuePeek(su_stream_frame_queue, &first, 0) != pdTRUE)
        return true;

    const bool codec_hq = first.codec_hq;
    const uint8_t frame_bytes = codec_hq ? V2C3_HQ_FRAME_BYTES : V2C3_LQ_FRAME_BYTES;
    const uint8_t frame_cap = codec_hq ? V2C3_HQ_FRAMES_PER_VOICE : V2C3_LQ_FRAMES_PER_VOICE;

    uint8_t payload_voice[VOICE_PLAINTEXT_BYTES];
    memset(payload_voice, 0, sizeof(payload_voice));

    uint8_t so_frame = 0;
    while (so_frame < frame_cap)
    {
        SUStreamAudioFrame peek = {};
        if (xQueuePeek(su_stream_frame_queue, &peek, 0) != pdTRUE)
            break;

        // Khong tron HQ/LQ trong cung packet. Profile switch duoc phep tai packet boundary.
        if (peek.codec_hq != codec_hq)
            break;

        SUStreamAudioFrame frame = {};
        if (xQueueReceive(su_stream_frame_queue, &frame, 0) != pdTRUE)
            break;

        const uint8_t copy_len = frame.frame_bytes < frame_bytes ? frame.frame_bytes : frame_bytes;
        memcpy(&payload_voice[so_frame * frame_bytes], frame.encoded, copy_len);
        so_frame++;
    }

    if (so_frame == 0U)
        return true;

    const bool la_packet_cuoi =
        su_stream_capture_done
        && uxQueueMessagesWaiting(su_stream_frame_queue) == 0;

    Tao_GoiTin_Voice(payload_voice, so_frame, la_packet_cuoi, codec_hq, out.raw);
    out.seq = ((uint32_t)out.raw[4] << 24) | ((uint32_t)out.raw[5] << 16)
            | ((uint32_t)out.raw[6] << 8) | (uint32_t)out.raw[7];
    out.frame_count = so_frame;
    out.last_audio = la_packet_cuoi;
    out.codec_hq = codec_hq;

    if (su_diag_first_voice_build_ms == 0)
    {
        su_diag_first_voice_build_ms = millis();
        Serial.printf(
            "[SU LAT] FIRST_VOICE_BUILT | SEQ=%u | FC=%u | PROFILE=%s | FROM_PTT=%u ms | QUEUE=%u | PREENC=1\n",
            (unsigned int)out.seq,
            (unsigned int)out.frame_count,
            codec_hq ? "HQ" : "LQ",
            (unsigned int)SU_DeltaMs(su_diag_first_voice_build_ms, thoi_diem_bat_dau_ghi_ms),
            (unsigned int)uxQueueMessagesWaiting(su_stream_frame_queue)
        );
    }

    su_oled_voice_packets++;
    packet_ready = true;
    return true;
}


struct SUV2BBeaconState
{
    bool valid;
    uint32_t frame_id;
    uint32_t ack_base_seq;
    uint8_t ack_count;
    uint8_t ack_bitmap;
    bool ack_fec_expected;
    bool ack_fec_ok;
    uint8_t schedule_mode;
    uint8_t transition_target;
    uint8_t jam_policy;
    uint8_t fec_grant_pair;
    uint32_t rx_ms;
    bool synthetic_holdover;
};

struct SUV2C2ProvisionalAck
{
    bool active;
    uint32_t base_seq;
    uint8_t relevant_mask;
    uint8_t expected_count;
};

static SUV2BBeaconState su_v2b_beacon = {};
static SUV2C2ProvisionalAck su_v2c2_provisional_ack = {};
static uint8_t su_v2b_consecutive_beacon_miss = 0;


// ========================================================
// V2C5.2 - FRIENDLY-JAM LEASE DISTRIBUTION (SIM ONLY)
//
// rBS dung bits3..2 cua schedule_ctl lam pair-mask:
//   bit0 = PAIR1 co lease, bit1 = PAIR2 co lease.
// SU CHI doi state PREPARED/ARMED de kiem tra giao thuc.
// KHONG co ham TX nhiem, KHONG doi cong suat, RF_JAM luon OFF.
// ========================================================
enum SUJam52State : uint8_t
{
    SU_JAM52_IDLE = 0,
    SU_JAM52_PREPARED = 1,
    SU_JAM52_ARMED = 2
};

static SUJam52State su_jam52_state = SU_JAM52_IDLE;
static uint64_t su_jam52_session_id = 0;
static uint32_t su_jam52_last_frame = 0;
static uint8_t su_jam52_last_mask = 0;
static constexpr bool SU_JAM52_RF_ENABLE = false;

static uint8_t SU_Jam52_MyMaskBit()
{
    return (uint8_t)(1U << (V2C_PAIR_INDEX - 1U));
}

static void SU_Jam52_Prepare(uint64_t session_id)
{
    su_jam52_session_id = session_id;
    su_jam52_state = SU_JAM52_PREPARED;
    su_jam52_last_mask = 0;
    Serial.printf(
        "[SU JAM V2C5.2 PREPARED] PAIR=%u | SESSION=%016llX | WAIT_BEACON_LEASE=1 | RF_JAM=OFF\n",
        (unsigned int)V2C_PAIR_INDEX,
        (unsigned long long)session_id
    );
}

static void SU_Jam52_Stop(const char *reason)
{
    if (su_jam52_state != SU_JAM52_IDLE || su_jam52_session_id != 0)
    {
        Serial.printf(
            "[SU JAM V2C5.2 STOP] PAIR=%u | SESSION=%016llX | REASON=%s | RF_JAM=OFF\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned long long)su_jam52_session_id,
            reason != nullptr ? reason : "UNKNOWN"
        );
    }
    su_jam52_state = SU_JAM52_IDLE;
    su_jam52_session_id = 0;
    su_jam52_last_mask = 0;
}

static void SU_Jam52_OnBeacon(const SUV2BBeaconState &b)
{
    if (SU_JAM52_RF_ENABLE)
    {
        // Hard fail-safe: V2C5.2 khong co duong phat RF.
        SU_Jam52_Stop("RF_ENABLE_FORBIDDEN");
        SU_Jam53B2_Stop("RF_ENABLE_FORBIDDEN");
        return;
    }

    if (su_jam52_session_id == 0 || su_jam52_state == SU_JAM52_IDLE)
        return;

    const uint8_t my_bit = SU_Jam52_MyMaskBit();
    const bool allowed =
        !b.synthetic_holdover
        && b.transition_target == 0U
        && ((b.jam_policy & my_bit) != 0U);

    const SUJam52State old_state = su_jam52_state;
    su_jam52_last_frame = b.frame_id;
    su_jam52_last_mask = b.jam_policy;

    if (allowed)
    {
        su_jam52_state = SU_JAM52_ARMED;
        if (old_state != SU_JAM52_ARMED)
        {
            Serial.printf(
                "[SU JAM V2C5.2 ARMED] PAIR=%u | FRAME=%u | MODE=%u | LEASE_MASK=0x%X | SESSION=%016llX | RF_JAM=OFF\n",
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned int)b.frame_id,
                (unsigned int)b.schedule_mode,
                (unsigned int)b.jam_policy,
                (unsigned long long)su_jam52_session_id
            );
        }
    }
    else
    {
        su_jam52_state = SU_JAM52_PREPARED;
        if (old_state == SU_JAM52_ARMED)
        {
            const char *why = b.synthetic_holdover
                ? "HOLDOVER_NO_LEASE"
                : (b.transition_target != 0U ? "PREPARE_TRANSITION" : "LEASE_REVOKED");
            Serial.printf(
                "[SU JAM V2C5.2 REVOKE] PAIR=%u | FRAME=%u | MODE=%u | LEASE_MASK=0x%X | REASON=%s | RF_JAM=OFF\n",
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned int)b.frame_id,
                (unsigned int)b.schedule_mode,
                (unsigned int)b.jam_policy,
                why
            );
        }
    }
}


static void SU_V2B_ResetSync()
{
    memset(&su_v2b_beacon, 0, sizeof(su_v2b_beacon));
    su_v2b_beacon.ack_base_seq = 0xFFFFFFFFUL;
    memset(&su_v2c2_provisional_ack, 0, sizeof(su_v2c2_provisional_ack));
    su_v2b_consecutive_beacon_miss = 0;
}


static bool SU_V2B_WaitBeacon(uint32_t timeout_ms)
{
    uint32_t frame_id = 0;
    uint32_t ack_base = 0xFFFFFFFFUL;
    uint8_t ack_count = 0;
    uint8_t ack_bitmap = 0;
    bool ack_fec_expected = false;
    bool ack_fec_ok = false;
    uint8_t schedule_mode = 0;
    uint8_t transition_target = 0;
    uint8_t jam_policy = 0;
    uint8_t fec_grant_pair = 0;
    uint32_t rx_ms = 0;

    const bool ok = Cho_SUPERFRAME_V2B(
        timeout_ms,
        V2B_SCHEDULE_VERSION,
        frame_id,
        ack_base,
        ack_count,
        ack_bitmap,
        ack_fec_expected,
        ack_fec_ok,
        schedule_mode,
        transition_target,
        jam_policy,
        fec_grant_pair,
        rx_ms
    );

    if (!ok)
    {
        su_v2b_beacon.valid = false;
        if (su_v2b_consecutive_beacon_miss < 255U)
        {
            su_v2b_consecutive_beacon_miss++;
        }

        Serial.printf(
            "[SU V2B SYNC MISS] MISS=%u/%u | KHONG TX NGOAI SLOT\n",
            (unsigned int)su_v2b_consecutive_beacon_miss,
            (unsigned int)V2B_MAX_CONSECUTIVE_BEACON_MISS
        );
        return false;
    }

    su_v2b_consecutive_beacon_miss = 0;
    su_v2b_beacon.valid = true;
    su_v2b_beacon.frame_id = frame_id;
    su_v2b_beacon.ack_base_seq = ack_base;
    su_v2b_beacon.ack_count = ack_count;
    su_v2b_beacon.ack_bitmap = ack_bitmap;
    su_v2b_beacon.ack_fec_expected = ack_fec_expected;
    su_v2b_beacon.ack_fec_ok = ack_fec_ok;
    su_v2b_beacon.schedule_mode = schedule_mode;
    su_v2b_beacon.transition_target = transition_target;
    su_v2b_beacon.jam_policy = jam_policy;
    su_v2b_beacon.fec_grant_pair = fec_grant_pair;
    su_v2b_beacon.rx_ms = rx_ms;
    su_v2b_beacon.synthetic_holdover = false;
    SU_Jam52_OnBeacon(su_v2b_beacon);
    return true;
}


static uint32_t SU_V2C2_FramePeriodMs(uint8_t mode)
{
    return (mode == V2C2_MODE_SINGLE1 || mode == V2C2_MODE_SINGLE2)
        ? V2C2_SINGLE_PERIOD_MS
        : V2C2_DUAL_PERIOD_MS;
}


static uint32_t SU_V2C2_VoiceLatestForMode(uint8_t mode)
{
    const bool single_for_me =
        (mode == V2C2_MODE_SINGLE1 && V2C_PAIR_INDEX == 1U)
        || (mode == V2C2_MODE_SINGLE2 && V2C_PAIR_INDEX == 2U);
    return single_for_me ? V2C2_SINGLE_ACTIVE_UL_LATEST_MS : V2C_UL_LATEST_START_MS;
}


// Wait for the exact next beacon only until the local predicted beacon time +
// a small grace. If it is lost, synthesize ONE schedule frame locally.
// PREPARE/COMMIT makes this safe: a PREPARE beacon keeps the old schedule; the
// following virtual frame may commit the advertised target. A second loss stops TX.
static bool SU_V2C2_WaitNextBeaconOrHoldover(
    const SUV2BBeaconState &frame_used)
{
    const uint32_t period_ms = SU_V2C2_FramePeriodMs(frame_used.schedule_mode);
    const uint32_t expected_rx_ms = frame_used.rx_ms + period_ms;
    const uint32_t deadline_ms = expected_rx_ms + V2C2_HOLDOVER_BEACON_GRACE_MS;
    const int32_t remain_ms = (int32_t)(deadline_ms - millis());

    if (remain_ms > 0)
    {
        if (SU_V2B_WaitBeacon((uint32_t)remain_ms))
        {
            const uint32_t expected_frame = frame_used.frame_id + 1U;
            if (su_v2b_beacon.frame_id != expected_frame)
            {
                Serial.printf(
                    "[SU V2C2.2 FAILSAFE] BEACON JUMP | EXPECT=%u GOT=%u -> DUNG TX\n",
                    (unsigned int)expected_frame,
                    (unsigned int)su_v2b_beacon.frame_id
                );
                return false;
            }
            return true;
        }
    }

    // A synthetic frame is already the single allowed holdover. Never chain it.
    if (frame_used.synthetic_holdover)
    {
        Serial.println(
            "[SU V2C2.2 FAILSAFE] MAT 2 BEACON LIEN TIEP -> KHONG HOLDOVER THEM"
        );
        return false;
    }

    const uint8_t predicted_mode =
        frame_used.transition_target != 0U
            ? frame_used.transition_target
            : frame_used.schedule_mode;
    const uint32_t virtual_age_ms = millis() - expected_rx_ms;
    const uint32_t latest_ms = SU_V2C2_VoiceLatestForMode(predicted_mode);

    if (virtual_age_ms > latest_ms)
    {
        Serial.printf(
            "[SU V2C2.2 HOLDOVER SKIP] QUA MUON | FRAME=%u | AGE=%u > %u ms\n",
            (unsigned int)(frame_used.frame_id + 1U),
            (unsigned int)virtual_age_ms,
            (unsigned int)latest_ms
        );
        return false;
    }

    memset(&su_v2b_beacon, 0, sizeof(su_v2b_beacon));
    su_v2b_beacon.valid = true;
    su_v2b_beacon.frame_id = frame_used.frame_id + 1U;
    su_v2b_beacon.ack_base_seq = 0xFFFFFFFFUL;
    su_v2b_beacon.schedule_mode = predicted_mode;
    su_v2b_beacon.transition_target = 0U;
    su_v2b_beacon.jam_policy = 0U; // JAM lease KHONG bao gio duoc holdover
    // Unknown grant/ACK on a lost beacon: never send parity in holdover.
    su_v2b_beacon.fec_grant_pair = 0U;
    su_v2b_beacon.rx_ms = expected_rx_ms;
    su_v2b_beacon.synthetic_holdover = true;
    SU_Jam52_OnBeacon(su_v2b_beacon);

    Serial.printf(
        "[SU V2C2.2 HOLDOVER] FRAME=%u | MODE=%u | FROM_FRAME=%u | PREP_TARGET=%u | FEC=OFF | VALID=1_FRAME\n",
        (unsigned int)su_v2b_beacon.frame_id,
        (unsigned int)predicted_mode,
        (unsigned int)frame_used.frame_id,
        (unsigned int)frame_used.transition_target
    );
    return true;
}


static bool SU_V2C2_ProvisionalAckConfirmed()
{
    if (!su_v2c2_provisional_ack.active) return true;
    if (su_v2b_beacon.synthetic_holdover) return false;

    bool confirmed = false;
    if (su_v2b_beacon.ack_base_seq == su_v2c2_provisional_ack.base_seq)
    {
        confirmed =
            su_v2b_beacon.ack_count == su_v2c2_provisional_ack.expected_count
            && ((su_v2b_beacon.ack_bitmap & su_v2c2_provisional_ack.relevant_mask)
                == su_v2c2_provisional_ack.relevant_mask);
    }
    else if (
        su_v2b_beacon.ack_base_seq != 0xFFFFFFFFUL
        && su_v2b_beacon.ack_base_seq > su_v2c2_provisional_ack.base_seq)
    {
        // rBS only advances to a new natural 2-packet base after the old block
        // was complete/ackable. A later base is therefore cumulative evidence.
        confirmed = true;
    }

    if (confirmed)
    {
        Serial.printf(
            "[SU V2C2.2 HOLDOVER ACK CONFIRMED] BASE=%u | VIA_ACK_BASE=%u | BITMAP=0x%02X\n",
            (unsigned int)su_v2c2_provisional_ack.base_seq,
            (unsigned int)su_v2b_beacon.ack_base_seq,
            (unsigned int)su_v2b_beacon.ack_bitmap
        );
        su_v2c2_provisional_ack.active = false;
        return true;
    }

    Serial.printf(
        "[SU V2C2.2 HOLDOVER ACK FAILSAFE] PROVISIONAL_BASE=%u | RX_ACK_BASE=%u | BITMAP=0x%02X -> DUNG PHIEN\n",
        (unsigned int)su_v2c2_provisional_ack.base_seq,
        (unsigned int)su_v2b_beacon.ack_base_seq,
        (unsigned int)su_v2b_beacon.ack_bitmap
    );
    return false;
}


static uint8_t SU_V2C2_ActivePairForMode(uint8_t mode)
{
    if (mode == V2C2_MODE_SINGLE1) return 1U;
    if (mode == V2C2_MODE_SINGLE2) return 2U;
    return 0U;
}


static bool SU_V2C2_ModeIsSingleForMe(uint8_t mode)
{
    return SU_V2C2_ActivePairForMode(mode) == V2C_PAIR_INDEX;
}


// V2C4.6 PROFILE TRANSITION SAFE:
// Chi encode HQ khi beacon DA COMMIT SINGLE cua chinh cap va KHONG co PREPARE
// chuyen mode. PREPARE -> DUAL phai doi sang LQ som mot frame de hang doi
// khong con HQ backlog luc COMMIT. Khi chua biet mode, LQ la profile an toan.
static bool SU_V2C46_CaptureShouldUseHQ()
{
    if (!su_v2b_beacon.valid)
        return false;

    if (!SU_V2C2_ModeIsSingleForMe(su_v2b_beacon.schedule_mode))
        return false;

    if (
        su_v2b_beacon.transition_target != 0U
        && su_v2b_beacon.transition_target != su_v2b_beacon.schedule_mode
    )
    {
        return false;
    }

    return true;
}


static uint32_t SU_V2C2_VoiceOffsetMs()
{
    if (SU_V2C2_ModeIsSingleForMe(su_v2b_beacon.schedule_mode))
        return V2C2_SINGLE_ACTIVE_UL_OFFSET_MS;

    // V2C4 DUAL=300 ms chi gui 1 LQ VOICE/pair, nen co the keo SU2 som hon
    // lich 600 ms cu. Hai packet van tach nhau ro tren mot E22 cua rBS.
    return (V2C_PAIR_INDEX == 1U) ? V2C4_DUAL_P1_OFFSET_MS : V2C4_DUAL_P2_OFFSET_MS;
}


static uint32_t SU_V2C2_VoiceLatestMs()
{
    if (SU_V2C2_ModeIsSingleForMe(su_v2b_beacon.schedule_mode))
        return V2C2_SINGLE_ACTIVE_UL_LATEST_MS;

    // V2C4: DUAL cung chay 300 ms; SU2 duoc keo som ve 80 ms va latest 108 ms
    // de co guard rong truoc downlink rBS.
    return (V2C_PAIR_INDEX == 1U) ? V2C4_DUAL_P1_LATEST_MS : V2C4_DUAL_P2_LATEST_MS;
}


static uint32_t SU_V2C2_FecOffsetMs()
{
    if (SU_V2C2_ModeIsSingleForMe(su_v2b_beacon.schedule_mode))
        return V2C2_SINGLE_FEC_OFFSET_MS;
    return V2C_FEC_SLOT_OFFSET_MS;
}


static uint8_t SU_V2C2_BlockTarget(bool codec_hq_build)
{
    // DUAL luon toi da 1 VOICE / pair / 300 ms de SU1 va SU2 khong dam nhau.
    // SINGLE: HQ can 2 packet (8+7 ~300 ms); LQ bridge chi can 1 packet 15 frame.
    if (!SU_V2C2_ModeIsSingleForMe(su_v2b_beacon.schedule_mode))
        return 1U;
    return codec_hq_build ? 2U : 1U;
}


static uint32_t SU_V2C2_SessionOffsetMs()
{
    const uint8_t active = SU_V2C2_ActivePairForMode(su_v2b_beacon.schedule_mode);
    if (active == 0U)
        return V2C_UL_VOICE_OFFSET_MS; // DUAL
    if (active == V2C_PAIR_INDEX)
        return V2C2_SINGLE_ACTIVE_UL_OFFSET_MS;
    return V2C2_SINGLE_JOIN_OFFSET_MS;
}


static uint32_t SU_V2C2_SessionLatestMs()
{
    const uint8_t active = SU_V2C2_ActivePairForMode(su_v2b_beacon.schedule_mode);
    if (active == 0U)
        return V2C_UL_LATEST_START_MS;
    if (active == V2C_PAIR_INDEX)
        return V2C2_SINGLE_ACTIVE_UL_LATEST_MS;
    return V2C2_SINGLE_JOIN_LATEST_MS;
}


static bool SU_V2B_EnsureFreshSlot()
{
    while (true)
    {
        SU_WDT_Feed();

        if (su_v2b_beacon.valid)
        {
            const uint8_t active_pair = SU_V2C2_ActivePairForMode(su_v2b_beacon.schedule_mode);
            if (active_pair != 0U && active_pair != V2C_PAIR_INDEX)
            {
                // Voice/FEC of this pair is forbidden in the other pair's SINGLE frame.
                // SESSION_START uses a separate JOIN opportunity and does not call here.
                Serial.printf(
                    "[SU V2C2 WAIT] FRAME=%u | MODE=%u belongs to PAIR%u -> NO VOICE TX\n",
                    (unsigned int)su_v2b_beacon.frame_id,
                    (unsigned int)su_v2b_beacon.schedule_mode,
                    (unsigned int)active_pair
                );
                su_v2b_beacon.valid = false;
            }
            else
            {
                const uint32_t age_ms = millis() - su_v2b_beacon.rx_ms;
                const uint32_t latest_ms = SU_V2C2_VoiceLatestMs();
                if (age_ms <= latest_ms)
                {
                    return true;
                }

                Serial.printf(
                    "[SU V2C2 SLOT SKIP] FRAME=%u | MODE=%u | AGE=%u ms > %u ms -> DOI BEACON MOI\n",
                    (unsigned int)su_v2b_beacon.frame_id,
                    (unsigned int)su_v2b_beacon.schedule_mode,
                    (unsigned int)age_ms,
                    (unsigned int)latest_ms
                );
                su_v2b_beacon.valid = false;
            }
        }

        if (SU_V2B_WaitBeacon(V2B_BEACON_WAIT_MS))
        {
            // Re-enter the validation path: a freshly received SINGLE beacon may
            // belong to the other pair and must never authorize VOICE/FEC here.
            continue;
        }

        if (su_v2b_consecutive_beacon_miss >= V2B_MAX_CONSECUTIVE_BEACON_MISS)
        {
            Serial.println(
                "[SU V2B FAILSAFE] MAT 3 BEACON LIEN TIEP -> DUNG TX PHIEN"
            );
            return false;
        }
    }
}


static void SU_V2B_WaitUplinkOffset()
{
    while (su_v2b_beacon.valid)
    {
        const uint32_t age_ms = millis() - su_v2b_beacon.rx_ms;
        if (age_ms >= SU_V2C2_VoiceOffsetMs())
        {
            break;
        }

        SU_WDT_Feed();
        delay(1);
    }
}


static bool SU_Stream_GuiVoiceBlock(
    SUStreamVoicePacket *block,
    uint8_t block_count,
    SUStreamFECPacket *fec_packet)
{
    if (block_count < 1 || block_count > V2B_VOICE_PER_BLOCK)
    {
        return false;
    }



    // V2B.1: FEC la BEST-EFFORT. FEC van duoc tao + gui + relay de DU co
    // kha nang khoi phuc 1 VOICE/group, nhung FEC KHONG con la dieu kien
    // de ACK VOICE. Neu parity mat, KHONG duoc retry lai ca block VOICE.
    const bool send_fec_best_effort =
        fec_packet != nullptr
        && fec_packet->valid;

    const uint32_t base_seq = block[0].seq;
    const uint8_t expected_mask = (uint8_t)((1U << block_count) - 1U);

    for (uint8_t lan = 0; lan <= MAX_TX_RETRY; lan++)
    {
        SU_WDT_Feed();

        if (!SU_V2B_EnsureFreshSlot())
        {
            break;
        }

        if (lan > 0)
        {
            // Chi tinh retry VOICE. Parity khong duoc phep keo ca block vao retry.
            su_oled_retransmissions += block_count;
        }

        SU_V2B_WaitUplinkOffset();

        const uint32_t slot_age_ms = millis() - su_v2b_beacon.rx_ms;
        const uint32_t tx_frame_id = su_v2b_beacon.frame_id;
        const bool single_now = SU_V2C2_ModeIsSingleForMe(su_v2b_beacon.schedule_mode);
        const bool fec_this_attempt =
            send_fec_best_effort
            && (lan == 0U)
            && su_v2b_beacon.fec_grant_pair == V2C_PAIR_INDEX
            && !(single_now && block_count > 1U);

        Serial.printf(
            "[SU V2B.1 UL] FRAME=%u | BASE=%u | COUNT=%u | FEC_BEST=%u | ATTEMPT=%u/%u | SLOT_AGE=%u ms | QUEUE=%u\n",
            (unsigned int)tx_frame_id,
            (unsigned int)base_seq,
            (unsigned int)block_count,
            fec_this_attempt ? 1U : 0U,
            (unsigned int)(lan + 1U),
            (unsigned int)(MAX_TX_RETRY + 1U),
            (unsigned int)slot_age_ms,
            (unsigned int)uxQueueMessagesWaiting(su_stream_frame_queue)
        );

        for (uint8_t i = 0; i < block_count; i++)
        {
            Serial.printf(
                "[SU V2B.1 TX] VOICE | FRAME_ID=%u | SEQ=%u | FRAME=%u | PROFILE=%s | LAST=%u | SLOT=%u/%u\n",
                (unsigned int)tx_frame_id,
                (unsigned int)block[i].seq,
                (unsigned int)block[i].frame_count,
                block[i].codec_hq ? "HQ" : "LQ",
                block[i].last_audio ? 1U : 0U,
                (unsigned int)(i + 1U),
                (unsigned int)block_count
            );

            if (su_diag_first_voice_tx_ms == 0)
            {
                su_diag_first_voice_tx_ms = millis();
                Serial.printf(
                    "[SU LAT] FIRST_VOICE_TX | FRAME_ID=%u | SEQ=%u | FROM_PTT=%u ms | FROM_READY=%u ms\n",
                    (unsigned int)su_v2b_beacon.frame_id,
                    (unsigned int)block[i].seq,
                    (unsigned int)SU_DeltaMs(su_diag_first_voice_tx_ms, thoi_diem_bat_dau_ghi_ms),
                    (unsigned int)SU_DeltaMs(su_diag_first_voice_tx_ms, su_diag_session_ready_ms)
                );
            }

            Dat_LED_SU(true);
            const bool tx_ok = Phat_GoiTin_LoRa(
                block[i].raw,
                SIZE_VOICE_PACKET_SU
            );
            Dat_LED_SU(false);

            if (!tx_ok)
            {
                Serial.printf(
                    "[SU V2B.1 WARN] TX DRIVER FAIL | VOICE SEQ=%u\n",
                    (unsigned int)block[i].seq
                );
            }

            if (i + 1U < block_count || fec_this_attempt)
            {
                delay(V2B_INTER_PACKET_GUARD_MS);
            }
        }

        if (fec_this_attempt)
        {
            while (su_v2b_beacon.valid && (uint32_t)(millis() - su_v2b_beacon.rx_ms) < SU_V2C2_FecOffsetMs())
            {
                SU_WDT_Feed();
                delay(1);
            }

            Serial.printf(
                "[SU V2C1 TX] FEC GRANTED | PAIR=%u | FEC BEST-EFFORT | FRAME_ID=%u | GROUP_START=%u | DATA=%u | HAS_LAST=%u | FINAL_FRAME=%u\n",
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned int)tx_frame_id,
                (unsigned int)fec_packet->group_start_seq,
                (unsigned int)fec_packet->data_count,
                fec_packet->has_last ? 1U : 0U,
                (unsigned int)fec_packet->final_frame_count
            );

            Dat_LED_SU(true);
            const bool fec_tx_ok = Phat_GoiTin_LoRa(
                const_cast<uint8_t *>(fec_packet->raw),
                SIZE_FEC_PACKET_SU
            );
            Dat_LED_SU(false);

            if (!fec_tx_ok)
            {
                Serial.printf(
                    "[SU V2C1 WARN] FEC TX FAIL | GROUP=%u | VOICE VAN TIEP TUC\n",
                    (unsigned int)fec_packet->group_start_seq
                );
            }
            // Best-effort: mot khi da duoc grant va da attempt TX, parity nay het vong doi.
            fec_packet->valid = false;
        }

        // ACK nam trong beacon frame ke tiep. V2C2.2 chi cho phep mat DUNG
        // mot beacon: SU du doan 1 frame de khong tao lo 295 ms o DU.
        const SUV2BBeaconState frame_used = su_v2b_beacon;
        su_v2b_beacon.valid = false;

        const bool got_next_beacon = SU_V2C2_WaitNextBeaconOrHoldover(frame_used);
        if (!got_next_beacon)
        {
            break;
        }

        // Neu frame truoc da duoc gui theo holdover, beacon that dau tien sau
        // holdover phai xac nhan no truc tiep hoac bang cumulative base advance.
        if (!SU_V2C2_ProvisionalAckConfirmed())
        {
            break;
        }

        // rBS keeps ACK state in natural 2-packet groups even in SINGLE.
        // Therefore a SINGLE frame may send only one member of that group:
        //   seq even -> expect bit0, seq odd -> expect bit1.
        const uint32_t ack_group_base = block[0].seq & ~1UL;
        uint8_t relevant_mask = 0;
        for (uint8_t i = 0; i < block_count; ++i)
        {
            const uint8_t ack_slot = (uint8_t)(block[i].seq - ack_group_base);
            if (ack_slot < 2U) relevant_mask |= (uint8_t)(1U << ack_slot);
        }
        uint8_t expected_ack_count = 2U;
        if (block_count == 1U && block[0].last_audio && ((block[0].seq & 1U) == 0U))
            expected_ack_count = 1U;

        if (su_v2b_beacon.synthetic_holdover)
        {
            // ACK itself was in the lost beacon. Keep exactly one provisional
            // block and continue with NEW audio in this virtual frame. The next
            // real beacon must prove delivery; otherwise fail-safe stops session.
            su_v2c2_provisional_ack.active = true;
            su_v2c2_provisional_ack.base_seq = ack_group_base;
            su_v2c2_provisional_ack.relevant_mask = relevant_mask;
            su_v2c2_provisional_ack.expected_count = expected_ack_count;
            Serial.printf(
                "[SU V2C2.2 HOLDOVER ACK PENDING] BASE=%u | MASK=0x%02X | NEXT_FRAME=%u\n",
                (unsigned int)ack_group_base,
                (unsigned int)relevant_mask,
                (unsigned int)su_v2b_beacon.frame_id
            );
            return true;
        }

        const bool ack_identity_ok =
            su_v2b_beacon.ack_base_seq == ack_group_base
            && su_v2b_beacon.ack_count == expected_ack_count;

        const bool voice_ok =
            ack_identity_ok
            && ((su_v2b_beacon.ack_bitmap & relevant_mask) == relevant_mask);

        if (voice_ok)
        {
            Serial.printf(
                "[SU V2B.1 ACK OK] RX_FRAME=%u | BASE=%u | BITMAP=0x%02X | FEC_BEST=%u\n",
                (unsigned int)su_v2b_beacon.frame_id,
                (unsigned int)ack_group_base,
                (unsigned int)su_v2b_beacon.ack_bitmap,
                send_fec_best_effort ? 1U : 0U
            );
            return true;
        }

        Serial.printf(
            "[SU V2B.1 RETRY VOICE] RX_FRAME=%u | BASE=%u | ACK_BASE=%u | ACK_COUNT=%u | BITMAP=0x%02X | EXPECT=0x%02X\n",
            (unsigned int)su_v2b_beacon.frame_id,
            (unsigned int)ack_group_base,
            (unsigned int)su_v2b_beacon.ack_base_seq,
            (unsigned int)su_v2b_beacon.ack_count,
            (unsigned int)su_v2b_beacon.ack_bitmap,
            (unsigned int)relevant_mask
        );

        // Beacon vua nhan dong thoi la sync cua frame retry. Vong sau chi
        // retry VOICE; FEC da la best-effort nen khong phat lai.
    }

    su_oled_fail_packets += block_count;

    Serial.printf(
        "[SU V2B.1 BLOCK FAIL] BASE=%u | COUNT=%u\n",
        (unsigned int)base_seq,
        (unsigned int)block_count
    );

    return false;
}


// FEC partial o CUOI cau noi co the khong trung frame VOICE dang gui. V2B.1
// cho phep mot frame tail chi gui FEC best-effort, cho qua downlink slot roi
// moi AUDIO_END. FEC tail khong bao gio lam fail phien voice.
static void SU_V2B1_GuiFECTailBestEffort(const SUStreamFECPacket &fec_packet)
{
    if (!fec_packet.valid) return;

    // Toi da cho 3 beacon de den grant cua pair minh; parity la best-effort.
    for (uint8_t tries=0; tries<3; ++tries)
    {
        if (!SU_V2B_EnsureFreshSlot()) break;
        if (su_v2b_beacon.fec_grant_pair == V2C_PAIR_INDEX)
        {
            while ((uint32_t)(millis() - su_v2b_beacon.rx_ms) < SU_V2C2_FecOffsetMs())
            {
                SU_WDT_Feed(); delay(1);
            }
            Serial.printf(
                "[SU V2C1 FEC TAIL] PAIR=%u | FRAME=%u | GROUP=%u | DATA=%u\n",
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned int)su_v2b_beacon.frame_id,
                (unsigned int)fec_packet.group_start_seq,
                (unsigned int)fec_packet.data_count
            );
            Dat_LED_SU(true);
            (void)Phat_GoiTin_LoRa(const_cast<uint8_t *>(fec_packet.raw), SIZE_FEC_PACKET_SU);
            Dat_LED_SU(false);
            su_v2b_beacon.valid=false;
            (void)SU_V2B_WaitBeacon(V2B_BEACON_WAIT_MS);
            return;
        }
        su_v2b_beacon.valid=false;
    }
    Serial.println("[SU V2C1 FEC TAIL] KHONG CO GRANT -> BO QUA PARITY TAIL");
}

static bool SU_Stream_ThietLapSession(uint64_t &session_id_tx)
{
    session_id_tx = Tao_Session_Moi();
    HMI_SU_Dat_Session(session_id_tx);
    SU_Jam52_Prepare(session_id_tx);
    SU_Jam53B2_Prepare(session_id_tx);

    uint8_t goi_session[SIZE_SESSION_PACKET_SU];
    Tao_GoiTin_SessionStart(goi_session);

    bool session_ready = false;
    bool session_fail_remote = false;

    // Admission bat dau o profile an toan. Neu day la pair dau tien, ngay khi
    // scheduler xac nhan SINGLE cua minh, capture se chuyen HQ. Neu la JOIN,
    // LQ da co san trong queue de vao DUAL khong bi nghen 8-frame HQ backlog.
    su_stream_encode_hq = false;

    // Xoa beacon state cu cua session truoc; network-busy lease van nam trong
    // phat_lora.cpp va se duoc cap nhat lien tuc boi beacon IDLE schedule v7.
    SU_V2B_ResetSync();

    bool tdma_locked = false;
    bool da_gui_bootstrap = false;

    Dat_LED_SU(true);

    for (
        uint8_t lan = 0;
        lan <= SESSION_REQUEST_RETRY;
        lan++
    )
    {
        SU_WDT_Feed();

        if (lan == 0U && V2C_BOOTSTRAP_SESSION_DELAY_MS > 0U)
        {
            delay(V2C_BOOTSTRAP_SESSION_DELAY_MS);
        }

        bool co_beacon_cho_luot = false;
        const bool network_hint = V2C1_NetworkBusy();

        if (tdma_locked || network_hint)
        {
            const uint32_t wait_ms = V2C46_TDMA_BEACON_WAIT_MS;
            Serial.printf(
                "[SU V2C4.6 ADMISSION] PAIR=%u | TRY=%u | TDMA_LOCK=%u | BUSY_HINT=%u | WAIT_BEACON=%u ms\n",
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned int)(lan + 1U),
                tdma_locked ? 1U : 0U,
                network_hint ? 1U : 0U,
                (unsigned int)wait_ms
            );

            if (SU_V2B_WaitBeacon(wait_ms))
            {
                co_beacon_cho_luot = true;
                tdma_locked = true;
            }
            else
            {
                // Mot khi da lock TDMA thi retry khong bao gio duoc TX tu do.
                if (tdma_locked || network_hint)
                {
                    Serial.printf(
                        "[SU V2C4.6 TDMA HOLD] PAIR=%u | TRY=%u | KHONG CO BEACON -> BO LUOT, KHONG BOOTSTRAP\n",
                        (unsigned int)V2C_PAIR_INDEX,
                        (unsigned int)(lan + 1U)
                    );
                    continue;
                }
            }
        }
        else
        {
            // Cache IDLE co the stale dung luc PTT. Nghe hon 1 superframe de
            // phan biet that su idle voi pair kia dang noi. Day la fallback;
            // khi idle beacon version 7 duoc poll dung, nhanh nay hiem xay ra.
            Serial.printf(
                "[SU V2C4.6 ADMISSION LISTEN] PAIR=%u | TRY=%u | CACHE_IDLE -> NGHE %u ms TRUOC BOOTSTRAP\n",
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned int)(lan + 1U),
                (unsigned int)V2C46_ADMISSION_LISTEN_MS
            );

            if (SU_V2B_WaitBeacon(V2C46_ADMISSION_LISTEN_MS))
            {
                co_beacon_cho_luot = true;
                tdma_locked = true;
                Serial.printf(
                    "[SU V2C4.6 TDMA LOCK] PAIR=%u | FRAME=%u | MODE=%u | NEXT=%u\n",
                    (unsigned int)V2C_PAIR_INDEX,
                    (unsigned int)su_v2b_beacon.frame_id,
                    (unsigned int)su_v2b_beacon.schedule_mode,
                    (unsigned int)su_v2b_beacon.transition_target
                );
            }
        }

        if (tdma_locked)
        {
            if (!co_beacon_cho_luot)
            {
                if (!SU_V2B_WaitBeacon(V2C46_TDMA_BEACON_WAIT_MS))
                {
                    Serial.printf(
                        "[SU V2C4.6 TDMA HOLD] PAIR=%u | TRY=%u | MAT BEACON -> KHONG TX\n",
                        (unsigned int)V2C_PAIR_INDEX,
                        (unsigned int)(lan + 1U)
                    );
                    continue;
                }
            }

            // PREPARE->DUAL doi capture sang LQ ngay tai beacon PREPARE.
            su_stream_encode_hq = SU_V2C46_CaptureShouldUseHQ();

            uint32_t age = millis() - su_v2b_beacon.rx_ms;
            if (age > SU_V2C2_SessionLatestMs())
            {
                su_v2b_beacon.valid = false;
                if (!SU_V2B_WaitBeacon(V2C46_TDMA_BEACON_WAIT_MS))
                {
                    Serial.printf(
                        "[SU V2C4.6 TDMA HOLD] PAIR=%u | TRY=%u | BEACON QUA HAN -> KHONG TX\n",
                        (unsigned int)V2C_PAIR_INDEX,
                        (unsigned int)(lan + 1U)
                    );
                    continue;
                }
                su_stream_encode_hq = SU_V2C46_CaptureShouldUseHQ();
            }

            const uint32_t offset_ms = SU_V2C2_SessionOffsetMs();
            const uint32_t latest_ms = SU_V2C2_SessionLatestMs();
            const uint8_t active_pair = SU_V2C2_ActivePairForMode(su_v2b_beacon.schedule_mode);
            const char *slot_role = active_pair == 0U
                ? "DUAL"
                : (active_pair == V2C_PAIR_INDEX ? "ACTIVE" : "JOIN");

            while ((uint32_t)(millis() - su_v2b_beacon.rx_ms) < offset_ms)
            {
                SU_WDT_Feed();
                delay(1);
            }

            const uint32_t age_tx = millis() - su_v2b_beacon.rx_ms;
            if (age_tx > latest_ms)
            {
                Serial.printf(
                    "[SU V2C4.6 SESSION SLOT MISS] PAIR=%u | FRAME=%u | ROLE=%s | AGE=%u > %u -> DOI BEACON\n",
                    (unsigned int)V2C_PAIR_INDEX,
                    (unsigned int)su_v2b_beacon.frame_id,
                    slot_role,
                    (unsigned int)age_tx,
                    (unsigned int)latest_ms
                );
                continue;
            }

            Serial.printf(
                "[SU V2C4.6 SESSION SLOT] PAIR=%u | FRAME=%u | MODE=%u | NEXT=%u | ROLE=%s | OFFSET=%u ms | LATEST=%u ms | TDMA_LOCK=1 | PROFILE_CAPTURE=%s | JAM=OFF\n",
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned int)su_v2b_beacon.frame_id,
                (unsigned int)su_v2b_beacon.schedule_mode,
                (unsigned int)su_v2b_beacon.transition_target,
                slot_role,
                (unsigned int)offset_ms,
                (unsigned int)latest_ms,
                su_stream_encode_hq ? "HQ" : "LQ"
            );
        }
        else
        {
            // Khong thay beacon sau admission listen => rBS dang IDLE that su.
            // Bootstrap chi duoc phep trong trang thai nay. Retry sau do se nghe
            // lai truoc; neu rBS da bat scheduler thi se TDMA-lock ngay.
            da_gui_bootstrap = true;
            su_stream_encode_hq = false;
            Serial.printf(
                "[SU V2C4.6 BOOTSTRAP SAFE] PAIR=%u | TRY=%u | KHONG CO BEACON %u ms -> SESSION_START TU DO 1 LAN\n",
                (unsigned int)V2C_PAIR_INDEX,
                (unsigned int)(lan + 1U),
                (unsigned int)V2C46_ADMISSION_LISTEN_MS
            );
        }

        Serial.printf(
            "[SU V2C4.6 SESSION] PAIR=%u | REQUEST %u/%u | SESSION=%016llX | TDMA_LOCK=%u | BOOTSTRAP_USED=%u\n",
            (unsigned int)V2C_PAIR_INDEX,
            (unsigned int)(lan + 1),
            (unsigned int)(SESSION_REQUEST_RETRY + 1),
            (unsigned long long)session_id_tx,
            tdma_locked ? 1U : 0U,
            da_gui_bootstrap ? 1U : 0U
        );

        Phat_GoiTin_LoRa(
            goi_session,
            sizeof(goi_session)
        );

        session_fail_remote = false;

        if (
            Cho_SESSION_READY_RBS(
                SESSION_TIMEOUT_SU_MS,
                session_id_tx,
                session_fail_remote
            )
        )
        {
            session_ready = true;
            break;
        }

        if (session_fail_remote)
        {
            break;
        }

        // Neu request da gui trong TDMA, lock giu nguyen qua moi retry.
        // Neu request bootstrap timeout, lan sau bat buoc admission-listen lai.
    }

    Dat_LED_SU(false);
    return session_ready;
}


static void SU_Stream_ChayMotPhien()
{
    HMI_SU_BatDau_CauMoi();
    Reset_ThongKe_OLED_SU();

    thoi_diem_bat_dau_ghi_ms = millis();
    e2e_ptt_release_ms = 0;

    su_diag_first_encoded_ms = 0;
    su_diag_queue_max_frames = 0;
    su_diag_session_ready_ms = 0;
    su_diag_first_voice_build_ms = 0;
    su_diag_first_voice_tx_ms = 0;
    Speex_SU_ResetDiag();

    Serial.printf(
        "[SU LAT] PTT_SESSION_START | T=%u ms | PAIR=%u\n",
        (unsigned int)thoi_diem_bat_dau_ghi_ms,
        (unsigned int)V2C_PAIR_INDEX
    );

    trang_thai = DANG_GHI_AM;

    Ve_GiaoDien_OLED(
        0,
        true
    );

    // V2C4.6: truoc khi biet scheduler, encode LQ an toan de neu pair kia dang
    // active thi khong tao HQ backlog lam nghen ngay luc COMMIT DUAL.
    su_stream_encode_hq = false;

    if (!SU_Stream_BatDauThu())
    {
        Serial.println(
            "[SU STREAM ERROR] KHONG KHOI DONG DUOC TASK THU AM"
        );
        trang_thai = NGHI_NGOI;
        su_stream_block_until_ptt_release = true;
        return;
    }

    Serial.println(
        ">> STREAM V2C5.2: ADMISSION_LOCK + JAM_LEASE_SIM + QUEUE_48 | SINGLE=HQ3950 | DUAL=LQ2150 | RF_JAM=OFF"
    );

    uint64_t session_id_tx = 0;

    if (!SU_Stream_ThietLapSession(session_id_tx))
    {
        Serial.println(
            "[SU STREAM SESSION FAIL] DUNG CAPTURE, BO AUDIO DANG CHO"
        );
        SU_Jam52_Stop("SESSION_FAIL");
        SU_Jam53B2_Stop("SESSION_FAIL");

        SU_Stream_DungThu();
        xQueueReset(su_stream_frame_queue);
        Bao_Loi_Session_SU();
        trang_thai = NGHI_NGOI;
        su_stream_block_until_ptt_release = true;
        Bat_CheDo_RX_LoRa();
        return;
    }

    su_diag_session_ready_ms = millis();
    Serial.printf(
        "[SU LAT] SESSION_READY | T=%u ms | FROM_PTT=%u ms | QUEUE=%u\n",
        (unsigned int)su_diag_session_ready_ms,
        (unsigned int)SU_DeltaMs(su_diag_session_ready_ms, thoi_diem_bat_dau_ghi_ms),
        (unsigned int)uxQueueMessagesWaiting(su_stream_frame_queue)
    );

    Serial.printf(
        "[SU V2C5.2 SESSION READY] PAIR=%u | SU=0x%02X DU=0x%02X | SESSION=%016llX | SINGLE=300ms/HQ8+7 | DUAL=300ms/LQ15 | SCHED=7 PREPARE_COMMIT | HOLDOVER=1 | FEC=8+1 | JAM_LEASE_SIM=ON | RF_JAM=OFF\n",
        (unsigned int)V2C_PAIR_INDEX,
        (unsigned int)V2C_SU_ID,
        (unsigned int)V2C_DU_ID,
        (unsigned long long)session_id_tx
    );

    SU_V2B_ResetSync();
    // Capture continues in its own task while we wait. Fresh beacon sau READY
    // chot profile ban dau; neu miss thi giu LQ fail-safe den beacon ke tiep.
    if (SU_V2B_WaitBeacon(V2B_BEACON_WAIT_MS))
        su_stream_encode_hq = SU_V2C46_CaptureShouldUseHQ();
    else
        su_stream_encode_hq = false;
    trang_thai = DANG_PHAT_SONG;

    SUStreamFECState fec_st;
    SU_Stream_ResetFEC(fec_st);

    // V2B.1 tach vong doi FEC khoi block VOICE. Khi du 4 DATA, parity duoc
    // snapshot ngay va fec_st reset de packet VOICE tiep theo vao group moi.
    // Moi superframe chi ghep TOI DA 1 parity best-effort sau VOICE burst.
    SUStreamFECPacket fec_ready;
    memset(&fec_ready, 0, sizeof(fec_ready));
    bool fec_ready_valid = false;

    SUStreamVoicePacket voice_block[V2B_VOICE_PER_BLOCK];
    uint8_t voice_block_count = 0;

    // V2C4.6: profile da duoc chot boi beacon sau READY. Khong ep HQ o day,
    // neu JOIN/DUAL hoac beacon chua chac chan thi LQ phai duoc giu.

    bool gui_that_bai = false;

    // V2C56G2B_SINGLE_BACKLOG_LQ_RESCUE
    // Chi la audio-profile rescue o SINGLE. KHONG lien quan friendly-jamming.
    bool v2c56g2b_lq_rescue = false;

    while (!gui_that_bai)
    {
        SU_WDT_Feed();

        // V2C4.6 + V2C56G2B:
        // - Scheduler van quyet dinh HQ/LQ theo SINGLE/PREPARE/DUAL nhu cu.
        // - Rieng SINGLE cua chinh Pair: neu queue cao, tam capture LQ de tao
        //   headroom. SINGLE gui 2 packet/300ms; 2 LQ packet co the mang 600ms
        //   audio, nen backlog co co hoi giam ma KHONG drop frame cu.
        // - Khi queue xuong thap, tu dong quay lai HQ.
        // - KHONG sua packet, TDMA, AES/FEC hay friendly-jamming.
        if (su_v2b_beacon.valid)
        {
            const bool scheduler_hq = SU_V2C46_CaptureShouldUseHQ();
            const bool single_for_me_rescue =
                SU_V2C2_ModeIsSingleForMe(su_v2b_beacon.schedule_mode);
            const UBaseType_t q_now =
                uxQueueMessagesWaiting(su_stream_frame_queue);

            if (scheduler_hq && single_for_me_rescue)
            {
                if (!v2c56g2b_lq_rescue && q_now >= 24U)
                {
                    v2c56g2b_lq_rescue = true;
                    Serial.printf(
                        "[SU V2C56G2B LQ RESCUE ON] QUEUE=%u | HIGH=24 | LOW=8 | MODE=%u\n",
                        (unsigned int)q_now,
                        (unsigned int)su_v2b_beacon.schedule_mode
                    );
                }
                else if (v2c56g2b_lq_rescue && q_now <= 8U)
                {
                    v2c56g2b_lq_rescue = false;
                    Serial.printf(
                        "[SU V2C56G2B LQ RESCUE OFF] QUEUE=%u | REASON=RECOVERED\n",
                        (unsigned int)q_now
                    );
                }

                su_stream_encode_hq = !v2c56g2b_lq_rescue;
            }
            else
            {
                if (v2c56g2b_lq_rescue)
                {
                    Serial.printf(
                        "[SU V2C56G2B LQ RESCUE OFF] QUEUE=%u | REASON=SCHEDULER_LQ_OR_TRANSITION | MODE=%u | NEXT=%u\n",
                        (unsigned int)q_now,
                        (unsigned int)su_v2b_beacon.schedule_mode,
                        (unsigned int)su_v2b_beacon.transition_target
                    );
                }

                v2c56g2b_lq_rescue = false;
                su_stream_encode_hq = scheduler_hq;
            }
        }

        // V2C4 packetization theo CHINH superframe 300 ms:
        // - SINGLE binh thuong: HQ 8+7 frame = 300 ms.
        // - DUAL: 1 LQ packet toi da 15 frame = 300 ms / pair.
        // - DUAL->SINGLE neu next SEQ le: 1 frame bridge LQ 15 frame de hoan tat
        //   ACK pair cu, sau do frame ke tiep moi vao HQ 8+7. Khong tao lo audio.
        const bool single_for_me = SU_V2C2_ModeIsSingleForMe(su_v2b_beacon.schedule_mode);
        // Profile duoc khoa theo BLOCK. Sau packet HQ thu nhat, next SEQ se thanh le;
        // khong duoc hieu nham do la bridge LQ cua mot transition moi.
        bool codec_hq_build = false;
        if (voice_block_count > 0U)
        {
            codec_hq_build = voice_block[0].codec_hq;
        }
        else
        {
            const bool ack_pair_bridge_lq =
                single_for_me && ((Lay_Seq_Voice_TiepTheo() & 1U) != 0U);
            codec_hq_build = single_for_me && !ack_pair_bridge_lq;
        }
        const uint8_t block_target_now = SU_V2C2_BlockTarget(codec_hq_build);

        uint8_t frame_cap = V2C3_LQ_FRAMES_PER_VOICE;
        if (codec_hq_build)
            frame_cap = (voice_block_count == 0U) ? 8U : 7U;

        bool packet_ready = false;
        if (!SU_Stream_TaoVoicePacketTuQueue(
                codec_hq_build,
                frame_cap,
                fec_st,
                voice_block[voice_block_count],
                packet_ready))
        {
            gui_that_bai = true;
            break;
        }

        if (!packet_ready)
        {
            if (voice_block_count > 0U)
            {
                // Da co audio cho frame nay: GUI NGAY, khong doi frame thu 2 roi
                // lam beacon gia. Steady-state pre-encode se thuong co du 8+7.
                if (!SU_Stream_GuiVoiceBlock(voice_block, voice_block_count, nullptr))
                    gui_that_bai = true;
                voice_block_count = 0;
                continue;
            }
            if (su_stream_capture_done && uxQueueMessagesWaiting(su_stream_frame_queue) == 0)
                break;
            delay(1);
            continue;
        }

        const bool la_packet_cuoi = voice_block[voice_block_count].last_audio;
        voice_block_count++;

        // Full FEC group 4+1: snapshot parity NGAY khi packet vua tao lam group
        // dat 4 DATA. Nhu vay block 3 VOICE co the di qua ranh gioi FEC ma
        // khong lam data_count tang thanh 5/6.
        if (fec_st.data_count >= FEC_DATA_PER_GROUP)
        {
            if (fec_ready_valid)
            {
                // V2C0 block toi da 2 VOICE va FEC_DATA_PER_GROUP=8 nen binh thuong
                // khong the co >1 full parity trong cung block.
                Serial.println(
                    "[SU V2B.1 ERROR] CO >1 FEC FULL TRONG 1 BLOCK -> DUNG PHIEN"
                );
                gui_that_bai = true;
                break;
            }

            if (SU_Stream_TaoFECPacket(fec_st, fec_ready))
            {
                fec_ready_valid = true;
            }
            SU_Stream_ResetFEC(fec_st);
        }

        const uint8_t block_target = block_target_now;
        const bool block_day = voice_block_count >= block_target;

        if (block_day || la_packet_cuoi)
        {
            // Neu day la packet cuoi va group hien tai chi la partial, co the
            // ghep parity partial vao frame nay NEU chua co full parity cho frame.
            if (
                la_packet_cuoi
                && fec_st.data_count > 0
                && fec_st.has_last
                && !fec_ready_valid
            )
            {
                if (SU_Stream_TaoFECPacket(fec_st, fec_ready))
                {
                    fec_ready_valid = true;
                    SU_Stream_ResetFEC(fec_st);
                }
            }

            if (!SU_Stream_GuiVoiceBlock(
                    voice_block,
                    voice_block_count,
                    fec_ready_valid ? &fec_ready : nullptr))
            {
                gui_that_bai = true;
                break;
            }

            voice_block_count = 0;
            if (fec_ready_valid && !fec_ready.valid)
            {
                memset(&fec_ready, 0, sizeof(fec_ready));
                fec_ready_valid = false;
            }
        }
    }

    // Edge hiem: block VOICE con lai chua flush do capture DONE roi dung tai
    // ranh gioi packet. Flush VOICE truoc, van uu tien VOICE hon FEC.
    if (!gui_that_bai && voice_block_count > 0)
    {
        if (!SU_Stream_GuiVoiceBlock(
                voice_block,
                voice_block_count,
                fec_ready_valid ? &fec_ready : nullptr))
        {
            gui_that_bai = true;
        }

        voice_block_count = 0;
        if (fec_ready_valid && !fec_ready.valid)
        {
            memset(&fec_ready, 0, sizeof(fec_ready));
            fec_ready_valid = false;
        }
    }

    // Full parity co the da san sang nhung chua dung frame duoc grant.
    // Cho toi da 3 beacon de gui trong dedicated FEC slot truoc khi AUDIO_END.
    if (!gui_that_bai && fec_ready_valid && fec_ready.valid)
    {
        SU_V2B1_GuiFECTailBestEffort(fec_ready);
        memset(&fec_ready, 0, sizeof(fec_ready));
        fec_ready_valid = false;
    }

    // Neu cau noi ket thuc ngay sau khi mot full parity da duoc ghep vao block,
    // fec_st co the con mot partial group cuoi. Gui no trong mot tail superframe
    // best-effort; KHONG retry VOICE va KHONG lam phien fail neu parity mat.
    if (!gui_that_bai && fec_st.data_count > 0)
    {
        SUStreamFECPacket tail_fec;
        if (SU_Stream_TaoFECPacket(fec_st, tail_fec))
        {
            SU_V2B1_GuiFECTailBestEffort(tail_fec);
        }
        SU_Stream_ResetFEC(fec_st);
    }

    SU_Stream_DungThu();

    uint32_t hold_ms =
        su_stream_ptt_release_ms != 0
        ? (su_stream_ptt_release_ms - thoi_diem_bat_dau_ghi_ms)
        : (millis() - thoi_diem_bat_dau_ghi_ms);

    Serial.printf(
        "[SU STREAM RECORD] HOLD_MS=%u | FRAME=%u | AUDIO_MS=%u | QUEUE_DROP_OLD=%u\n",
        (unsigned int)hold_ms,
        (unsigned int)tong_so_khung_da_ghi,
        (unsigned int)(tong_so_khung_da_ghi * 20U),
        (unsigned int)su_stream_queue_overflow
    );

    SpeexAudioDiagSU audio_diag = Speex_SU_LayDiag();
    Serial.printf(
        "[SU LAT SUMMARY] PTT_TO_ENCODE=%u ms | PTT_TO_BUILD=%u ms | PTT_TO_TX=%u ms | READY_TO_TX=%u ms | QMAX=%u frames (%u ms)\n",
        (unsigned int)SU_DeltaMs(su_diag_first_encoded_ms, thoi_diem_bat_dau_ghi_ms),
        (unsigned int)SU_DeltaMs(su_diag_first_voice_build_ms, thoi_diem_bat_dau_ghi_ms),
        (unsigned int)SU_DeltaMs(su_diag_first_voice_tx_ms, thoi_diem_bat_dau_ghi_ms),
        (unsigned int)SU_DeltaMs(su_diag_first_voice_tx_ms, su_diag_session_ready_ms),
        (unsigned int)su_diag_queue_max_frames,
        (unsigned int)(su_diag_queue_max_frames * 20U)
    );
    Serial.printf(
        "[SU AUDIO SUMMARY] PROFILE=ADAPTIVE | SAMPLES=%u | PEAK_PRE=%d | PEAK_POST=%d | LIMIT=%d | CLIPPED=%u | SPEEX_SIZE_ERR=%u | HQ=3950/10B | LQ=2150/6B | HPF~150Hz | GAIN=3.2 | PREEMPH=0.08 | COMPLEXITY=10\n",
        (unsigned int)audio_diag.sample_count,
        (int)audio_diag.peak_pre_gain,
        (int)audio_diag.peak_post_gain,
        (int)15000,
        (unsigned int)audio_diag.clip_count,
        (unsigned int)audio_diag.frame_size_error_count
    );

    // V2C5.2: het luong VOICE cuc bo -> thu hoi lease truoc AUDIO_END.
    // START/STOP RF chua duoc trien khai; day chi la state machine gia lap.
    SU_Jam52_Stop(gui_that_bai ? "ARQ_FAIL" : "LOCAL_AUDIO_END");
    SU_Jam53B2_Stop(gui_that_bai ? "ARQ_FAIL" : "LOCAL_AUDIO_END");

    // V2C2.4: PLAY_STARTED chi con la diagnostic legacy.
    // Khong block sau AUDIO_END nua vi streaming DU da phat tu truoc END;
    // viec cho report nay tung tao TIMEOUT vo ich sau khi cau thoai da xong.
    if (!gui_that_bai)
    {
        uint8_t goi_audio_end[5] =
        {
            ID_TRAM_DU_CTRL,
            ID_TRAM_SU_CTRL,
            TYPE_AUDIO_END_SU,
            0x00,
            0x00
        };

        Dat_LED_SU(true);
        Phat_GoiTin_LoRa(
            goi_audio_end,
            sizeof(goi_audio_end)
        );
        Dat_LED_SU(false);

        Serial.println(
            "[SU STREAM CTRL] AUDIO_END -> rBS"
        );

        // Khong cho PLAY_STARTED dong bo nua. Neu report legacy den sau,
        // main RX co the bo qua; no khong con nam tren critical path audio.
        Serial.println(
            "[SU STREAM] PLAY_REPORT_AFTER_END=NONBLOCKING_DISABLED | E2E_LEGACY=DISABLED"
        );
    }
    else
    {
        Serial.println(
            "[SU STREAM] DUNG PHIEN DO ARQ FAIL"
        );
    }

    su_telemetry_post_pending = true;
    moc_gui_vi_tri_su_tiep_theo_ms = millis();

    // Neu cham gioi han MAX_KHUNG_THOAI trong khi nguoi dung van giu PTT,
    // khong tu mo ngay mot session moi. Cho nha PTT roi moi cho bat dau lai.
    if (su_stream_capture_forced_stop && Nut_PTT_Dang_Bam_SU())
    {
        su_stream_block_until_ptt_release = true;
    }

    trang_thai = NGHI_NGOI;

    HienThi_SU_KetQua(
        su_oled_voice_packets,
        su_oled_retransmissions,
        su_oled_fail_packets,
        false,
        0
    );

    Bat_CheDo_RX_LoRa();
}


static void SU_Stream_LoopV1()
{
    SU_WDT_SelfTest_Check();
    SU_WDT_Feed();

    bool nut_dang_bam = Nut_PTT_Dang_Bam_SU();

    if (su_stream_block_until_ptt_release)
    {
        if (!nut_dang_bam)
        {
            su_stream_block_until_ptt_release = false;
        }

        delay(5);
        return;
    }

    if (
        nut_dang_bam
        && trang_thai == NGHI_NGOI
    )
    {
        SU_Stream_ChayMotPhien();
        return;
    }

    if (
        !nut_dang_bam
        && trang_thai == NGHI_NGOI
    )
    {
        CapNhat_LED_PhanHoi_SU();

        uint64_t ma_phien_hmi = HMI_SU_Lay_Session();
        if (ma_phien_hmi == 0)
        {
            uint8_t dummy_response = 0;
            (void)KiemTra_USER_RESPONSE_RBS(0, dummy_response);
        }
        if (ma_phien_hmi != 0)
        {
            uint8_t ma_phan_hoi = 0;
            if (KiemTra_USER_RESPONSE_RBS(ma_phien_hmi, ma_phan_hoi))
            {
                Dat_PhanHoi_SU(ma_phan_hoi);
                Gui_USER_CONFIRM_RBS(ma_phien_hmi, ma_phan_hoi);
            }
        }

        XuLy_BaoCao_ViTri_DinhKy_SU();
        SU_WDT_Feed();
        delay(5);
        return;
    }

    delay(1);
}


// ========================================================
// SETUP
// ========================================================

void setup()
{
    Serial.begin(
        115200
    );

    delay(100);
    SU_WDT_Init();
    SU_WDT_Feed();

    // ====================================================
    // CRYPTO FINAL SESSION-ID V1
    // Reserve persistent SESSION_ID block tai BOOT.
    // NVS write khong nam tren critical path PTT.
    // ====================================================
    if (!KhoiTao_Session_ID_BenVung())
    {
        Serial.println(
            "[SU CRYPTO FATAL] KHOI TAO SESSION-ID THAT BAI -> REBOOT"
        );
        Serial.flush();
        delay(1000);
        ESP.restart();
        return;
    }

    SU_WDT_Feed();

    // GPS NEO-6M doc lien tuc trong task rieng, khong block audio/PTT.
    KhoiTao_GPS_SU();
    SU_WDT_Feed();

    moc_gui_vi_tri_su_tiep_theo_ms =
        millis() + TRE_BAO_CAO_VI_TRI_SU_LUC_KHOI_DONG_MS;


    // Nut PTT + LED duoc khoi tao trong module HMI rieng.
    KhoiTao_HMI_SU();
    SU_WDT_Feed();


    // ====================================================
    // OLED
    // ====================================================

    Wire.begin(
        4,
        5
    );


    KhoiTao_OLED();
    SU_WDT_Feed();


    Ve_GiaoDien_OLED(
        0,
        false
    );


    // ====================================================
    // AUDIO ADC
    // ====================================================

    KhoiTao_ThuAm_DMA();
    SU_WDT_Feed();


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
    SU_WDT_Feed();


    // ====================================================
    // LORA
    // ====================================================

    KhoiTao_LoRa();
    SU_WDT_Feed();


    // ====================================================
    // SU AUDIO PSRAM V1
    // PSRAM-FIRST, SAFE FALLBACK
    // ====================================================

    const size_t psram_total =
        heap_caps_get_total_size(
            MALLOC_CAP_SPIRAM
        );

    const size_t psram_free_before =
        heap_caps_get_free_size(
            MALLOC_CAP_SPIRAM
        );

    const size_t internal_free_before =
        heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        );

    Serial.printf(
        "[SU PSRAM] TOTAL=%u | FREE BEFORE=%u | AUDIO NEED=%u bytes\n",
        (unsigned int)psram_total,
        (unsigned int)psram_free_before,
        (unsigned int)su_audio_buffer_bytes
    );

    if (
        psram_total > 0
        &&
        psram_free_before >= su_audio_buffer_bytes
    )
    {
        Kho_Chua_AmThanh =
            (uint8_t *)
            heap_caps_malloc(
                su_audio_buffer_bytes,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
            );

        if (Kho_Chua_AmThanh != NULL)
        {
            su_audio_buffer_in_psram = true;
        }
    }

    // Fallback an toan:
    // Neu SU thuc te khong co PSRAM, hoac PlatformIO chua expose PSRAM,
    // van giu he thong chay nhu baseline cu de test/diagnose.
    if (Kho_Chua_AmThanh == NULL)
    {
        Serial.println(
            "[SU PSRAM WARN] KHONG CAP PHAT DUOC PSRAM -> FALLBACK INTERNAL RAM"
        );

        Kho_Chua_AmThanh =
            (uint8_t *)
            heap_caps_malloc(
                su_audio_buffer_bytes,
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            );

        su_audio_buffer_in_psram = false;
    }

    if (Kho_Chua_AmThanh == NULL)
    {
        Serial.println(
            "[SU ERROR] CAP PHAT AUDIO BUFFER THAT BAI!"
        );

        Serial.printf(
            "[SU MEM] PSRAM_FREE=%u | INTERNAL_FREE=%u\n",
            (unsigned int)
            heap_caps_get_free_size(
                MALLOC_CAP_SPIRAM
            ),
            (unsigned int)
            heap_caps_get_free_size(
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            )
        );

        Serial.println(
            "[SU SELF-HEAL] audio buffer init fail -> reboot sau 1s"
        );

        delay(1000);
        ESP.restart();
    }

    Serial.printf(
        "[SU AUDIO BUFFER] %s | SIZE=%u bytes | MAX_AUDIO=%u ms\n",
        su_audio_buffer_in_psram ? "PSRAM" : "INTERNAL_RAM",
        (unsigned int)su_audio_buffer_bytes,
        (unsigned int)(MAX_KHUNG_THOAI * 20U)
    );

    Serial.printf(
        "[SU PSRAM] FREE AFTER=%u | INTERNAL FREE AFTER=%u\n",
        (unsigned int)
        heap_caps_get_free_size(
            MALLOC_CAP_SPIRAM
        ),
        (unsigned int)
        heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        )
    );

#if SU_STREAMING_V1
    su_stream_frame_queue = xQueueCreate(
        SU_STREAM_FRAME_QUEUE_DEPTH,
        sizeof(SUStreamAudioFrame)
    );

    if (su_stream_frame_queue == nullptr)
    {
        Serial.println(
            "[SU STREAM FATAL] TAO FRAME QUEUE THAT BAI"
        );
        delay(500);
        ESP.restart();
    }

    BaseType_t stream_task_ok = xTaskCreatePinnedToCore(
        TacVu_ThuAm_Streaming_SU,
        "SU_AUDIO_CAPTURE",
        8192,
        nullptr,
        4,
        &su_stream_capture_task,
        0
    );

    if (stream_task_ok != pdPASS)
    {
        Serial.println(
            "[SU STREAM FATAL] TAO AUDIO CAPTURE TASK THAT BAI"
        );
        delay(500);
        ESP.restart();
    }

    Serial.printf(
        "[SU BUILD] TAG=V2C4_6_ADMISSION_PROFILE_FAILSAFE | PAIR=%u | SU=0x%02X | DU=0x%02X | SPEEX=ADAPTIVE(HQ3950/LQ2150) | HQ_BYTES=10 | LQ_BYTES=6 | FRAME_MS=20 | VOICE_MAX_FRAMES=15 | VOICE_BYTES=%u | FEC=OFF_TRANSITION_SAFE | SF=7 | BW=500k\n",
        (unsigned int)V2C_PAIR_INDEX,
        (unsigned int)V2C_SU_ID,
        (unsigned int)V2C_DU_ID,
        (unsigned int)SIZE_VOICE_PACKET_SU
    );

    Serial.printf(
        "[SU STREAM V2C4.2] READY | PAIR=%u | SU=0x%02X DU=0x%02X | QUEUE=%u | SINGLE=300ms/HQ8+7 | DUAL=300ms/LQ15 | FEC=OFF_TRANSITION_SAFE | JAM=OFF\n",
        (unsigned int)V2C_PAIR_INDEX,
        (unsigned int)V2C_SU_ID,
        (unsigned int)V2C_DU_ID,
        (unsigned int)SU_STREAM_FRAME_QUEUE_DEPTH
    );
#endif

    Serial.println(
        "[SU] Khoi tao thanh cong!"
    );
}


// ========================================================
// GUI VI TRI DINH KY KHI RADIO RANH
// - Khong chen vao RECORD / SESSION / VOICE / FEC / ARQ.
// - Neu SU dang cho ACK/NACK nguoi dung cua DU thi tam hoan.
// - Best-effort: khong tao them ARQ de khong tang tai kenh.
// ========================================================
static void XuLy_BaoCao_ViTri_DinhKy_SU()
{
    if (trang_thai != NGHI_NGOI)
        return;

    // Chua nhan phan hoi cua DU cho cau vua roi -> uu tien nghe HMI.
    if (HMI_SU_Dang_Cho_PhanHoi())
        return;

    uint32_t bay_gio_ms = millis();
    if ((int32_t)(bay_gio_ms - moc_gui_vi_tri_su_tiep_theo_ms) < 0)
        return;

    DuLieuGPS_SU du_lieu_gps_su = Lay_DuLieu_GPS_SU();
    In_TrangThai_GPS_SU(du_lieu_gps_su);

    // V10: chi "tieu thu" STT khi TX thanh cong.
    // Neu radio ban/TX loi, lan thu lai van dung cung STT -> rBS co the
    // phan biet packet mat tren khong trung voi beacon bi hoan tai SU.
    uint64_t stt_du_kien = so_thu_tu_bao_cao_vi_tri_su + 1;
    if (stt_du_kien == 0)
        stt_du_kien = 1;

    bool gui_thanh_cong = Gui_VI_TRI_DINH_KY_SU(
        stt_du_kien,
        du_lieu_gps_su
    );

    if (gui_thanh_cong)
    {
        so_thu_tu_bao_cao_vi_tri_su = stt_du_kien;
        su_telemetry_last_tx_ms = bay_gio_ms;

        if (su_telemetry_post_pending)
        {
            su_telemetry_post_pending = false;

            Serial.printf(
                "[SU TELEMETRY] POST -> rBS | STT=%llu\n",
                (unsigned long long)stt_du_kien
            );
        }
    }

    moc_gui_vi_tri_su_tiep_theo_ms = bay_gio_ms +
        (gui_thanh_cong ? CHU_KY_BAO_CAO_VI_TRI_SU_MS : 500UL);
}


// ========================================================
// LOOP
// ========================================================

void loop()
{
#if SU_STREAMING_V1
    SU_Stream_LoopV1();
    return;
#endif

    SU_WDT_SelfTest_Check();
    SU_WDT_Feed();

    bool nut_dang_bam = Nut_PTT_Dang_Bam_SU();


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
        // TELEMETRY PRE/POST V1:
        // Khong TX telemetry moi tai thoi diem PTT.
        // Dung snapshot IDLE gan nhat lam PRE de khong chen delay vao voice.
        if (su_telemetry_last_tx_ms != 0)
        {
            Serial.printf(
                "[SU TELEMETRY] PRE SNAPSHOT | AGE=%u ms | STT=%llu | LOCK TRONG PHIEN\n",
                (unsigned int)(millis() - su_telemetry_last_tx_ms),
                (unsigned long long)so_thu_tu_bao_cao_vi_tri_su
            );
        }
        else
        {
            Serial.println(
                "[SU TELEMETRY] PRE SNAPSHOT CHUA CO | LOCK TRONG PHIEN"
            );
        }

        trang_thai =
            DANG_GHI_AM;

        // Cau moi -> module HMI xoa ACK/NACK/session cua cau truoc.
        HMI_SU_BatDau_CauMoi();


        tong_so_khung_da_ghi =
            0;


        // Ghi lại thời điểm PTT thực sự chuyển sang trạng thái RECORD.
        thoi_diem_bat_dau_ghi_ms =
            millis();


        // OLED cap nhat TRUOC khi bat ADC-DMA.
        // Trong luc dang thu, KHONG goi I2C/OLED de tranh can thiệp
        // vao duong thu am DMA da duoc kiem chung on dinh.
        Ve_GiaoDien_OLED(
            0,
            true
        );


        adc_digi_start();


        Serial.printf(
            ">> BAT DAU GHI AM -> SPEEX BUFFER %s...\n",
            su_audio_buffer_in_psram ? "PSRAM" : "INTERNAL_RAM"
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

        uint8_t khung_speex_20b[SPEEX_BYTES_PER_FRAME];


        if (
            LayMau_AmThanh(
                mang_tam_PCM
            )
        )
        {
            // ============================================
            // KHONG CAP NHAT OLED TRONG LUC ADC-DMA DANG THU
            //
            // Ban OLED se giu nguyen man hinh "PHAT AM".
            // Muc dich: tach hoan toan I2C/OLED khoi duong thu am.
            // Sau khi nha PTT va adc_digi_stop(), OLED moi cap nhat lai.
            // ============================================


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
                            SPEEX_BYTES_PER_FRAME
                        ],

                        khung_speex_20b,

                        SPEEX_BYTES_PER_FRAME
                    );


                    tong_so_khung_da_ghi++;
                    SU_WDT_Feed();

                    // Log nhe de xac nhan ADC/Speex van dang thu,
                    // khong cap nhat OLED trong luc DMA hoat dong.
                    if ((tong_so_khung_da_ghi % 50) == 0)
                    {
                        Serial.printf(
                            "[SU REC] FRAME=%u | AUDIO_MS=%u\n",
                            (unsigned int)tong_so_khung_da_ghi,
                            (unsigned int)(tong_so_khung_da_ghi * 20)
                        );
                    }
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
        // T0 cua phep do E2E: bat ngay khi SU phat hien PTT da nha.
        // Dat truoc ADC stop/OLED de tinh ca processing local sau khi nha nut.
        e2e_ptt_release_ms =
            millis();

        trang_thai =
            DANG_PHAT_SONG;

        Reset_ThongKe_OLED_SU();


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

        HienThi_SU_DangGui(
            0,
            0,
            0
        );


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
        // NATIVE 8-FRAME / 96B + ARQ + FEC 3+1
        // =================================================

        uint8_t goi_voice[SIZE_VOICE_PACKET_SU];
        uint8_t goi_fec[SIZE_FEC_PACKET_SU];

        uint8_t parity_group[FEC_PARITY_BYTES];

        memset(
            parity_group,
            0,
            sizeof(parity_group)
        );

        uint8_t data_count_group = 0;
        uint32_t group_start_seq = 0;

        bool group_has_last = false;
        uint8_t final_frame_count = 0;

        bool gui_that_bai = false;


        // =================================================
        // SESSION SETUP
        //
        // Nhả PTT -> SU gui SESSION_START.
        // rBS tu relay SESSION_START toi DU toi da 3 lan.
        // DU nhan duoc -> tu dong gui SESSION_READY.
        // rBS forward SESSION_READY ve SU.
        // CHI KHI READY thanh cong SU moi gui VOICE/FEC.
        // =================================================

        uint64_t session_id_tx =
            Tao_Session_Moi();

        HMI_SU_Dat_Session(session_id_tx);

        uint8_t goi_session[12];

        Tao_GoiTin_SessionStart(
            goi_session
        );

        bool session_ready = false;
        bool session_fail_remote = false;

        // LED sang dung = dang cho handshake SESSION_READY.
        Dat_LED_SU(true);

        for (
            uint8_t lan = 0;
            lan <= SESSION_REQUEST_RETRY;
            lan++
        )
        {
            SU_WDT_Feed();

            Serial.printf(
                "[SU SESSION] REQUEST %u/%u | SESSION=%016llX\n",
                (unsigned int)(lan + 1),
                (unsigned int)(SESSION_REQUEST_RETRY + 1),
                (unsigned long long)session_id_tx
            );

            Phat_GoiTin_LoRa(
                goi_session,
                sizeof(goi_session)
            );

            session_fail_remote = false;

            bool ready_ok =
                Cho_SESSION_READY_RBS(
                    SESSION_TIMEOUT_SU_MS,
                    session_id_tx,
                    session_fail_remote
                );

            SU_WDT_Feed();

            if (ready_ok)
            {
                session_ready = true;
                break;
            }

            if (session_fail_remote)
            {
                break;
            }
        }

        Dat_LED_SU(false);

        if (!session_ready)
        {
            gui_that_bai = true;

            Serial.printf(
                "[SU SESSION FAIL] KHONG THIET LAP DUOC SESSION=%016llX\n",
                (unsigned long long)session_id_tx
            );

            Bao_Loi_Session_SU();
        }
        else
        {
            Serial.printf(
                "[SU SESSION READY] DU DA SAN SANG | SESSION=%016llX\n",
                (unsigned long long)session_id_tx
            );

            Serial.println(
                "[SU TELEMETRY] READY->VOICE | KHONG GPS CHEN GIUA"
            );

            // GPS la packet rieng, khong chen vao SESSION_START/VOICE.
            // Gui snapshot moi nhat NGAY SAU READY, truoc VOICE.
        }


        // =================================================
        // MỖI VOICE = TỐI ĐA 8 FRAME = 160ms
        // =================================================

        for (
            uint32_t i = 0;
            i < tong_so_khung_da_ghi && !gui_that_bai;
            i += MAX_FRAME_PER_PACKET
        )
        {
            SU_WDT_Feed();

            uint32_t con_lai =
                tong_so_khung_da_ghi - i;

            uint8_t so_frame =
                (
                    con_lai >= MAX_FRAME_PER_PACKET
                    ? MAX_FRAME_PER_PACKET
                    : (uint8_t)con_lai
                );

            bool la_packet_cuoi =
                (
                    i + so_frame
                    >=
                    tong_so_khung_da_ghi
                );


            // =============================================
            // PLAINTEXT BLOCK 80B
            //
            // Packet cuối thiếu frame được zero-pad.
            // Sau khi AES-GCM tạo VOICE, parity KHÔNG XOR
            // plaintext này; parity XOR protected block
            // 88B = ciphertext80 + original GCM tag8.
            // =============================================

            uint8_t payload_voice[
                VOICE_PLAINTEXT_BYTES
            ];

            memset(
                payload_voice,
                0,
                sizeof(payload_voice)
            );

            memcpy(
                payload_voice,
                &Kho_Chua_AmThanh[
                    i * SPEEX_BYTES_PER_FRAME
                ],
                so_frame * SPEEX_BYTES_PER_FRAME
            );


            Tao_GoiTin_Voice(
                payload_voice,
                so_frame,
                la_packet_cuoi,
                false,
                goi_voice
            );


            uint32_t seq_num_tx =
                ((uint32_t)goi_voice[4] << 24)
                |
                ((uint32_t)goi_voice[5] << 16)
                |
                ((uint32_t)goi_voice[6] << 8)
                |
                ((uint32_t)goi_voice[7]);


            if (data_count_group == 0)
            {
                group_start_seq =
                    seq_num_tx;

                memset(
                    parity_group,
                    0,
                    sizeof(parity_group)
                );

                group_has_last =
                    false;

                final_frame_count =
                    0;
            }


            // =============================================
            // CẬP NHẬT XOR PARITY TRÊN DỮ LIỆU ĐÃ MÃ HÓA
            //
            // protected block = ciphertext80 + original GCM tag8 = 88B.
            // =============================================

            for (
                size_t b = 0;
                b < FEC_PARITY_BYTES;
                b++
            )
            {
                parity_group[b] ^=
                    goi_voice[8 + b];
            }

            data_count_group++;

            if (la_packet_cuoi)
            {
                group_has_last =
                    true;

                final_frame_count =
                    so_frame;
            }


            su_oled_voice_packets++;

            // Khong refresh OLED theo tung VOICE packet.
            // Counter van duoc cap nhat trong RAM; tranh chen I2C vao latency.

            Serial.printf(
                "[SU TX] VOICE | SEQ=%u | FRAME=%u | LAST=%u | SIZE=%uB\n",
                seq_num_tx,
                so_frame,
                la_packet_cuoi ? 1 : 0,
                (unsigned int)SIZE_VOICE_PACKET_SU
            );


            // =============================================
            // ARQ SU -> rBS
            // =============================================

            if (
                !Gui_Packet_Co_ACK(
                    goi_voice,
                    SIZE_VOICE_PACKET_SU,
                    TYPE_VOICE_SU,
                    seq_num_tx
                )
            )
            {
                gui_that_bai =
                    true;

                break;
            }

            SU_WDT_Feed();


            // =============================================
            // DU FEC_DATA_PER_GROUP DATA HOẶC ĐẾN PACKET CUỐI
            // -> PHÁT 1 PARITY PACKET
            //
            // FEC overhead:
            //   STREAM V1 full group: 1 / 4 = 25%
            // =============================================

            if (
                data_count_group
                    >= FEC_DATA_PER_GROUP
                ||
                la_packet_cuoi
            )
            {
                Tao_GoiTin_FEC(
                    parity_group,
                    group_start_seq,
                    data_count_group,
                    group_has_last,
                    final_frame_count,
                    goi_fec
                );

                Serial.printf(
                    "[SU TX] FEC | GROUP_START=%u | DATA=%u | HAS_LAST=%u | FINAL_FRAME=%u | SIZE=%uB | PARITY=CIPHERTEXT+TAG\n",
                    group_start_seq,
                    data_count_group,
                    group_has_last ? 1 : 0,
                    final_frame_count,
                    (unsigned int)SIZE_FEC_PACKET_SU
                );


                if (
                    !Gui_Packet_Co_ACK(
                        goi_fec,
                        SIZE_FEC_PACKET_SU,
                        TYPE_FEC_SU,
                        group_start_seq
                    )
                )
                {
                    gui_that_bai =
                        true;

                    break;
                }

                SU_WDT_Feed();


                data_count_group =
                    0;

                memset(
                    parity_group,
                    0,
                    sizeof(parity_group)
                );
            }
        }


        // =================================================
        // AUDIO_END 5B
        //
        // Chỉ gửi sau khi final FEC đã được ACK.
        // =================================================

        bool e2e_hop_le = false;
        uint32_t e2e_observed_ms = 0;
        uint32_t e2e_est_ms = 0;

        if (!gui_that_bai)
        {
            uint8_t goi_audio_end[5] =
            {
                ID_TRAM_DU_CTRL,
                ID_TRAM_SU_CTRL,
                TYPE_AUDIO_END_SU,
                0x00,
                0x00
            };

            Dat_LED_SU(true);
            Phat_GoiTin_LoRa(
                goi_audio_end,
                sizeof(goi_audio_end)
            );
            Dat_LED_SU(false);

            Serial.println(
                "[SU CTRL] AUDIO_END 5B -> rBS"
            );

            // Sau AUDIO_END, SU vao RX va cho tin PLAY_STARTED da duoc
            // rBS forward tu DU. Timer van chay tren CHINH dong ho SU.
            SU_WDT_Feed();

            bool play_started_ok =
                Cho_PLAY_STARTED_RBS(
                    PLAY_REPORT_TIMEOUT_MS,
                    session_id_tx
                );

            SU_WDT_Feed();

            if (play_started_ok)
            {
                e2e_observed_ms =
                    millis() - e2e_ptt_release_ms;

                // Tru airtime uoc tinh cua duong report quay ve 2 hop.
                // Neu sau nay doi SF/BW/CR hoac format report, cap nhat hang so nay.
                uint32_t thoi_gian_phan_hoi_uoc_tinh_ms =
                    THOI_GIAN_PHAN_HOI_E2E_UOC_TINH_MS;

                e2e_est_ms =
                    (
                        e2e_observed_ms > thoi_gian_phan_hoi_uoc_tinh_ms
                        ? e2e_observed_ms - thoi_gian_phan_hoi_uoc_tinh_ms
                        : e2e_observed_ms
                    );

                e2e_hop_le = true;

                Serial.printf(
                    "[SU E2E] OBS=%u ms | RETURN_EST=%u ms | PTT_RELEASE->DU_PLAY ~= %u ms\n",
                    (unsigned int)e2e_observed_ms,
                    (unsigned int)thoi_gian_phan_hoi_uoc_tinh_ms,
                    (unsigned int)e2e_est_ms
                );
            }
            else
            {
                Serial.println(
                    "[SU E2E] KHONG NHAN DUOC PLAY_STARTED -> E2E KHONG CO SO LIEU"
                );
            }

            Serial.println(
                ">> DA GUI XONG!"
            );
        }
        else
        {
            Serial.println(
                ">> GUI BI DUNG DO ARQ FAIL!"
            );
        }


        // =================================================
        // VỀ TRẠNG THÁI NGHỈ
        // =================================================

        // TELEMETRY PRE/POST V1:
        // Dat POST pending NGAY khi voice/session da ket thuc.
        // XuLy_BaoCao... van tu cho neu HMI dang cho phan hoi,
        // nen packet POST khong chen vao control cua session.
        su_telemetry_post_pending = true;
        moc_gui_vi_tri_su_tiep_theo_ms = millis();

        Serial.println(
            "[SU TELEMETRY] POST PENDING | CHO HMI/RADIO RANH"
        );

        trang_thai =
            NGHI_NGOI;


        HienThi_SU_KetQua(
            su_oled_voice_packets,
            su_oled_retransmissions,
            su_oled_fail_packets,
            e2e_hop_le,
            e2e_est_ms
        );

        Bat_CheDo_RX_LoRa();
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
        CapNhat_LED_PhanHoi_SU();

        uint64_t ma_phien_hmi = HMI_SU_Lay_Session();
        if (ma_phien_hmi == 0)
        {
            uint8_t dummy_response = 0;
            (void)KiemTra_USER_RESPONSE_RBS(0, dummy_response);
        }
        if (ma_phien_hmi != 0)
        {
            uint8_t ma_phan_hoi = 0;
            if (KiemTra_USER_RESPONSE_RBS(ma_phien_hmi, ma_phan_hoi))
            {
                Dat_PhanHoi_SU(ma_phan_hoi);
                Gui_USER_CONFIRM_RBS(ma_phien_hmi, ma_phan_hoi);
            }
        }

        XuLy_BaoCao_ViTri_DinhKy_SU();

        SU_WDT_Feed();
        delay(5);
    }
}