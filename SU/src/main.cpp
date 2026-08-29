#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "driver/adc.h"

#include "thu_am.h"
#include "nen_speex.h"
#include "dong_goi.h"
#include "phat_lora.h"
#include "hien_thi.h"
#include "gps_su.h"
#include "hmi_su.h"


extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;


// ========================================================
// CẤU HÌNH
// ========================================================

#define MAX_KHUNG_THOAI 3000


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
// PROTOCOL NATIVE 8-FRAME + ARQ + PACKET-FEC
//
// 1 VOICE packet = tối đa 8 frame = 160 ms audio.
// VOICE inner     = 176 byte.
//
// rBS relay NGAY từng packet (BURST_SIZE logic cũ bỏ).
//
// First hop SU->rBS:
//   stop-and-wait ACK có KIND + SEQ.
//   Nếu ACK mất / packet mất: retransmit chính packet đã mã hóa.
//
// End-to-end packet FEC:
//   8 DATA + 1 PARITY.
//   Parity XOR tren protected block 168B = ciphertext160 + original GCM tag8.
//   FEC parity packet tu duoc AES-GCM bao ve.
//   DU co the khoi phuc 1 VOICE mat trong moi group va verify GCM goc.
//
// CR LoRa vẫn = 4/5.
// ========================================================

#define READY_TIMEOUT_MS          350
#define MAX_TX_RETRY                2

// SESSION_START -> rBS -> DU -> SESSION_READY -> rBS -> SU.
// V10: cho rong hon de DU co GPS report + SESSION_READY ma khong bi ep timeout.
// Khi READY den som, SU thoat ngay nen khong lam tang do tre binh thuong.
#define SESSION_SETUP_TIMEOUT_MS  1200
#define SESSION_REQUEST_RETRY        2

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

#define ID_TRAM_SU_CTRL   0x01
#define ID_TRAM_DU_CTRL   0x02


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

#define PLAY_REPORT_TIMEOUT_MS       500
#define PLAY_REPORT_RETURN_EST_MS     29


// ========================================================
// BỘ NHỚ AUDIO
// ========================================================

uint8_t *Kho_Chua_AmThanh;

uint32_t tong_so_khung_da_ghi = 0;

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

        if (
            Cho_READY_RBS(
                READY_TIMEOUT_MS,
                ack_kind,
                ack_seq
            )
        )
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


// ========================================================
// SETUP
// ========================================================

void setup()
{
    Serial.begin(
        115200
    );

    // GPS NEO-6M doc lien tuc trong task rieng, khong block audio/PTT.
    KhoiTao_GPS_SU();

    moc_gui_vi_tri_su_tiep_theo_ms =
        millis() + TRE_BAO_CAO_VI_TRI_SU_LUC_KHOI_DONG_MS;


    // Nut PTT + LED duoc khoi tao trong module HMI rieng.
    KhoiTao_HMI_SU();


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
        so_thu_tu_bao_cao_vi_tri_su = stt_du_kien;

    moc_gui_vi_tri_su_tiep_theo_ms = bay_gio_ms +
        (gui_thanh_cong ? CHU_KY_BAO_CAO_VI_TRI_SU_MS : 500UL);
}


// ========================================================
// LOOP
// ========================================================

void loop()
{
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
                            20
                        ],

                        khung_speex_20b,

                        20
                    );


                    tong_so_khung_da_ghi++;

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
        // NATIVE 8-FRAME / 176B + ARQ + FEC 8+1
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

            if (
                Cho_SESSION_READY_RBS(
                    SESSION_SETUP_TIMEOUT_MS,
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

            // GPS la packet rieng, khong chen vao SESSION_START/VOICE.
            // Gui snapshot moi nhat NGAY SAU READY, truoc VOICE.
            DuLieuGPS_SU du_lieu_gps_su = Lay_DuLieu_GPS_SU();
            In_TrangThai_GPS_SU(du_lieu_gps_su);

            Dat_LED_SU(true);
            Gui_GPS_REPORT_SU(session_id_tx, du_lieu_gps_su);
            Dat_LED_SU(false);

            // Cho rBS doc GPS_REPORT + re-arm RX truoc VOICE dau tien.
            delay(8);
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
            // PLAINTEXT BLOCK 160B
            //
            // Packet cuối thiếu frame được zero-pad.
            // Sau khi AES-GCM tạo VOICE, parity KHÔNG XOR
            // plaintext này; parity XOR protected block
            // 168B = ciphertext160 + original GCM tag8.
            // =============================================

            uint8_t payload_160[
                VOICE_PLAINTEXT_BYTES
            ];

            memset(
                payload_160,
                0,
                sizeof(payload_160)
            );

            memcpy(
                payload_160,
                &Kho_Chua_AmThanh[
                    i * SPEEX_BYTES_PER_FRAME
                ],
                so_frame * SPEEX_BYTES_PER_FRAME
            );


            Tao_GoiTin_Voice(
                payload_160,
                so_frame,
                la_packet_cuoi,
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
            // goi_voice[8..167]   = ciphertext 160B
            // goi_voice[168..175] = original GCM tag 8B
            //
            // Tổng protected block = 168B.
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
                "[SU TX] VOICE | SEQ=%u | FRAME=%u | LAST=%u | SIZE=176B\n",
                seq_num_tx,
                so_frame,
                la_packet_cuoi ? 1 : 0
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


            // =============================================
            // ĐỦ 8 DATA HOẶC ĐẾN PACKET CUỐI
            // -> PHÁT 1 PARITY PACKET
            //
            // FEC overhead:
            //   full group: 1 / 8 = 12.5%
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
                    "[SU TX] FEC | GROUP_START=%u | DATA=%u | HAS_LAST=%u | FINAL_FRAME=%u | SIZE=184B | PARITY=CIPHERTEXT+TAG\n",
                    group_start_seq,
                    data_count_group,
                    group_has_last ? 1 : 0,
                    final_frame_count
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
            if (
                Cho_PLAY_STARTED_RBS(
                    PLAY_REPORT_TIMEOUT_MS,
                    session_id_tx
                )
            )
            {
                e2e_observed_ms =
                    millis() - e2e_ptt_release_ms;

                // Tru airtime uoc tinh cua duong report quay ve 2 hop.
                // Neu sau nay doi SF/BW/CR hoac format report, cap nhat hang so nay.
                e2e_est_ms =
                    (
                        e2e_observed_ms > PLAY_REPORT_RETURN_EST_MS
                        ? e2e_observed_ms - PLAY_REPORT_RETURN_EST_MS
                        : e2e_observed_ms
                    );

                e2e_hop_le = true;

                Serial.printf(
                    "[SU E2E] OBS=%u ms | RETURN_EST=%u ms | PTT_RELEASE->DU_PLAY ~= %u ms\n",
                    (unsigned int)e2e_observed_ms,
                    (unsigned int)PLAY_REPORT_RETURN_EST_MS,
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

        delay(5);
    }
}