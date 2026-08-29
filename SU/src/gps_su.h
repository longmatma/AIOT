#ifndef GPS_SU_H
#define GPS_SU_H

#include <Arduino.h>

// =====================================================
// NEO-6M / GNSS UART - SU
// Cau hinh da xac nhan theo phan cung hien tai cua ban:
// GPS TX -> ESP32-S3 GPIO18 (RX)
// GPS RX -> ESP32-S3 GPIO17 (TX, co the bo neu chi doc GPS)
// =====================================================
#define GPS_SU_UART_RX_PIN 18
#define GPS_SU_UART_TX_PIN 17
#define GPS_SU_UART_BAUD   4800

// Fix cu hon nguong nay se khong duoc coi la toa do hop le.
#define GPS_SU_TUOI_FIX_TOI_DA_MS 5000UL

struct DuLieuGPS_SU
{
    bool gps_hop_le;
    bool do_cao_hop_le;
    bool toc_do_hop_le;
    bool hdop_hop_le;

    double vi_do;
    double kinh_do;
    double do_cao_m;
    double toc_do_m_s;

    uint8_t so_ve_tinh;
    double hdop;
    uint32_t tuoi_fix_ms;

    // UTC tu NMEA. =0 neu chua co ngay/gio hop le.
    uint32_t ngay_utc_yyyymmdd;
    uint32_t gio_utc_ms_trong_ngay;

    // Chan doan UART/NMEA, chi dung local, khong bat buoc gui qua LoRa.
    uint32_t so_ky_tu_nmea;
};

void KhoiTao_GPS_SU();
DuLieuGPS_SU Lay_DuLieu_GPS_SU();
void In_TrangThai_GPS_SU(const DuLieuGPS_SU &du_lieu_gps);

#endif
