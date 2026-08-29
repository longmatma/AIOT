#ifndef GPS_DU_H
#define GPS_DU_H

#include <Arduino.h>

// =====================================================
// NEO-6M / GNSS UART - DU
// Cau hinh da xac nhan theo phan cung hien tai cua ban:
// GPS TX -> ESP32-S3 GPIO18 (RX)
// GPS RX -> ESP32-S3 GPIO17 (TX, co the bo neu chi doc GPS)
// =====================================================
#define GPS_DU_UART_RX_PIN 18
#define GPS_DU_UART_TX_PIN 17
#define GPS_DU_UART_BAUD   9600

#define GPS_DU_TUOI_FIX_TOI_DA_MS 5000UL

struct DuLieuGPS_DU
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

    uint32_t ngay_utc_yyyymmdd;
    uint32_t gio_utc_ms_trong_ngay;

    uint32_t so_ky_tu_nmea;
};

void KhoiTao_GPS_DU();
DuLieuGPS_DU Lay_DuLieu_GPS_DU();
void In_TrangThai_GPS_DU(const DuLieuGPS_DU &du_lieu_gps);

#endif
