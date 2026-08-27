#ifndef GPS_DU_H
#define GPS_DU_H

#include <Arduino.h>

// =====================================================
// NEO-6M / GNSS UART - DU
// Mac dinh da chon cac GPIO dang trong trong baseline hien tai.
// Neu PCB cua ban noi khac, CHI CAN doi 2 define nay.
// GPS TX -> ESP32-S3 GPIO17 (RX)
// GPS RX -> ESP32-S3 GPIO18 (TX, co the bo trong neu chi doc GPS)
// =====================================================
#define GPS_DU_UART_RX_PIN 17
#define GPS_DU_UART_TX_PIN 18
#define GPS_DU_UART_BAUD   9600

// Fix cu hon nguong nay se khong duoc danh dau GPS_VALID.
#define GPS_DU_MAX_FIX_AGE_MS 5000UL

struct DuLieuGPS_DU
{
    bool gps_valid;
    bool altitude_valid;
    bool speed_valid;
    bool hdop_valid;

    double latitude;
    double longitude;
    double altitude_m;
    double speed_mps;

    uint8_t satellites;
    double hdop;
    uint32_t fix_age_ms;

    // UTC tu NMEA. =0 neu chua co date/time hop le.
    uint32_t utc_date_yyyymmdd;
    uint32_t utc_time_ms_of_day;
};

void KhoiTao_GPS_DU();
DuLieuGPS_DU Lay_DuLieu_GPS_DU();
void In_TrangThai_GPS_DU(const DuLieuGPS_DU &gps);

#endif
