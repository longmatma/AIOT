#include "gps_su.h"

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

static TinyGPSPlus gps_parser_su;
static HardwareSerial gps_uart_su(1);
static portMUX_TYPE gps_su_mux = portMUX_INITIALIZER_UNLOCKED;
static DuLieuGPS_SU gps_su_snapshot = {};

static uint32_t Tao_Ngay_UTC()
{
    if (!gps_parser_su.date.isValid()) return 0;
    return (uint32_t)gps_parser_su.date.year() * 10000UL
         + (uint32_t)gps_parser_su.date.month() * 100UL
         + (uint32_t)gps_parser_su.date.day();
}

static uint32_t Tao_Gio_UTC_ms()
{
    if (!gps_parser_su.time.isValid()) return 0;
    return ((uint32_t)gps_parser_su.time.hour() * 3600UL
          + (uint32_t)gps_parser_su.time.minute() * 60UL
          + (uint32_t)gps_parser_su.time.second()) * 1000UL
          + (uint32_t)gps_parser_su.time.centisecond() * 10UL;
}

static void CapNhat_Snapshot_SU()
{
    DuLieuGPS_SU s = {};

    uint32_t age = gps_parser_su.location.isValid()
        ? gps_parser_su.location.age()
        : UINT32_MAX;

    s.fix_age_ms = age;
    s.gps_valid = gps_parser_su.location.isValid()
               && age <= GPS_SU_MAX_FIX_AGE_MS;

    if (gps_parser_su.location.isValid())
    {
        s.latitude = gps_parser_su.location.lat();
        s.longitude = gps_parser_su.location.lng();
    }

    s.altitude_valid = gps_parser_su.altitude.isValid();
    if (s.altitude_valid) s.altitude_m = gps_parser_su.altitude.meters();

    s.speed_valid = gps_parser_su.speed.isValid();
    if (s.speed_valid) s.speed_mps = gps_parser_su.speed.mps();

    s.satellites = gps_parser_su.satellites.isValid()
        ? (uint8_t)min((uint32_t)255, gps_parser_su.satellites.value())
        : 0;

    s.hdop_valid = gps_parser_su.hdop.isValid();
    if (s.hdop_valid) s.hdop = gps_parser_su.hdop.hdop();

    s.utc_date_yyyymmdd = Tao_Ngay_UTC();
    s.utc_time_ms_of_day = Tao_Gio_UTC_ms();

    portENTER_CRITICAL(&gps_su_mux);
    gps_su_snapshot = s;
    portEXIT_CRITICAL(&gps_su_mux);
}

static void TacVu_GPS_SU(void *tham_so)
{
    (void)tham_so;

    while (true)
    {
        bool co_du_lieu_moi = false;

        while (gps_uart_su.available() > 0)
        {
            char c = (char)gps_uart_su.read();
            if (gps_parser_su.encode(c))
            {
                co_du_lieu_moi = true;
            }
        }

        if (co_du_lieu_moi)
        {
            CapNhat_Snapshot_SU();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void KhoiTao_GPS_SU()
{
    gps_uart_su.setRxBufferSize(2048);
    gps_uart_su.begin(
        GPS_SU_UART_BAUD,
        SERIAL_8N1,
        GPS_SU_UART_RX_PIN,
        GPS_SU_UART_TX_PIN
    );

    xTaskCreatePinnedToCore(
        TacVu_GPS_SU,
        "GPS_SU",
        4096,
        nullptr,
        1,
        nullptr,
        0
    );

    Serial.printf(
        "[SU GPS] NEO-6M UART khoi tao | RX=%d | TX=%d | BAUD=%d\n",
        GPS_SU_UART_RX_PIN,
        GPS_SU_UART_TX_PIN,
        GPS_SU_UART_BAUD
    );
}

DuLieuGPS_SU Lay_DuLieu_GPS_SU()
{
    DuLieuGPS_SU s;
    portENTER_CRITICAL(&gps_su_mux);
    s = gps_su_snapshot;
    portEXIT_CRITICAL(&gps_su_mux);
    return s;
}

void In_TrangThai_GPS_SU(const DuLieuGPS_SU &gps)
{
    Serial.printf(
        "[SU GPS] VALID=%u | LAT=%.7f | LON=%.7f | ALT=%.1fm | SPEED=%.2fm/s | SAT=%u | HDOP=%.2f | AGE=%u ms | UTC_DATE=%u | UTC_MS=%u\n",
        gps.gps_valid ? 1 : 0,
        gps.latitude,
        gps.longitude,
        gps.altitude_m,
        gps.speed_mps,
        gps.satellites,
        gps.hdop,
        (unsigned int)gps.fix_age_ms,
        (unsigned int)gps.utc_date_yyyymmdd,
        (unsigned int)gps.utc_time_ms_of_day
    );
}
