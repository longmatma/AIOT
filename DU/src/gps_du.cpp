#include "gps_du.h"

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

static TinyGPSPlus gps_parser_du;
static HardwareSerial gps_uart_du(1);
static portMUX_TYPE gps_du_mux = portMUX_INITIALIZER_UNLOCKED;
static DuLieuGPS_DU gps_du_snapshot = {};

static uint32_t Tao_Ngay_UTC()
{
    if (!gps_parser_du.date.isValid()) return 0;
    return (uint32_t)gps_parser_du.date.year() * 10000UL
         + (uint32_t)gps_parser_du.date.month() * 100UL
         + (uint32_t)gps_parser_du.date.day();
}

static uint32_t Tao_Gio_UTC_ms()
{
    if (!gps_parser_du.time.isValid()) return 0;
    return ((uint32_t)gps_parser_du.time.hour() * 3600UL
          + (uint32_t)gps_parser_du.time.minute() * 60UL
          + (uint32_t)gps_parser_du.time.second()) * 1000UL
          + (uint32_t)gps_parser_du.time.centisecond() * 10UL;
}

static void CapNhat_Snapshot_DU()
{
    DuLieuGPS_DU s = {};

    uint32_t age = gps_parser_du.location.isValid()
        ? gps_parser_du.location.age()
        : UINT32_MAX;

    s.fix_age_ms = age;
    s.gps_valid = gps_parser_du.location.isValid()
               && age <= GPS_DU_MAX_FIX_AGE_MS;

    if (gps_parser_du.location.isValid())
    {
        s.latitude = gps_parser_du.location.lat();
        s.longitude = gps_parser_du.location.lng();
    }

    s.altitude_valid = gps_parser_du.altitude.isValid();
    if (s.altitude_valid) s.altitude_m = gps_parser_du.altitude.meters();

    s.speed_valid = gps_parser_du.speed.isValid();
    if (s.speed_valid) s.speed_mps = gps_parser_du.speed.mps();

    s.satellites = gps_parser_du.satellites.isValid()
        ? (uint8_t)min((uint32_t)255, gps_parser_du.satellites.value())
        : 0;

    s.hdop_valid = gps_parser_du.hdop.isValid();
    if (s.hdop_valid) s.hdop = gps_parser_du.hdop.hdop();

    s.utc_date_yyyymmdd = Tao_Ngay_UTC();
    s.utc_time_ms_of_day = Tao_Gio_UTC_ms();

    portENTER_CRITICAL(&gps_du_mux);
    gps_du_snapshot = s;
    portEXIT_CRITICAL(&gps_du_mux);
}

static void TacVu_GPS_DU(void *tham_so)
{
    (void)tham_so;

    while (true)
    {
        bool co_du_lieu_moi = false;

        while (gps_uart_du.available() > 0)
        {
            char c = (char)gps_uart_du.read();
            if (gps_parser_du.encode(c))
            {
                co_du_lieu_moi = true;
            }
        }

        if (co_du_lieu_moi)
        {
            CapNhat_Snapshot_DU();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void KhoiTao_GPS_DU()
{
    gps_uart_du.setRxBufferSize(2048);
    gps_uart_du.begin(
        GPS_DU_UART_BAUD,
        SERIAL_8N1,
        GPS_DU_UART_RX_PIN,
        GPS_DU_UART_TX_PIN
    );

    xTaskCreatePinnedToCore(
        TacVu_GPS_DU,
        "GPS_DU",
        4096,
        nullptr,
        1,
        nullptr,
        0
    );

    Serial.printf(
        "[DU GPS] NEO-6M UART khoi tao | RX=%d | TX=%d | BAUD=%d\n",
        GPS_DU_UART_RX_PIN,
        GPS_DU_UART_TX_PIN,
        GPS_DU_UART_BAUD
    );
}

DuLieuGPS_DU Lay_DuLieu_GPS_DU()
{
    DuLieuGPS_DU s;
    portENTER_CRITICAL(&gps_du_mux);
    s = gps_du_snapshot;
    portEXIT_CRITICAL(&gps_du_mux);
    return s;
}

void In_TrangThai_GPS_DU(const DuLieuGPS_DU &gps)
{
    Serial.printf(
        "[DU GPS] VALID=%u | LAT=%.7f | LON=%.7f | ALT=%.1fm | SPEED=%.2fm/s | SAT=%u | HDOP=%.2f | AGE=%u ms | UTC_DATE=%u | UTC_MS=%u\n",
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
