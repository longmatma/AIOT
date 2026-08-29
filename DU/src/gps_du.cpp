#include "gps_du.h"

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

static TinyGPSPlus bo_phan_tich_gps_du;
static HardwareSerial uart_gps_du(1);
static portMUX_TYPE khoa_gps_du = portMUX_INITIALIZER_UNLOCKED;
static DuLieuGPS_DU anh_chup_gps_du = {};

static uint32_t Tao_Ngay_UTC_DU()
{
    if (!bo_phan_tich_gps_du.date.isValid()) return 0;
    return (uint32_t)bo_phan_tich_gps_du.date.year() * 10000UL
         + (uint32_t)bo_phan_tich_gps_du.date.month() * 100UL
         + (uint32_t)bo_phan_tich_gps_du.date.day();
}

static uint32_t Tao_Gio_UTC_ms_DU()
{
    if (!bo_phan_tich_gps_du.time.isValid()) return 0;
    return ((uint32_t)bo_phan_tich_gps_du.time.hour() * 3600UL
          + (uint32_t)bo_phan_tich_gps_du.time.minute() * 60UL
          + (uint32_t)bo_phan_tich_gps_du.time.second()) * 1000UL
          + (uint32_t)bo_phan_tich_gps_du.time.centisecond() * 10UL;
}

static void CapNhat_AnhChup_GPS_DU()
{
    DuLieuGPS_DU du_lieu = {};

    uint32_t tuoi_fix = bo_phan_tich_gps_du.location.isValid()
        ? bo_phan_tich_gps_du.location.age()
        : UINT32_MAX;

    du_lieu.tuoi_fix_ms = tuoi_fix;
    du_lieu.gps_hop_le = bo_phan_tich_gps_du.location.isValid()
                      && tuoi_fix <= GPS_DU_TUOI_FIX_TOI_DA_MS;

    if (bo_phan_tich_gps_du.location.isValid())
    {
        du_lieu.vi_do = bo_phan_tich_gps_du.location.lat();
        du_lieu.kinh_do = bo_phan_tich_gps_du.location.lng();
    }

    du_lieu.do_cao_hop_le = bo_phan_tich_gps_du.altitude.isValid();
    if (du_lieu.do_cao_hop_le)
        du_lieu.do_cao_m = bo_phan_tich_gps_du.altitude.meters();

    du_lieu.toc_do_hop_le = bo_phan_tich_gps_du.speed.isValid();
    if (du_lieu.toc_do_hop_le)
        du_lieu.toc_do_m_s = bo_phan_tich_gps_du.speed.mps();

    du_lieu.so_ve_tinh = bo_phan_tich_gps_du.satellites.isValid()
        ? (uint8_t)min((uint32_t)255, bo_phan_tich_gps_du.satellites.value())
        : 0;

    du_lieu.hdop_hop_le = bo_phan_tich_gps_du.hdop.isValid();
    if (du_lieu.hdop_hop_le)
        du_lieu.hdop = bo_phan_tich_gps_du.hdop.hdop();

    du_lieu.ngay_utc_yyyymmdd = Tao_Ngay_UTC_DU();
    du_lieu.gio_utc_ms_trong_ngay = Tao_Gio_UTC_ms_DU();
    du_lieu.so_ky_tu_nmea = bo_phan_tich_gps_du.charsProcessed();

    portENTER_CRITICAL(&khoa_gps_du);
    anh_chup_gps_du = du_lieu;
    portEXIT_CRITICAL(&khoa_gps_du);
}

static void TacVu_GPS_DU(void *tham_so)
{
    (void)tham_so;

    while (true)
    {
        while (uart_gps_du.available() > 0)
        {
            char ky_tu = (char)uart_gps_du.read();
            bo_phan_tich_gps_du.encode(ky_tu);
        }

        // Cap nhat snapshot lien tuc: toa do, tuoi fix va bo dem NMEA.
        CapNhat_AnhChup_GPS_DU();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void KhoiTao_GPS_DU()
{
    uart_gps_du.setRxBufferSize(2048);
    uart_gps_du.begin(
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
        "[DU GPS] UART khoi tao | RX=%d | TX=%d | BAUD=%d\n",
        GPS_DU_UART_RX_PIN,
        GPS_DU_UART_TX_PIN,
        GPS_DU_UART_BAUD
    );
}

DuLieuGPS_DU Lay_DuLieu_GPS_DU()
{
    DuLieuGPS_DU du_lieu;
    portENTER_CRITICAL(&khoa_gps_du);
    du_lieu = anh_chup_gps_du;
    portEXIT_CRITICAL(&khoa_gps_du);
    return du_lieu;
}

void In_TrangThai_GPS_DU(const DuLieuGPS_DU &du_lieu_gps)
{
    Serial.printf(
        "[DU GPS] HOP_LE=%u | VI_DO=%.7f | KINH_DO=%.7f | DO_CAO=%.1fm | TOC_DO=%.2fm/s | VE_TINH=%u | HDOP=%.2f | TUOI_FIX=%u ms | NGAY_UTC=%u | GIO_UTC_MS=%u | NMEA_CHARS=%u\n",
        du_lieu_gps.gps_hop_le ? 1 : 0,
        du_lieu_gps.vi_do,
        du_lieu_gps.kinh_do,
        du_lieu_gps.do_cao_m,
        du_lieu_gps.toc_do_m_s,
        du_lieu_gps.so_ve_tinh,
        du_lieu_gps.hdop,
        (unsigned int)du_lieu_gps.tuoi_fix_ms,
        (unsigned int)du_lieu_gps.ngay_utc_yyyymmdd,
        (unsigned int)du_lieu_gps.gio_utc_ms_trong_ngay,
        (unsigned int)du_lieu_gps.so_ky_tu_nmea
    );
}
