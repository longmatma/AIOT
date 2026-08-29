#include "gps_su.h"

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

static TinyGPSPlus bo_phan_tich_gps_su;
static HardwareSerial uart_gps_su(1);
static portMUX_TYPE khoa_gps_su = portMUX_INITIALIZER_UNLOCKED;
static DuLieuGPS_SU anh_chup_gps_su = {};

static uint32_t Tao_Ngay_UTC_SU()
{
    if (!bo_phan_tich_gps_su.date.isValid()) return 0;
    return (uint32_t)bo_phan_tich_gps_su.date.year() * 10000UL
         + (uint32_t)bo_phan_tich_gps_su.date.month() * 100UL
         + (uint32_t)bo_phan_tich_gps_su.date.day();
}

static uint32_t Tao_Gio_UTC_ms_SU()
{
    if (!bo_phan_tich_gps_su.time.isValid()) return 0;
    return ((uint32_t)bo_phan_tich_gps_su.time.hour() * 3600UL
          + (uint32_t)bo_phan_tich_gps_su.time.minute() * 60UL
          + (uint32_t)bo_phan_tich_gps_su.time.second()) * 1000UL
          + (uint32_t)bo_phan_tich_gps_su.time.centisecond() * 10UL;
}

static void CapNhat_AnhChup_GPS_SU()
{
    DuLieuGPS_SU du_lieu = {};

    uint32_t tuoi_fix = bo_phan_tich_gps_su.location.isValid()
        ? bo_phan_tich_gps_su.location.age()
        : UINT32_MAX;

    du_lieu.tuoi_fix_ms = tuoi_fix;
    du_lieu.gps_hop_le = bo_phan_tich_gps_su.location.isValid()
                      && tuoi_fix <= GPS_SU_TUOI_FIX_TOI_DA_MS;

    if (bo_phan_tich_gps_su.location.isValid())
    {
        du_lieu.vi_do = bo_phan_tich_gps_su.location.lat();
        du_lieu.kinh_do = bo_phan_tich_gps_su.location.lng();
    }

    du_lieu.do_cao_hop_le = bo_phan_tich_gps_su.altitude.isValid();
    if (du_lieu.do_cao_hop_le)
        du_lieu.do_cao_m = bo_phan_tich_gps_su.altitude.meters();

    du_lieu.toc_do_hop_le = bo_phan_tich_gps_su.speed.isValid();
    if (du_lieu.toc_do_hop_le)
        du_lieu.toc_do_m_s = bo_phan_tich_gps_su.speed.mps();

    du_lieu.so_ve_tinh = bo_phan_tich_gps_su.satellites.isValid()
        ? (uint8_t)min((uint32_t)255, bo_phan_tich_gps_su.satellites.value())
        : 0;

    du_lieu.hdop_hop_le = bo_phan_tich_gps_su.hdop.isValid();
    if (du_lieu.hdop_hop_le)
        du_lieu.hdop = bo_phan_tich_gps_su.hdop.hdop();

    du_lieu.ngay_utc_yyyymmdd = Tao_Ngay_UTC_SU();
    du_lieu.gio_utc_ms_trong_ngay = Tao_Gio_UTC_ms_SU();
    du_lieu.so_ky_tu_nmea = bo_phan_tich_gps_su.charsProcessed();

    portENTER_CRITICAL(&khoa_gps_su);
    anh_chup_gps_su = du_lieu;
    portEXIT_CRITICAL(&khoa_gps_su);
}

static void TacVu_GPS_SU(void *tham_so)
{
    (void)tham_so;

    while (true)
    {
        while (uart_gps_su.available() > 0)
        {
            char ky_tu = (char)uart_gps_su.read();
            bo_phan_tich_gps_su.encode(ky_tu);
        }

        // Cap nhat snapshot lien tuc: toa do, tuoi fix va bo dem NMEA.
        CapNhat_AnhChup_GPS_SU();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void KhoiTao_GPS_SU()
{
    uart_gps_su.setRxBufferSize(2048);
    uart_gps_su.begin(
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
        "[SU GPS] UART khoi tao | RX=%d | TX=%d | BAUD=%d\n",
        GPS_SU_UART_RX_PIN,
        GPS_SU_UART_TX_PIN,
        GPS_SU_UART_BAUD
    );
}

DuLieuGPS_SU Lay_DuLieu_GPS_SU()
{
    DuLieuGPS_SU du_lieu;
    portENTER_CRITICAL(&khoa_gps_su);
    du_lieu = anh_chup_gps_su;
    portEXIT_CRITICAL(&khoa_gps_su);
    return du_lieu;
}

void In_TrangThai_GPS_SU(const DuLieuGPS_SU &du_lieu_gps)
{
    Serial.printf(
        "[SU GPS] HOP_LE=%u | VI_DO=%.7f | KINH_DO=%.7f | DO_CAO=%.1fm | TOC_DO=%.2fm/s | VE_TINH=%u | HDOP=%.2f | TUOI_FIX=%u ms | NGAY_UTC=%u | GIO_UTC_MS=%u | NMEA_CHARS=%u\n",
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
