import csv
import math
from datetime import datetime, timezone
from pathlib import Path

# ============================================================
# CẤU HÌNH TỌA ĐỘ rBS CỐ ĐỊNH
#
# KHÔNG điền số giả. Khi đo thực địa, thay None bằng tọa độ thật.
# Ví dụ:
# RBS_LAT = 21.028511
# RBS_LON = 105.804817
# ============================================================
RBS_LAT = None
RBS_LON = None

TYPE_GPS_REPORT = 0x17
SIZE_GPS_REPORT = 44

ID_TRAM_SU = 0x01
ID_TRAM_DU = 0x02
ID_TRAM_RBS = 0x03

CSV_PATH = Path('/home/admin/rbs_gps_link_log.csv')


def _u32(p):
    return int.from_bytes(p, 'big', signed=False)


def _i32(p):
    return int.from_bytes(p, 'big', signed=True)


def parse_gps_report(packet_bytes):
    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)
    if len(p) != SIZE_GPS_REPORT:
        return None

    if p[0] != ID_TRAM_RBS or p[1] not in (ID_TRAM_SU, ID_TRAM_DU) or p[2] != TYPE_GPS_REPORT:
        return None

    flags = p[3]
    sid = int.from_bytes(p[4:12], 'big', signed=False)
    if sid == 0:
        return None

    return {
        'src': p[1],
        'session_id': sid,
        'gps_valid': bool(flags & 0x01),
        'altitude_valid': bool(flags & 0x02),
        'speed_valid': bool(flags & 0x04),
        'hdop_valid': bool(flags & 0x08),
        'latitude': _i32(p[12:16]) / 1e7,
        'longitude': _i32(p[16:20]) / 1e7,
        'altitude_m': _i32(p[20:24]) / 100.0,
        'speed_mps': _u32(p[24:28]) / 100.0,
        'satellites': p[28],
        'hdop': int.from_bytes(p[30:32], 'big', signed=False) / 100.0,
        'fix_age_ms': _u32(p[32:36]),
        'utc_date_yyyymmdd': _u32(p[36:40]),
        'utc_time_ms_of_day': _u32(p[40:44]),
    }


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r * c


def _avg(link, key_sum, key_count):
    n = link.get(key_count, 0)
    return '' if n <= 0 else link.get(key_sum, 0.0) / n


class QuanLyGPSRBS:
    def __init__(self, csv_path=CSV_PATH):
        self.csv_path = Path(csv_path)
        self.latest = {ID_TRAM_SU: None, ID_TRAM_DU: None}
        self.theo_session = {}
        self._csv_header_written = self.csv_path.exists() and self.csv_path.stat().st_size > 0

        if RBS_LAT is None or RBS_LON is None:
            print('[rBS GPS] CHƯA KHAI BÁO RBS_LAT/RBS_LON trong gps_rbs.py -> khoảng cách tới rBS sẽ là N/A.')
        else:
            print(f'[rBS GPS] Tọa độ cố định rBS = {RBS_LAT:.7f}, {RBS_LON:.7f}')

    def cap_nhat(self, report, rssi=None, snr=None):
        if report is None:
            return

        x = dict(report)
        x['rssi'] = rssi
        x['snr'] = snr
        x['rbs_rx_timestamp_utc'] = datetime.now(timezone.utc).isoformat()

        self.latest[x['src']] = x
        sess = self.theo_session.setdefault(x['session_id'], {})
        sess[x['src']] = x

    def lay(self, session_id, src):
        return self.theo_session.get(session_id, {}).get(src)

    def tinh_khoang_cach(self, session_id):
        su = self.lay(session_id, ID_TRAM_SU)
        du = self.lay(session_id, ID_TRAM_DU)

        d_su_du = None
        d_su_rbs = None
        d_du_rbs = None

        if su and du and su['gps_valid'] and du['gps_valid']:
            d_su_du = haversine_m(su['latitude'], su['longitude'], du['latitude'], du['longitude'])

        if RBS_LAT is not None and RBS_LON is not None:
            if su and su['gps_valid']:
                d_su_rbs = haversine_m(su['latitude'], su['longitude'], RBS_LAT, RBS_LON)
            if du and du['gps_valid']:
                d_du_rbs = haversine_m(du['latitude'], du['longitude'], RBS_LAT, RBS_LON)

        return d_su_du, d_su_rbs, d_du_rbs

    def in_tom_tat(self, session_id):
        su = self.lay(session_id, ID_TRAM_SU)
        du = self.lay(session_id, ID_TRAM_DU)
        d_su_du, d_su_rbs, d_du_rbs = self.tinh_khoang_cach(session_id)

        print('[GPS / KHOẢNG CÁCH]')
        if su:
            print(
                f"  SU: VALID={1 if su['gps_valid'] else 0} | LAT={su['latitude']:.7f} | LON={su['longitude']:.7f} | "
                f"ALT={su['altitude_m']:.1f}m | SPEED={su['speed_mps']:.2f}m/s | SAT={su['satellites']} | "
                f"HDOP={su['hdop']:.2f} | AGE={su['fix_age_ms']}ms"
            )
        else:
            print('  SU: CHƯA NHẬN GPS_REPORT')

        if du:
            print(
                f"  DU: VALID={1 if du['gps_valid'] else 0} | LAT={du['latitude']:.7f} | LON={du['longitude']:.7f} | "
                f"ALT={du['altitude_m']:.1f}m | SPEED={du['speed_mps']:.2f}m/s | SAT={du['satellites']} | "
                f"HDOP={du['hdop']:.2f} | AGE={du['fix_age_ms']}ms"
            )
        else:
            print('  DU: CHƯA NHẬN GPS_REPORT')

        print(f"  DIST SU<->DU  = {'N/A' if d_su_du is None else f'{d_su_du:.1f} m'}")
        print(f"  DIST SU<->rBS = {'N/A' if d_su_rbs is None else f'{d_su_rbs:.1f} m'}")
        print(f"  DIST DU<->rBS = {'N/A' if d_du_rbs is None else f'{d_du_rbs:.1f} m'}")

    def ghi_csv_session(self, diag, ly_do):
        if diag is None:
            return

        sid = diag['session_id']
        su = self.lay(sid, ID_TRAM_SU) or {}
        du = self.lay(sid, ID_TRAM_DU) or {}
        d_su_du, d_su_rbs, d_du_rbs = self.tinh_khoang_cach(sid)
        ls = diag['link_su_rbs']
        ld = diag['link_du_rbs']

        row = {
            'Timestamp_UTC': datetime.now(timezone.utc).isoformat(),
            'Record_Reason': ly_do,
            'Session_ID': f'{sid:016X}',
            'SU_GPS_Valid': int(bool(su.get('gps_valid', False))),
            'SU_Latitude': su.get('latitude', ''),
            'SU_Longitude': su.get('longitude', ''),
            'SU_Altitude_m': su.get('altitude_m', ''),
            'SU_Speed_mps': su.get('speed_mps', ''),
            'SU_Satellites': su.get('satellites', ''),
            'SU_HDOP': su.get('hdop', ''),
            'SU_Fix_Age_ms': su.get('fix_age_ms', ''),
            'SU_GPS_UTC_Date': su.get('utc_date_yyyymmdd', ''),
            'SU_GPS_UTC_ms': su.get('utc_time_ms_of_day', ''),
            'DU_GPS_Valid': int(bool(du.get('gps_valid', False))),
            'DU_Latitude': du.get('latitude', ''),
            'DU_Longitude': du.get('longitude', ''),
            'DU_Altitude_m': du.get('altitude_m', ''),
            'DU_Speed_mps': du.get('speed_mps', ''),
            'DU_Satellites': du.get('satellites', ''),
            'DU_HDOP': du.get('hdop', ''),
            'DU_Fix_Age_ms': du.get('fix_age_ms', ''),
            'DU_GPS_UTC_Date': du.get('utc_date_yyyymmdd', ''),
            'DU_GPS_UTC_ms': du.get('utc_time_ms_of_day', ''),
            'rBS_Latitude': '' if RBS_LAT is None else RBS_LAT,
            'rBS_Longitude': '' if RBS_LON is None else RBS_LON,
            'Distance_SU_DU_m': '' if d_su_du is None else round(d_su_du, 2),
            'Distance_SU_rBS_m': '' if d_su_rbs is None else round(d_su_rbs, 2),
            'Distance_DU_rBS_m': '' if d_du_rbs is None else round(d_du_rbs, 2),
            'RSSI_SU_rBS_Avg_dBm': _avg(ls, 'rssi_sum', 'rssi_count'),
            'SNR_SU_rBS_Avg_dB': _avg(ls, 'snr_sum', 'snr_count'),
            'RSSI_DU_rBS_Avg_dBm': _avg(ld, 'rssi_sum', 'rssi_count'),
            'SNR_DU_rBS_Avg_dB': _avg(ld, 'snr_sum', 'snr_count'),
            'ARQ_Retry_Observed': diag.get('so_goi_trung_arq', 0),
            'Session_Start_Attempts': diag.get('so_lan_start_toi_du', 0),
            'ACK_NACK': diag.get('hmi_ket_qua', ''),
            'Frequency_MHz': 433.0,
            'SF': 7,
            'BW_Hz': 500000,
            'CR': '4/5',
            'rBS_Tx_Power_dBm': 23,
            # Chua du du lieu de tinh dung cac cot nay tai rBS.
            'PDR': '',
            'Packet_Loss': '',
            'FEC_Recovered': '',
            'E2E_Latency_ms': '',
        }

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self.csv_path.exists() or self.csv_path.stat().st_size == 0
        with self.csv_path.open('a', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            if write_header:
                w.writeheader()
            w.writerow(row)

        print(f'[rBS GPS CSV] Đã ghi: {self.csv_path}')
