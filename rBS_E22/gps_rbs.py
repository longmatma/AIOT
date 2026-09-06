import csv
import math
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from statistics import median

# ============================================================
# GPS / TELEMETRY V11.1
#
# Mục tiêu:
# - rBS có vị trí CỐ ĐỊNH.
# - SU và DU gửi GPS + trạng thái RF về rBS như trước.
# - Chỉ tính khoảng cách rBS <-> SU và rBS <-> DU.
# - KHÔNG còn tính khoảng cách SU <-> DU.
# - Giữ dữ liệu GPS thô trong CSV để phục vụ nghiên cứu về sau.
# - Dùng bộ lọc chất lượng + trung vị 3 mẫu cho khoảng cách định kỳ.
# ============================================================

# ============================================================
# TỌA ĐỘ CỐ ĐỊNH CỦA rBS
#
# Đây là tọa độ TẠM lấy từ SU/DU khi đặt cạnh rBS.
# Khi có phép đo ngoài trời tốt hơn chỉ cần sửa đúng 2 biến này.
# ============================================================
VI_DO_RBS = 20.9810400
KINH_DO_RBS = 105.7996883

# ============================================================
# LOẠI GÓI / ID
# ============================================================
LOAI_BAO_CAO_GPS_PHIEN = 0x17
LOAI_BAO_CAO_VI_TRI_DINH_KY = 0x18
KICH_THUOC_BAO_CAO_GPS = 44

MA_TRAM_SU = 0x01
MA_TRAM_DU = 0x02
MA_TRAM_RBS = 0x03

# Packet V11.1 vẫn giữ nguyên 44 byte của V11.
# Byte 29 = công suất phát hiện tại của node.
# 4 bit cao của byte flags (byte 3) mã hóa SF:
#   1 -> SF7, 2 -> SF8, 3 -> SF9.
CONG_SUAT_PHAT_MAC_DINH_DBM = {
    MA_TRAM_SU: 17.0,
    MA_TRAM_DU: 17.0,
}
HE_SO_TRAI_PHO_MAC_DINH = 7

# ============================================================
# NGƯỠNG GPS DÙNG ĐỂ CÔNG BỐ KHOẢNG CÁCH TIN CẬY
# ============================================================
GPS_MIN_VE_TINH_2D = 5
GPS_HDOP_TOI_DA_2D = 2.0
GPS_TUOI_FIX_TOI_DA_MS = 2000
GPS_SO_MAU_LOC = 3
GPS_MAU_TOI_DA_CU_S = 20.0

# Dùng file mới để không trộn header V10 với V11.
DUONG_DAN_CSV_PHIEN = Path('/home/admin/rbs_phien_v11.csv')
DUONG_DAN_CSV_DINH_KY = Path('/home/admin/rbs_lien_ket_dinh_ky_v11.csv')


def _doc_u32(du_lieu):
    return int.from_bytes(du_lieu, 'big', signed=False)


def _doc_i32(du_lieu):
    return int.from_bytes(du_lieu, 'big', signed=True)


def _doc_i8(gia_tri):
    return gia_tri - 256 if gia_tri >= 128 else gia_tri


def _ten_thiet_bi(nguon):
    return 'SU' if nguon == MA_TRAM_SU else 'DU'


def _dinh_dang_so(gia_tri, so_le=1, don_vi=''):
    if gia_tri is None:
        return 'N/A'
    return f'{gia_tri:.{so_le}f}{don_vi}'


def phan_tich_bao_cao_gps(goi_tin):
    """Phân tích packet GPS/telemetry 44B; chưa lọc tại bước này."""
    if goi_tin is None:
        return None

    du_lieu = bytes(goi_tin)
    if len(du_lieu) != KICH_THUOC_BAO_CAO_GPS:
        return None

    if du_lieu[0] != MA_TRAM_RBS or du_lieu[1] not in (MA_TRAM_SU, MA_TRAM_DU):
        return None

    loai_bao_cao = du_lieu[2]
    if loai_bao_cao not in (LOAI_BAO_CAO_GPS_PHIEN, LOAI_BAO_CAO_VI_TRI_DINH_KY):
        return None

    co_hieu = du_lieu[3]
    ma_tham_chieu = int.from_bytes(du_lieu[4:12], 'big', signed=False)
    if ma_tham_chieu == 0:
        return None

    cong_suat_phat_dbm = float(_doc_i8(du_lieu[29]))
    if cong_suat_phat_dbm == 0:
        cong_suat_phat_dbm = CONG_SUAT_PHAT_MAC_DINH_DBM[du_lieu[1]]

    # Packet V10 có 4 bit cao = 0, vì vậy vẫn tương thích ngược về SF7.
    ma_sf = (co_hieu >> 4) & 0x0F
    he_so_trai_pho = 6 + ma_sf if 1 <= ma_sf <= 6 else HE_SO_TRAI_PHO_MAC_DINH

    la_dinh_ky = loai_bao_cao == LOAI_BAO_CAO_VI_TRI_DINH_KY

    return {
        'nguon': du_lieu[1],
        'loai_bao_cao': loai_bao_cao,
        'la_dinh_ky': la_dinh_ky,
        'ma_phien': 0 if la_dinh_ky else ma_tham_chieu,
        'so_thu_tu_bao_cao': ma_tham_chieu if la_dinh_ky else 0,
        'gps_hop_le': bool(co_hieu & 0x01),
        'do_cao_hop_le': bool(co_hieu & 0x02),
        'toc_do_hop_le': bool(co_hieu & 0x04),
        'hdop_hop_le': bool(co_hieu & 0x08),
        'vi_do': _doc_i32(du_lieu[12:16]) / 1e7,
        'kinh_do': _doc_i32(du_lieu[16:20]) / 1e7,
        'do_cao_m': _doc_i32(du_lieu[20:24]) / 100.0,
        'toc_do_m_s': _doc_u32(du_lieu[24:28]) / 100.0,
        'so_ve_tinh': du_lieu[28],
        'cong_suat_phat_dbm': cong_suat_phat_dbm,
        'he_so_trai_pho': int(he_so_trai_pho),
        'hdop': int.from_bytes(du_lieu[30:32], 'big', signed=False) / 100.0,
        'tuoi_fix_ms': _doc_u32(du_lieu[32:36]),
        'ngay_utc_yyyymmdd': _doc_u32(du_lieu[36:40]),
        'gio_utc_ms_trong_ngay': _doc_u32(du_lieu[40:44]),
    }


def tinh_khoang_cach_haversine_m(vi_do_1, kinh_do_1, vi_do_2, kinh_do_2):
    """Khoảng cách 2D theo cung lớn của Trái Đất, đơn vị mét."""
    ban_kinh_trai_dat_m = 6371000.0
    p1 = math.radians(vi_do_1)
    p2 = math.radians(vi_do_2)
    dp = math.radians(vi_do_2 - vi_do_1)
    dl = math.radians(kinh_do_2 - kinh_do_1)

    a = (
        math.sin(dp / 2.0) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    )
    a = max(0.0, min(1.0, a))
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return ban_kinh_trai_dat_m * c


def _trung_binh(thong_ke, khoa_tong, khoa_dem):
    so_mau = thong_ke.get(khoa_dem, 0)
    return '' if so_mau <= 0 else thong_ke.get(khoa_tong, 0.0) / so_mau


class QuanLyGPSRBS:
    def __init__(
        self,
        duong_dan_csv_phien=DUONG_DAN_CSV_PHIEN,
        duong_dan_csv_dinh_ky=DUONG_DAN_CSV_DINH_KY,
    ):
        self.duong_dan_csv_phien = Path(duong_dan_csv_phien)
        self.duong_dan_csv_dinh_ky = Path(duong_dan_csv_dinh_ky)

        self.gan_nhat = {MA_TRAM_SU: None, MA_TRAM_DU: None}
        self.theo_phien = {}

        # Mỗi node có bộ lọc riêng; không còn ghép SU với DU để tính khoảng cách.
        self.lich_su_vi_tri_tot = {
            MA_TRAM_SU: deque(maxlen=GPS_SO_MAU_LOC),
            MA_TRAM_DU: deque(maxlen=GPS_SO_MAU_LOC),
        }

        self.thong_ke_beacon = {
            MA_TRAM_SU: self._tao_thong_ke_beacon(),
            MA_TRAM_DU: self._tao_thong_ke_beacon(),
        }

        print(
            f'[HỆ THỐNG] Tọa độ rBS cố định = '
            f'{VI_DO_RBS:.7f}, {KINH_DO_RBS:.7f} (tọa độ tạm để thử nghiệm)'
        )
        print(
            '[HỆ THỐNG] Bộ lọc GPS = '
            f'>={GPS_MIN_VE_TINH_2D} vệ tinh | '
            f'HDOP<={GPS_HDOP_TOI_DA_2D:.1f} | '
            f'tuổi fix<={GPS_TUOI_FIX_TOI_DA_MS}ms | '
            f'trung vị {GPS_SO_MAU_LOC} mẫu'
        )

    @staticmethod
    def _tao_thong_ke_beacon():
        return {
            'stt_cuoi': None,
            'so_goi_nhan': 0,
            'so_goi_ky_vong': 0,
            'so_goi_mat_uoc_tinh': 0,
            'so_goi_trung': 0,
        }

    @staticmethod
    def _tinh_kenh(bao_cao, rssi_dbm, snr_db):
        ket_qua = dict(bao_cao)
        ket_qua['rssi_dbm'] = None if rssi_dbm is None else float(rssi_dbm)
        ket_qua['snr_db'] = None if snr_db is None else float(snr_db)
        ket_qua['thoi_gian_rbs_nhan_utc'] = datetime.now(timezone.utc).isoformat()
        ket_qua['moc_rbs_nhan_monotonic'] = time.monotonic()

        if rssi_dbm is None:
            ket_qua['he_so_kenh_db'] = None
            ket_qua['suy_hao_kenh_db'] = None
            ket_qua['he_so_cong_suat_kenh'] = None
            return ket_qua

        # Đây là ước lượng channel power gain theo RSSI, không phải CSI phức.
        # H_kênh[dB] = P_rx[dBm] - P_tx[dBm].
        he_so_kenh_db = float(rssi_dbm) - float(bao_cao['cong_suat_phat_dbm'])
        ket_qua['he_so_kenh_db'] = he_so_kenh_db
        ket_qua['suy_hao_kenh_db'] = -he_so_kenh_db
        ket_qua['he_so_cong_suat_kenh'] = 10.0 ** (he_so_kenh_db / 10.0)
        return ket_qua

    @staticmethod
    def _gps_du_tin_cay_2d(du_lieu):
        if not du_lieu:
            return False

        return (
            bool(du_lieu.get('gps_hop_le'))
            and bool(du_lieu.get('hdop_hop_le'))
            and du_lieu.get('so_ve_tinh', 0) >= GPS_MIN_VE_TINH_2D
            and 0.0 < du_lieu.get('hdop', 999.0) <= GPS_HDOP_TOI_DA_2D
            and du_lieu.get('tuoi_fix_ms', 0xFFFFFFFF) <= GPS_TUOI_FIX_TOI_DA_MS
            and -90.0 <= du_lieu.get('vi_do', 999.0) <= 90.0
            and -180.0 <= du_lieu.get('kinh_do', 999.0) <= 180.0
        )

    def _cap_nhat_pdr(self, du_lieu):
        nguon = du_lieu['nguon']
        stt = int(du_lieu['so_thu_tu_bao_cao'])
        tk = self.thong_ke_beacon[nguon]
        stt_cuoi = tk['stt_cuoi']

        if stt_cuoi is None:
            tk['stt_cuoi'] = stt
            tk['so_goi_nhan'] = 1
            tk['so_goi_ky_vong'] = 1
        elif stt == stt_cuoi:
            tk['so_goi_trung'] += 1
        elif stt > stt_cuoi:
            khoang_nhay = stt - stt_cuoi
            tk['so_goi_ky_vong'] += khoang_nhay
            tk['so_goi_nhan'] += 1
            tk['so_goi_mat_uoc_tinh'] += max(0, khoang_nhay - 1)
            tk['stt_cuoi'] = stt
        else:
            # Node reboot / counter reset -> mở một cửa sổ thống kê mới.
            self.thong_ke_beacon[nguon] = self._tao_thong_ke_beacon()
            tk = self.thong_ke_beacon[nguon]
            tk['stt_cuoi'] = stt
            tk['so_goi_nhan'] = 1
            tk['so_goi_ky_vong'] = 1

        du_lieu['pdr_beacon_phan_tram'] = (
            100.0 * tk['so_goi_nhan'] / tk['so_goi_ky_vong']
            if tk['so_goi_ky_vong'] > 0
            else None
        )
        du_lieu['so_goi_mat_uoc_tinh'] = tk['so_goi_mat_uoc_tinh']
        du_lieu['so_goi_trung'] = tk['so_goi_trung']

    def _cap_nhat_bo_loc(self, du_lieu):
        du_lieu['gps_tin_cay_2d'] = self._gps_du_tin_cay_2d(du_lieu)

        if not du_lieu['la_dinh_ky']:
            return

        # V11.1: khoang cach chi duoc cong bo sau 3 mau GPS TOT lien tiep.
        # Neu mau hien tai khong dat nguong, xoa cua so loc cu de khong dung
        # lai toa do tot cua 10-20 giay truoc trong luc GPS hien tai dang xau.
        if not du_lieu['gps_tin_cay_2d']:
            self.lich_su_vi_tri_tot[du_lieu['nguon']].clear()
            return

        self.lich_su_vi_tri_tot[du_lieu['nguon']].append({
            'vi_do': du_lieu['vi_do'],
            'kinh_do': du_lieu['kinh_do'],
            'moc': du_lieu['moc_rbs_nhan_monotonic'],
        })

    def _lay_vi_tri_loc(self, nguon):
        lich_su = self.lich_su_vi_tri_tot[nguon]
        if len(lich_su) < GPS_SO_MAU_LOC:
            return None

        bay_gio = time.monotonic()
        mau = [x for x in lich_su if bay_gio - x['moc'] <= GPS_MAU_TOI_DA_CU_S]
        if len(mau) < GPS_SO_MAU_LOC:
            return None

        return {
            'vi_do': median([x['vi_do'] for x in mau]),
            'kinh_do': median([x['kinh_do'] for x in mau]),
            'moc': max(x['moc'] for x in mau),
        }

    def _tinh_khoang_cach_rbs_tu_vi_tri(self, vi_tri):
        if not vi_tri:
            return None
        return tinh_khoang_cach_haversine_m(
            VI_DO_RBS,
            KINH_DO_RBS,
            vi_tri['vi_do'],
            vi_tri['kinh_do'],
        )

    def khoang_cach_rbs_node_gan_nhat(self, nguon):
        """Khoảng cách định kỳ dùng vị trí đã lọc trung vị 3 mẫu."""
        return self._tinh_khoang_cach_rbs_tu_vi_tri(self._lay_vi_tri_loc(nguon))

    def khoang_cach_rbs_node_theo_phien(self, ma_phien, nguon):
        """Khoảng cách theo snapshot của phiên nếu snapshot đạt ngưỡng GPS."""
        du_lieu = self.lay(ma_phien, nguon)
        if not self._gps_du_tin_cay_2d(du_lieu):
            return None
        return tinh_khoang_cach_haversine_m(
            VI_DO_RBS,
            KINH_DO_RBS,
            du_lieu['vi_do'],
            du_lieu['kinh_do'],
        )

    def cap_nhat(self, bao_cao, rssi=None, snr=None):
        if bao_cao is None:
            return None

        du_lieu = self._tinh_kenh(bao_cao, rssi, snr)
        self._cap_nhat_bo_loc(du_lieu)

        if du_lieu['la_dinh_ky']:
            self._cap_nhat_pdr(du_lieu)

        self.gan_nhat[du_lieu['nguon']] = du_lieu

        if du_lieu['ma_phien'] != 0:
            phien = self.theo_phien.setdefault(du_lieu['ma_phien'], {})
            phien[du_lieu['nguon']] = du_lieu

        if du_lieu['la_dinh_ky']:
            self.ghi_csv_dinh_ky(du_lieu)

        return du_lieu

    def lay(self, ma_phien, nguon):
        return self.theo_phien.get(ma_phien, {}).get(nguon)

    def in_bao_cao_dinh_ky(self, du_lieu):
        if du_lieu is None:
            return

        ten = _ten_thiet_bi(du_lieu['nguon'])
        gps_tin_cay = bool(du_lieu.get('gps_tin_cay_2d'))

        # Chỉ hiện khoảng cách khi CHÍNH MẪU HIỆN TẠI đạt ngưỡng GPS
        # và bộ lọc đã có đủ 3 mẫu tốt liên tiếp.
        khoang_cach_m = (
            self.khoang_cach_rbs_node_gan_nhat(du_lieu['nguon'])
            if gps_tin_cay
            else None
        )

        gps_text = 'TỐT' if gps_tin_cay else 'CHƯA_TỐT'
        vi_do_text = (
            f"{du_lieu['vi_do']:.7f}"
            if du_lieu.get('gps_hop_le')
            else 'N/A'
        )
        kinh_do_text = (
            f"{du_lieu['kinh_do']:.7f}"
            if du_lieu.get('gps_hop_le')
            else 'N/A'
        )

        # PDR vẫn được tính và giữ nội bộ/CSV để bộ điều khiển RF sử dụng,
        # nhưng không in ở log vận hành bình thường để log gọn hơn.
        print(
            f'[LIÊN KẾT] {ten}->rBS | '
            f'VĨ_ĐỘ={vi_do_text} | KINH_ĐỘ={kinh_do_text} | '
            f'GPS={gps_text} | KC_rBS={_dinh_dang_so(khoang_cach_m, 1, "m")} | '
            f'VỆ_TINH={du_lieu["so_ve_tinh"]} | HDOP={du_lieu["hdop"]:.2f} | '
            f'CÔNG_SUẤT={du_lieu["cong_suat_phat_dbm"]:.0f}dBm | SF={du_lieu["he_so_trai_pho"]} | '
            f'RSSI={_dinh_dang_so(du_lieu["rssi_dbm"], 1, "dBm")} | '
            f'SNR={_dinh_dang_so(du_lieu["snr_db"], 1, "dB")} | '
            f'HỆ_SỐ_KÊNH={_dinh_dang_so(du_lieu["he_so_kenh_db"], 1, "dB")}'
        )

    def ghi_csv_dinh_ky(self, du_lieu):
        khoang_cach_m = self.khoang_cach_rbs_node_gan_nhat(du_lieu['nguon'])

        dong = {
            'Thoi_gian_rBS_nhan_UTC': du_lieu['thoi_gian_rbs_nhan_utc'],
            'Thiet_bi': _ten_thiet_bi(du_lieu['nguon']),
            'So_thu_tu_bao_cao': du_lieu['so_thu_tu_bao_cao'],
            'GPS_hop_le': int(du_lieu['gps_hop_le']),
            'GPS_tin_cay_2D': int(bool(du_lieu.get('gps_tin_cay_2d'))),
            'Vi_do_tho': du_lieu['vi_do'],
            'Kinh_do_tho': du_lieu['kinh_do'],
            'Do_cao_tho_m': du_lieu['do_cao_m'],
            'Toc_do_m_s': du_lieu['toc_do_m_s'],
            'So_ve_tinh': du_lieu['so_ve_tinh'],
            'HDOP': du_lieu['hdop'],
            'Tuoi_fix_ms': du_lieu['tuoi_fix_ms'],
            'Cong_suat_phat_dBm': du_lieu['cong_suat_phat_dbm'],
            'SF': du_lieu['he_so_trai_pho'],
            'RSSI_dBm': '' if du_lieu['rssi_dbm'] is None else du_lieu['rssi_dbm'],
            'SNR_dB': '' if du_lieu['snr_db'] is None else du_lieu['snr_db'],
            'He_so_kenh_dB': '' if du_lieu['he_so_kenh_db'] is None else du_lieu['he_so_kenh_db'],
            'Suy_hao_kenh_dB': '' if du_lieu['suy_hao_kenh_db'] is None else du_lieu['suy_hao_kenh_db'],
            'PDR_beacon_phan_tram': '' if du_lieu.get('pdr_beacon_phan_tram') is None else round(du_lieu['pdr_beacon_phan_tram'], 3),
            'So_goi_mat_uoc_tinh': du_lieu.get('so_goi_mat_uoc_tinh', 0),
            'Vi_do_rBS': VI_DO_RBS,
            'Kinh_do_rBS': KINH_DO_RBS,
            'Khoang_cach_rBS_node_m': '' if khoang_cach_m is None else round(khoang_cach_m, 2),
        }

        self.duong_dan_csv_dinh_ky.parent.mkdir(parents=True, exist_ok=True)
        can_ghi_tieu_de = (
            not self.duong_dan_csv_dinh_ky.exists()
            or self.duong_dan_csv_dinh_ky.stat().st_size == 0
        )
        with self.duong_dan_csv_dinh_ky.open('a', newline='', encoding='utf-8') as tep:
            bo_ghi = csv.DictWriter(tep, fieldnames=list(dong.keys()))
            if can_ghi_tieu_de:
                bo_ghi.writeheader()
            bo_ghi.writerow(dong)

    def in_tom_tat(self, ma_phien):
        for nguon in (MA_TRAM_SU, MA_TRAM_DU):
            du_lieu = self.lay(ma_phien, nguon)
            ten = _ten_thiet_bi(nguon)
            if not du_lieu:
                print(f'[PHIÊN THOẠI] {ten}: chưa có GPS trong phiên')
                continue

            gps_tin_cay = self._gps_du_tin_cay_2d(du_lieu)
            kc = self.khoang_cach_rbs_node_theo_phien(ma_phien, nguon)
            vi_do_text = (
                f"{du_lieu.get('vi_do', 0.0):.7f}"
                if du_lieu.get('gps_hop_le')
                else 'N/A'
            )
            kinh_do_text = (
                f"{du_lieu.get('kinh_do', 0.0):.7f}"
                if du_lieu.get('gps_hop_le')
                else 'N/A'
            )

            print(
                f'[PHIÊN THOẠI] {ten}->rBS | '
                f'VĨ_ĐỘ={vi_do_text} | KINH_ĐỘ={kinh_do_text} | '
                f'GPS={"TỐT" if gps_tin_cay else "CHƯA_TỐT"} | '
                f'KC_rBS={_dinh_dang_so(kc, 1, "m")} | '
                f'RSSI={_dinh_dang_so(du_lieu.get("rssi_dbm"), 1, "dBm")} | '
                f'SNR={_dinh_dang_so(du_lieu.get("snr_db"), 1, "dB")} | '
                f'HỆ_SỐ_KÊNH={_dinh_dang_so(du_lieu.get("he_so_kenh_db"), 1, "dB")}'
            )

    def ghi_csv_session(self, diag, ly_do):
        if diag is None:
            return

        ma_phien = diag['session_id']
        su = self.lay(ma_phien, MA_TRAM_SU) or {}
        du = self.lay(ma_phien, MA_TRAM_DU) or {}
        kc_su_rbs = self.khoang_cach_rbs_node_theo_phien(ma_phien, MA_TRAM_SU)
        kc_du_rbs = self.khoang_cach_rbs_node_theo_phien(ma_phien, MA_TRAM_DU)
        thong_ke_su = diag['link_su_rbs']
        thong_ke_du = diag['link_du_rbs']

        dong = {
            'Thoi_gian_UTC': datetime.now(timezone.utc).isoformat(),
            'Ly_do_ghi': ly_do,
            'Ma_phien': f'{ma_phien:016X}',
            'Vi_do_rBS': VI_DO_RBS,
            'Kinh_do_rBS': KINH_DO_RBS,
            'SU_Vi_do': su.get('vi_do', ''),
            'SU_Kinh_do': su.get('kinh_do', ''),
            'SU_GPS_tin_cay_2D': int(self._gps_du_tin_cay_2d(su)),
            'SU_Khoang_cach_rBS_m': '' if kc_su_rbs is None else round(kc_su_rbs, 2),
            'SU_Cong_suat_phat_dBm': su.get('cong_suat_phat_dbm', ''),
            'SU_SF': su.get('he_so_trai_pho', ''),
            'SU_He_so_kenh_dB': su.get('he_so_kenh_db', ''),
            'DU_Vi_do': du.get('vi_do', ''),
            'DU_Kinh_do': du.get('kinh_do', ''),
            'DU_GPS_tin_cay_2D': int(self._gps_du_tin_cay_2d(du)),
            'DU_Khoang_cach_rBS_m': '' if kc_du_rbs is None else round(kc_du_rbs, 2),
            'DU_Cong_suat_phat_dBm': du.get('cong_suat_phat_dbm', ''),
            'DU_SF': du.get('he_so_trai_pho', ''),
            'DU_He_so_kenh_dB': du.get('he_so_kenh_db', ''),
            'RSSI_SU_rBS_TB_dBm': _trung_binh(thong_ke_su, 'rssi_sum', 'rssi_count'),
            'SNR_SU_rBS_TB_dB': _trung_binh(thong_ke_su, 'snr_sum', 'snr_count'),
            'RSSI_DU_rBS_TB_dBm': _trung_binh(thong_ke_du, 'rssi_sum', 'rssi_count'),
            'SNR_DU_rBS_TB_dB': _trung_binh(thong_ke_du, 'snr_sum', 'snr_count'),
            'ARQ_retry_quan_sat': diag.get('so_goi_trung_arq', 0),
            'ACK_NACK': diag.get('hmi_ket_qua', ''),
            'Tan_so_MHz': 433.0,
            'BW_Hz': 500000,
            'CR': '4/5',
        }

        self.duong_dan_csv_phien.parent.mkdir(parents=True, exist_ok=True)
        can_ghi_tieu_de = (
            not self.duong_dan_csv_phien.exists()
            or self.duong_dan_csv_phien.stat().st_size == 0
        )
        with self.duong_dan_csv_phien.open('a', newline='', encoding='utf-8') as tep:
            bo_ghi = csv.DictWriter(tep, fieldnames=list(dong.keys()))
            if can_ghi_tieu_de:
                bo_ghi.writeheader()
            bo_ghi.writerow(dong)
