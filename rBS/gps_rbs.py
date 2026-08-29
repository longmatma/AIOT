import csv
import math
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from statistics import median

# ============================================================
# GPS / TELEMETRY V10
#
# Muc tieu:
# 1) Giu NGUYEN packet GPS 44B cua V9 -> khong pha giao thuc tren khong trung.
# 2) Luu du lieu GPS THO de phuc vu nghien cuu/AI ve sau.
# 3) Khong coi "location valid" cua TinyGPSPlus la "GPS du tin cay".
# 4) Loc trung vi 3 mau cho vi tri dung de tinh khoang cach 2D.
# 5) Khong dung do cao NEO-6M de cong bo khoang cach 3D tin cay.
# 6) Theo doi STT beacon/PDR sau khi V10 SU/DU chi tang STT khi TX thanh cong.
# ============================================================

# ============================================================
# TOA DO rBS CO DINH
# Neu chua co toa do rBS thi de None; SU <-> DU van co the tinh.
# ============================================================
VI_DO_RBS = None
KINH_DO_RBS = None
DO_CAO_RBS_M = None

LOAI_BAO_CAO_GPS_PHIEN = 0x17
LOAI_BAO_CAO_VI_TRI_DINH_KY = 0x18
KICH_THUOC_BAO_CAO_GPS = 44

MA_TRAM_SU = 0x01
MA_TRAM_DU = 0x02
MA_TRAM_RBS = 0x03

# Cong suat duoc SU/DU V10 ghi vao byte 29 cua packet telemetry.
CONG_SUAT_PHAT_MAC_DINH_DBM = {
    MA_TRAM_SU: 17.0,
    MA_TRAM_DU: 17.0,
}

# ============================================================
# NGUONG CHAT LUONG GPS DUNG CHO KHOANG CACH 2D
#
# HOP_LE=1 chi co nghia parser co location va fix chua qua cu.
# De tinh khoang cach "TIN_CAY", V10 yeu cau chat hon:
# - >= 5 ve tinh
# - HDOP <= 2.0
# - fix age <= 2 s
# - co du 3 mau tot de loc trung vi
# ============================================================
GPS_MIN_VE_TINH_2D = 5
GPS_HDOP_TOI_DA_2D = 2.0
GPS_TUOI_FIX_TOI_DA_MS = 2000
GPS_SO_MAU_LOC = 3
GPS_MAU_TOI_DA_CU_S = 20.0
GPS_CAP_SU_DU_TOI_DA_LECH_S = 10.0

# NEO-6M khong cung cap vertical accuracy/RTK trong packet hien tai.
# Do cao tho van duoc log, nhung KC_3D "tin cay" mac dinh khong cong bo.
CHO_PHEP_KHOANG_CACH_3D_TIN_CAY = False

DUONG_DAN_CSV_PHIEN = Path('/home/admin/rbs_gps_link_log_v10.csv')
DUONG_DAN_CSV_DINH_KY = Path('/home/admin/rbs_gps_dinh_ky_v10.csv')


def _doc_u32(du_lieu):
    return int.from_bytes(du_lieu, 'big', signed=False)


def _doc_i32(du_lieu):
    return int.from_bytes(du_lieu, 'big', signed=True)


def _doc_i8(gia_tri):
    return gia_tri - 256 if gia_tri >= 128 else gia_tri


def phan_tich_bao_cao_gps(goi_tin):
    """Phan tich packet GPS/vi tri 44B. Khong loc o day de giu du lieu tho."""
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
    # Tuong thich packet cu: byte29 reserved=0.
    if cong_suat_phat_dbm == 0:
        cong_suat_phat_dbm = CONG_SUAT_PHAT_MAC_DINH_DBM[du_lieu[1]]

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
        'hdop': int.from_bytes(du_lieu[30:32], 'big', signed=False) / 100.0,
        'tuoi_fix_ms': _doc_u32(du_lieu[32:36]),
        'ngay_utc_yyyymmdd': _doc_u32(du_lieu[36:40]),
        'gio_utc_ms_trong_ngay': _doc_u32(du_lieu[40:44]),
    }


def tinh_khoang_cach_haversine_m(vi_do_1, kinh_do_1, vi_do_2, kinh_do_2):
    """Khoang cach cung lon 2D tren mat dat, don vi met."""
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


def _ten_thiet_bi(nguon):
    return 'SU' if nguon == MA_TRAM_SU else 'DU'


class QuanLyGPSRBS:
    def __init__(
        self,
        duong_dan_csv_phien=DUONG_DAN_CSV_PHIEN,
        duong_dan_csv_dinh_ky=DUONG_DAN_CSV_DINH_KY,
    ):
        self.duong_dan_csv_phien = Path(duong_dan_csv_phien)
        self.duong_dan_csv_dinh_ky = Path(duong_dan_csv_dinh_ky)

        # Mau gan nhat THO cua tung thiet bi.
        self.gan_nhat = {MA_TRAM_SU: None, MA_TRAM_DU: None}
        self.theo_phien = {}

        # Lich su chi luu beacon dat nguong chat luong de loc 2D.
        self.lich_su_vi_tri_tot = {
            MA_TRAM_SU: deque(maxlen=GPS_SO_MAU_LOC),
            MA_TRAM_DU: deque(maxlen=GPS_SO_MAU_LOC),
        }

        # PDR quan sat theo STT beacon. Lan dau rBS nhin thay la moc bat dau,
        # khong gia dinh cac STT truoc do bi mat.
        self.thong_ke_beacon = {
            MA_TRAM_SU: self._tao_thong_ke_beacon(),
            MA_TRAM_DU: self._tao_thong_ke_beacon(),
        }

        if VI_DO_RBS is None or KINH_DO_RBS is None:
            print('[rBS GPS] CHUA KHAI BAO TOA DO rBS -> khoang cach SU/DU den rBS = N/A.')
        else:
            print(f'[rBS GPS] TOA DO rBS = {VI_DO_RBS:.7f}, {KINH_DO_RBS:.7f}')

        print(
            '[rBS GPS V10] LOC 2D: SAT>={} | HDOP<={:.1f} | AGE<={}ms | MEDIAN={} MAU | 3D_TIN_CAY={}'.format(
                GPS_MIN_VE_TINH_2D,
                GPS_HDOP_TOI_DA_2D,
                GPS_TUOI_FIX_TOI_DA_MS,
                GPS_SO_MAU_LOC,
                1 if CHO_PHEP_KHOANG_CACH_3D_TIN_CAY else 0,
            )
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
            ket_qua['bien_do_kenh_uoc_luong'] = None
            return ket_qua

        # Day la UOC LUONG packet-level tu RSSI, KHONG phai complex CSI h.
        # G_kenh[dB] = P_rx[dBm] - P_tx[dBm]
        # PL[dB]     = P_tx[dBm] - P_rx[dBm]
        he_so_kenh_db = float(rssi_dbm) - float(bao_cao['cong_suat_phat_dbm'])
        ket_qua['he_so_kenh_db'] = he_so_kenh_db
        ket_qua['suy_hao_kenh_db'] = -he_so_kenh_db
        ket_qua['he_so_cong_suat_kenh'] = 10.0 ** (he_so_kenh_db / 10.0)
        ket_qua['bien_do_kenh_uoc_luong'] = 10.0 ** (he_so_kenh_db / 20.0)
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
        """PDR quan sat theo STT. Co y nghia chuan voi firmware SU/DU V10."""
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
            # ESP reboot / counter reset: bat dau cua so quan sat moi.
            self.thong_ke_beacon[nguon] = self._tao_thong_ke_beacon()
            tk = self.thong_ke_beacon[nguon]
            tk['stt_cuoi'] = stt
            tk['so_goi_nhan'] = 1
            tk['so_goi_ky_vong'] = 1

        pdr = (
            100.0 * tk['so_goi_nhan'] / tk['so_goi_ky_vong']
            if tk['so_goi_ky_vong'] > 0
            else None
        )

        du_lieu['pdr_beacon_phan_tram'] = pdr
        du_lieu['so_goi_mat_uoc_tinh'] = tk['so_goi_mat_uoc_tinh']
        du_lieu['so_goi_trung'] = tk['so_goi_trung']

    def _cap_nhat_bo_loc(self, du_lieu):
        du_lieu['gps_tin_cay_2d'] = self._gps_du_tin_cay_2d(du_lieu)

        if not du_lieu['la_dinh_ky'] or not du_lieu['gps_tin_cay_2d']:
            return

        self.lich_su_vi_tri_tot[du_lieu['nguon']].append({
            'vi_do': du_lieu['vi_do'],
            'kinh_do': du_lieu['kinh_do'],
            'do_cao_m': du_lieu['do_cao_m'],
            'moc': du_lieu['moc_rbs_nhan_monotonic'],
        })

    def _lay_vi_tri_loc(self, nguon):
        lich_su = self.lich_su_vi_tri_tot[nguon]
        if len(lich_su) < GPS_SO_MAU_LOC:
            return None

        now = time.monotonic()
        mau = [x for x in lich_su if now - x['moc'] <= GPS_MAU_TOI_DA_CU_S]
        if len(mau) < GPS_SO_MAU_LOC:
            return None

        return {
            'vi_do': median([x['vi_do'] for x in mau]),
            'kinh_do': median([x['kinh_do'] for x in mau]),
            'moc': max(x['moc'] for x in mau),
        }

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

    @staticmethod
    def _khoang_cach_tho_hai_thiet_bi(du_lieu_1, du_lieu_2):
        """Khoang cach GPS THO. Chi dung de chan doan, khong gan nhan tin cay."""
        khoang_cach_2d_m = None
        khoang_cach_3d_m = None

        if (
            du_lieu_1
            and du_lieu_2
            and du_lieu_1.get('gps_hop_le')
            and du_lieu_2.get('gps_hop_le')
        ):
            khoang_cach_2d_m = tinh_khoang_cach_haversine_m(
                du_lieu_1['vi_do'], du_lieu_1['kinh_do'],
                du_lieu_2['vi_do'], du_lieu_2['kinh_do'],
            )

            if du_lieu_1.get('do_cao_hop_le') and du_lieu_2.get('do_cao_hop_le'):
                chenh_do_cao_m = du_lieu_2['do_cao_m'] - du_lieu_1['do_cao_m']
                khoang_cach_3d_m = math.sqrt(khoang_cach_2d_m ** 2 + chenh_do_cao_m ** 2)

        return khoang_cach_2d_m, khoang_cach_3d_m

    def _khoang_cach_loc_gan_nhat(self):
        su = self._lay_vi_tri_loc(MA_TRAM_SU)
        du = self._lay_vi_tri_loc(MA_TRAM_DU)

        if not su or not du:
            return None

        if abs(su['moc'] - du['moc']) > GPS_CAP_SU_DU_TOI_DA_LECH_S:
            return None

        return tinh_khoang_cach_haversine_m(
            su['vi_do'], su['kinh_do'], du['vi_do'], du['kinh_do']
        )

    def tinh_khoang_cach(self, ma_phien):
        """API giu tuong thich rbs_gateway.py cho bao cao theo SESSION."""
        su = self.lay(ma_phien, MA_TRAM_SU)
        du = self.lay(ma_phien, MA_TRAM_DU)

        # Theo SESSION chi co 1 snapshot, nen neu ca hai dat nguong chat luong
        # thi tinh 2D raw; khong the median 3 mau nhu beacon dinh ky.
        khoang_cach_su_du_m = None
        khoang_cach_3d_su_du_m = None
        if self._gps_du_tin_cay_2d(su) and self._gps_du_tin_cay_2d(du):
            khoang_cach_su_du_m = tinh_khoang_cach_haversine_m(
                su['vi_do'], su['kinh_do'], du['vi_do'], du['kinh_do']
            )

            if CHO_PHEP_KHOANG_CACH_3D_TIN_CAY:
                _, khoang_cach_3d_su_du_m = self._khoang_cach_tho_hai_thiet_bi(su, du)

        khoang_cach_su_rbs_m = None
        khoang_cach_du_rbs_m = None

        if VI_DO_RBS is not None and KINH_DO_RBS is not None:
            if self._gps_du_tin_cay_2d(su):
                khoang_cach_su_rbs_m = tinh_khoang_cach_haversine_m(
                    su['vi_do'], su['kinh_do'], VI_DO_RBS, KINH_DO_RBS
                )
            if self._gps_du_tin_cay_2d(du):
                khoang_cach_du_rbs_m = tinh_khoang_cach_haversine_m(
                    du['vi_do'], du['kinh_do'], VI_DO_RBS, KINH_DO_RBS
                )

        return (
            khoang_cach_su_du_m,
            khoang_cach_3d_su_du_m,
            khoang_cach_su_rbs_m,
            khoang_cach_du_rbs_m,
        )

    def tinh_khoang_cach_gan_nhat(self):
        # API cu: tra 2D loc + 3D tin cay (mac dinh None).
        return self._khoang_cach_loc_gan_nhat(), None

    def in_bao_cao_dinh_ky(self, du_lieu):
        if du_lieu is None:
            return

        ten = _ten_thiet_bi(du_lieu['nguon'])
        su_raw = self.gan_nhat[MA_TRAM_SU]
        du_raw = self.gan_nhat[MA_TRAM_DU]
        kc_tho_2d, kc_tho_3d = self._khoang_cach_tho_hai_thiet_bi(su_raw, du_raw)
        kc_loc_2d = self._khoang_cach_loc_gan_nhat()

        he_so_kenh = 'N/A' if du_lieu['he_so_kenh_db'] is None else f"{du_lieu['he_so_kenh_db']:.1f} dB"
        suy_hao = 'N/A' if du_lieu['suy_hao_kenh_db'] is None else f"{du_lieu['suy_hao_kenh_db']:.1f} dB"
        rssi = 'N/A' if du_lieu['rssi_dbm'] is None else f"{du_lieu['rssi_dbm']:.1f} dBm"
        snr = 'N/A' if du_lieu['snr_db'] is None else f"{du_lieu['snr_db']:.1f} dB"
        pdr = du_lieu.get('pdr_beacon_phan_tram')

        print(
            f"[rBS VI TRI DINH KY] {ten} | STT={du_lieu['so_thu_tu_bao_cao']} | "
            f"HOP_LE={1 if du_lieu['gps_hop_le'] else 0} | "
            f"GPS_TIN_CAY={1 if du_lieu.get('gps_tin_cay_2d') else 0} | "
            f"VI_DO={du_lieu['vi_do']:.7f} | KINH_DO={du_lieu['kinh_do']:.7f} | "
            f"DO_CAO={du_lieu['do_cao_m']:.1f}m | TOC_DO={du_lieu['toc_do_m_s']:.2f}m/s | "
            f"VE_TINH={du_lieu['so_ve_tinh']} | HDOP={du_lieu['hdop']:.2f} | "
            f"P_TX={du_lieu['cong_suat_phat_dbm']:.0f}dBm | RSSI={rssi} | SNR={snr} | "
            f"HE_SO_KENH={he_so_kenh} | SUY_HAO={suy_hao} | "
            f"PDR={'N/A' if pdr is None else f'{pdr:.1f}%'} | "
            f"MAT_UOC_TINH={du_lieu.get('so_goi_mat_uoc_tinh', 0)} | "
            f"KC_THO={'N/A' if kc_tho_2d is None else f'{kc_tho_2d:.1f}m'} | "
            f"KC_SU_DU={'N/A' if kc_loc_2d is None else f'{kc_loc_2d:.1f}m'} | "
            f"KC_3D_THO={'N/A' if kc_tho_3d is None else f'{kc_tho_3d:.1f}m'} | "
            f"KC_3D_SU_DU=N/A"
        )

    def ghi_csv_dinh_ky(self, du_lieu):
        su_raw = self.gan_nhat[MA_TRAM_SU]
        du_raw = self.gan_nhat[MA_TRAM_DU]
        kc_tho_2d, kc_tho_3d = self._khoang_cach_tho_hai_thiet_bi(su_raw, du_raw)
        kc_loc_2d = self._khoang_cach_loc_gan_nhat()
        ten = _ten_thiet_bi(du_lieu['nguon'])

        dong = {
            'Thoi_gian_rBS_nhan_UTC': du_lieu['thoi_gian_rbs_nhan_utc'],
            'Thiet_bi': ten,
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
            'Ngay_UTC': du_lieu['ngay_utc_yyyymmdd'],
            'Gio_UTC_ms_trong_ngay': du_lieu['gio_utc_ms_trong_ngay'],
            'Cong_suat_phat_dBm': du_lieu['cong_suat_phat_dbm'],
            'RSSI_dBm': '' if du_lieu['rssi_dbm'] is None else du_lieu['rssi_dbm'],
            'SNR_dB': '' if du_lieu['snr_db'] is None else du_lieu['snr_db'],
            'He_so_kenh_dB': '' if du_lieu['he_so_kenh_db'] is None else du_lieu['he_so_kenh_db'],
            'Suy_hao_kenh_dB': '' if du_lieu['suy_hao_kenh_db'] is None else du_lieu['suy_hao_kenh_db'],
            'He_so_cong_suat_kenh': '' if du_lieu['he_so_cong_suat_kenh'] is None else du_lieu['he_so_cong_suat_kenh'],
            'Bien_do_kenh_uoc_luong': '' if du_lieu['bien_do_kenh_uoc_luong'] is None else du_lieu['bien_do_kenh_uoc_luong'],
            'PDR_beacon_phan_tram': '' if du_lieu.get('pdr_beacon_phan_tram') is None else round(du_lieu['pdr_beacon_phan_tram'], 3),
            'So_goi_mat_uoc_tinh': du_lieu.get('so_goi_mat_uoc_tinh', 0),
            'So_goi_trung': du_lieu.get('so_goi_trung', 0),
            'Khoang_cach_SU_DU_2D_tho_m': '' if kc_tho_2d is None else round(kc_tho_2d, 2),
            'Khoang_cach_SU_DU_2D_loc_m': '' if kc_loc_2d is None else round(kc_loc_2d, 2),
            'Khoang_cach_SU_DU_3D_tho_m': '' if kc_tho_3d is None else round(kc_tho_3d, 2),
            'Khoang_cach_SU_DU_3D_tin_cay_m': '',
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
        su = self.lay(ma_phien, MA_TRAM_SU)
        du = self.lay(ma_phien, MA_TRAM_DU)
        kc_su_du, kc_3d_su_du, kc_su_rbs, kc_du_rbs = self.tinh_khoang_cach(ma_phien)

        print('[GPS / KHOANG CACH]')
        if su:
            print(
                f"  SU: HOP_LE={1 if su['gps_hop_le'] else 0} | TIN_CAY_2D={1 if self._gps_du_tin_cay_2d(su) else 0} | "
                f"VI_DO={su['vi_do']:.7f} | KINH_DO={su['kinh_do']:.7f} | "
                f"DO_CAO_THO={su['do_cao_m']:.1f}m | TOC_DO={su['toc_do_m_s']:.2f}m/s | "
                f"VE_TINH={su['so_ve_tinh']} | HDOP={su['hdop']:.2f} | "
                f"RSSI={su.get('rssi_dbm')} | HE_SO_KENH={su.get('he_so_kenh_db')}"
            )
        else:
            print('  SU: CHUA NHAN GPS_REPORT')

        if du:
            print(
                f"  DU: HOP_LE={1 if du['gps_hop_le'] else 0} | TIN_CAY_2D={1 if self._gps_du_tin_cay_2d(du) else 0} | "
                f"VI_DO={du['vi_do']:.7f} | KINH_DO={du['kinh_do']:.7f} | "
                f"DO_CAO_THO={du['do_cao_m']:.1f}m | TOC_DO={du['toc_do_m_s']:.2f}m/s | "
                f"VE_TINH={du['so_ve_tinh']} | HDOP={du['hdop']:.2f} | "
                f"RSSI={du.get('rssi_dbm')} | HE_SO_KENH={du.get('he_so_kenh_db')}"
            )
        else:
            print('  DU: CHUA NHAN GPS_REPORT')

        print(f"  KHOANG CACH SU<->DU 2D = {'N/A' if kc_su_du is None else f'{kc_su_du:.1f} m'}")
        print('  KHOANG CACH SU<->DU 3D = N/A (do cao NEO-6M chi luu de tham khao)')
        print(f"  KHOANG CACH SU<->rBS   = {'N/A' if kc_su_rbs is None else f'{kc_su_rbs:.1f} m'}")
        print(f"  KHOANG CACH DU<->rBS   = {'N/A' if kc_du_rbs is None else f'{kc_du_rbs:.1f} m'}")

    def ghi_csv_session(self, diag, ly_do):
        if diag is None:
            return

        ma_phien = diag['session_id']
        su = self.lay(ma_phien, MA_TRAM_SU) or {}
        du = self.lay(ma_phien, MA_TRAM_DU) or {}
        kc_su_du, kc_3d_su_du, kc_su_rbs, kc_du_rbs = self.tinh_khoang_cach(ma_phien)
        thong_ke_su = diag['link_su_rbs']
        thong_ke_du = diag['link_du_rbs']

        dong = {
            'Thoi_gian_UTC': datetime.now(timezone.utc).isoformat(),
            'Ly_do_ghi': ly_do,
            'Ma_phien': f'{ma_phien:016X}',
            'SU_GPS_hop_le': int(bool(su.get('gps_hop_le', False))),
            'SU_GPS_tin_cay_2D': int(self._gps_du_tin_cay_2d(su)),
            'SU_Vi_do': su.get('vi_do', ''),
            'SU_Kinh_do': su.get('kinh_do', ''),
            'SU_Do_cao_tho_m': su.get('do_cao_m', ''),
            'SU_Toc_do_m_s': su.get('toc_do_m_s', ''),
            'SU_So_ve_tinh': su.get('so_ve_tinh', ''),
            'SU_HDOP': su.get('hdop', ''),
            'SU_He_so_kenh_dB': su.get('he_so_kenh_db', ''),
            'DU_GPS_hop_le': int(bool(du.get('gps_hop_le', False))),
            'DU_GPS_tin_cay_2D': int(self._gps_du_tin_cay_2d(du)),
            'DU_Vi_do': du.get('vi_do', ''),
            'DU_Kinh_do': du.get('kinh_do', ''),
            'DU_Do_cao_tho_m': du.get('do_cao_m', ''),
            'DU_Toc_do_m_s': du.get('toc_do_m_s', ''),
            'DU_So_ve_tinh': du.get('so_ve_tinh', ''),
            'DU_HDOP': du.get('hdop', ''),
            'DU_He_so_kenh_dB': du.get('he_so_kenh_db', ''),
            'Vi_do_rBS': '' if VI_DO_RBS is None else VI_DO_RBS,
            'Kinh_do_rBS': '' if KINH_DO_RBS is None else KINH_DO_RBS,
            'Khoang_cach_SU_DU_2D_m': '' if kc_su_du is None else round(kc_su_du, 2),
            'Khoang_cach_SU_DU_3D_m': '' if kc_3d_su_du is None else round(kc_3d_su_du, 2),
            'Khoang_cach_SU_rBS_m': '' if kc_su_rbs is None else round(kc_su_rbs, 2),
            'Khoang_cach_DU_rBS_m': '' if kc_du_rbs is None else round(kc_du_rbs, 2),
            'RSSI_SU_rBS_TB_dBm': _trung_binh(thong_ke_su, 'rssi_sum', 'rssi_count'),
            'SNR_SU_rBS_TB_dB': _trung_binh(thong_ke_su, 'snr_sum', 'snr_count'),
            'RSSI_DU_rBS_TB_dBm': _trung_binh(thong_ke_du, 'rssi_sum', 'rssi_count'),
            'SNR_DU_rBS_TB_dB': _trung_binh(thong_ke_du, 'snr_sum', 'snr_count'),
            'ARQ_retry_quan_sat': diag.get('so_goi_trung_arq', 0),
            'So_lan_SESSION_START_rBS_DU': diag.get('so_lan_start_toi_du', 0),
            'ACK_NACK': diag.get('hmi_ket_qua', ''),
            'Tan_so_MHz': 433.0,
            'SF': 7,
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

        print(f'[rBS GPS CSV] DA GHI PHIEN: {self.duong_dan_csv_phien}')
