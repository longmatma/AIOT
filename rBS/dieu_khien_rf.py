import time
from collections import deque

# ============================================================
# BỘ ĐIỀU KHIỂN RF V11 - rBS LÀ TRUNG TÂM
#
# Điều khiển:
#   1) Công suất phát của SU.
#   2) Công suất phát của DU.
#   3) Hệ số trải phổ chung SF7/SF8/SF9.
#
# Chưa triển khai nguồn nhiễu/jammer ở giai đoạn này.
#
# Nguyên tắc an toàn:
# - Không đổi RF trong lúc đang có phiên thoại.
# - Chỉ đánh giá sau khi có đủ dữ liệu của CẢ SU và DU.
# - Công suất thay đổi từng nấc, có hysteresis/cooldown.
# - SF chỉ tăng khi công suất đã lên mức tối đa mà link vẫn xấu.
# - Đổi SF theo 2 pha: CHUẨN_BỊ -> COMMIT có trì hoãn.
# - Nếu sau đổi cấu hình không xác minh được cả 2 node, rBS quét SF7/8/9
#   và ép toàn hệ thống quay về baseline SF7 / 17 dBm.
# ============================================================

ID_TRAM_SU = 0x01
ID_TRAM_DU = 0x02
ID_TRAM_RBS = 0x03
ID_QUANG_BA = 0xFF

TYPE_DIEU_KHIEN_RF = 0x19
TYPE_XAC_NHAN_RF = 0x1A

LENH_CHUAN_BI = 0x01
LENH_AP_DUNG = 0x02
LENH_KHOI_PHUC_MAC_DINH = 0x04

PHIEN_BAN_LENH_RF = 0x01

CAC_MUC_CONG_SUAT_DBM = (8, 11, 14, 17, 20)
CAC_MUC_SF = (7, 8, 9)

CONG_SUAT_MAC_DINH_DBM = 17
SF_MAC_DINH = 7

# Dùng 6 beacon gần nhất (~30 s) để tránh phản ứng theo 1 mẫu nhiễu.
SO_MAU_DANH_GIA = 6

# Không điều chỉnh liên tục. Cho kênh ổn định sau mỗi lần đổi.
THOI_GIAN_CHO_GIUA_HAI_LAN_DIEU_CHINH_S = 60.0

# Chỉ đổi RF khi đã rảnh khỏi phiên thoại một khoảng thời gian.
THOI_GIAN_RANH_TOI_THIEU_S = 8.0

# Sau COMMIT, chờ tối đa 18 s để nhận beacon xác nhận từ cả SU và DU.
THOI_GIAN_XAC_MINH_CAU_HINH_S = 18.0

# Node áp dụng cấu hình sau 0.8 s kể từ lúc nhận COMMIT.
DO_TRE_AP_DUNG_DON_VI_100MS = 8

# Retry PREPARE giúp tránh đổi SF khi một node chưa nhận được cấu hình.
SO_LAN_THU_CHUAN_BI = 3
CUA_SO_CHO_XAC_NHAN_S = 0.25


class BoDieuKhienRF:
    def __init__(self):
        self.cong_suat_hien_tai = {
            ID_TRAM_SU: CONG_SUAT_MAC_DINH_DBM,
            ID_TRAM_DU: CONG_SUAT_MAC_DINH_DBM,
        }
        self.he_so_trai_pho_hien_tai = SF_MAC_DINH

        self.lich_su = {
            ID_TRAM_SU: deque(maxlen=SO_MAU_DANH_GIA),
            ID_TRAM_DU: deque(maxlen=SO_MAU_DANH_GIA),
        }
        self.stt_cuoi = {ID_TRAM_SU: None, ID_TRAM_DU: None}
        self.moc_nhan_cuoi = {ID_TRAM_SU: None, ID_TRAM_DU: None}

        self.ma_lenh = 0
        self.moc_dieu_chinh_cuoi = time.monotonic()

        self.cau_hinh_dang_xac_minh = None
        self.xac_minh_node = {ID_TRAM_SU: False, ID_TRAM_DU: False}
        self.han_xac_minh = None

    @staticmethod
    def _ten_node(node):
        return 'SU' if node == ID_TRAM_SU else 'DU'

    @staticmethod
    def _muc_lien_ke(gia_tri, cac_muc, huong):
        """Lấy một nấc liền kề: huong=+1 tăng, huong=-1 giảm."""
        try:
            vi_tri = cac_muc.index(int(gia_tri))
        except ValueError:
            # Nếu firmware báo giá trị lạ, quay về phần tử gần nhất.
            vi_tri = min(range(len(cac_muc)), key=lambda i: abs(cac_muc[i] - int(gia_tri)))

        vi_tri_moi = max(0, min(len(cac_muc) - 1, vi_tri + huong))
        return cac_muc[vi_tri_moi]

    def _ma_lenh_moi(self):
        self.ma_lenh = (self.ma_lenh + 1) & 0xFFFF
        if self.ma_lenh == 0:
            self.ma_lenh = 1
        return self.ma_lenh

    def cap_nhat_du_lieu(self, du_lieu):
        """Nhận một beacon đã được gps_rbs.py gắn RSSI/SNR/PDR/channel gain."""
        if not du_lieu or not du_lieu.get('la_dinh_ky'):
            return

        node = int(du_lieu['nguon'])
        if node not in (ID_TRAM_SU, ID_TRAM_DU):
            return

        rssi = du_lieu.get('rssi_dbm')
        snr = du_lieu.get('snr_db')
        stt = int(du_lieu.get('so_thu_tu_bao_cao', 0))
        if rssi is None or snr is None or stt <= 0:
            return

        stt_truoc = self.stt_cuoi[node]
        if stt_truoc is None or stt <= stt_truoc or (stt - stt_truoc) > 100:
            so_goi_ky_vong = 1
        else:
            so_goi_ky_vong = stt - stt_truoc

        self.stt_cuoi[node] = stt
        self.moc_nhan_cuoi[node] = time.monotonic()

        self.lich_su[node].append({
            'rssi': float(rssi),
            'snr': float(snr),
            'so_goi_ky_vong': max(1, int(so_goi_ky_vong)),
        })

        # Đồng bộ trạng thái thực tế từ chính telemetry node gửi về.
        self.cong_suat_hien_tai[node] = int(round(du_lieu.get('cong_suat_phat_dbm', CONG_SUAT_MAC_DINH_DBM)))
        sf_bao_cao = int(du_lieu.get('he_so_trai_pho', SF_MAC_DINH))

        # Khi hai node đang cùng SF thì đây là trạng thái RF thực tế đáng tin cậy.
        if sf_bao_cao in CAC_MUC_SF:
            if self.cau_hinh_dang_xac_minh is None:
                self.he_so_trai_pho_hien_tai = sf_bao_cao

        # Xác minh cấu hình sau khi COMMIT.
        if self.cau_hinh_dang_xac_minh is not None:
            muc_tieu = self.cau_hinh_dang_xac_minh
            cong_suat_muc_tieu = (
                muc_tieu['cong_suat_su'] if node == ID_TRAM_SU else muc_tieu['cong_suat_du']
            )
            if (
                int(round(du_lieu.get('cong_suat_phat_dbm', -99))) == cong_suat_muc_tieu
                and sf_bao_cao == muc_tieu['sf']
            ):
                self.xac_minh_node[node] = True

            if self.xac_minh_node[ID_TRAM_SU] and self.xac_minh_node[ID_TRAM_DU]:
                self.cong_suat_hien_tai[ID_TRAM_SU] = muc_tieu['cong_suat_su']
                self.cong_suat_hien_tai[ID_TRAM_DU] = muc_tieu['cong_suat_du']
                self.he_so_trai_pho_hien_tai = muc_tieu['sf']
                print(
                    '[ĐIỀU KHIỂN RF] ĐÃ XÁC NHẬN CẢ SU VÀ DU | '
                    f'P_SU={muc_tieu["cong_suat_su"]}dBm | '
                    f'P_DU={muc_tieu["cong_suat_du"]}dBm | SF={muc_tieu["sf"]}'
                )
                self.cau_hinh_dang_xac_minh = None
                self.han_xac_minh = None

    def _thong_ke_cua_so(self, node):
        mau = self.lich_su[node]
        if len(mau) < SO_MAU_DANH_GIA:
            return None

        rssi_tb = sum(x['rssi'] for x in mau) / len(mau)
        snr_tb = sum(x['snr'] for x in mau) / len(mau)
        so_goi_ky_vong = sum(x['so_goi_ky_vong'] for x in mau)
        pdr_cua_so = 100.0 * len(mau) / so_goi_ky_vong if so_goi_ky_vong > 0 else 100.0

        return {
            'rssi_tb': rssi_tb,
            'snr_tb': snr_tb,
            'pdr_cua_so': pdr_cua_so,
        }

    def _du_lieu_du_moi(self):
        bay_gio = time.monotonic()
        for node in (ID_TRAM_SU, ID_TRAM_DU):
            moc = self.moc_nhan_cuoi[node]
            if moc is None or (bay_gio - moc) > 12.0:
                return False
        return True

    def _de_xuat_cong_suat(self, node, thong_ke):
        hien_tai = int(self.cong_suat_hien_tai[node])

        # Link xấu: tăng một nấc.
        if (
            thong_ke['pdr_cua_so'] < 96.0
            or thong_ke['snr_tb'] < -2.0
            or thong_ke['rssi_tb'] < -105.0
        ):
            return self._muc_lien_ke(hien_tai, CAC_MUC_CONG_SUAT_DBM, +1)

        # Link rất dư: giảm một nấc để tiết kiệm năng lượng.
        if (
            thong_ke['pdr_cua_so'] >= 99.5
            and thong_ke['snr_tb'] >= 5.0
            and thong_ke['rssi_tb'] > -70.0
        ):
            return self._muc_lien_ke(hien_tai, CAC_MUC_CONG_SUAT_DBM, -1)

        return hien_tai

    def _de_xuat_sf(self, tk_su, tk_du, cong_suat_su, cong_suat_du):
        sf = int(self.he_so_trai_pho_hien_tai)

        # Nếu đang thay đổi công suất, giữ SF nguyên ở vòng này.
        if (
            cong_suat_su != self.cong_suat_hien_tai[ID_TRAM_SU]
            or cong_suat_du != self.cong_suat_hien_tai[ID_TRAM_DU]
        ):
            return sf

        xau_nhat_pdr = min(tk_su['pdr_cua_so'], tk_du['pdr_cua_so'])
        xau_nhat_snr = min(tk_su['snr_tb'], tk_du['snr_tb'])
        xau_nhat_rssi = min(tk_su['rssi_tb'], tk_du['rssi_tb'])

        # Chỉ tăng SF khi node yếu đã ở 20 dBm nhưng link vẫn xấu.
        da_toi_cong_suat_cao = (
            self.cong_suat_hien_tai[ID_TRAM_SU] >= 20
            or self.cong_suat_hien_tai[ID_TRAM_DU] >= 20
        )

        if sf == 7 and da_toi_cong_suat_cao:
            if xau_nhat_pdr < 90.0 or xau_nhat_snr < -6.0 or xau_nhat_rssi < -115.0:
                return 8

        if sf == 8:
            if da_toi_cong_suat_cao and (
                xau_nhat_pdr < 85.0 or xau_nhat_snr < -8.0 or xau_nhat_rssi < -118.0
            ):
                return 9

            # Hạ SF khi cả hai link đã khỏe lại.
            if (
                tk_su['pdr_cua_so'] >= 99.0 and tk_du['pdr_cua_so'] >= 99.0
                and tk_su['snr_tb'] >= 2.0 and tk_du['snr_tb'] >= 2.0
                and tk_su['rssi_tb'] > -105.0 and tk_du['rssi_tb'] > -105.0
            ):
                return 7

        if sf == 9:
            if (
                tk_su['pdr_cua_so'] >= 98.0 and tk_du['pdr_cua_so'] >= 98.0
                and tk_su['snr_tb'] >= 0.0 and tk_du['snr_tb'] >= 0.0
                and tk_su['rssi_tb'] > -110.0 and tk_du['rssi_tb'] > -110.0
            ):
                return 8

        return sf

    @staticmethod
    def _tao_payload_lenh(ma_lenh, cong_suat_su, cong_suat_du, sf, do_tre_100ms):
        return bytes([
            PHIEN_BAN_LENH_RF,
            (ma_lenh >> 8) & 0xFF,
            ma_lenh & 0xFF,
            cong_suat_su & 0xFF,
            cong_suat_du & 0xFF,
            sf & 0xFF,
            do_tre_100ms & 0xFF,
            0x00,
        ])

    @staticmethod
    def _phan_tich_xac_nhan(goi_tin, node_mong_doi, ma_lenh_mong_doi):
        if goi_tin is None:
            return False
        p = bytes(goi_tin)
        if len(p) != 8:
            return False
        if p[0] != ID_TRAM_RBS or p[1] != node_mong_doi or p[2] != TYPE_XAC_NHAN_RF:
            return False
        if p[3] != 0x01:
            return False
        ma_lenh = (p[4] << 8) | p[5]
        return ma_lenh == ma_lenh_mong_doi

    def _gui_chuan_bi_cho_node(self, rfm9x, node, ma_lenh, cong_suat_su, cong_suat_du, sf):
        payload = self._tao_payload_lenh(
            ma_lenh,
            cong_suat_su,
            cong_suat_du,
            sf,
            DO_TRE_AP_DUNG_DON_VI_100MS,
        )

        for lan in range(1, SO_LAN_THU_CHUAN_BI + 1):
            rfm9x.send(
                payload,
                destination=node,
                node=ID_TRAM_RBS,
                identifier=TYPE_DIEU_KHIEN_RF,
                flags=LENH_CHUAN_BI,
            )
            rfm9x.listen()

            han = time.monotonic() + CUA_SO_CHO_XAC_NHAN_S
            while time.monotonic() < han:
                goi = rfm9x.receive(timeout=0.01, with_header=True)
                if self._phan_tich_xac_nhan(goi, node, ma_lenh):
                    return True

            print(
                f'[CẢNH BÁO] {self._ten_node(node)} chưa xác nhận chuẩn bị RF | '
                f'lần={lan}/{SO_LAN_THU_CHUAN_BI}'
            )

        return False

    def _gui_commit(self, rfm9x, ma_lenh, cong_suat_su, cong_suat_du, sf):
        payload = self._tao_payload_lenh(
            ma_lenh,
            cong_suat_su,
            cong_suat_du,
            sf,
            DO_TRE_AP_DUNG_DON_VI_100MS,
        )

        # Gửi lặp 5 lần trong SF hiện tại. Node chỉ lên lịch đổi sau 0.8 s,
        # nên toàn bộ bản sao COMMIT được phát xong trước khi SF thực sự đổi.
        for _ in range(5):
            rfm9x.send(
                payload,
                destination=ID_QUANG_BA,
                node=ID_TRAM_RBS,
                identifier=TYPE_DIEU_KHIEN_RF,
                flags=LENH_AP_DUNG,
            )
            time.sleep(0.015)

        # Chờ node tới thời điểm áp dụng rồi rBS mới chuyển SF.
        time.sleep(DO_TRE_AP_DUNG_DON_VI_100MS * 0.1 + 0.12)
        rfm9x.idle()
        rfm9x.spreading_factor = sf
        rfm9x.listen()

    def thu_dieu_chinh(self, rfm9x, duoc_phep_dieu_chinh, thoi_gian_ranh_s):
        """Được gọi khi rBS đang xử lý beacon định kỳ."""
        if not duoc_phep_dieu_chinh:
            return False
        if thoi_gian_ranh_s < THOI_GIAN_RANH_TOI_THIEU_S:
            return False
        if self.cau_hinh_dang_xac_minh is not None:
            return False
        if not self._du_lieu_du_moi():
            return False
        if time.monotonic() - self.moc_dieu_chinh_cuoi < THOI_GIAN_CHO_GIUA_HAI_LAN_DIEU_CHINH_S:
            return False

        tk_su = self._thong_ke_cua_so(ID_TRAM_SU)
        tk_du = self._thong_ke_cua_so(ID_TRAM_DU)
        if tk_su is None or tk_du is None:
            return False

        cong_suat_su_moi = self._de_xuat_cong_suat(ID_TRAM_SU, tk_su)
        cong_suat_du_moi = self._de_xuat_cong_suat(ID_TRAM_DU, tk_du)
        sf_moi = self._de_xuat_sf(tk_su, tk_du, cong_suat_su_moi, cong_suat_du_moi)

        if (
            cong_suat_su_moi == self.cong_suat_hien_tai[ID_TRAM_SU]
            and cong_suat_du_moi == self.cong_suat_hien_tai[ID_TRAM_DU]
            and sf_moi == self.he_so_trai_pho_hien_tai
        ):
            return False

        ma_lenh = self._ma_lenh_moi()

        print(
            '[ĐIỀU KHIỂN RF] ĐỀ XUẤT | '
            f'P_SU={self.cong_suat_hien_tai[ID_TRAM_SU]}->{cong_suat_su_moi}dBm | '
            f'P_DU={self.cong_suat_hien_tai[ID_TRAM_DU]}->{cong_suat_du_moi}dBm | '
            f'SF={self.he_so_trai_pho_hien_tai}->{sf_moi}'
        )

        # Pha 1: cả SU và DU phải xác nhận đã nhận cấu hình mới khi vẫn ở SF cũ.
        if not self._gui_chuan_bi_cho_node(
            rfm9x, ID_TRAM_SU, ma_lenh, cong_suat_su_moi, cong_suat_du_moi, sf_moi
        ):
            print('[CẢNH BÁO] Hủy thay đổi RF vì SU không xác nhận chuẩn bị')
            self.moc_dieu_chinh_cuoi = time.monotonic()
            return False

        if not self._gui_chuan_bi_cho_node(
            rfm9x, ID_TRAM_DU, ma_lenh, cong_suat_su_moi, cong_suat_du_moi, sf_moi
        ):
            print('[CẢNH BÁO] Hủy thay đổi RF vì DU không xác nhận chuẩn bị')
            self.moc_dieu_chinh_cuoi = time.monotonic()
            return False

        # Pha 2: COMMIT quảng bá có trì hoãn để ba thiết bị cùng chuyển cấu hình.
        self._gui_commit(
            rfm9x,
            ma_lenh,
            cong_suat_su_moi,
            cong_suat_du_moi,
            sf_moi,
        )

        self.cau_hinh_dang_xac_minh = {
            'ma_lenh': ma_lenh,
            'cong_suat_su': cong_suat_su_moi,
            'cong_suat_du': cong_suat_du_moi,
            'sf': sf_moi,
        }
        self.xac_minh_node = {ID_TRAM_SU: False, ID_TRAM_DU: False}
        self.han_xac_minh = time.monotonic() + THOI_GIAN_XAC_MINH_CAU_HINH_S
        self.he_so_trai_pho_hien_tai = sf_moi
        self.moc_dieu_chinh_cuoi = time.monotonic()

        print(
            '[ĐIỀU KHIỂN RF] ĐÃ COMMIT | '
            f'P_SU={cong_suat_su_moi}dBm | P_DU={cong_suat_du_moi}dBm | SF={sf_moi} | '
            'đang chờ beacon xác minh'
        )
        return True

    def kiem_tra_xac_minh(self, rfm9x):
        """Nếu cấu hình mới bị lệch giữa các node, tự đưa toàn hệ về baseline."""
        if self.cau_hinh_dang_xac_minh is None or self.han_xac_minh is None:
            return False

        if time.monotonic() <= self.han_xac_minh:
            return False

        print(
            '[CẢNH BÁO] Không xác minh được cấu hình RF trên cả SU và DU -> '
            'khôi phục SF7 / 17 dBm'
        )
        self.khoi_phuc_mac_dinh(rfm9x)
        return True

    def khoi_phuc_mac_dinh(self, rfm9x):
        """Quét SF7/SF8/SF9 và gửi lệnh ép baseline để xử lý mất đồng bộ SF."""
        ma_lenh = self._ma_lenh_moi()
        payload = self._tao_payload_lenh(
            ma_lenh,
            CONG_SUAT_MAC_DINH_DBM,
            CONG_SUAT_MAC_DINH_DBM,
            SF_MAC_DINH,
            DO_TRE_AP_DUNG_DON_VI_100MS,
        )

        for sf_quet in CAC_MUC_SF:
            rfm9x.idle()
            rfm9x.spreading_factor = sf_quet
            for _ in range(3):
                rfm9x.send(
                    payload,
                    destination=ID_QUANG_BA,
                    node=ID_TRAM_RBS,
                    identifier=TYPE_DIEU_KHIEN_RF,
                    flags=LENH_KHOI_PHUC_MAC_DINH,
                )
                time.sleep(0.020)

        # Đợi các node nhận lệnh ở SF hiện tại của chúng rồi cùng quay về SF7.
        time.sleep(DO_TRE_AP_DUNG_DON_VI_100MS * 0.1 + 0.15)
        rfm9x.idle()
        rfm9x.spreading_factor = SF_MAC_DINH
        rfm9x.listen()

        self.cong_suat_hien_tai = {
            ID_TRAM_SU: CONG_SUAT_MAC_DINH_DBM,
            ID_TRAM_DU: CONG_SUAT_MAC_DINH_DBM,
        }
        self.he_so_trai_pho_hien_tai = SF_MAC_DINH
        self.cau_hinh_dang_xac_minh = None
        self.han_xac_minh = None
        self.xac_minh_node = {ID_TRAM_SU: False, ID_TRAM_DU: False}
        self.lich_su[ID_TRAM_SU].clear()
        self.lich_su[ID_TRAM_DU].clear()
        self.moc_dieu_chinh_cuoi = time.monotonic()

        print('[ĐIỀU KHIỂN RF] KHÔI PHỤC XONG | P_SU=17dBm | P_DU=17dBm | SF=7')
