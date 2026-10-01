import os
import time
import builtins
import csv
import math
import socket
import threading
from pathlib import Path
from datetime import datetime, timezone

from stm32_e22_bridge import STM32E22Bridge, BridgeError
from gps_rbs import (
    QuanLyGPSRBS,
    phan_tich_bao_cao_gps,
    LOAI_BAO_CAO_GPS_PHIEN,
    LOAI_BAO_CAO_VI_TRI_DINH_KY,
    KICH_THUOC_BAO_CAO_GPS,
    MA_TRAM_SU as MA_GPS_SU,
    MA_TRAM_DU as MA_GPS_DU,
)


# ============================================================
# LOG VẬN HÀNH TỐI GIẢN
#
# Mặc định chỉ in những sự kiện cần thiết để giảm I/O journal:
# - khởi động / radio sẵn sàng
# - bắt đầu / hoàn tất / thất bại phiên thoại
# - phản hồi cuối phiên
# - cảnh báo / lỗi
# - watchdog khi phải HARD RESET / reconnect
#
# Toàn bộ log legacy vẫn còn trong code. Khi cần debug chỉ cần chạy với:
#   RBS_LOG_DEBUG=1
# mà không phải sửa lại source.
# ============================================================
_in_goc = builtins.print
_LOG_DEBUG = os.environ.get("RBS_LOG_DEBUG", "0").strip().lower() in {
    "1", "true", "yes", "on"
}


def _la_log_quan_trong(noi_dung: str) -> bool:
    if noi_dung.startswith("[LỖI]") or noi_dung.startswith("[CẢNH BÁO]"):
        return True

    if noi_dung.startswith("[PHẢN HỒI]"):
        return True

    if noi_dung.startswith("[PHIÊN THOẠI]"):
        return any(
            cum in noi_dung
            for cum in (
                "BẮT ĐẦU",
                "HOÀN TẤT",
                "BẮT TAY THẤT BẠI",
            )
        )

    if noi_dung.startswith("[HỆ THỐNG]"):
        return (
            "rBS E22-400M30S + STM32 BRIDGE" in noi_dung
            or "Pi->STM32 UART=" in noi_dung
            or "E22 rBS sẵn sàng" in noi_dung
        )

    if noi_dung.startswith("[WATCHDOG]"):
        return any(
            cum in noi_dung
            for cum in (
                "BẬT",
                "HARD RESET",
                "reconnect",
                "TỰ PHỤC HỒI",
                "RECONFIG",
                "RE-ARM",
            )
        )

    if noi_dung.startswith("[SYSTEMD WDT]"):
        return True

    # V2A la giai doan bench slot/block; giu log nay ngay ca khi DEBUG=0
    # de co the doi chieu SU <-> rBS ma khong bat lai toan bo legacy log.
    if (
        noi_dung.startswith("[rBS V2A")
        or noi_dung.startswith("[rBS V2B")
        or noi_dung.startswith("[rBS V2C1")
        or noi_dung.startswith("[rBS V2C2")
        or noi_dung.startswith("[rBS BUILD")
        or noi_dung.startswith("[rBS LAT")
    ):
        return True

    return False


def print(*args, **kwargs):  # noqa: A001 - lọc log legacy trong module này
    if _LOG_DEBUG:
        _in_goc(*args, **kwargs)
        return

    if not args:
        return

    noi_dung = " ".join(str(x) for x in args)
    if noi_dung and _la_log_quan_trong(noi_dung):
        _in_goc(*args, **kwargs)



# ============================================================
# SELF-HEAL V2 - SYSTEMD SERVICE WATCHDOG
#
# Mục tiêu:
#   - Python crash -> systemd Restart=always đã xử lý.
#   - Python còn process nhưng main loop bị treo/deadlock -> systemd WatchdogSec
#     phải phát hiện và restart service.
#
# Cách làm:
#   - main loop gọi progress() ở mỗi vòng xử lý.
#   - một thread rất nhẹ chỉ gửi WATCHDOG=1 khi main loop còn tiến triển.
#   - nếu main loop đứng quá RBS_SYSTEMD_MAIN_STALL_S, thread NGỪNG báo sống.
#   - systemd hết WatchdogSec -> kill + restart service.
#
# Không cài thêm package Python; sd_notify được gửi trực tiếp qua NOTIFY_SOCKET.
# ============================================================

RBS_SYSTEMD_MAIN_STALL_S = float(
    os.environ.get(
        "RBS_SYSTEMD_MAIN_STALL_S",
        "12.0",
    )
)


class SystemdServiceWatchdog:
    def __init__(self):
        self._notify_socket = os.environ.get(
            "NOTIFY_SOCKET",
            "",
        )

        watchdog_usec_text = os.environ.get(
            "WATCHDOG_USEC",
            "0",
        )

        try:
            watchdog_usec = int(
                watchdog_usec_text
            )
        except (TypeError, ValueError):
            watchdog_usec = 0

        self.enabled = bool(
            self._notify_socket
            and watchdog_usec > 0
        )

        self._watchdog_period_s = (
            watchdog_usec / 1_000_000.0
            if watchdog_usec > 0
            else 0.0
        )

        # Gửi keepalive nhanh hơn nhiều so với WatchdogSec.
        self._notify_interval_s = (
            max(
                1.0,
                min(
                    5.0,
                    self._watchdog_period_s / 3.0,
                ),
            )
            if self.enabled
            else 5.0
        )

        self._last_main_progress = (
            time.monotonic()
        )

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._stall_logged = False

    def _send_notify(self, message: str) -> bool:
        if not self.enabled:
            return False

        address = self._notify_socket

        # systemd dùng '@name' để biểu diễn abstract UNIX socket.
        if address.startswith("@"):
            address = (
                "\0"
                +
                address[1:]
            )

        sock = socket.socket(
            socket.AF_UNIX,
            socket.SOCK_DGRAM,
        )

        try:
            sock.connect(address)
            sock.sendall(
                message.encode(
                    "utf-8",
                    errors="strict",
                )
            )
            return True
        except OSError:
            return False
        finally:
            sock.close()

    def progress(self):
        with self._lock:
            self._last_main_progress = (
                time.monotonic()
            )
            self._stall_logged = False

    def status(self, text_status: str):
        if self.enabled:
            self._send_notify(
                f"STATUS={text_status}"
            )

    def ready(self):
        if self.enabled:
            self._send_notify(
                "READY=1\n"
                "STATUS=rBS gateway running"
            )

    def _run(self):
        while not self._stop_event.wait(
            self._notify_interval_s
        ):
            with self._lock:
                age_s = (
                    time.monotonic()
                    -
                    self._last_main_progress
                )

                stall_logged = (
                    self._stall_logged
                )

            if (
                age_s
                <=
                RBS_SYSTEMD_MAIN_STALL_S
            ):
                self._send_notify(
                    "WATCHDOG=1"
                )
                continue

            # Cố ý KHÔNG gửi WATCHDOG=1.
            # systemd sẽ xử lý process treo.
            if not stall_logged:
                print(
                    f"[SYSTEMD WDT] MAIN LOOP KHONG TIEN TRIEN "
                    f"{age_s:.1f}s -> NGUNG HEARTBEAT, "
                    "CHO SYSTEMD RESTART SERVICE"
                )

                with self._lock:
                    self._stall_logged = True

    def start(self):
        if not self.enabled:
            print(
                "[SYSTEMD WDT] KHONG CO NOTIFY_SOCKET/WATCHDOG_USEC "
                "-> che do systemd watchdog khong kich hoat"
            )
            return

        self._thread = threading.Thread(
            target=self._run,
            name="rbs-systemd-watchdog",
            daemon=True,
        )
        self._thread.start()

        print(
            f"[SYSTEMD WDT] BAT | "
            f"WATCHDOG_SEC={self._watchdog_period_s:.1f}s | "
            f"MAIN_STALL={RBS_SYSTEMD_MAIN_STALL_S:.1f}s | "
            f"HEARTBEAT={self._notify_interval_s:.1f}s"
        )

    def stop(self):
        self._stop_event.set()


def now_us():
    return time.monotonic_ns() // 1000


def lay_rssi_snr(rfm9x):
    """Đọc RSSI/SNR của gói vừa nhận. Nếu thư viện không hỗ trợ thì trả None."""
    rssi = None
    snr = None

    try:
        value = getattr(rfm9x, "last_rssi", None)
        if value is not None:
            rssi = float(value)
    except Exception:
        rssi = None

    try:
        value = getattr(rfm9x, "last_snr", None)
        if value is not None:
            snr = float(value)
    except Exception:
        snr = None

    return rssi, snr


def chuoi_rssi_snr(rssi, snr):
    rssi_text = "N/A" if rssi is None else f"{rssi:.1f} dBm"
    snr_text = "N/A" if snr is None else f"{snr:.1f} dB"
    return f"RSSI={rssi_text} | SNR={snr_text}"


def tao_thong_ke_lien_ket():
    return {
        "so_goi_rx": 0,
        "rssi_count": 0,
        "rssi_sum": 0.0,
        "rssi_min": None,
        "rssi_max": None,
        "snr_count": 0,
        "snr_sum": 0.0,
        "snr_min": None,
        "snr_max": None,
        "theo_loai": {},
    }


def cap_nhat_thong_ke_lien_ket(thong_ke, rssi, snr, loai_goi):
    if thong_ke is None:
        return

    thong_ke["so_goi_rx"] += 1
    thong_ke["theo_loai"][loai_goi] = thong_ke["theo_loai"].get(loai_goi, 0) + 1

    if rssi is not None:
        thong_ke["rssi_count"] += 1
        thong_ke["rssi_sum"] += float(rssi)
        thong_ke["rssi_min"] = (
            float(rssi)
            if thong_ke["rssi_min"] is None
            else min(thong_ke["rssi_min"], float(rssi))
        )
        thong_ke["rssi_max"] = (
            float(rssi)
            if thong_ke["rssi_max"] is None
            else max(thong_ke["rssi_max"], float(rssi))
        )

    if snr is not None:
        thong_ke["snr_count"] += 1
        thong_ke["snr_sum"] += float(snr)
        thong_ke["snr_min"] = (
            float(snr)
            if thong_ke["snr_min"] is None
            else min(thong_ke["snr_min"], float(snr))
        )
        thong_ke["snr_max"] = (
            float(snr)
            if thong_ke["snr_max"] is None
            else max(thong_ke["snr_max"], float(snr))
        )


def _gia_tri_trung_binh(thong_ke, khoa_tong, khoa_dem):
    so_mau = int(thong_ke.get(khoa_dem, 0))
    if so_mau <= 0:
        return None
    return float(thong_ke.get(khoa_tong, 0.0)) / so_mau


def _fmt_link(value, unit):
    return "N/A" if value is None else f"{value:.1f}{unit}"


def in_tong_ket_lien_ket(diag, ly_do, gps_manager=None):
    """Một dòng tổng kết RF đủ dùng để test thoại/khoảng cách, tránh spam log."""
    if diag is None:
        return

    su = diag["link_su_rbs"]
    du = diag["link_du_rbs"]
    su_rssi = _gia_tri_trung_binh(su, "rssi_sum", "rssi_count")
    su_snr = _gia_tri_trung_binh(su, "snr_sum", "snr_count")
    du_rssi = _gia_tri_trung_binh(du, "rssi_sum", "rssi_count")
    du_snr = _gia_tri_trung_binh(du, "snr_sum", "snr_count")

    print(
        f"[PHIÊN THOẠI] LIÊN KẾT | MÃ={diag['session_id']:016X} | "
        f"SU->rBS RSSI_TB={_fmt_link(su_rssi, 'dBm')} SNR_TB={_fmt_link(su_snr, 'dB')} | "
        f"DU->rBS RSSI_TB={_fmt_link(du_rssi, 'dBm')} SNR_TB={_fmt_link(du_snr, 'dB')} | "
        f"ARQ_LẶP={diag['so_goi_trung_arq']} | KẾT_THÚC={ly_do}"
    )

    if gps_manager is not None:
        gps_manager.in_tom_tat(diag["session_id"])
        gps_manager.ghi_csv_session(diag, ly_do)


def tao_chan_doan_session(session_id):
    return {
        "session_id": session_id,
        "so_lan_start_toi_du": 0,
        "da_nhan_ready_tu_du": False,
        "so_lan_ready_toi_su": 0,
        "so_lan_su_gui_lai_start": 0,
        "da_nhan_voice_dau_tien": False,
        "so_goi_trung_arq": 0,
        "link_su_rbs": tao_thong_ke_lien_ket(),
        "link_du_rbs": tao_thong_ke_lien_ket(),
        "hmi_ket_qua": "",
    }


def in_tom_tat_chan_doan(diag):
    if diag is None:
        return

    print(
        "[THỐNG KÊ BẮT TAY] "
        f"START_rBS->DU={diag['so_lan_start_toi_du']} | "
        f"READY_DU->rBS={1 if diag['da_nhan_ready_tu_du'] else 0} | "
        f"READY_rBS->SU={diag['so_lan_ready_toi_su']} | "
        f"SU_GỬI_LẠI_START={diag['so_lan_su_gui_lai_start']} | "
        f"VOICE_ĐẦU={1 if diag['da_nhan_voice_dau_tien'] else 0}"
    )


# ============================================================
# ID / RADIO
# ============================================================

ID_TRAM_SU = 0x01
ID_TRAM_DU = 0x02
ID_TRAM_RBS = 0x03

TAN_SO_LORA = 433.0
TAN_SO_LORA_HZ = 433_000_000
BANG_THONG_LORA_HZ = 500_000
HE_SO_TRAI_PHO_CO_DINH = 7
MA_HOA_KENH_MAU_SO = 5  # CR 4/5
CONG_SUAT_RBS_DBM = 30  # E22-400M30S: dùng mức công suất tối đa của module


TYPE_VOICE = 0x01
TYPE_SESSION_START = 0x02
TYPE_READY = 0x03
TYPE_AUDIO_END_SU = 0x04
TYPE_FEC = 0x05

TYPE_MASK = 0x0F
FLAG_LAST = 0x10
COUNT_SHIFT = 5

SIZE_SESSION = 12
SIZE_VOICE_PACKET = 106
SIZE_FEC_PACKET = 114
SIZE_CONTROL_SU = 5

VOICE_LENGTH = 98

TYPE_RELAY = 0x10
TYPE_RELAY_END = 0x11

# Control DU -> rBS -> SU de do PTT_RELEASE -> DU_PLAY.
TYPE_PLAY_STARTED = 0x12
SIZE_PLAY_STARTED = 12

# HMI user feedback
TYPE_USER_RESPONSE = 0x13
TYPE_USER_CONFIRM = 0x14
TYPE_SESSION_READY = 0x15
TYPE_SESSION_FAIL = 0x16
TYPE_BLOCK_ACK_V2A = 0x1A
TYPE_SUPERFRAME_V2B = 0x1B
SIZE_USER_HMI = 12
SIZE_SESSION_CTRL = 12
USER_RESPONSE_ACK = 0x01
USER_RESPONSE_NACK = 0x02
USER_HMI_FORWARD_GUARD = 0.008

# Measurement-only DU -> rBS: DU nghe ke beacon dinh ky cua SU de do kenh SU->DU.
TYPE_BAO_CAO_KENH_SU_DU = 0x19
SIZE_BAO_CAO_KENH_SU_DU = 20
DUONG_DAN_CSV_KENH_SU_DU = Path(__file__).resolve().parent / "rbs_kenh_su_du.csv"

# SESSION_START -> SESSION_READY handshake.
# rBS tu retry SESSION_START toi DU toi da 3 lan.
SESSION_MAX_ATTEMPTS = 3
# V10: tang cua so READY de du thoi gian cho DU xu ly START + GPS_PHIEN + READY.
# Neu READY den som thi thoat ngay, khong cong them delay vao truong hop tot.
SESSION_READY_WINDOW = 0.300
SESSION_READY_FORWARD_GUARD = 0.008

# Sau END #1, rBS mo RX window cho DU bao PLAY.
# Playback DU da bat dau ngay tu END #1; window nay KHONG chen delay vao loa.
PLAY_REPORT_WINDOW = 0.120
PLAY_REPORT_FORWARD_GUARD = 0.008

# Giữ các guard đã chứng minh ổn định.
PHASE2_RX_GUARD = 0.008
READY_GUARD = 0.010

# ============================================================
# STREAMING V2A.1 - BLOCK/SLOT + FEC IN-BURST (1 SU + 1 DU)
#
# Uplink collect slot: rBS giu RX va gom toi da 2 VOICE lien tiep.
# Downlink slot: rBS relay cac VOICE da nhan theo thu tu SEQ.
# Cuoi block: 1 BLOCK_ACK xac nhan VOICE bitmap + FEC status.
# Neu parity den han, SU gui FEC ngay sau VOICE trong cung uplink burst; rBS
# relay VOICE+FEC trong cung downlink burst, KHONG co ACK rieng cho FEC.
# ============================================================
V2A_VOICE_PER_BLOCK = 2
V2A_COLLECT_WINDOW_S = 0.090
V2A_DOWNLINK_GUARD_S = 0.004
V2A_POST_DOWNLINK_ACK_GUARD_S = 0.008
V2A_FEC_BIT = 0x04
V2A_FEC_DATA_PER_GROUP = 8

# ============================================================
# STREAMING V2C0 - CAPACITY PROFILE 600 ms, BENCH 1 SU + 1 DU
#
# t=0       rBS bat dau phat beacon/ACK frame truoc
# ~t=33 ms  beacon RX_DONE tai SU (thuc te bench)
# SU RX_DONE+12 ms: uplink burst toi da 3 VOICE + toi da 1 FEC best-effort
# t=265 ms  rBS bat dau downlink burst
# t=480 ms  beacon ke tiep; ACK chi xac nhan VOICE bitmap
#
# V2B.1 sua loi bench V2B: FEC bi day sang frame sau lam retry VOICE va
# queue SU tran. FEC 4+1 van duoc relay nhung la BEST-EFFORT, khong gate ACK.
# 480 ms + 3 VOICE tao dung 480 ms audio/superframe va co guard cho 1 parity.
# Python chi ra lenh tai moc lich; STM32 van la radio driver TX_DONE/RX.
# ============================================================
V2B_SCHEDULE_VERSION = 3
V2B_SUPERFRAME_S = 0.600
V2B_DOWNLINK_OFFSET_S = 0.300
V2B_FIRST_BEACON_DELAY_S = 0.050
V2B_INTER_PACKET_GUARD_S = 0.003
V2B_STM32_BURST_GUARD_MS = 3
V2B_ACK_NONE_BASE = 0xFFFFFFFF
V2B_BEACON_LATE_WARN_S = 0.015

# RX watchdog cho rBS qua STM32 bridge.
# DU/SU co beacon dinh ky ~5 s, nen 20 s im lang la bat thuong trong che do van hanh.
# Khong thay doi protocol/ARQ/FEC/AES hay cac guard da on dinh.
# SELF-HEAL V1:
# SU/DU đang phát beacon/telemetry khoảng 5 s/lần. Sau 15 s không thấy packet,
# chỉ kiểm tra trạng thái bridge/radio; KHÔNG hard-reset E22 chỉ vì im sóng.
RX_WATCHDOG_IM_LANG_S = 15.0
RX_IDLE_YIELD_S = 0.001

# Explicit AUDIO_END là đường bình thường.
# Fallback để dài hơn ARQ retry, không ảnh hưởng latency bình thường.
END_OF_AUDIO_IDLE = 1.500


# ============================================================
# KENH THAT SU -> DU
# DU nghe ke packet 0x18 cua SU va bao RSSI do duoc ve rBS.
# ============================================================
def _i16_be(data):
    return int.from_bytes(bytes(data), byteorder="big", signed=True)


def tinh_kenh_tu_rssi_pt(rssi_dbm, pt_dbm):
    """Dung dung 2 cong thuc do that: RSSI->Pr, Pr=Pt|H|^2."""
    pr_mw = 10.0 ** (float(rssi_dbm) / 10.0)
    pt_mw = 10.0 ** (float(pt_dbm) / 10.0)
    h2 = pr_mw / pt_mw
    h_abs = math.sqrt(max(h2, 0.0))
    h_db = 10.0 * math.log10(h2) if h2 > 0.0 else None
    return pr_mw, pt_mw, h2, h_abs, h_db


def parse_bao_cao_kenh_su_du(packet_bytes):
    if packet_bytes is None:
        return None
    p = bytes(packet_bytes)
    if len(p) != SIZE_BAO_CAO_KENH_SU_DU:
        return None
    if p[0] != ID_TRAM_RBS or p[1] != ID_TRAM_DU or p[2] != TYPE_BAO_CAO_KENH_SU_DU or p[3] != 1:
        return None

    stt_su = int.from_bytes(p[4:12], byteorder="big", signed=False)
    rssi_dbm = _i16_be(p[12:14]) / 10.0
    snr_db = _i16_be(p[14:16]) / 10.0
    pt_su_dbm = int.from_bytes(p[16:17], byteorder="big", signed=True)
    sf_su = int(p[17])
    pr_mw, pt_mw, h2, h_abs, h_db = tinh_kenh_tu_rssi_pt(rssi_dbm, pt_su_dbm)
    return {
        "stt_su": stt_su,
        "rssi_dbm": rssi_dbm,
        "snr_db": snr_db,
        "pt_su_dbm": pt_su_dbm,
        "sf_su": sf_su,
        "pr_mw": pr_mw,
        "pt_mw": pt_mw,
        "h2": h2,
        "h_abs": h_abs,
        "h_db": h_db,
    }


def ghi_csv_kenh_su_du(mau):
    if mau is None:
        return
    dong = {
        "Thoi_gian_rBS_nhan_UTC": datetime.now(timezone.utc).isoformat(),
        "Lien_ket": "SU-DU",
        "Nguon_do": "DU_nghe_beacon_SU",
        "STT_beacon_SU": mau["stt_su"],
        "RSSI_SU_DU_dBm": mau["rssi_dbm"],
        "SNR_SU_DU_dB": mau["snr_db"],
        "P_TX_SU_dBm": mau["pt_su_dbm"],
        "P_r_mW": mau["pr_mw"],
        "P_t_mW": mau["pt_mw"],
        "H_abs_binh_phuong": mau["h2"],
        "H_abs": mau["h_abs"],
        "H_dB": mau["h_db"],
        "SF_SU": mau["sf_su"],
    }
    DUONG_DAN_CSV_KENH_SU_DU.parent.mkdir(parents=True, exist_ok=True)
    new_file = not DUONG_DAN_CSV_KENH_SU_DU.exists() or DUONG_DAN_CSV_KENH_SU_DU.stat().st_size == 0
    with DUONG_DAN_CSV_KENH_SU_DU.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(dong.keys()))
        if new_file:
            writer.writeheader()
        writer.writerow(dong)

    # Log nay duoc coi la quan trong de test kenh that.
    _in_goc(
        f"[KENH THAT SU-DU] RSSI={mau['rssi_dbm']:.1f} dBm | "
        f"P_TX={mau['pt_su_dbm']} dBm | |H|^2={mau['h2']:.3e} | "
        f"|H|={mau['h_abs']:.3e} | H={mau['h_db']:.1f} dB"
    )


# ============================================================
# PARSER SU
# ============================================================

def parse_packet_su(packet_bytes):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    if len(p) not in (
        SIZE_SESSION,
        SIZE_VOICE_PACKET,
        SIZE_FEC_PACKET,
        SIZE_CONTROL_SU,
        KICH_THUOC_BAO_CAO_GPS,
    ):
        return None

    dst = p[0]
    src = p[1]

    # GPS packet 0x17 (gan phien) hoac 0x18 (dinh ky), dich truc tiep rBS.
    if len(p) == KICH_THUOC_BAO_CAO_GPS and p[2] in (LOAI_BAO_CAO_GPS_PHIEN, LOAI_BAO_CAO_VI_TRI_DINH_KY):
        bao_cao_gps = phan_tich_bao_cao_gps(p)
        if bao_cao_gps is not None and bao_cao_gps["nguon"] == MA_GPS_SU:
            return {"kind": "gps", "raw": p, "gps": bao_cao_gps}
        return None

    packet_type = p[2] & TYPE_MASK

    if dst != ID_TRAM_DU or src != ID_TRAM_SU:
        return None

    # --------------------------------------------------------
    # AUDIO_END 5B
    # --------------------------------------------------------
    if len(p) == SIZE_CONTROL_SU:
        if packet_type == TYPE_AUDIO_END_SU:
            return {
                "kind": "audio_end",
                "raw": p,
            }

        return None

    # --------------------------------------------------------
    # SESSION
    # --------------------------------------------------------
    if packet_type == TYPE_SESSION_START:
        if len(p) != SIZE_SESSION or p[3] != 8:
            return None

        session_id = int.from_bytes(
            p[4:12],
            byteorder="big",
            signed=False,
        )

        if session_id == 0:
            return None

        return {
            "kind": "session",
            "raw": p,
            "session_id": session_id,
        }

    # --------------------------------------------------------
    # VOICE 96B
    #
    # byte2:
    #   bits7..5 frame_count-1
    #   bit4     LAST
    #   bits3..0 TYPE
    #
    # byte3 = 88
    # --------------------------------------------------------
    if packet_type == TYPE_VOICE:
        if len(p) != SIZE_VOICE_PACKET:
            return None

        # V2C0: byte3 mang frame_count truc tiep (1..15).
        frames = int(p[3])
        if frames < 1 or frames > 15:
            return None
        last_audio = (p[2] & FLAG_LAST) != 0

        seq = int.from_bytes(
            p[4:8],
            byteorder="big",
            signed=False,
        )

        if seq & 0x80000000:
            return None

        return {
            "kind": "voice",
            "raw": p,
            "seq": seq,
            "frames": frames,
            "last_audio": last_audio,
            "ack_kind": TYPE_VOICE,
            "ack_seq": seq,
        }

    # --------------------------------------------------------
    # FEC 104B
    #
    # byte2:
    #   bits7..5 data_count-1
    #   bit4     GROUP_HAS_LAST
    #   bits3..0 TYPE_FEC
    #
    # byte3:
    #   0 hoặc final_frame_count 1..8
    #
    # byte4..7 group_start_seq
    # --------------------------------------------------------
    if packet_type == TYPE_FEC:
        if len(p) != SIZE_FEC_PACKET:
            return None

        data_count = ((p[2] >> COUNT_SHIFT) & 0x07) + 1
        has_last = (p[2] & FLAG_LAST) != 0
        final_frames = p[3]

        if has_last:
            if final_frames < 1 or final_frames > 15:
                return None
        else:
            if final_frames != 0:
                return None

        group_start = int.from_bytes(
            p[4:8],
            byteorder="big",
            signed=False,
        )

        if group_start & 0x80000000:
            return None

        return {
            "kind": "fec",
            "raw": p,
            "group_start": group_start,
            "data_count": data_count,
            "has_last": has_last,
            "final_frames": final_frames,
            "ack_kind": TYPE_FEC,
            "ack_seq": group_start,
        }

    return None


# ============================================================
# RELAY / CONTROL
# ============================================================

def gui_relay_wrapper(rfm9x, packet_goc):

    packet_goc = bytes(packet_goc)

    if len(packet_goc) not in (
        SIZE_SESSION,
        SIZE_VOICE_PACKET,
        SIZE_FEC_PACKET,
    ):
        raise ValueError(
            f"Do dai packet goc khong hop le: {len(packet_goc)}"
        )

    # RadioHead 4B outer header.
    # VOICE 96B  -> 100B physical relay.
    # FEC   104B -> 108B physical relay.
    rfm9x.send(
        packet_goc,
        destination=ID_TRAM_DU,
        node=ID_TRAM_RBS,
        identifier=TYPE_RELAY,
        flags=len(packet_goc),
    )


def gui_ready_ack(
    rfm9x,
    ack_kind,
    ack_seq,
):
    # 4B RadioHead header + 4B payload seq = 8B.
    rfm9x.send(
        int(ack_seq).to_bytes(
            4,
            byteorder="big",
            signed=False,
        ),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_READY,
        flags=ack_kind,
    )

    print(
        f"[rBS GỬI ACK] -> SU | "
        f"LOẠI=0x{ack_kind:02X} | SEQ={ack_seq}"
    )


def parse_session_ready_du(packet_bytes, expected_session_id):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    if len(p) != SIZE_SESSION_CTRL:
        return None

    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_DU
        or p[2] != TYPE_SESSION_READY
    ):
        return None

    sid = int.from_bytes(
        p[4:12],
        byteorder="big",
        signed=False,
    )

    if sid == 0:
        return None

    if expected_session_id is not None and sid != expected_session_id:
        return None

    return sid


def gui_session_ready_cho_su(rfm9x, session_id):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_SESSION_READY,
        flags=0,
    )

    print(
        f"[rBS GỬI] SESSION_READY -> SU | SESSION={session_id:016X}"
    )


def gui_session_fail_cho_su(rfm9x, session_id):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_SESSION_FAIL,
        flags=0,
    )

    print(
        f"[rBS GỬI] SESSION_FAIL -> SU | SESSION={session_id:016X}"
    )


def thiet_lap_session_voi_du(rfm9x, goi_session, session_id, diag_session=None, gps_manager=None):
    """
    rBS tự động bắt tay với DU:
      SESSION_START -> chờ SESSION_READY.
    Tổng cộng tối đa 3 lần gửi SESSION_START.

    Trả về dict để main dùng cho chẩn đoán.
    """

    ket_qua = {
        "ok": False,
        "so_lan_start_toi_du": 0,
        "da_nhan_ready_tu_du": False,
        "so_lan_ready_toi_su": 0,
    }

    for attempt in range(1, SESSION_MAX_ATTEMPTS + 1):
        ket_qua["so_lan_start_toi_du"] += 1

        print(
            f"[CHẨN ĐOÁN S1] rBS -> DU: ĐÃ PHÁT SESSION_START | "
            f"LẦN={attempt}/{SESSION_MAX_ATTEMPTS} | "
            f"SESSION={session_id:016X}"
        )

        gui_relay_wrapper(
            rfm9x,
            goi_session,
        )

        deadline = time.monotonic() + SESSION_READY_WINDOW

        while time.monotonic() < deadline:
            p = rfm9x.receive(
                timeout=0.005,
                with_header=True,
            )

            if p is None:
                continue

            rssi, snr = lay_rssi_snr(rfm9x)

            bao_cao_gps = phan_tich_bao_cao_gps(p)
            if bao_cao_gps is not None:
                du_lieu_gps = gps_manager.cap_nhat(bao_cao_gps, rssi, snr) if gps_manager is not None else bao_cao_gps

                if bao_cao_gps["la_dinh_ky"]:
                    ten_tb = "SU" if bao_cao_gps["nguon"] == MA_GPS_SU else "DU"
                    print(
                        f"[CHẨN ĐOÁN GPS ĐỊNH KỲ] NHẬN 0x18 từ {ten_tb} "
                        f"| STT={bao_cao_gps['so_thu_tu_bao_cao']} "
                        f"| SIZE={len(p)}B | {chuoi_rssi_snr(rssi, snr)}"
                    )
                    if gps_manager is not None:
                        gps_manager.in_bao_cao_dinh_ky(du_lieu_gps)
                    continue

                if (
                    bao_cao_gps["nguon"] == MA_GPS_DU
                    and bao_cao_gps["ma_phien"] == session_id
                ):
                    if diag_session is not None:
                        cap_nhat_thong_ke_lien_ket(
                            diag_session["link_du_rbs"],
                            rssi,
                            snr,
                            "GPS_REPORT",
                        )
                    print(
                        f"[rBS GPS] NHAN GPS_PHIEN tu DU | SESSION={session_id:016X} | "
                        f"HOP_LE={1 if bao_cao_gps['gps_hop_le'] else 0} | "
                        f"VI_DO={bao_cao_gps['vi_do']:.7f} | KINH_DO={bao_cao_gps['kinh_do']:.7f} | "
                        f"VE_TINH={bao_cao_gps['so_ve_tinh']} | HDOP={bao_cao_gps['hdop']:.2f} | "
                        f"TUOI_FIX={bao_cao_gps['tuoi_fix_ms']} ms | {chuoi_rssi_snr(rssi, snr)}"
                    )
                    continue

            sid = parse_session_ready_du(
                p,
                session_id,
            )

            if sid is None:
                continue

            ket_qua["da_nhan_ready_tu_du"] = True

            if diag_session is not None:
                cap_nhat_thong_ke_lien_ket(
                    diag_session["link_du_rbs"],
                    rssi,
                    snr,
                    "SESSION_READY",
                )

            print(
                f"[CHẨN ĐOÁN S2] DU -> rBS: NHẬN SESSION_READY = OK | "
                f"SESSION={sid:016X} | {chuoi_rssi_snr(rssi, snr)}"
            )

            # SU có thể nghe ké DU->rBS READY. Cho SU xả packet + re-arm RX.
            time.sleep(SESSION_READY_FORWARD_GUARD)

            print(
                f"[THỜI GIAN][rBS] GUARD trước khi chuyển READY sang SU = "
                f"{SESSION_READY_FORWARD_GUARD * 1000:.1f} ms"
            )

            gui_session_ready_cho_su(
                rfm9x,
                sid,
            )
            ket_qua["so_lan_ready_toi_su"] += 1

            print(
                f"[CHẨN ĐOÁN S3] rBS -> SU: ĐÃ PHÁT SESSION_READY | "
                f"SESSION={sid:016X}"
            )

            ket_qua["ok"] = True
            return ket_qua

        print(
            f"[rBS BẮT TAY] CHƯA NHẬN SESSION_READY từ DU | "
            f"LẦN={attempt}/{SESSION_MAX_ATTEMPTS}"
        )

    gui_session_fail_cho_su(
        rfm9x,
        session_id,
    )

    print(
        "[KẾT LUẬN CHẨN ĐOÁN] BẮT TAY SESSION THẤT BẠI: "
        "rBS không nhận được SESSION_READY sau 3 lần."
    )
    print(
        "[KẾT LUẬN CHẨN ĐOÁN] CHƯA XÁC ĐỊNH chính xác: "
        "có thể mất SESSION_START ở rBS->DU hoặc mất SESSION_READY ở DU->rBS."
    )

    return ket_qua


def parse_play_started_du(packet_bytes, expected_session_id):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    # DU tu dong tao RadioHead-compatible header 4B + session 8B.
    if len(p) != SIZE_PLAY_STARTED:
        return None

    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_DU
        or p[2] != TYPE_PLAY_STARTED
    ):
        return None

    session_id = int.from_bytes(
        p[4:12],
        byteorder="big",
        signed=False,
    )

    if session_id == 0:
        return None

    if (
        expected_session_id is not None
        and session_id != expected_session_id
    ):
        return None

    return session_id


def gui_play_started_cho_su(rfm9x, session_id):

    rfm9x.send(
        int(session_id).to_bytes(
            8,
            byteorder="big",
            signed=False,
        ),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_PLAY_STARTED,
        flags=0,
    )

    print(
        f"[rBS GỬI] PLAY_STARTED -> SU | "
        f"SESSION={session_id:016X}"
    )


def parse_user_response_du(packet_bytes):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    if len(p) != SIZE_USER_HMI:
        return None

    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_DU
        or p[2] != TYPE_USER_RESPONSE
        or p[3] not in (USER_RESPONSE_ACK, USER_RESPONSE_NACK)
    ):
        return None

    sid = int.from_bytes(p[4:12], byteorder="big", signed=False)
    if sid == 0:
        return None

    return {"session_id": sid, "code": p[3]}


def gui_user_response_cho_su(rfm9x, session_id, code):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_USER_RESPONSE,
        flags=code,
    )

    print(
        f"[rBS PHẢN HỒI] {'ACK TỰ ĐỘNG' if code == USER_RESPONSE_ACK else 'NACK THỦ CÔNG'} "
        f"-> SU | SESSION={session_id:016X}"
    )


def parse_user_confirm_su(packet_bytes):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    if len(p) != SIZE_USER_HMI:
        return None

    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_SU
        or p[2] != TYPE_USER_CONFIRM
        or p[3] not in (USER_RESPONSE_ACK, USER_RESPONSE_NACK)
    ):
        return None

    sid = int.from_bytes(p[4:12], byteorder="big", signed=False)
    if sid == 0:
        return None

    return {"session_id": sid, "code": p[3]}


def gui_user_confirm_cho_du(rfm9x, session_id, code):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_DU,
        node=ID_TRAM_RBS,
        identifier=TYPE_USER_CONFIRM,
        flags=code,
    )

    print(
        f"[rBS XÁC NHẬN] SU đã nhận {'ACK' if code == USER_RESPONSE_ACK else 'NACK'} -> DU | "
        f"SESSION={session_id:016X}"
    )


def gui_end_audio_cho_du(rfm9x, expected_session_id=None, diag_session=None):
    """
    END #1 -> DU bat dau PLAY.
    rBS lap tuc mo RX de nhan PLAY_STARTED cua DU, forward ve SU.
    Sau do moi gui END #2/#3 lam redundancy nhu baseline cu.

    Nhu vay reverse report khong can cho het 3 END, nen phep do E2E
    chi bi cong them airtime cua 2 packet report nho (~20.6 ms).
    """

    # END #1 - day la END lam DU mo play gate binh thuong.
    rfm9x.send(
        b"\x00",
        destination=ID_TRAM_DU,
        node=ID_TRAM_RBS,
        identifier=TYPE_RELAY_END,
        flags=0,
    )

    print("[rBS GỬI] END_AUDIO #1 -> DU | MỞ CỬA SỔ CHỜ PLAY_STARTED")

    got_play_report = False
    deadline = time.monotonic() + PLAY_REPORT_WINDOW

    while time.monotonic() < deadline:
        p = rfm9x.receive(
            timeout=0.005,
            with_header=True,
        )

        if p is None:
            continue

        sid = parse_play_started_du(
            p,
            expected_session_id,
        )

        if sid is None:
            continue

        rssi_play, snr_play = lay_rssi_snr(rfm9x)

        if diag_session is not None:
            cap_nhat_thong_ke_lien_ket(
                diag_session["link_du_rbs"],
                rssi_play,
                snr_play,
                "PLAY_STARTED",
            )

        print(
            f"[rBS NHẬN] PLAY_STARTED từ DU | "
            f"SESSION={sid:016X} | {chuoi_rssi_snr(rssi_play, snr_play)}"
        )

        # SU cung nghe thay packet DU->rBS. Cho SU doc/xả packet do
        # va re-arm RX continuous truoc khi rBS forward report.
        time.sleep(PLAY_REPORT_FORWARD_GUARD)

        print(
            f"[THỜI GIAN][rBS] GUARD trước khi chuyển PLAY_STARTED sang SU = "
            f"{PLAY_REPORT_FORWARD_GUARD * 1000:.1f} ms"
        )

        gui_play_started_cho_su(
            rfm9x,
            sid,
        )

        got_play_report = True
        break

    if not got_play_report:
        print(
            "[rBS CẢNH BÁO] HẾT THỜI GIAN CHỜ PLAY_STARTED -> SU sẽ hiện E2E ---"
        )

    # Giu redundancy END x3: gui not END #2 va #3.
    for lan in range(2):
        rfm9x.send(
            b"\x00",
            destination=ID_TRAM_DU,
            node=ID_TRAM_RBS,
            identifier=TYPE_RELAY_END,
            flags=0,
        )

        if lan == 0:
            time.sleep(0.020)

    print("[rBS GỬI] END_AUDIO #2/#3 -> DU")


# ============================================================
# RELAY 1 PACKET DATA (VOICE/FEC)
# ============================================================

def relay_one_data_packet(
    rfm9x,
    info,
    goi_session,
    session_da_gui,
    t_rx_us,
):

    t_start = now_us()

    print()
    print("----------------------------------------------------")
    print(
        f"[rBS] BẮT ĐẦU PHA 2 | CHUYỂN TIẾP 1 GÓI {info['kind'].upper()}"
    )
    print("----------------------------------------------------")

    if t_rx_us is not None:
        print(
            f"[THỜI GIAN][rBS] TỪ LÚC NHẬN GÓI -> BẮT ĐẦU PHA 2 = "
            f"{t_start - t_rx_us} us"
        )

    if session_da_gui:
        t0 = now_us()
        time.sleep(PHASE2_RX_GUARD)
        t1 = now_us()

        print(
            f"[THỜI GIAN][rBS] PHASE2_RX_GUARD = "
            f"{(t1 - t0) / 1000:.3f} ms"
        )

    # Nhánh dự phòng cũ. Với SESSION_READY bình thường, DU đã READY trước DATA.
    if (
        not session_da_gui
        and goi_session is not None
    ):
        session_id = int.from_bytes(
            goi_session[4:12],
            byteorder="big",
            signed=False,
        )

        for lan in range(2):
            gui_relay_wrapper(
                rfm9x,
                goi_session,
            )

            if lan == 0:
                time.sleep(0.005)

        print(
            f"[rBS GỬI] SESSION_START dự phòng -> DU | "
            f"SESSION={session_id:016X}"
        )

        session_da_gui = True

    t_tx0 = now_us()

    gui_relay_wrapper(
        rfm9x,
        info["raw"],
    )

    t_tx1 = now_us()

    if info["kind"] == "voice":
        print(
            f"[rBS GỬI DATA] VOICE -> DU | "
            f"SEQ={info['seq']} | "
            f"FRAME={info['frames']} | "
            f"LAST={1 if info['last_audio'] else 0}"
        )
    else:
        print(
            f"[rBS GỬI DATA] FEC -> DU | "
            f"GROUP_START={info['group_start']} | "
            f"DATA={info['data_count']} | "
            f"HAS_LAST={1 if info['has_last'] else 0}"
        )

    print(
        f"[THỜI GIAN][rBS GỬI] THỜI GIAN PHÁT = "
        f"{t_tx1 - t_tx0} us "
        f"({(t_tx1 - t_tx0) / 1000:.3f} ms)"
    )

    # Cho SU xa packet relay toi 108B nghe ke va quay lai RX.
    tg0 = now_us()
    time.sleep(READY_GUARD)
    tg1 = now_us()

    print(
        f"[THỜI GIAN][rBS] READY_GUARD = "
        f"{(tg1 - tg0) / 1000:.3f} ms"
    )

    ta0 = now_us()

    gui_ready_ack(
        rfm9x,
        info["ack_kind"],
        info["ack_seq"],
    )

    ta1 = now_us()

    print(
        f"[THỜI GIAN][rBS] THỜI GIAN PHÁT ACK = "
        f"{ta1 - ta0} us "
        f"({(ta1 - ta0) / 1000:.3f} ms)"
    )

    print("[rBS] KẾT THÚC PHA 2")
    print("[rBS] QUAY LẠI PHA 1 -> NGHE SU")
    print("----------------------------------------------------")
    print()

    return session_da_gui, ta1


def gui_block_ack_v2a(
    rfm9x,
    base_seq,
    expected_count,
    bitmap,
    expected_fec=False,
    fec_received=False,
):
    expected_count = int(expected_count)
    bitmap = int(bitmap) & 0x03

    # flags V2A.1:
    # bits5..4 count, bit3 FEC_OK, bit2 FEC_EXPECTED, bits1..0 voice bitmap.
    flags = ((expected_count & 0x03) << 4) | bitmap
    if expected_fec:
        flags |= 0x04
    if fec_received:
        flags |= 0x08

    rfm9x.send(
        int(base_seq).to_bytes(4, byteorder="big", signed=False),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_BLOCK_ACK_V2A,
        flags=flags,
    )

    print(
        f"[rBS V2A.1 ACK] BLOCK_ACK -> SU | BASE={base_seq} | "
        f"COUNT={expected_count} | BITMAP=0x{bitmap:02X} | "
        f"FEC_EXPECT={1 if expected_fec else 0} | FEC_OK={1 if fec_received else 0}"
    )


def v2a_new_voice_state():
    return {
        "base": None,
        "expected_count": 2,
        "expected_fec": False,
        "received_mask": 0,
        "relayed_mask": 0,
        "packets": {},
        "fec_received": False,
        "fec_relayed": False,
        "fec_info": None,
        "deadline": None,
        "retry_seen_mask": 0,
        "ack_sent_once": False,
        "retry_packet_total": 0,
    }


def v2a_reset_voice_state(state):
    state.clear()
    state.update(v2a_new_voice_state())


def _v2a_expected_mask(state):
    return (1 << int(state["expected_count"])) - 1


def _v2a_expected_attempt_mask(state):
    mask = _v2a_expected_mask(state)
    if state.get("expected_fec", False):
        mask |= V2A_FEC_BIT
    return mask


def _v2a_expected_fec_group_start(state):
    if state.get("base") is None:
        return None
    base = int(state["base"])
    return base - (base % V2A_FEC_DATA_PER_GROUP)


def _v2a_init_base(state, base_seq, expected_count=2):
    state["base"] = int(base_seq)
    state["expected_count"] = int(expected_count)

    # Block base 2,6,10,... ket thuc mot group 4 DATA nen chac chan co FEC.
    # Block base 0,4,8,... chi co FEC neu LAST xuat hien trong block.
    state["expected_fec"] = (
        (int(base_seq) % V2A_FEC_DATA_PER_GROUP)
        == (V2A_FEC_DATA_PER_GROUP - V2A_VOICE_PER_BLOCK)
    )

    state["received_mask"] = 0
    state["relayed_mask"] = 0
    state["packets"] = {}
    state["fec_received"] = False
    state["fec_relayed"] = False
    state["fec_info"] = None
    state["deadline"] = None
    state["retry_seen_mask"] = 0
    state["ack_sent_once"] = False


def _v2a_refresh_deadline(state):
    # Refresh sau MOI packet uplink. Quan trong voi burst 2 VOICE + FEC:
    # 90 ms tinh tu packet dau co the het truoc khi parity 104B den.
    state["deadline"] = time.monotonic() + V2A_COLLECT_WINDOW_S


def _v2a_voice_complete(state):
    expected = _v2a_expected_mask(state)
    return (int(state["received_mask"]) & expected) == expected


def _v2a_all_received(state):
    if not _v2a_voice_complete(state):
        return False
    if state.get("expected_fec", False) and not state.get("fec_received", False):
        return False
    return True


def _v2a_all_relayed(state):
    expected = _v2a_expected_mask(state)
    if (int(state["relayed_mask"]) & expected) != expected:
        return False
    if state.get("expected_fec", False) and not state.get("fec_relayed", False):
        return False
    return True


def _v2a_relay_pending_and_ack(rfm9x, state, reason):
    if state["base"] is None:
        return 0

    expected_mask = _v2a_expected_mask(state)
    received_mask = int(state["received_mask"]) & expected_mask
    pending_voice_mask = received_mask & (~int(state["relayed_mask"]))
    pending_fec = bool(
        state.get("expected_fec", False)
        and state.get("fec_received", False)
        and not state.get("fec_relayed", False)
        and state.get("fec_info") is not None
    )

    relayed_now = 0
    have_downlink = bool(pending_voice_mask or pending_fec)

    if have_downlink:
        # SU da ket thuc uplink burst va chuyen RX. Guard mot lan truoc downlink.
        time.sleep(V2A_DOWNLINK_GUARD_S)

        print(
            f"[rBS V2A.1 SLOT] DOWNLINK BAT DAU | BASE={state['base']} | "
            f"RX_BITMAP=0x{received_mask:02X} | "
            f"FEC_RX={1 if state.get('fec_received', False) else 0} | REASON={reason}"
        )

        pending_items = []
        for slot in range(int(state["expected_count"])):
            bit = 1 << slot
            if pending_voice_mask & bit:
                pending_items.append(("voice", slot))
        if pending_fec:
            pending_items.append(("fec", None))

        for item_index, (kind, slot) in enumerate(pending_items):
            if kind == "voice":
                bit = 1 << slot
                info = state["packets"].get(slot)
                if info is None:
                    continue

                gui_relay_wrapper(rfm9x, info["raw"])
                state["relayed_mask"] |= bit
                relayed_now += 1

                print(
                    f"[rBS V2A.1 DL] VOICE -> DU | BASE={state['base']} | "
                    f"SLOT={slot} | SEQ={info['seq']} | FRAME={info['frames']} | "
                    f"LAST={1 if info['last_audio'] else 0}"
                )
            else:
                info = state.get("fec_info")
                if info is None:
                    continue

                gui_relay_wrapper(rfm9x, info["raw"])
                state["fec_relayed"] = True
                relayed_now += 1

                print(
                    f"[rBS V2A.1 DL] FEC -> DU | BASE={state['base']} | "
                    f"GROUP_START={info['group_start']} | DATA={info['data_count']} | "
                    f"HAS_LAST={1 if info['has_last'] else 0}"
                )

            if item_index + 1 < len(pending_items):
                time.sleep(V2A_DOWNLINK_GUARD_S)

        print(
            f"[rBS V2A.1 SLOT] DOWNLINK KET THUC | BASE={state['base']} | "
            f"RELAYED=0x{int(state['relayed_mask']):02X} | "
            f"FEC_RELAYED={1 if state.get('fec_relayed', False) else 0}"
        )

    # ACK chung cho VOICE + optional FEC. Neu partial, bitmap/FEC_OK noi ro
    # thanh phan nao da nhan; SU retry TOAN burst de giu scheduler don gian.
    time.sleep(V2A_POST_DOWNLINK_ACK_GUARD_S)
    gui_block_ack_v2a(
        rfm9x,
        state["base"],
        state["expected_count"],
        received_mask,
        expected_fec=bool(state.get("expected_fec", False)),
        fec_received=bool(state.get("fec_received", False)),
    )

    state["deadline"] = None
    state["retry_seen_mask"] = 0
    state["ack_sent_once"] = True
    return relayed_now


def v2a_handle_voice_packet(rfm9x, state, info):
    seq = int(info["seq"])
    base_seq = seq & ~1
    slot = seq - base_seq

    if slot not in (0, 1):
        return 0

    expected_count_for_packet = 1 if (info["last_audio"] and slot == 0) else 2

    if state["base"] is None:
        _v2a_init_base(state, base_seq, expected_count_for_packet)
        print(
            f"[rBS V2A.1 SLOT] UPLINK BAT DAU | BASE={base_seq} | "
            f"COUNT={expected_count_for_packet} | "
            f"FEC_EXPECT={1 if state['expected_fec'] else 0}"
        )
    elif int(state["base"]) != base_seq:
        old_expected = _v2a_expected_mask(state)
        old_complete = (
            (int(state["received_mask"]) & old_expected) == old_expected
            and (
                not state.get("expected_fec", False)
                or state.get("fec_received", False)
            )
        )
        if not old_complete:
            print(
                f"[CẢNH BÁO] V2A.1 BLOCK DOI SOM | OLD_BASE={state['base']} | "
                f"RX=0x{int(state['received_mask']):02X} | "
                f"FEC_RX={1 if state.get('fec_received', False) else 0} | "
                f"NEW_BASE={base_seq}"
            )
        _v2a_init_base(state, base_seq, expected_count_for_packet)
        print(
            f"[rBS V2A.1 SLOT] UPLINK BAT DAU | BASE={base_seq} | "
            f"COUNT={expected_count_for_packet} | "
            f"FEC_EXPECT={1 if state['expected_fec'] else 0}"
        )

    # LAST bat ky trong block => parity partial/final chac chan theo sau.
    if info["last_audio"]:
        state["expected_count"] = expected_count_for_packet
        state["expected_fec"] = True

    bit = 1 << slot

    if state.get("ack_sent_once", False):
        state["retry_packet_total"] = int(state.get("retry_packet_total", 0)) + 1
        state["retry_seen_mask"] |= bit

        if not (int(state["received_mask"]) & bit):
            state["received_mask"] |= bit
            state["packets"][slot] = info
            print(
                f"[rBS V2A.1 RETRY UL] NHAN VOICE CON THIEU | BASE={base_seq} | "
                f"SLOT={slot} | RX_BITMAP=0x{int(state['received_mask']):02X} | "
                f"ATTEMPT=0x{int(state['retry_seen_mask']):02X}"
            )
        else:
            print(
                f"[rBS V2A.1 RETRY UL] DUP VOICE | BASE={base_seq} | SLOT={slot} | "
                f"ATTEMPT=0x{int(state['retry_seen_mask']):02X}"
            )

        _v2a_refresh_deadline(state)

        if (
            int(state["retry_seen_mask"]) & _v2a_expected_attempt_mask(state)
        ) == _v2a_expected_attempt_mask(state):
            return _v2a_relay_pending_and_ack(rfm9x, state, "RETRY_BURST_FULL")
        return 0

    if not (int(state["received_mask"]) & bit):
        state["received_mask"] |= bit
        state["packets"][slot] = info
        print(
            f"[rBS V2A.1 UL] VOICE NHAN | BASE={base_seq} | SLOT={slot} | "
            f"SEQ={seq} | RX_BITMAP=0x{int(state['received_mask']):02X} | "
            f"FEC_EXPECT={1 if state.get('expected_fec', False) else 0}"
        )
    else:
        print(
            f"[rBS V2A.1 DUP] VOICE TRUNG TRONG BLOCK | BASE={base_seq} | SLOT={slot}"
        )

    _v2a_refresh_deadline(state)

    if _v2a_voice_complete(state):
        if state.get("expected_fec", False):
            if state.get("fec_received", False):
                return _v2a_relay_pending_and_ack(rfm9x, state, "BLOCK_PLUS_FEC_FULL")

            print(
                f"[rBS V2A.1 SLOT] VOICE DU | BASE={base_seq} | CHO FEC TRONG CUNG UPLINK BURST"
            )
            return 0

        return _v2a_relay_pending_and_ack(rfm9x, state, "BLOCK_FULL")

    return 0


def v2a_handle_fec_packet(rfm9x, state, info):
    """Tra (handled, relayed_count). FEC partial fallback khong thuoc burst V2A.1
    se tra handled=False de di qua legacy ACK cu.
    """
    if state.get("base") is None:
        return False, 0

    # V2A.1 chi ghep FEC khi du group 4 DATA hoac group final HAS_LAST.
    # FEC partial khong-LAST hiem do race capture van giu legacy fallback.
    if not (
        int(info.get("data_count", 0)) >= V2A_FEC_DATA_PER_GROUP
        or bool(info.get("has_last", False))
    ):
        return False, 0

    expected_group = _v2a_expected_fec_group_start(state)
    if expected_group is None or int(info["group_start"]) != int(expected_group):
        return False, 0

    state["expected_fec"] = True
    bit = V2A_FEC_BIT

    if state.get("ack_sent_once", False):
        state["retry_packet_total"] = int(state.get("retry_packet_total", 0)) + 1
        state["retry_seen_mask"] |= bit

        if not state.get("fec_received", False):
            state["fec_received"] = True
            state["fec_info"] = info
            print(
                f"[rBS V2A.1 RETRY UL] NHAN FEC CON THIEU | BASE={state['base']} | "
                f"GROUP_START={info['group_start']} | ATTEMPT=0x{int(state['retry_seen_mask']):02X}"
            )
        else:
            print(
                f"[rBS V2A.1 RETRY UL] DUP FEC | BASE={state['base']} | "
                f"GROUP_START={info['group_start']} | ATTEMPT=0x{int(state['retry_seen_mask']):02X}"
            )

        _v2a_refresh_deadline(state)

        if (
            int(state["retry_seen_mask"]) & _v2a_expected_attempt_mask(state)
        ) == _v2a_expected_attempt_mask(state):
            return True, _v2a_relay_pending_and_ack(
                rfm9x, state, "RETRY_BURST_FULL_WITH_FEC"
            )
        return True, 0

    if not state.get("fec_received", False):
        state["fec_received"] = True
        state["fec_info"] = info
        print(
            f"[rBS V2A.1 UL] FEC NHAN | BASE={state['base']} | "
            f"GROUP_START={info['group_start']} | DATA={info['data_count']} | "
            f"HAS_LAST={1 if info['has_last'] else 0}"
        )
    else:
        print(
            f"[rBS V2A.1 DUP] FEC TRUNG | BASE={state['base']} | GROUP_START={info['group_start']}"
        )

    _v2a_refresh_deadline(state)

    if _v2a_voice_complete(state):
        return True, _v2a_relay_pending_and_ack(rfm9x, state, "BLOCK_PLUS_FEC_FULL")

    return True, 0


def v2a_service_voice_deadline(rfm9x, state):
    deadline = state.get("deadline")
    if deadline is None or time.monotonic() < deadline:
        return 0

    if state["base"] is None:
        state["deadline"] = None
        return 0

    if _v2a_all_relayed(state):
        # ACK cu co the bi mat; da cho du burst retry window, gui lai full ACK.
        return _v2a_relay_pending_and_ack(rfm9x, state, "ACK_RETRY_TIMEOUT")

    return _v2a_relay_pending_and_ack(rfm9x, state, "COLLECT_TIMEOUT")


def reset_session_state():
    return (
        None,   # session_id
        None,   # goi_session
        False,  # session_da_gui / DU READY
        False,  # session_setup_failed
        False,  # da_relay_data
        None,   # deadline
        set(),  # seen keys
    )


# ============================================================
# E22-400M30S QUA STM32 UART BRIDGE
# ============================================================


# ============================================================
# V2B - SUPERFRAME SCHEDULER CO DINH
# ============================================================
def v2b_new_block_state():
    return {
        "base": None,
        "expected_count": 2,
        "received_mask": 0,
        "relayed_mask": 0,
        "packets": {},
        "complete_acked": False,
        "final_seen": False,
    }


def v2b_new_scheduler_state():
    return {
        "active": False,
        "frame_id": 0,
        "next_beacon_at": None,
        "frame_start": None,
        "downlink_due": None,
        "downlink_done": True,
        "block": v2b_new_block_state(),
        # V2B.1: parity tach khoi ACK VOICE. FEC den luc nao thi dua vao hang
        # pending va relay toi da 1 parity/downlink frame. Khong FEC nao duoc
        # phep giu block VOICE cu lai de retry.
        "fec_pending": {},
        "fec_relayed_groups": set(),
        "beacon_count": 0,
        "late_max_ms": 0.0,
    }


def v2b_reset_block(block):
    block.clear()
    block.update(v2b_new_block_state())


def v2b_activate(sched):
    sched.clear()
    sched.update(v2b_new_scheduler_state())
    sched["active"] = True
    sched["next_beacon_at"] = time.monotonic() + V2B_FIRST_BEACON_DELAY_S
    print(
        f"[rBS V2C0] ACTIVE | PERIOD={V2B_SUPERFRAME_S*1000:.0f} ms | "
        f"DL_OFFSET={V2B_DOWNLINK_OFFSET_S*1000:.0f} ms | "
        f"VOICE_PER_FRAME=2 | PACKET_AUDIO=300ms | FEC=8+1 BEST_EFFORT | "
        f"SCHEDULE_VER={V2B_SCHEDULE_VERSION} | STM32_BURST=ON"
    )


def v2b_deactivate(sched, reason=""):
    if sched.get("active", False):
        print(
            f"[rBS V2B.1] DEACTIVE | REASON={reason or 'NONE'} | "
            f"BEACON={sched.get('beacon_count', 0)} | "
            f"FEC_PENDING={len(sched.get('fec_pending', {}))} | "
            f"LATE_MAX={sched.get('late_max_ms', 0.0):.1f} ms"
        )
    sched.clear()
    sched.update(v2b_new_scheduler_state())


def _v2b_expected_mask(block):
    count = max(1, min(2, int(block.get("expected_count", 2))))
    return (1 << count) - 1


def _v2b_voice_complete(block):
    if block.get("base") is None:
        return False
    mask = _v2b_expected_mask(block)
    return (int(block.get("received_mask", 0)) & mask) == mask


def _v2b_ack_fields(block):
    if block.get("base") is None:
        return V2B_ACK_NONE_BASE, 0

    count = max(1, min(2, int(block.get("expected_count", 2))))
    bitmap = int(block.get("received_mask", 0)) & ((1 << count) - 1)

    # V2B.1 ACK_FLAGS:
    #   bits7..6 = VOICE count (1..3)
    #   bits2..0 = VOICE bitmap
    # FEC khong nam trong ACK/retry path.
    flags = ((count & 0x03) << 6) | (bitmap & 0x07)
    return int(block["base"]), flags


def gui_superframe_beacon_v2b(rfm9x, frame_id, block):
    ack_base, ack_flags = _v2b_ack_fields(block)
    payload = (
        int(frame_id).to_bytes(4, byteorder="big", signed=False)
        + int(ack_base).to_bytes(4, byteorder="big", signed=False)
        + bytes((ack_flags & 0xFF,))
    )

    t0 = time.monotonic()
    rfm9x.send(
        payload,
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_SUPERFRAME_V2B,
        flags=V2B_SCHEDULE_VERSION,
    )
    try:
        rfm9x.listen()
    except Exception:
        pass
    tx_ms = (time.monotonic() - t0) * 1000.0

    ack_count = (ack_flags >> 6) & 0x03
    ack_bitmap = ack_flags & 0x07

    print(
        f"[rBS V2B.1 BEACON] FRAME={frame_id} | TX={tx_ms:.1f} ms | "
        f"ACK_BASE={'NONE' if ack_base == V2B_ACK_NONE_BASE else ack_base} | "
        f"COUNT={ack_count} | BITMAP=0x{ack_bitmap:02X}"
    )

    if _v2b_voice_complete(block):
        block["complete_acked"] = True


def _v2b_init_block(block, base_seq, expected_count=2):
    v2b_reset_block(block)
    block["base"] = int(base_seq)
    block["expected_count"] = max(1, min(2, int(expected_count)))


def v2b_handle_voice_packet(sched, info):
    if not sched.get("active", False):
        return False

    block = sched["block"]
    seq = int(info["seq"])

    # V2C0 gom 2 VOICE/superframe: base 0,2,4,6,...
    base_seq = (seq // 2) * 2
    slot = seq - base_seq
    if slot not in (0, 1):
        return True

    expected_count = (slot + 1) if info["last_audio"] else 2

    if block.get("base") is None:
        _v2b_init_block(block, base_seq, expected_count)
        print(
            f"[rBS V2B.1 UL] NEW BLOCK | FRAME={sched.get('frame_id', 0)-1} | "
            f"BASE={base_seq} | COUNT={expected_count}"
        )
    elif int(block["base"]) != base_seq:
        if block.get("complete_acked", False) and _v2b_voice_complete(block):
            _v2b_init_block(block, base_seq, expected_count)
            print(
                f"[rBS V2B.1 UL] NEXT BLOCK | FRAME={sched.get('frame_id', 0)-1} | "
                f"BASE={base_seq} | COUNT={expected_count}"
            )
        else:
            print(
                f"[CẢNH BÁO] V2B.1 NEW BASE KHI BLOCK CU CHUA ACK | "
                f"OLD={block.get('base')} | NEW={base_seq} | "
                f"RX=0x{int(block.get('received_mask',0)):02X}"
            )
            return True

    if info["last_audio"]:
        block["expected_count"] = expected_count
        block["final_seen"] = True

    bit = 1 << slot
    if not (int(block["received_mask"]) & bit):
        block["received_mask"] |= bit
        block["packets"][slot] = info
        print(
            f"[rBS V2B.1 UL] VOICE | FRAME={sched.get('frame_id', 0)-1} | "
            f"BASE={base_seq} | SLOT={slot} | SEQ={seq} | "
            f"RX_BITMAP=0x{int(block['received_mask']):02X}"
        )
    else:
        print(
            f"[rBS V2B.1 UL] DUP VOICE | FRAME={sched.get('frame_id', 0)-1} | "
            f"BASE={base_seq} | SLOT={slot} | SEQ={seq}"
        )
    return True


def v2b_handle_fec_packet(sched, info):
    if not sched.get("active", False):
        return False

    group_start = int(info["group_start"])
    pending = sched.setdefault("fec_pending", {})
    relayed_groups = sched.setdefault("fec_relayed_groups", set())

    if group_start in relayed_groups:
        print(
            f"[rBS V2B.1 UL] DUP FEC DA RELAY | FRAME={sched.get('frame_id', 0)-1} | "
            f"GROUP_START={group_start}"
        )
        return True

    if group_start in pending:
        print(
            f"[rBS V2B.1 UL] DUP FEC PENDING | FRAME={sched.get('frame_id', 0)-1} | "
            f"GROUP_START={group_start}"
        )
        return True

    pending[group_start] = info
    print(
        f"[rBS V2B.1 UL] FEC BEST-EFFORT | FRAME={sched.get('frame_id', 0)-1} | "
        f"GROUP_START={group_start} | DATA={info['data_count']} | "
        f"HAS_LAST={1 if info['has_last'] else 0} | PENDING={len(pending)}"
    )
    return True


def v2b_relay_downlink(rfm9x, sched):
    """Queue one downlink burst.

    V2B.2 keeps application scheduling on Pi but moves the *within-downlink*
    packet train to STM32. This prevents Python from blocking ~40-50 ms for
    every VOICE/FEC packet and then missing the next beacon deadline.
    """
    block = sched["block"]
    frame_open = int(sched.get("frame_id", 0)) - 1

    pending_items = []

    if block.get("base") is not None:
        count = max(1, min(2, int(block.get("expected_count", 2))))
        for slot in range(count):
            bit = 1 << slot
            if (
                (int(block.get("received_mask", 0)) & bit)
                and not (int(block.get("relayed_mask", 0)) & bit)
            ):
                pending_items.append(("voice", slot, None))

    fec_pending = sched.setdefault("fec_pending", {})
    if fec_pending:
        fec_group = next(iter(fec_pending))
        pending_items.append(("fec", None, fec_group))

    if not pending_items:
        if block.get("base") is None:
            print(f"[rBS V2B.2 DL] FRAME={frame_open} | IDLE - KHONG CO UPLINK")
        else:
            print(
                f"[rBS V2B.2 DL] FRAME={frame_open} | BASE={block['base']} | "
                "KHONG CO ITEM MOI DE RELAY"
            )
        return 0

    print(
        f"[rBS V2B.2 DL] BAT DAU | FRAME={frame_open} | "
        f"BASE={'NONE' if block.get('base') is None else block['base']} | "
        f"RX_BITMAP=0x{int(block.get('received_mask',0)):02X} | "
        f"FEC_PENDING={len(fec_pending)}"
    )

    raw_burst = []
    accepted_meta = []
    for kind, slot, fec_group in pending_items:
        if kind == "voice":
            info = block["packets"].get(slot)
            if info is None:
                continue
            raw = bytes((
                ID_TRAM_DU & 0xFF,
                ID_TRAM_RBS & 0xFF,
                TYPE_RELAY & 0xFF,
                len(info["raw"]) & 0xFF,
            )) + bytes(info["raw"])
            raw_burst.append(raw)
            accepted_meta.append(("voice", slot, info))
        else:
            info = fec_pending.get(fec_group)
            if info is None:
                continue
            raw = bytes((
                ID_TRAM_DU & 0xFF,
                ID_TRAM_RBS & 0xFF,
                TYPE_RELAY & 0xFF,
                len(info["raw"]) & 0xFF,
            )) + bytes(info["raw"])
            raw_burst.append(raw)
            accepted_meta.append(("fec", fec_group, info))

    if not raw_burst:
        return 0

    t0 = time.monotonic()
    use_hw_burst = hasattr(rfm9x, "send_burst_raw")
    if use_hw_burst:
        burst_seq = rfm9x.send_burst_raw(
            raw_burst,
            guard_ms=V2B_STM32_BURST_GUARD_MS,
        )
        enqueue_ms = (time.monotonic() - t0) * 1000.0
        print(
            f"[rBS V2B.2 DL BURST] QUEUED STM32 | FRAME={frame_open} | "
            f"ITEMS={len(raw_burst)} | CMD_SEQ={burst_seq} | "
            f"PI_BLOCK={enqueue_ms:.1f} ms"
        )
    else:
        # Fallback for old bridge during rollback/testing.
        for idx, raw in enumerate(raw_burst):
            rfm9x.send(
                raw[4:],
                destination=raw[0],
                node=raw[1],
                identifier=raw[2],
                flags=raw[3],
            )
            if idx + 1 < len(raw_burst):
                time.sleep(V2B_INTER_PACKET_GUARD_S)
        try:
            rfm9x.listen()
        except Exception:
            pass
        print(
            f"[rBS V2B.2 DL BURST] FALLBACK SEQUENTIAL | "
            f"FRAME={frame_open} | ITEMS={len(raw_burst)}"
        )

    # Once STM32 accepts the burst, ownership of those relay packets has moved
    # out of Python. A later bridge/radio error is handled by the existing
    # health-recovery path; do not requeue duplicates in the normal case.
    relayed = 0
    for kind, key, info in accepted_meta:
        relayed += 1
        if kind == "voice":
            slot = key
            block["relayed_mask"] |= 1 << slot
            print(
                f"[rBS V2B.2 DL] VOICE QUEUED | FRAME={frame_open} | "
                f"BASE={block['base']} | SLOT={slot} | SEQ={info['seq']} | "
                f"FRAME_CNT={info['frames']} | LAST={1 if info['last_audio'] else 0}"
            )
        else:
            fec_group = int(key)
            fec_pending.pop(fec_group, None)
            relayed_groups = sched.setdefault("fec_relayed_groups", set())
            relayed_groups.add(fec_group)
            if len(relayed_groups) > 64:
                keep = sorted(relayed_groups)[-32:]
                sched["fec_relayed_groups"] = set(keep)
            print(
                f"[rBS V2B.2 DL] FEC QUEUED | FRAME={frame_open} | "
                f"GROUP_START={info['group_start']} | DATA={info['data_count']} | "
                f"HAS_LAST={1 if info['has_last'] else 0}"
            )

    print(
        f"[rBS V2B.2 DL] KET THUC QUEUE | FRAME={frame_open} | "
        f"RELAYED=0x{int(block.get('relayed_mask',0)):02X} | "
        f"FEC_PENDING={len(sched.get('fec_pending',{}))}"
    )
    return relayed

def v2b_service_timing(rfm9x, sched):
    if not sched.get("active", False):
        return 0

    relayed = 0
    now = time.monotonic()

    downlink_due = sched.get("downlink_due")
    if (
        downlink_due is not None
        and not sched.get("downlink_done", True)
        and now >= downlink_due
    ):
        late_ms = max(0.0, (now - downlink_due) * 1000.0)
        if late_ms > 5.0:
            print(f"[rBS V2B.1 WARN] DOWNLINK LATE={late_ms:.1f} ms")
        relayed += v2b_relay_downlink(rfm9x, sched)
        sched["downlink_done"] = True
        now = time.monotonic()

    next_beacon_at = sched.get("next_beacon_at")
    if next_beacon_at is None or now < next_beacon_at:
        return relayed

    # Neu bi treo/late qua mot chu ky, re-anchor de khong tao bao beacon catch-up.
    late_s = max(0.0, now - next_beacon_at)
    late_ms = late_s * 1000.0
    sched["late_max_ms"] = max(float(sched.get("late_max_ms", 0.0)), late_ms)
    if late_s > V2B_BEACON_LATE_WARN_S:
        print(f"[rBS V2B.1 WARN] BEACON LATE={late_ms:.1f} ms")

    frame_id = int(sched.get("frame_id", 0))
    actual_start = time.monotonic()
    gui_superframe_beacon_v2b(rfm9x, frame_id, sched["block"])

    sched["beacon_count"] = int(sched.get("beacon_count", 0)) + 1
    sched["frame_start"] = actual_start
    sched["downlink_due"] = actual_start + V2B_DOWNLINK_OFFSET_S
    sched["downlink_done"] = False
    sched["frame_id"] = (frame_id + 1) & 0xFFFFFFFF

    if late_s > (V2B_SUPERFRAME_S / 2.0):
        sched["next_beacon_at"] = actual_start + V2B_SUPERFRAME_S
        print("[rBS V2B.1] REALIGN SUPERFRAME SAU LATE LON")
    else:
        sched["next_beacon_at"] = next_beacon_at + V2B_SUPERFRAME_S

    return relayed

def khoi_tao_lora_rbs():
    """Mở UART tới STM32; STM32 sở hữu SPI/BUSY/DIO/RESET của E22.

    Port/baud để STM32E22Bridge (hoặc biến môi trường) làm nguồn cấu hình duy nhất,
    tránh lệch baud giữa gateway và bridge.
    """
    return STM32E22Bridge(
        frequency_hz=TAN_SO_LORA_HZ,
        bandwidth_hz=BANG_THONG_LORA_HZ,
        spreading_factor=HE_SO_TRAI_PHO_CO_DINH,
        coding_rate_denominator=MA_HOA_KENH_MAU_SO,
        tx_power_dbm=CONG_SUAT_RBS_DBM,
        preamble_symbols=8,
        sync_word=0x12,
        crc_enabled=True,
        iq_inverted=False,
    )


def phuc_hoi_rx_mem(radio):
    """Yêu cầu STM32 đưa E22 về RX liên tục, không reset toàn hệ thống."""
    radio.listen()


# ============================================================
# MAIN
# ============================================================

def main():

    print(
        "[HỆ THỐNG] rBS E22-400M30S + STM32 BRIDGE | "
        "433MHz | BW=500kHz | CR=4/5 | SF7 cố định | TX_rBS=30dBm"
    )
    try:
        rfm9x = khoi_tao_lora_rbs()
    except (RuntimeError, BridgeError, OSError) as error:
        print("[LỖI] Không kết nối được STM32/E22:", error)
        return

    print(
        f"[HỆ THỐNG] Pi->STM32 UART={rfm9x.port} | baud={rfm9x.baudrate} | "
        "STM32->E22 bằng SPI"
    )
    print("[HỆ THỐNG] E22 rBS sẵn sàng | SF7 cố định | TX_rBS=30 dBm")

    systemd_wdt = SystemdServiceWatchdog()
    systemd_wdt.start()
    systemd_wdt.progress()
    systemd_wdt.ready()

    gps_manager = QuanLyGPSRBS()

    (
        session_id_hien_tai,
        goi_session,
        session_da_gui,
        session_setup_failed,
        da_relay_data,
        deadline_end_audio,
        seen_keys,
     ) = reset_session_state()

    v2a_voice_state = v2a_new_voice_state()
    v2b_sched = v2b_new_scheduler_state()

    t_ack_end_us = None
    diag_session = None
    diag_hoan_tat_gan_nhat = None

    # SELF-HEAL V1:
    # Im sóng chỉ là tín hiệu để KIỂM TRA health, không phải bằng chứng E22 chết.
    # Khi STM32 reboot do IWDG, START_RX sẽ fail vì s_configured=0; bridge
    # ensure_ready() sẽ tự PING -> CONFIG -> START_RX mà không cần restart service.
    thoi_diem_rx_cuoi = time.monotonic()

    print(
        f"[WATCHDOG] BẬT | IM_LẶNG={RX_WATCHDOG_IM_LANG_S:.0f}s | "
        "SELF_HEAL=PING->RX/RECONFIG/RECONNECT"
    )
    print(
        f"[rBS V2C0] SUPERFRAME READY | PERIOD={V2B_SUPERFRAME_S*1000:.0f} ms | "
        f"DL_OFFSET={V2B_DOWNLINK_OFFSET_S*1000:.0f} ms | "
        f"SCHEDULE_VER={V2B_SCHEDULE_VERSION}"
    )

    while True:

        # Heartbeat của service phải gắn với tiến triển thật của main loop.
        # Nếu main loop deadlock/hang, dòng này không còn chạy; thread watchdog
        # sẽ ngừng sd_notify và systemd tự restart service.
        systemd_wdt.progress()

        # V2B scheduler duoc service TRUOC receive() de beacon/downlink khong bi
        # tre them mot RX timeout 5 ms. Moi action van cho TX_DONE tu STM32.
        try:
            relayed_v2b = v2b_service_timing(rfm9x, v2b_sched)
            if relayed_v2b > 0:
                da_relay_data = True
        except (BridgeError, OSError) as v2b_error:
            print(f"[CẢNH BÁO] V2B.1 timing service lỗi: {v2b_error}")

        t_rx_call_us = now_us()

        if t_ack_end_us is not None:
            print(
                f"[THỜI GIAN][rBS] TỪ ACK CUỐI -> GỌI HÀM NHẬN = "
                f"{t_rx_call_us - t_ack_end_us} us"
            )
            t_ack_end_us = None

        try:
            packet = rfm9x.receive(
                timeout=0.005,
                with_header=True,
            )
        except (BridgeError, OSError) as rx_error:
            # UART/radio vừa báo lỗi: phục hồi ngay, không chờ 15 s.
            print(
                f"[CẢNH BÁO] RX bridge lỗi: {rx_error} -> TỰ PHỤC HỒI"
            )
            packet = None
            try:
                hanh_dong = rfm9x.ensure_ready()
                print(
                    f"[WATCHDOG] TỰ PHỤC HỒI thành công | ACTION={hanh_dong}"
                )
            except Exception as recover_error:  # noqa: BLE001
                print(
                    f"[CẢNH BÁO] TỰ PHỤC HỒI nhanh lỗi: {recover_error} "
                    "-> HARD RESET E22"
                )
                try:
                    rfm9x.reset_radio()
                    print(
                        "[WATCHDOG] HARD RESET + RECONFIG + START_RX thành công"
                    )
                except Exception as hard_error:  # noqa: BLE001
                    print(
                        f"[CẢNH BÁO] HARD RESET lỗi: {hard_error} "
                        "-> reconnect UART/STM32"
                    )
                    try:
                        rfm9x.reconnect()
                        print("[WATCHDOG] STM32/E22 reconnect thành công")
                    except Exception as reconnect_error:  # noqa: BLE001
                        print(
                            f"[LỖI] WATCHDOG reconnect thất bại: "
                            f"{reconnect_error}"
                        )
            thoi_diem_rx_cuoi = time.monotonic()
            time.sleep(0.02)
            continue

        now = time.monotonic()

        # V2A.1 deadline chi con la fallback neu V2B chua ACTIVE.
        if not v2b_sched.get("active", False):
            try:
                relayed_v2a = v2a_service_voice_deadline(rfm9x, v2a_voice_state)
                if relayed_v2a > 0:
                    da_relay_data = True
            except (BridgeError, OSError) as v2a_error:
                print(f"[CẢNH BÁO] V2A fallback deadline service lỗi: {v2a_error}")

        t_packet_rx_us = (
            now_us()
            if packet is not None
            else None
        )

        if packet is not None:
            # Bất kỳ packet RF hợp lệ nào đều chứng minh RX đang sống.
            thoi_diem_rx_cuoi = now
        else:
            im_lang_s = now - thoi_diem_rx_cuoi

            if im_lang_s >= RX_WATCHDOG_IM_LANG_S:
                # Không còn suy luận "im sóng = radio chết".
                # Health path:
                #   PING STM32
                #     -> START_RX nếu E22 còn cấu hình
                #     -> CONFIG + START_RX nếu STM32 vừa reboot
                #     -> reconnect UART + PING + CONFIG + START_RX nếu UART lỗi
                try:
                    hanh_dong = rfm9x.ensure_ready()
                    print(
                        f"[WATCHDOG] TỰ PHỤC HỒI RX | "
                        f"IM_LẶNG={im_lang_s:.1f}s | ACTION={hanh_dong}"
                    )
                except Exception as recover_error:  # noqa: BLE001
                    print(
                        f"[CẢNH BÁO] WATCHDOG health recovery lỗi: "
                        f"{recover_error} -> HARD RESET E22"
                    )
                    try:
                        rfm9x.reset_radio()
                        print(
                            "[WATCHDOG] HARD RESET + RECONFIG + START_RX "
                            "thành công"
                        )
                    except Exception as hard_error:  # noqa: BLE001
                        print(
                            f"[CẢNH BÁO] HARD RESET lỗi: {hard_error} "
                            "-> reconnect UART/STM32"
                        )
                        try:
                            rfm9x.reconnect()
                            print("[WATCHDOG] STM32/E22 reconnect thành công")
                        except Exception as reconnect_error:  # noqa: BLE001
                            print(
                                f"[LỖI] WATCHDOG reconnect thất bại: "
                                f"{reconnect_error}"
                            )

                # Mở cửa sổ health mới sau mỗi lần kiểm tra/phục hồi.
                thoi_diem_rx_cuoi = time.monotonic()

            # Nhường CPU rất ngắn khi không có packet.
            time.sleep(RX_IDLE_YIELD_S)

        if packet is not None:

            rssi_goi, snr_goi = lay_rssi_snr(rfm9x)

            # KENH THAT SU->DU: report measurement-only tu DU.
            mau_kenh_su_du = parse_bao_cao_kenh_su_du(packet)
            if mau_kenh_su_du is not None:
                ghi_csv_kenh_su_du(mau_kenh_su_du)
                continue

            # GPS/VI TRI SU/DU -> rBS. Packet rieng, khong gate VOICE.
            bao_cao_gps = phan_tich_bao_cao_gps(packet)
            if bao_cao_gps is not None:
                du_lieu_gps = gps_manager.cap_nhat(bao_cao_gps, rssi_goi, snr_goi)

                if bao_cao_gps["la_dinh_ky"]:
                    # Giữ telemetry/CSV để đo khoảng cách và chất lượng link,
                    # nhưng không còn điều khiển P_TX/SF tự động.
                    ten_tb = "SU" if bao_cao_gps["nguon"] == MA_GPS_SU else "DU"
                    print(
                        f"[CHẨN ĐOÁN GPS ĐỊNH KỲ] NHẬN 0x18 từ {ten_tb} "
                        f"| STT={bao_cao_gps['so_thu_tu_bao_cao']} "
                        f"| SIZE={len(packet)}B | {chuoi_rssi_snr(rssi_goi, snr_goi)}"
                    )
                    gps_manager.in_bao_cao_dinh_ky(du_lieu_gps)
                    continue

                thong_ke_phien_gps = None
                if diag_session is not None and diag_session["session_id"] == bao_cao_gps["ma_phien"]:
                    thong_ke_phien_gps = diag_session
                elif diag_hoan_tat_gan_nhat is not None and diag_hoan_tat_gan_nhat["session_id"] == bao_cao_gps["ma_phien"]:
                    thong_ke_phien_gps = diag_hoan_tat_gan_nhat

                if thong_ke_phien_gps is not None:
                    khoa_lien_ket = "link_su_rbs" if bao_cao_gps["nguon"] == MA_GPS_SU else "link_du_rbs"
                    cap_nhat_thong_ke_lien_ket(
                        thong_ke_phien_gps[khoa_lien_ket],
                        rssi_goi,
                        snr_goi,
                        "GPS_REPORT",
                    )

                ten_thiet_bi = "SU" if bao_cao_gps["nguon"] == MA_GPS_SU else "DU"
                print(
                    f"[rBS GPS] NHAN GPS_PHIEN tu {ten_thiet_bi} | SESSION={bao_cao_gps['ma_phien']:016X} | "
                    f"HOP_LE={1 if bao_cao_gps['gps_hop_le'] else 0} | "
                    f"VI_DO={bao_cao_gps['vi_do']:.7f} | KINH_DO={bao_cao_gps['kinh_do']:.7f} | "
                    f"DO_CAO={bao_cao_gps['do_cao_m']:.1f}m | TOC_DO={bao_cao_gps['toc_do_m_s']:.2f}m/s | "
                    f"VE_TINH={bao_cao_gps['so_ve_tinh']} | HDOP={bao_cao_gps['hdop']:.2f} | "
                    f"TUOI_FIX={bao_cao_gps['tuoi_fix_ms']} ms | {chuoi_rssi_snr(rssi_goi, snr_goi)}"
                )
                continue

            # HMI: DU -> rBS -> SU
            user_response = parse_user_response_du(packet)
            if user_response is not None:
                diag_hmi = None
                if (
                    diag_session is not None
                    and diag_session["session_id"] == user_response["session_id"]
                ):
                    diag_hmi = diag_session
                elif (
                    diag_hoan_tat_gan_nhat is not None
                    and diag_hoan_tat_gan_nhat["session_id"] == user_response["session_id"]
                ):
                    diag_hmi = diag_hoan_tat_gan_nhat

                if diag_hmi is not None:
                    cap_nhat_thong_ke_lien_ket(
                        diag_hmi["link_du_rbs"],
                        rssi_goi,
                        snr_goi,
                        "ACK_TỰ_ĐỘNG" if user_response["code"] == USER_RESPONSE_ACK else "NACK_THỦ_CÔNG",
                    )
                    diag_hmi["hmi_ket_qua"] = (
                        "ACK" if user_response["code"] == USER_RESPONSE_ACK else "NACK"
                    )

                print(
                    f"[PHẢN HỒI] DU->rBS | "
                    f"KẾT_QUẢ={'ACK_TỰ_ĐỘNG' if user_response['code'] == USER_RESPONSE_ACK else 'NACK_THỦ_CÔNG'} | "
                    f"MÃ={user_response['session_id']:016X}"
                )
                time.sleep(USER_HMI_FORWARD_GUARD)
                gui_user_response_cho_su(
                    rfm9x,
                    user_response["session_id"],
                    user_response["code"],
                )
                continue

            # HMI: SU confirm -> rBS -> DU
            user_confirm = parse_user_confirm_su(packet)
            if user_confirm is not None:
                diag_hmi = None
                if (
                    diag_session is not None
                    and diag_session["session_id"] == user_confirm["session_id"]
                ):
                    diag_hmi = diag_session
                elif (
                    diag_hoan_tat_gan_nhat is not None
                    and diag_hoan_tat_gan_nhat["session_id"] == user_confirm["session_id"]
                ):
                    diag_hmi = diag_hoan_tat_gan_nhat

                if diag_hmi is not None:
                    cap_nhat_thong_ke_lien_ket(
                        diag_hmi["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "USER_CONFIRM",
                    )

                print(
                    f"[PHẢN HỒI] SU->rBS | "
                    f"ĐÃ_NHẬN={'ACK' if user_confirm['code'] == USER_RESPONSE_ACK else 'NACK'} | "
                    f"MÃ={user_confirm['session_id']:016X}"
                )
                time.sleep(USER_HMI_FORWARD_GUARD)
                gui_user_confirm_cho_du(
                    rfm9x,
                    user_confirm["session_id"],
                    user_confirm["code"],
                )

                # V11.1: không in lại tổng kết phiên ở USER_CONFIRM.
                # Tổng kết đã được in đúng một lần khi AUDIO_END/fallback.
                # Việc gọi lại ở đây trước kia làm log phiên thoại lặp 2-3 lần.
                continue

            info = parse_packet_su(
                packet
            )

            if info is None:
                continue

            # ----------------------------------------------------
            # AUDIO_END
            # ----------------------------------------------------
            if info["kind"] == "audio_end":

                # Final block da duoc ACK bang beacon truoc khi SU gui AUDIO_END.
                # Dung superframe truoc khi relay END/PLAY control de control plane
                # khong bi beacon chen vao giua.
                v2b_deactivate(v2b_sched, "AUDIO_END")

                if diag_session is not None:
                    cap_nhat_thong_ke_lien_ket(
                        diag_session["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "AUDIO_END",
                    )

                print(
                    f"[rBS NHẬN] AUDIO_END 5B từ SU | "
                    f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                )

                if da_relay_data:
                    gui_end_audio_cho_du(
                        rfm9x,
                        session_id_hien_tai,
                        diag_session,
                    )

                    print("[rBS] PHIÊN THOẠI KẾT THÚC BẰNG AUDIO_END")

                if diag_session is not None:
                    print(
                        f"[PHIÊN THOẠI] HOÀN TẤT | MÃ={diag_session['session_id']:016X} | "
                        f"ARQ_LẶP={diag_session['so_goi_trung_arq']}"
                    )
                    in_tong_ket_lien_ket(diag_session, "AUDIO_END", gps_manager)
                    diag_hoan_tat_gan_nhat = diag_session

                (
                    session_id_hien_tai,
                    goi_session,
                    session_da_gui,
                    session_setup_failed,
                    da_relay_data,
                    deadline_end_audio,
                    seen_keys,
                ) = reset_session_state()
                v2a_reset_voice_state(v2a_voice_state)
                v2b_deactivate(v2b_sched, "SESSION_RESET")
                diag_session = None

                print("[HỆ THỐNG] rBS sẵn sàng cho phiên tiếp theo")
                print()
                continue

            # ----------------------------------------------------
            # SESSION START -> SESSION_READY HANDSHAKE
            # ----------------------------------------------------
            if info["kind"] == "session":

                session_moi = info["session_id"]

                if session_id_hien_tai != session_moi:
                    if da_relay_data:
                        gui_end_audio_cho_du(
                            rfm9x,
                            session_id_hien_tai,
                            diag_session,
                        )
                        if diag_session is not None:
                            in_tong_ket_lien_ket(diag_session, "SESSION_MỚI ĐẾN TRƯỚC AUDIO_END", gps_manager)
                            diag_hoan_tat_gan_nhat = diag_session

                    session_id_hien_tai = session_moi
                    goi_session = info["raw"]
                    session_da_gui = False
                    session_setup_failed = False
                    da_relay_data = False
                    deadline_end_audio = None
                    seen_keys.clear()
                    v2a_reset_voice_state(v2a_voice_state)
                    v2b_deactivate(v2b_sched, "NEW_SESSION")

                    diag_session = tao_chan_doan_session(session_moi)
                    print(
                        f"[PHIÊN THOẠI] BẮT ĐẦU | MÃ={session_moi:016X} | "
                        f"SF={HE_SO_TRAI_PHO_CO_DINH}"
                    )
                    cap_nhat_thong_ke_lien_ket(
                        diag_session["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "SESSION_START",
                    )
                    print(
                        f"[CHẨN ĐOÁN S0] SU -> rBS: NHẬN SESSION_START = OK | "
                        f"SESSION={session_moi:016X} | "
                        f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                    )

                else:
                    goi_session = info["raw"]

                    if diag_session is None:
                        diag_session = tao_chan_doan_session(session_moi)
                    cap_nhat_thong_ke_lien_ket(
                        diag_session["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "SESSION_START_LẶP",
                    )
                    diag_session["so_lan_su_gui_lai_start"] += 1

                    print(
                        f"[rBS NHẬN] SU gửi lại SESSION_START | "
                        f"SESSION={session_moi:016X} | "
                        f"LẦN_LẶP={diag_session['so_lan_su_gui_lai_start']} | "
                        f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                    )

                # Neu READY da thanh cong nhung SU bi mat packet READY,
                # SU se gui lai cung SESSION_START. Khong bat tay lai DU;
                # chi forward READY lai cho SU.
                if session_da_gui:
                    print(
                        "[CHẨN ĐOÁN] DU đã READY từ trước nhưng SU lại gửi SESSION_START."
                    )
                    print(
                        "[CHẨN ĐOÁN] Khả năng cao gói SESSION_READY rBS->SU trước đó đã bị mất; "
                        "rBS sẽ gửi lại READY cho SU."
                    )
                    gui_session_ready_cho_su(
                        rfm9x,
                        session_id_hien_tai,
                    )
                    if diag_session is not None:
                        diag_session["so_lan_ready_toi_su"] += 1
                        in_tom_tat_chan_doan(diag_session)
                    continue

                # Neu da fail 3 lan, duplicate tu SU chi nhan FAIL lai,
                # khong khoi dong them 3 retry moi.
                if session_setup_failed:
                    print(
                        "[rBS BẮT TAY] Session này đã thất bại trước đó -> gửi lại SESSION_FAIL cho SU."
                    )
                    gui_session_fail_cho_su(
                        rfm9x,
                        session_id_hien_tai,
                    )
                    if diag_session is not None:
                        in_tom_tat_chan_doan(diag_session)
                    continue

                ket_qua_bat_tay = thiet_lap_session_voi_du(
                    rfm9x,
                    goi_session,
                    session_id_hien_tai,
                    diag_session,
                    gps_manager,
                )

                if diag_session is None:
                    diag_session = tao_chan_doan_session(session_id_hien_tai)
                diag_session["so_lan_start_toi_du"] += ket_qua_bat_tay["so_lan_start_toi_du"]
                diag_session["da_nhan_ready_tu_du"] = (
                    diag_session["da_nhan_ready_tu_du"]
                    or ket_qua_bat_tay["da_nhan_ready_tu_du"]
                )
                diag_session["so_lan_ready_toi_su"] += ket_qua_bat_tay["so_lan_ready_toi_su"]

                if ket_qua_bat_tay["ok"]:
                    session_da_gui = True
                    session_setup_failed = False
                    v2a_reset_voice_state(v2a_voice_state)
                    v2b_activate(v2b_sched)
                    print(
                        f"[rBS BẮT TAY] THÀNH CÔNG | SESSION={session_id_hien_tai:016X}"
                    )
                    print(
                        "[CHẨN ĐOÁN] Đường DU -> rBS = OK vì rBS đã nhận SESSION_READY."
                    )
                    print(
                        "[CHẨN ĐOÁN] Đường rBS -> SU chưa được xác nhận tuyệt đối; "
                        "sẽ xác nhận gián tiếp khi rBS nhận VOICE đầu tiên từ SU."
                    )
                else:
                    session_da_gui = False
                    session_setup_failed = True
                    print(
                        f"[PHIÊN THOẠI] BẮT TAY THẤT BẠI | MÃ={session_id_hien_tai:016X}"
                    )

                in_tom_tat_chan_doan(diag_session)
                if not ket_qua_bat_tay["ok"]:
                    in_tong_ket_lien_ket(diag_session, "SESSION_FAIL", gps_manager)
                continue

            # ----------------------------------------------------
            # VOICE / FEC
            # ----------------------------------------------------
            if info["kind"] in ("voice", "fec"):

                if session_id_hien_tai is None:
                    print(
                        "[rBS BỎ GÓI] DATA đến khi chưa có SESSION"
                    )
                    continue

                if not session_da_gui:
                    print(
                        "[rBS BỎ GÓI] DATA đến khi DU chưa SESSION_READY"
                    )
                    continue

                if diag_session is not None:
                    cap_nhat_thong_ke_lien_ket(
                        diag_session["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "VOICE" if info["kind"] == "voice" else "FEC",
                    )

                if info["kind"] == "voice":
                    key = (
                        TYPE_VOICE,
                        info["seq"],
                    )

                    print(
                        f"[rBS NHẬN DATA] VOICE từ SU | "
                        f"SEQ={info['seq']} | "
                        f"FRAME={info['frames']} | "
                        f"LAST={1 if info['last_audio'] else 0} | "
                        f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                    )

                    if diag_session is not None and not diag_session["da_nhan_voice_dau_tien"]:
                        diag_session["da_nhan_voice_dau_tien"] = True
                        print(
                            f"[CHẨN ĐOÁN S4] SU -> rBS: NHẬN VOICE ĐẦU TIÊN = OK | "
                            f"SEQ={info['seq']}"
                        )
                        print(
                            "[KẾT LUẬN CHẨN ĐOÁN] BẮT TAY SESSION THÀNH CÔNG."
                        )
                        print(
                            "[KẾT LUẬN CHẨN ĐOÁN] DU -> rBS = OK "
                            "(rBS đã nhận SESSION_READY)."
                        )
                        print(
                            "[KẾT LUẬN CHẨN ĐOÁN] rBS -> SU = OK GIÁN TIẾP "
                            "(SU chỉ gửi VOICE sau khi nhận SESSION_READY)."
                        )
                        in_tom_tat_chan_doan(diag_session)
                else:
                    key = (
                        TYPE_FEC,
                        info["group_start"],
                    )

                    print(
                        f"[rBS NHẬN DATA] FEC từ SU | "
                        f"GROUP_START={info['group_start']} | "
                        f"DATA={info['data_count']} | "
                        f"HAS_LAST={1 if info['has_last'] else 0} | "
                        f"FINAL_FRAME={info['final_frames']} | "
                        f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                    )

                # Có data mới => chưa end trong lúc xử lý nó.
                deadline_end_audio = None

                # ------------------------------------------------
                # V2B.1 SUPERFRAME
                # Uplink gom toi da 3 VOICE. FEC duoc ghi nhan doc lap/best-effort.
                # v2b_service_timing() relay tai moc downlink t=265 ms; beacon
                # frame ke tiep chi ACK VOICE bitmap, khong gate theo FEC.
                # ------------------------------------------------
                if v2b_sched.get("active", False):
                    if info["kind"] == "voice":
                        try:
                            v2b_handle_voice_packet(v2b_sched, info)
                        except Exception as v2b_error:  # noqa: BLE001
                            print(f"[LỖI] V2B.1 voice handle lỗi: {v2b_error}")

                        if info["last_audio"]:
                            deadline_end_audio = time.monotonic() + END_OF_AUDIO_IDLE
                        continue

                    if info["kind"] == "fec":
                        try:
                            handled_v2b_fec = v2b_handle_fec_packet(v2b_sched, info)
                            if handled_v2b_fec:
                                if info["has_last"]:
                                    deadline_end_audio = time.monotonic() + END_OF_AUDIO_IDLE
                                continue
                        except Exception as v2b_error:  # noqa: BLE001
                            print(f"[LỖI] V2B.1 FEC handle lỗi: {v2b_error}")

                # ------------------------------------------------
                # V2A.1 BLOCK/SLOT + FEC IN-BURST (fallback)
                # VOICE khong relay+ACK tung packet. Neu FEC den han, no duoc
                # gom trong cung uplink burst va relay cung downlink burst;
                # BLOCK_ACK chung xac nhan ca VOICE bitmap va FEC status.
                # ------------------------------------------------
                if info["kind"] == "voice":
                    try:
                        retry_before = int(v2a_voice_state.get("retry_packet_total", 0))
                        relayed_v2a = v2a_handle_voice_packet(
                            rfm9x,
                            v2a_voice_state,
                            info,
                        )
                        retry_after = int(v2a_voice_state.get("retry_packet_total", 0))
                        if diag_session is not None and retry_after > retry_before:
                            diag_session["so_goi_trung_arq"] += retry_after - retry_before
                        if relayed_v2a > 0:
                            da_relay_data = True
                    except (BridgeError, OSError) as v2a_error:
                        print(f"[LỖI] V2A voice block lỗi: {v2a_error}")

                    # Final VOICE co the co FEC trong cung burst; deadline nay
                    # chi la fallback neu explicit AUDIO_END bi mat.
                    if info["last_audio"]:
                        deadline_end_audio = time.monotonic() + END_OF_AUDIO_IDLE
                    continue

                # ------------------------------------------------
                # V2A.1 FEC IN-BURST
                # FEC full-group (DATA=4) hoac final HAS_LAST se di cung block
                # VOICE va dung BLOCK_ACK chung. FEC partial hiem do race capture
                # se tra handled=False va roi xuong legacy ACK ben duoi.
                # ------------------------------------------------
                if info["kind"] == "fec":
                    try:
                        retry_before = int(v2a_voice_state.get("retry_packet_total", 0))
                        handled_v2a_fec, relayed_v2a_fec = v2a_handle_fec_packet(
                            rfm9x,
                            v2a_voice_state,
                            info,
                        )
                        retry_after = int(v2a_voice_state.get("retry_packet_total", 0))
                        if diag_session is not None and retry_after > retry_before:
                            diag_session["so_goi_trung_arq"] += retry_after - retry_before

                        if handled_v2a_fec:
                            if relayed_v2a_fec > 0:
                                da_relay_data = True
                            if info["has_last"]:
                                deadline_end_audio = time.monotonic() + END_OF_AUDIO_IDLE
                            continue
                    except (BridgeError, OSError) as v2a_error:
                        print(f"[LỖI] V2A.1 FEC block lỗi: {v2a_error}")

                # ------------------------------------------------
                # LEGACY DUPLICATE/ACK CHI CON DUNG CHO FEC FALLBACK
                #
                # Không relay lại cho DU.
                # Chỉ phát lại đúng ACK KIND+SEQ.
                # ------------------------------------------------
                if key in seen_keys:

                    if diag_session is not None:
                        diag_session["so_goi_trung_arq"] += 1

                    print(
                        f"[rBS ARQ] GÓI TRÙNG -> KHÔNG CHUYỂN TIẾP LẠI | "
                        f"LOẠI=0x{info['ack_kind']:02X} | "
                        f"SEQ={info['ack_seq']}"
                    )

                    time.sleep(
                        READY_GUARD
                    )

                    ta0 = now_us()

                    gui_ready_ack(
                        rfm9x,
                        info["ack_kind"],
                        info["ack_seq"],
                    )

                    t_ack_end_us = now_us()

                    print(
                        f"[THỜI GIAN][rBS] THỜI GIAN ACK CHO GÓI TRÙNG = "
                        f"{t_ack_end_us - ta0} us"
                    )

                    # Nếu duplicate thuộc final path, vẫn giữ fallback.
                    is_final_marker = (
                        (
                            info["kind"] == "voice"
                            and info["last_audio"]
                        )
                        or
                        (
                            info["kind"] == "fec"
                            and info["has_last"]
                        )
                    )

                    if is_final_marker:
                        deadline_end_audio = (
                            time.monotonic()
                            + END_OF_AUDIO_IDLE
                        )

                    continue

                # ------------------------------------------------
                # PACKET MỚI -> RELAY 1 LẦN
                # ------------------------------------------------
                seen_keys.add(
                    key
                )

                (
                    session_da_gui,
                    t_ack_end_us,
                ) = relay_one_data_packet(
                    rfm9x,
                    info,
                    goi_session,
                    session_da_gui,
                    t_packet_rx_us,
                )

                da_relay_data = True

                # Final VOICE còn phải có FEC sau nó.
                # Deadline chỉ là fallback dài.
                if (
                    (
                        info["kind"] == "voice"
                        and info["last_audio"]
                    )
                    or
                    (
                        info["kind"] == "fec"
                        and info["has_last"]
                    )
                ):
                    deadline_end_audio = (
                        time.monotonic()
                        + END_OF_AUDIO_IDLE
                    )

                continue

        # ========================================================
        # FALLBACK END — chỉ khi explicit AUDIO_END mất/hỏng
        # ========================================================

        if (
            deadline_end_audio is not None
            and
            now >= deadline_end_audio
        ):
            print(
                "[rBS DỰ PHÒNG] HẾT THỜI GIAN CHỜ AUDIO_END -> GỬI END SANG DU"
            )

            if da_relay_data:
                gui_end_audio_cho_du(
                    rfm9x,
                    session_id_hien_tai,
                    diag_session,
                )

            if diag_session is not None:
                print(
                    f"[PHIÊN THOẠI] HOÀN TẤT BẰNG DỰ PHÒNG | "
                    f"MÃ={diag_session['session_id']:016X}"
                )
                in_tong_ket_lien_ket(diag_session, "FALLBACK HẾT THỜI GIAN AUDIO_END", gps_manager)
                diag_hoan_tat_gan_nhat = diag_session

            (
                session_id_hien_tai,
                goi_session,
                session_da_gui,
                session_setup_failed,
                da_relay_data,
                deadline_end_audio,
                seen_keys,
            ) = reset_session_state()
            v2a_reset_voice_state(v2a_voice_state)
            v2b_deactivate(v2b_sched, "FALLBACK_END")
            diag_session = None

            print("[HỆ THỐNG] rBS sẵn sàng cho phiên tiếp theo")
            print()



# ============================================================
# STREAMING V2C1 - DUAL PAIR TDMA CORE
#
# Pair1: SU1=0x01 -> DU1=0x02
# Pair2: SU2=0x04 -> DU2=0x05
# rBS  : 0x03
#
# 600 ms superframe, one broadcast beacon, two independent uplink slots.
# FEC has a dedicated slot and exactly one pair receives FEC_GRANT per frame.
# STM32 autonomous burst relays up to 4 VOICE + 1 FEC + 1 control packet.
# ============================================================
V2C1_ENABLED = os.environ.get("RBS_V2C1", "1").strip().lower() not in {"0", "false", "off", "no"}
V2C1_TYPE_BEACON = 0x1C
V2C1_SCHEDULE_VERSION = 7
V2C1_BROADCAST = 0xFF

# V2C2 adaptive scheduler.  DUAL keeps the proven 600 ms layout.  SINGLE
# uses one 300 ms VOICE per cycle and runs slightly faster than the audio
# production rate, so a small backlog can be drained instead of accumulating.
V2C2_MODE_SINGLE1 = 1
V2C2_MODE_SINGLE2 = 2
V2C2_MODE_DUAL = 3
V2C2_DUAL_SUPERFRAME_S = 0.300
V2C2_DUAL_DOWNLINK_OFFSET_S = 0.185
V2C2_SINGLE_SUPERFRAME_S = 0.300
V2C2_SINGLE_DOWNLINK_OFFSET_S = 0.170
V2C2_SINGLE_JOIN_OFFSET_MS = 80
V2C2_SINGLE_FEC_OFFSET_MS = 105

# Compatibility aliases for older helpers/comments.
V2C1_SUPERFRAME_S = V2C2_DUAL_SUPERFRAME_S
V2C1_DOWNLINK_OFFSET_S = V2C2_DUAL_DOWNLINK_OFFSET_S
# V2C2.3: align first beacon with first 300 ms Speex VOICE readiness.
# Bench showed frame-0 was ~90 ms stale when VOICE became ready, causing
# a full 295 ms skip. 130 ms preserves guard while targeting frame-0 TX.
V2C1_FIRST_BEACON_DELAY_S = 0.240
V2C1_BEACON_LATE_WARN_S = 0.015
V2C1_IDLE_HOLD_S = 1.8
V2C3_PREVOICE_LEASE_S = 2.5
V2C4_JOIN_RETRY_S = 0.360
V2C4_JOIN_MAX_START_TX = 3
V2C4_JOIN_RELEASE_S = 0.650

# V2C5.1A - DUAL -> SINGLE stale-voice release
# Trong DUAL, neu mot pair khong con VOICE trong ~3 chu ky trong khi pair kia
# van dang gui deu, coi pair im la da roi PTT / mat AUDIO_END va thu hoi slot.
# HARD_RELEASE chi la failsafe cho session da tung co VOICE nhung bi mat END_AUDIO.
V2C5_DUAL_FAST_RELEASE_S = 0.95
V2C5_OTHER_PAIR_FRESH_S = 0.65
V2C5_VOICE_HARD_RELEASE_S = 2.20
V2C1_FEC_DATA_PER_GROUP = 8
V2C1_MAX_BURST_ITEMS = 6
V2C1_BURST_GUARD_MS = 3
V2C4_SINGLE_DL_GUARD_MS = 10  # same-DU 2-VOICE burst: cho DU doc FIFO + re-arm RX
V2C4_JOIN_DL_GUARD_MS = 10    # DUAL: 1 VOICE + JOIN/READY control, cho node khac re-arm RX

# V2C5 JAM DRY-RUN - RBS ONLY
# --------------------------
# CHI LAP KE HOACH/IN LOG; KHONG bat them bat ky TX RF nao tren SU/DU/rBS.
# Bits JAM trong beacon van = 0, de firmware SU/DU hien tai khong bi thay doi hanh vi.
# V2C5.1: DRY-RUN logging is forced ON in this diagnostic build.
# This avoids a stale shell/systemd environment accidentally disabling only the
# candidate log while the rest of V2C5 still prints JAM=DRYRUN/RF_OFF.
V2C5_JAM_DRYRUN = True
V2C5_JAM_RF_ENABLE = False  # SAFETY HARD-LOCK: ban nay TUYET DOI khong phat nhiem that.

# V2C5.2 - JAM LEASE DISTRIBUTION (SIM ONLY)
# bits3..2 cua schedule_ctl khong con de 0 co dinh. Chung mang MASK 2-bit:
#   bit0 = PAIR1 duoc cap quyen ho tro GIA LAP trong frame steady
#   bit1 = PAIR2 duoc cap quyen ho tro GIA LAP trong frame steady
# PREPARE/COMMIT luon mask=0. Day CHI la quyen/lease; khong kich TX RF.
V2C52_LEASE_DISTRIBUTION = True

V2C1_BUILD_TAG = os.environ.get("RBS_BUILD_TAG", "V2C5_2_JAM_LEASE_DISTRIBUTION_SIM")

V2C1_PAIRS = {
    1: {"su": 0x01, "du": 0x02, "name": "PAIR1"},
    2: {"su": 0x04, "du": 0x05, "name": "PAIR2"},
}
V2C1_SU_TO_PAIR = {v["su"]: k for k, v in V2C1_PAIRS.items()}
V2C1_DU_TO_PAIR = {v["du"]: k for k, v in V2C1_PAIRS.items()}


def _v2c1_u32be(data):
    return int.from_bytes(bytes(data), "big", signed=False)


def _v2c1_u64be(data):
    return int.from_bytes(bytes(data), "big", signed=False)


def _v2c1_new_block():
    return {
        "base": None,
        "expected_count": 2,
        "received_mask": 0,
        "relayed_mask": 0,
        "packets": {},
        "complete_acked": False,
        "final_seen": False,
    }


def _v2c1_new_pair(pair_index):
    cfg = V2C1_PAIRS[pair_index]
    return {
        "pair": pair_index,
        "name": cfg["name"],
        "su": cfg["su"],
        "du": cfg["du"],
        "session_id": None,
        "session_inner": None,
        "du_ready": False,
        "voice_active": False,
        "ended": False,
        "block": _v2c1_new_block(),
        "fec_pending": None,
        "fec_relayed_groups": set(),
        "last_activity": 0.0,
        # Thoi diem rBS nhan VOICE gan nhat cua pair. Tach rieng khoi
        # last_activity vi FEC/HMI khong duoc phep giu mot slot VOICE stale.
        "last_voice_rx_at": None,
        "voice_rx": 0,
        "voice_dup": 0,
        "voice_retry": 0,
        "fec_rx": 0,
        # Latency baseline (monotonic local clock at rBS).
        "diag_session_rx_at": None,
        "diag_du_ready_at": None,
        "diag_first_voice_rx_at": None,
        "diag_first_voice_dl_accept_at": None,
        "fast_setup_pending": False,
        # V2C4.4 JOIN state: second pair may request entry while the first pair
        # is already streaming.  This state is intentionally separate from
        # voice_active to avoid the circular wait: no DUAL slot -> no VOICE -> no DUAL.
        "join_pending": False,
        "join_start_attempts": 0,
        "join_last_tx_at": None,
        "join_requested_at": None,
    }


def v2c1_new_state():
    now = time.monotonic()
    return {
        "active": False,
        "frame_id": 0,
        "frame_start": None,
        "next_beacon_at": None,
        "downlink_due": None,
        "downlink_done": False,
        "grant_pair": 0,
        "late_max_s": 0.0,
        "beacon_count": 0,
        "pairs": {1: _v2c1_new_pair(1), 2: _v2c1_new_pair(2)},
        "control_queue": [],
        "control_keys": set(),
        "last_activity": now,
        "mode": V2C2_MODE_DUAL,
        "desired_mode": V2C2_MODE_DUAL,
        # Two-beacon mode transition: PREPARE advertises target while current
        # mode remains active; COMMIT changes mode on the following beacon.
        # This gives an SU that loses one beacon a safe one-frame holdover rule.
        "transition_target": 0,
        "frame_period_s": V2C2_DUAL_SUPERFRAME_S,
        "downlink_offset_s": V2C2_DUAL_DOWNLINK_OFFSET_S,
        # V2C5 DRY-RUN: chi luu dau vet cua frame hien tai de doi chieu
        # "ai DA gui voice" va "rBS DA relay voice". Khong dieu khien RF.
        "jam_dryrun_frame": None,
        "jam_dryrun_ul_pairs": set(),
        "jam_dryrun_phase": "STEADY",
        "jam_dryrun_stats": {
            "frames": 0,
            "ul_candidates": 0,
            "dl_candidates": 0,
            "blocked_transition": 0,
            "blocked_control": 0,
        },
    }


def _v2c2_mode_name(mode):
    return {
        V2C2_MODE_SINGLE1: "SINGLE_P1",
        V2C2_MODE_SINGLE2: "SINGLE_P2",
        V2C2_MODE_DUAL: "DUAL",
    }.get(int(mode), "UNKNOWN")


def _v2c2_desired_mode(st):
    # V2C4.4 JOIN PROMOTION
    # ----------------------
    # A second SU cannot send its first VOICE while rBS is still in SINGLE for
    # the first pair. Therefore DUAL promotion must be triggered by the JOIN
    # request/session state, NOT by first VOICE from the second pair.
    #
    # Normal steady-state still uses DU_READY.  join_pending is only the short
    # bridge that breaks the circular dependency during SINGLE -> DUAL.
    work = [i for i, p in st["pairs"].items() if _v2c1_pair_has_work(p)]
    ready = [i for i, p in st["pairs"].items() if _v2c1_pair_has_work(p) and bool(p.get("du_ready"))]
    joining = [i for i, p in st["pairs"].items() if _v2c1_pair_has_work(p) and bool(p.get("join_pending"))]

    if len(work) >= 2 and (len(ready) >= 2 or bool(joining)):
        return V2C2_MODE_DUAL
    if len(ready) == 1:
        return V2C2_MODE_SINGLE1 if ready[0] == 1 else V2C2_MODE_SINGLE2
    if len(work) == 1:
        # Initial/idle-hold handshake: keep a deterministic SINGLE layout for
        # that pair while SESSION_START/READY completes.
        return V2C2_MODE_SINGLE1 if work[0] == 1 else V2C2_MODE_SINGLE2
    return int(st.get("mode", V2C2_MODE_DUAL))


def _v2c2_frame_period(mode):
    return V2C2_SINGLE_SUPERFRAME_S if mode in (V2C2_MODE_SINGLE1, V2C2_MODE_SINGLE2) else V2C2_DUAL_SUPERFRAME_S


def _v2c2_downlink_offset(mode):
    return V2C2_SINGLE_DOWNLINK_OFFSET_S if mode in (V2C2_MODE_SINGLE1, V2C2_MODE_SINGLE2) else V2C2_DUAL_DOWNLINK_OFFSET_S


def _v2c2_resolve_mode_for_next_beacon(st):
    """Return (advertised_mode, prepare_target, phase).

    Mode changes are deliberately two-beacon transactions:
      PREPARE: keep current MODE, advertise TARGET in schedule_ctl bits5..4.
      COMMIT:  next beacon switches MODE and clears TARGET.

    Therefore a node that misses exactly one beacon can safely run one local
    holdover frame: if the previous beacon was PREPARE it predicts COMMIT; if
    it was steady it keeps the same mode.  Two consecutive misses still stop TX.
    """
    desired = _v2c2_desired_mode(st)
    current = int(st.get("mode", V2C2_MODE_DUAL))
    pending = int(st.get("transition_target", 0))
    st["desired_mode"] = desired

    # Once PREPARE has been advertised, COMMIT MUST happen on the next beacon
    # even if desired state changes in the meantime. Otherwise a node that lost
    # exactly that COMMIT beacon could predict the prepared mode while rBS
    # silently cancels it, which would break the one-frame holdover safety rule.
    if pending != 0:
        commit_mode = pending
        st["transition_target"] = 0
        return commit_mode, 0, "COMMIT"

    if desired == current:
        return current, 0, "STEADY"

    # First boundary: PREPARE only. Current slot layout remains valid.
    st["transition_target"] = desired
    return current, desired, "PREPARE"


def _v2c52_jam_lease_mask(st, mode, transition_phase):
    """Tra ve mask 2-bit quyen ho tro GIA LAP cho beacon hien tai.

    Quyen chi duoc cap khi:
    - frame STEADY (PREPARE/COMMIT = 0),
    - pair co session con song va DU da READY,
    - pair nam trong layout mode hien tai.

    Day khong phai lenh TX RF. START/STOP RF chua ton tai trong V2C5.2.
    """
    if not V2C52_LEASE_DISTRIBUTION:
        return 0
    if str(transition_phase) != "STEADY":
        return 0

    mode = int(mode)
    mask = 0
    for pi in (1, 2):
        p = st["pairs"][pi]
        if not _v2c1_pair_has_work(p):
            continue
        if not bool(p.get("du_ready")):
            continue
        if mode == V2C2_MODE_SINGLE1 and pi != 1:
            continue
        if mode == V2C2_MODE_SINGLE2 and pi != 2:
            continue
        if mode not in (V2C2_MODE_SINGLE1, V2C2_MODE_SINGLE2, V2C2_MODE_DUAL):
            continue
        mask |= (1 << (pi - 1))
    return mask & 0x03


def _v2c5_jam_pair_label(pair_index, direction):
    """Tra ve mo ta quyen ho tro NHIEM GIA LAP cho mot pair.

    direction="UL": SUx -> rBS, DUx la node ho tro.
    direction="DL": rBS -> DUx, SUx la node ho tro.
    Ham nay CHI tao chuoi log, khong goi radio/LoRa.
    """
    pi = int(pair_index)
    if pi not in (1, 2):
        return "NONE"
    if direction == "UL":
        return f"DU{pi}->HO_TRO(SU{pi}->rBS)"
    return f"SU{pi}->HO_TRO(rBS->DU{pi})"


def _v2c5_jam_dryrun_log_frame(st, frame_id, meta):
    """In mot dong ke hoach friendly-jamming GIA LAP cho frame da xay xong.

    Nguyen tac bao thu:
    - Beacon/control: luon sach.
    - PREPARE/COMMIT: khong cap quyen jam trong ca frame dry-run.
    - UL: chi danh dau pair THUC SU da co VOICE toi rBS.
    - DL: chi danh dau VOICE THUC SU duoc rBS dua vao burst.
    - Neu burst DL co control, KHONG cap jam DL vi chua co co che cat chinh xac
      theo tung packet con trong burst. Day la fail-safe de khong de len control.
    - KHONG ghi bit JAM vao beacon, KHONG phat RF.
    """
    # V2C5.1 diagnostic build: always emit the dry-run decision.
    # RF remains hard-locked OFF by V2C5_JAM_RF_ENABLE=False.
    stats = st.get("jam_dryrun_stats") or {}
    stats["frames"] = int(stats.get("frames", 0)) + 1
    st["jam_dryrun_stats"] = stats

    mode = int(st.get("mode", V2C2_MODE_DUAL))
    phase = str(st.get("jam_dryrun_phase", "STEADY"))
    transition_block = phase in ("PREPARE", "COMMIT")

    ul_pairs = sorted(int(x) for x in st.get("jam_dryrun_ul_pairs", set()) if int(x) in (1, 2))
    dl_pairs = sorted({int(m[1]) for m in meta if m[0] == "voice" and int(m[1]) in (1, 2)})
    has_control = any(m[0] == "control" for m in meta)

    if transition_block:
        stats["blocked_transition"] = int(stats.get("blocked_transition", 0)) + 1
        ul_text = "OFF(CHUYEN_CHE_DO)" if ul_pairs else "NONE"
        dl_text = "OFF(CHUYEN_CHE_DO)" if dl_pairs else "NONE"
    else:
        ul_text = ",".join(_v2c5_jam_pair_label(pi, "UL") for pi in ul_pairs) if ul_pairs else "NONE"
        if has_control and dl_pairs:
            stats["blocked_control"] = int(stats.get("blocked_control", 0)) + 1
            dl_text = "OFF(CO_GOI_DIEU_KHIEN_TRONG_BURST)"
        else:
            dl_text = ",".join(_v2c5_jam_pair_label(pi, "DL") for pi in dl_pairs) if dl_pairs else "NONE"

        stats["ul_candidates"] = int(stats.get("ul_candidates", 0)) + len(ul_pairs)
        if not has_control:
            stats["dl_candidates"] = int(stats.get("dl_candidates", 0)) + len(dl_pairs)

    print(
        f"[rBS JAM DRYRUN] FRAME={int(frame_id)} | MODE={_v2c2_mode_name(mode)} | "
        f"PHA={phase} | UL={ul_text} | DL={dl_text} | "
        f"BEACON_CTRL=SACH | RF_JAM=OFF"
    )

    # Moi frame chi dung VOICE da thay trong frame do.
    st["jam_dryrun_ul_pairs"] = set()



# ============================================================
# V2C5.3B-1 - DOWNLINK PERMISSION SIM (rBS ONLY)
#
# Tinh quyen ho tro downlink tu BURST THUC TE cua rBS.
# CHI IN LOG. KHONG doi beacon/protocol, KHONG gui control moi, KHONG RF jam.
#
# DL_SAFE_MASK:
#   bit0 = Pair1 co VOICE downlink sach trong burst
#   bit1 = Pair2 co VOICE downlink sach trong burst
#
# Fail-safe:
#   PREPARE/COMMIT -> 0
#   burst co CONTROL -> 0
#   khong co VOICE -> 0
# ============================================================
V2C53B1_DL_PERMISSION_SIM = True


def _v2c53b1_dl_safe_mask(st, frame_id, meta, *, burst_accepted=True):
    if not V2C53B1_DL_PERMISSION_SIM:
        return 0

    mode = int(st.get("mode", V2C2_MODE_DUAL))
    phase = str(st.get("jam_dryrun_phase", "STEADY"))

    voice_pairs = sorted({
        int(m[1])
        for m in meta
        if m[0] == "voice" and int(m[1]) in (1, 2)
    })
    has_control = any(m[0] == "control" for m in meta)

    mask = 0
    reason = "VOICE_ONLY"

    if not burst_accepted:
        reason = "BURST_NOT_ACCEPTED"
    elif phase in ("PREPARE", "COMMIT"):
        reason = "TRANSITION"
    elif has_control:
        reason = "CONTROL_IN_BURST"
    elif not voice_pairs:
        reason = "NO_VOICE"
    else:
        for pi in voice_pairs:
            mask |= (1 << (pi - 1))

    mask &= 0x03

    voice_text = ",".join(str(x) for x in voice_pairs) if voice_pairs else "NONE"
    print(
        f"[rBS V2C5.3B1 DL_PERMISSION] FRAME={int(frame_id)} | "
        f"MODE={_v2c2_mode_name(mode)} | PHA={phase} | "
        f"VOICE_PAIRS={voice_text} | CONTROL={1 if has_control else 0} | "
        f"DL_SAFE_MASK=0x{mask:X} | REASON={reason} | RF_JAM=OFF"
    )

    st["v2c53b1_last_dl_safe_mask"] = mask
    st["v2c53b1_last_dl_frame"] = int(frame_id)
    return mask




# ============================================================
# V2C5.3B-2 - DL PERMISSION DISTRIBUTION (SIM ONLY)
#
# Tiny permission packet is prepended to the SAME accepted STM32 DL burst.
# Physical packet is ONLY the 4-byte RadioHead-compatible header:
#   DST=0xFF | SRC=rBS | TYPE=0x1D | FLAGS
# FLAGS: bits7..4=frame low4, bits3..2=mode, bits1..0=DL_SAFE_MASK.
# No helper/jammer RF TX is enabled by this.
# ============================================================
V2C53B2_DL_PERMISSION_SIM = True
V2C53B2_TYPE_DL_PERMISSION = 0x1D


def _v2c53b2_calc_permission_mask(st, meta):
    if not V2C53B2_DL_PERMISSION_SIM:
        return 0
    phase = str(st.get("jam_dryrun_phase", "STEADY"))
    if phase in ("PREPARE", "COMMIT"):
        return 0
    if any(m[0] == "control" for m in meta):
        return 0

    mask = 0
    for m in meta:
        if m[0] != "voice":
            continue
        pi = int(m[1])
        if pi in (1, 2):
            mask |= 1 << (pi - 1)
    return mask & 0x03


def _v2c53b2_permission_raw(frame_id, mode, mask):
    flags = (
        ((int(frame_id) & 0x0F) << 4)
        | ((int(mode) & 0x03) << 2)
        | (int(mask) & 0x03)
    )
    return bytes((
        V2C1_BROADCAST & 0xFF,
        ID_TRAM_RBS & 0xFF,
        V2C53B2_TYPE_DL_PERMISSION,
        flags & 0xFF,
    ))



def _v2c1_activate(st):
    if st["active"]:
        return
    if V2C5_JAM_RF_ENABLE:
        raise RuntimeError("V2C5 DRY-RUN SAFETY: RF JAM phai luon OFF")
    st["active"] = True
    st["frame_id"] = 0
    st["next_beacon_at"] = time.monotonic() + V2C1_FIRST_BEACON_DELAY_S
    st["frame_start"] = None
    st["downlink_due"] = None
    st["downlink_done"] = False
    st["last_activity"] = time.monotonic()
    # The first session is already registered before activation, so start in
    # SINGLE immediately.  A second session will promote the next beacon to DUAL.
    mode = _v2c2_desired_mode(st)
    st["mode"] = mode
    st["desired_mode"] = mode
    st["transition_target"] = 0
    st["frame_period_s"] = _v2c2_frame_period(mode)
    st["downlink_offset_s"] = _v2c2_downlink_offset(mode)
    print(
        f"[rBS V2C2] ACTIVE | MODE={_v2c2_mode_name(mode)} | "
        f"PERIOD={st['frame_period_s']*1000:.0f}ms | DL_OFFSET={st['downlink_offset_s']*1000:.0f}ms | "
        "SINGLE_UL=12ms/2xHQ(8+7) | SINGLE_DL=170ms | CODEC=SINGLE_HQ/DUAL_LQ | "
        "DUAL_SU1=12ms/1xLQ DUAL_SU2=80ms/1xLQ | DUAL_DL=185ms | JAM=DRYRUN/RF_OFF"
    )


def _v2c1_pair_has_work(pair):
    return bool(pair["session_id"] is not None and not pair["ended"])


def _v2c1_select_grant(st, frame_id, mode=None):
    mode = int(st.get("mode", V2C2_MODE_DUAL) if mode is None else mode)
    if mode == V2C2_MODE_SINGLE1:
        return 1
    if mode == V2C2_MODE_SINGLE2:
        return 2

    eligible = [i for i, p in st["pairs"].items() if p["du_ready"] and p["voice_active"]]
    if len(eligible) == 1:
        return eligible[0]
    if len(eligible) >= 2:
        return 1 if (frame_id & 1) == 0 else 2
    pending = [i for i, p in st["pairs"].items() if _v2c1_pair_has_work(p)]
    if len(pending) == 1:
        return pending[0]
    if len(pending) >= 2:
        return 1 if (frame_id & 1) == 0 else 2
    return 0


def _v2c1_ack_tuple(pair):
    block = pair["block"]
    if block["base"] is None:
        return 0xFFFFFFFF, 0
    count = max(1, min(2, int(block["expected_count"])))
    # ACK chi bit da nhan UPLINK VA da queue thanh cong vao STM32 downlink.
    # Neu DL burst queue fail/late, SU se retry thay vi mat chu am tham o DU.
    mask = int(block["received_mask"]) & int(block["relayed_mask"]) & ((1 << count) - 1)
    return int(block["base"]), ((count & 0x03) << 6) | (mask & 0x03)


def _v2c1_build_beacon_payload(st, frame_id, grant_pair, mode, transition_target=0, jam_lease_mask=0):
    a1, f1 = _v2c1_ack_tuple(st["pairs"][1])
    a2, f2 = _v2c1_ack_tuple(st["pairs"][2])
    payload = bytearray()
    payload += int(frame_id).to_bytes(4, "big")
    payload += int(a1).to_bytes(4, "big")
    payload.append(f1 & 0xFF)
    payload += int(a2).to_bytes(4, "big")
    payload.append(f2 & 0xFF)
    # byte18 schedule_ctl V2C2.2:
    #   bits7..6 = MODE currently valid for this frame
    #   bits5..4 = PREPARE target mode for the NEXT beacon (0 = no prepare)
    #   bits3..2 = V2C5.2 JAM LEASE MASK: bit0=PAIR1, bit1=PAIR2
    #   bits1..0 = FEC grant pair
    # Lease chi la QUYEN GIA LAP. PREPARE/COMMIT mask=0; RF_JAM van hard OFF.
    schedule_ctl = (
        ((int(mode) & 0x03) << 6)
        | ((int(transition_target) & 0x03) << 4)
        | ((int(jam_lease_mask) & 0x03) << 2)
        | (int(grant_pair) & 0x03)
    )
    payload.append(schedule_ctl)
    return bytes(payload)


def _v2c1_mark_acked(pair):
    b = pair["block"]
    if b["base"] is None:
        return
    need = (1 << int(b["expected_count"])) - 1
    ackable = int(b["received_mask"]) & int(b["relayed_mask"])
    if (ackable & need) == need:
        b["complete_acked"] = True


def _v2c1_send_beacon(radio, st, scheduled_at):
    actual = time.monotonic()
    late = max(0.0, actual - scheduled_at)
    st["late_max_s"] = max(st["late_max_s"], late)
    if late > V2C1_BEACON_LATE_WARN_S:
        print(f"[rBS V2C1 WARN] BEACON LATE={late*1000:.1f} ms")

    frame_id = int(st["frame_id"]) & 0xFFFFFFFF
    previous_mode = int(st.get("mode", V2C2_MODE_DUAL))
    mode, transition_target, transition_phase = _v2c2_resolve_mode_for_next_beacon(st)
    if transition_phase == "PREPARE":
        print(
            f"[rBS V2C2 MODE PREPARE] {_v2c2_mode_name(previous_mode)} -> "
            f"{_v2c2_mode_name(transition_target)} | FRAME={frame_id} | "
            "COMMIT=NEXT_BEACON | JAM=OFF"
        )
    elif transition_phase == "COMMIT":
        print(
            f"[rBS V2C2 MODE COMMIT] {_v2c2_mode_name(previous_mode)} -> "
            f"{_v2c2_mode_name(mode)} | FRAME={frame_id} | JAM=OFF"
        )
    st["mode"] = mode
    st["frame_period_s"] = _v2c2_frame_period(mode)
    st["downlink_offset_s"] = _v2c2_downlink_offset(mode)
    st["jam_dryrun_frame"] = frame_id
    st["jam_dryrun_ul_pairs"] = set()
    st["jam_dryrun_phase"] = transition_phase

    grant = _v2c1_select_grant(st, frame_id, mode)
    jam_lease_mask = _v2c52_jam_lease_mask(st, mode, transition_phase)
    payload = _v2c1_build_beacon_payload(
        st, frame_id, grant, mode, transition_target, jam_lease_mask
    )
    t0 = time.monotonic()
    radio.send(
        payload,
        destination=V2C1_BROADCAST,
        node=ID_TRAM_RBS,
        identifier=V2C1_TYPE_BEACON,
        flags=V2C1_SCHEDULE_VERSION,
    )
    tx_ms = (time.monotonic() - t0) * 1000.0
    try:
        radio.listen()
    except Exception:
        pass

    a1, f1 = _v2c1_ack_tuple(st["pairs"][1])
    a2, f2 = _v2c1_ack_tuple(st["pairs"][2])
    print(
        f"[rBS V2C1 BEACON] FRAME={frame_id} | TX={tx_ms:.1f}ms | "
        f"P1_BASE={'NONE' if a1==0xFFFFFFFF else a1} P1_ACK=0x{f1&3:02X} | "
        f"P2_BASE={'NONE' if a2==0xFFFFFFFF else a2} P2_ACK=0x{f2&3:02X} | "
        f"MODE={_v2c2_mode_name(mode)} | "
        f"NEXT={_v2c2_mode_name(transition_target) if transition_target else 'NONE'} | "
        f"FEC_GRANT={grant} | JAM_LEASE=0x{jam_lease_mask:01X} | RF_JAM=OFF"
    )

    _v2c1_mark_acked(st["pairs"][1])
    _v2c1_mark_acked(st["pairs"][2])
    st["grant_pair"] = grant
    st["frame_start"] = actual
    st["downlink_due"] = actual + st["downlink_offset_s"]
    st["downlink_done"] = False
    st["beacon_count"] += 1
    st["frame_id"] = (frame_id + 1) & 0xFFFFFFFF
    st["next_beacon_at"] = scheduled_at + st["frame_period_s"]


def _v2c1_init_block(pair, base, expected_count):
    pair["block"] = _v2c1_new_block()
    pair["block"]["base"] = int(base)
    pair["block"]["expected_count"] = max(1, min(2, int(expected_count)))


def _v2c1_handle_voice(st, pair, raw):
    seq = _v2c1_u32be(raw[4:8])
    frames = int(raw[3])
    last = bool(raw[2] & FLAG_LAST)
    if not (1 <= frames <= 15):
        return
    base = (seq // 2) * 2
    slot = seq - base
    if slot not in (0, 1):
        return
    expected = (slot + 1) if last else 2
    b = pair["block"]
    if b["base"] is None:
        _v2c1_init_block(pair, base, expected)
        b = pair["block"]
    elif int(b["base"]) != base:
        if b["complete_acked"]:
            _v2c1_init_block(pair, base, expected)
            b = pair["block"]
        else:
            print(
                f"[rBS V2C1 WARN] {pair['name']} NEW_BASE={base} khi OLD_BASE={b['base']} chua ACK"
            )
            return
    if last:
        b["expected_count"] = expected
        b["final_seen"] = True
    bit = 1 << slot
    # Ke ca retry/duplicate cung chung minh SU van dang song va con giu PTT,
    # nen cap nhat voice lease truoc nhanh DUP.
    voice_now = time.monotonic()
    pair["last_voice_rx_at"] = voice_now
    pair["last_activity"] = voice_now
    st["last_activity"] = voice_now
    if b["received_mask"] & bit:
        pair["voice_dup"] += 1
        print(f"[rBS V2C1 UL] {pair['name']} DUP VOICE | SEQ={seq}")
        return
    b["received_mask"] |= bit
    b["packets"][slot] = bytes(raw)
    pair["voice_rx"] += 1
    pair["voice_active"] = True
    if V2C5_JAM_DRYRUN:
        st.setdefault("jam_dryrun_ul_pairs", set()).add(int(pair["pair"]))

    if pair["diag_first_voice_rx_at"] is None:
        pair["diag_first_voice_rx_at"] = pair["last_activity"]
        session_at = pair.get("diag_session_rx_at")
        delta_ms = (pair["last_activity"] - session_at) * 1000.0 if session_at else -1.0
        print(
            f"[rBS LAT] {pair['name']} FIRST_VOICE_RX | SEQ={seq} | "
            f"FROM_SESSION_RX={delta_ms:.1f} ms"
        )

    print(
        f"[rBS V2C1 UL] {pair['name']} VOICE | FRAME={int(st['frame_id'])-1} | "
        f"BASE={base} SLOT={slot} SEQ={seq} FC={frames} LAST={1 if last else 0} "
        f"RX=0x{b['received_mask']:02X}"
    )


def _v2c1_handle_fec(st, pair, raw):
    group = _v2c1_u32be(raw[4:8])
    data_count = ((raw[2] >> 5) & 0x07) + 1
    has_last = bool(raw[2] & FLAG_LAST)
    if int(st.get("grant_pair", 0)) != int(pair["pair"]):
        print(
            f"[rBS V2C1 DROP] FEC KHONG DUOC GRANT | {pair['name']} | GROUP={group} | GRANT={st.get('grant_pair',0)}"
        )
        return
    if group in pair["fec_relayed_groups"]:
        return
    pair["fec_pending"] = bytes(raw)
    pair["fec_rx"] += 1
    pair["last_activity"] = time.monotonic()
    st["last_activity"] = pair["last_activity"]
    print(
        f"[rBS V2C1 UL] {pair['name']} FEC GRANTED | GROUP={group} | DATA={data_count} | LAST={1 if has_last else 0}"
    )


def _v2c1_control_raw(dst, typ, flags, payload=b""):
    return bytes((dst & 0xFF, ID_TRAM_RBS & 0xFF, typ & 0xFF, flags & 0xFF)) + bytes(payload)


def _v2c1_relay_raw(pair, inner):
    inner = bytes(inner)
    return bytes((pair["du"], ID_TRAM_RBS, TYPE_RELAY, len(inner))) + inner


def _v2c1_end_raw(pair):
    return bytes((pair["du"], ID_TRAM_RBS, TYPE_RELAY_END, 0, 0))


def _v2c2_send_control_now(radio, raw, label):
    """FAST control path chi dung khi chua co lich RF dang active.

    FAST_SETUP_3A: dung CHINH duong CMD_TX_BURST da duoc bench on dinh
    cho VOICE/FEC/control thay vi CMD_TX_RAW. Voi control nhanh chi co 1
    packet/burst, sau do doi EVT_BURST_DONE de chac chan RF TX da ket thuc
    va E22 da quay ve RX truoc khi cho DU phan hoi.
    """
    raw = bytes(raw)
    if len(raw) < 4:
        return False
    try:
        if not hasattr(radio, "send_burst_raw"):
            raise BridgeError("STM32 bridge khong ho tro CMD_TX_BURST")
        if hasattr(radio, "burst_busy") and radio.burst_busy():
            return False

        t0 = time.monotonic()
        cmd_seq = radio.send_burst_raw([raw], guard_ms=0)
        done = radio.wait_burst_done(timeout=0.30)
        elapsed_ms = (time.monotonic() - t0) * 1000.0

        if not done:
            print(
                f"[rBS V2C2 FAST CTRL FAIL] {label} | "
                f"BURST_DONE timeout | CMD_SEQ={cmd_seq} | JAM=OFF"
            )
            return False

        print(
            f"[rBS V2C2 FAST CTRL] {label} | "
            f"BURST_DONE={elapsed_ms:.1f} ms | CMD_SEQ={cmd_seq} | JAM=OFF"
        )
        return True
    except (BridgeError, OSError) as exc:
        print(f"[rBS V2C2 FAST CTRL FAIL] {label} | {exc}")
        return False


def _v2c1_queue_control(st, key, raw, label, *, session_start=False):
    if key in st["control_keys"]:
        return False
    st["control_keys"].add(key)
    st["control_queue"].append({
        "key": key,
        "raw": bytes(raw),
        "label": str(label),
        "session_start": bool(session_start),
    })
    st["last_activity"] = time.monotonic()
    return True


def _v2c1_pop_control(st, *, allow_session_start=True, single_mode_pair=0):
    for i, item in enumerate(st["control_queue"]):
        if item["session_start"]:
            if not allow_session_start:
                continue
            # While SINGLE_P1/P2 is still active, defer SESSION_START for the
            # *other* pair until the PREPARE/COMMIT transaction has promoted
            # the scheduler to DUAL.  Sending it as the 3rd packet behind two
            # HQ VOICE packets left no safe SESSION_READY response window.
            key = item.get("key")
            start_pair = 0
            if isinstance(key, tuple) and len(key) >= 2 and key[0] == "start":
                try:
                    start_pair = int(key[1])
                except Exception:
                    start_pair = 0
            if single_mode_pair in (1, 2) and start_pair in (1, 2) and start_pair != single_mode_pair:
                continue
        st["control_queue"].pop(i)
        st["control_keys"].discard(item["key"])
        return item
    return None


def _v2c1_downlink(radio, st):
    frame_open = (int(st["frame_id"]) - 1) & 0xFFFFFFFF
    items = []
    meta = []

    # Deterministic order: P1 voices, P2 voices, granted FEC, then one control.
    for pi in (1, 2):
        pair = st["pairs"][pi]
        b = pair["block"]
        if b["base"] is None:
            continue
        for slot in range(int(b["expected_count"])):
            bit = 1 << slot
            if (b["received_mask"] & bit) and not (b["relayed_mask"] & bit):
                raw = b["packets"].get(slot)
                if raw is not None:
                    items.append(_v2c1_relay_raw(pair, raw))
                    meta.append(("voice", pi, slot, _v2c1_u32be(raw[4:8])))
                    if len(items) >= V2C1_MAX_BURST_ITEMS:
                        break
        if len(items) >= V2C1_MAX_BURST_ITEMS:
            break

    gp = int(st.get("grant_pair", 0))
    single_mode = int(st.get("mode", V2C2_MODE_DUAL)) in (V2C2_MODE_SINGLE1, V2C2_MODE_SINGLE2)
    # In a short SINGLE frame, control (especially the other pair's SESSION_START)
    # outranks parity.  FEC is best-effort and remains pending for a later frame.
    allow_fec_now = not (single_mode and bool(st["control_queue"]))
    if allow_fec_now and len(items) < V2C1_MAX_BURST_ITEMS and gp in (1, 2):
        pair = st["pairs"][gp]
        if pair["fec_pending"] is not None:
            raw = pair["fec_pending"]
            items.append(_v2c1_relay_raw(pair, raw))
            meta.append(("fec", gp, None, _v2c1_u32be(raw[4:8])))

    # V2C4 300 ms airtime guard:
    # - SINGLE DL bat dau 170 ms: 2 HQ relay + 1 control van vua frame.
    # - DUAL DL bat dau 185 ms: 2 LQ relay da gan day ngan sach; neu da co
    #   2 VOICE thi de control sang frame sau de beacon 300 ms khong bi tre.
    session_start_used = False
    selected_controls = []
    dual_mode = int(st.get("mode", V2C2_MODE_DUAL)) == V2C2_MODE_DUAL
    voice_items = sum(1 for m in meta if m[0] == "voice")
    allow_control_this_frame = not (dual_mode and voice_items >= 2)
    single_mode_pair = 1 if int(st.get("mode", V2C2_MODE_DUAL)) == V2C2_MODE_SINGLE1 else (2 if int(st.get("mode", V2C2_MODE_DUAL)) == V2C2_MODE_SINGLE2 else 0)
    while allow_control_this_frame and len(items) < V2C1_MAX_BURST_ITEMS:
        ctrl = _v2c1_pop_control(
            st,
            allow_session_start=not session_start_used,
            single_mode_pair=single_mode_pair,
        )
        if ctrl is None:
            break
        items.append(ctrl["raw"])
        selected_controls.append(ctrl)
        meta.append(("control", 0, None, ctrl["label"]))
        if ctrl["session_start"]:
            session_start_used = True
            break
        if any(m[0] in ("voice", "fec") for m in meta):
            break

    if not items:
        _v2c5_jam_dryrun_log_frame(st, frame_open, meta)
        _v2c53b1_dl_safe_mask(st, frame_open, meta, burst_accepted=True)
        print(f"[rBS V2C1 DL] FRAME={frame_open} | IDLE")
        return

    # V2C4.3 downlink pacing:
    # Log DU cho thay rBS nhan du SEQ nhung DU thuong mat packet DAU cua burst
    # 2 VOICE lien tiep.  SX1278 can mot khoang de task DU doc FIFO, clear RX_DONE
    # va goi LoRa.receive() truoc packet tiep theo.  Chi tang guard khi SINGLE co
    # dung 2 VOICE cua cung mot pair va KHONG chen control/FEC; DUAL van giu 3 ms
    # de bao toan ngan sach 300 ms.
    voice_meta = [m for m in meta if m[0] == "voice"]
    pure_two_voice_single = (
        single_mode
        and len(voice_meta) == 2
        and len(meta) == 2
        and int(voice_meta[0][1]) == int(voice_meta[1][1])
    )
    # In the first DUAL frames of a JOIN, only the already-active pair has a
    # VOICE packet.  The second item is SESSION_START or SESSION_READY for the
    # joining pair.  Give the other node the same proven 10 ms RX re-arm guard.
    # Budget stays safe: 185 + ~46 + 10 + ~19 = ~260 ms < 300 ms.
    one_voice_one_control_dual = (
        dual_mode
        and len(voice_meta) == 1
        and len(meta) == 2
        and any(m[0] == "control" for m in meta)
    )
    if pure_two_voice_single:
        burst_guard_ms = V2C4_SINGLE_DL_GUARD_MS
    elif one_voice_one_control_dual:
        burst_guard_ms = V2C4_JOIN_DL_GUARD_MS
    else:
        burst_guard_ms = V2C1_BURST_GUARD_MS

    # V2C5.3B-2: add permission ONLY for the actual clean STEADY
    # downlink burst. meta remains unchanged so B1 audits original content.
    v2c53b2_mask = _v2c53b2_calc_permission_mask(st, meta)
    v2c53b2_permission_added = False
    if v2c53b2_mask:
        if len(items) < V2C1_MAX_BURST_ITEMS:
            v2c53b2_mode = int(st.get("mode", V2C2_MODE_DUAL))
            items.insert(
                0,
                _v2c53b2_permission_raw(
                    frame_open,
                    v2c53b2_mode,
                    v2c53b2_mask,
                ),
            )
            v2c53b2_permission_added = True
        else:
            print(
                f"[rBS V2C5.3B2 DL_PERMISSION SKIP] FRAME={frame_open} | "
                f"MASK=0x{v2c53b2_mask:X} | REASON=NO_BURST_ROOM | RF_JAM=OFF"
            )

    t0 = time.monotonic()
    try:
        cmd_seq = radio.send_burst_raw(items, guard_ms=burst_guard_ms)
        block_ms = (time.monotonic() - t0) * 1000.0
        print(
            f"[rBS V2C1 DL BURST] FRAME={frame_open} | MODE={_v2c2_mode_name(st.get('mode', V2C2_MODE_DUAL))} | "
            f"ITEMS={len(items)} | GUARD={burst_guard_ms}ms | PACED={1 if (pure_two_voice_single or one_voice_one_control_dual) else 0} | "
            f"CMD_SEQ={cmd_seq} | PI_BLOCK={block_ms:.1f}ms"
        )
    except (BridgeError, OSError) as exc:
        print(f"[LỖI] V2C1 STM32 BURST FAIL: {exc}")
        for ctrl in reversed(selected_controls):
            st["control_queue"].insert(0, ctrl)
            st["control_keys"].add(ctrl["key"])
        return

    if v2c53b2_permission_added:
        print(
            f"[rBS V2C5.3B2 DL_PERMISSION_TX] FRAME={frame_open} | "
            f"MODE={_v2c2_mode_name(st.get('mode', V2C2_MODE_DUAL))} | "
            f"DL_SAFE_MASK=0x{v2c53b2_mask:X} | "
            f"BURST_ITEMS_WITH_PERMISSION={len(items)} | RF_JAM=OFF"
        )

    # V2C5: tai day STM32 da chap nhan burst. Chi IN KE HOACH jam gia lap;
    # khong co lenh radio moi nao duoc gui.
    _v2c5_jam_dryrun_log_frame(st, frame_open, meta)
    _v2c53b1_dl_safe_mask(st, frame_open, meta, burst_accepted=True)

    # Commit only after STM32 accepted the burst.
    # Track JOIN SESSION_START attempts only after the STM32 accepted the RF burst.
    for ctrl in selected_controls:
        if not ctrl.get("session_start"):
            continue
        key = ctrl.get("key")
        if not (isinstance(key, tuple) and len(key) >= 3 and key[0] == "start"):
            continue
        try:
            join_pi = int(key[1])
        except Exception:
            continue
        join_pair = st["pairs"].get(join_pi)
        if join_pair is None or join_pair.get("session_id") != key[2]:
            continue
        join_pair["join_start_attempts"] = int(join_pair.get("join_start_attempts", 0)) + 1
        join_pair["join_last_tx_at"] = time.monotonic()
        print(
            f"[rBS V2C4.4 JOIN TX] {join_pair['name']} SESSION_START | "
            f"ATTEMPT={join_pair['join_start_attempts']}/{V2C4_JOIN_MAX_START_TX} | "
            f"MODE={_v2c2_mode_name(st.get('mode', V2C2_MODE_DUAL))}"
        )

    for kind, pi, slot, val in meta:
        if kind == "voice":
            pair = st["pairs"][pi]
            pair["block"]["relayed_mask"] |= (1 << int(slot))
            if pair["diag_first_voice_dl_accept_at"] is None:
                pair["diag_first_voice_dl_accept_at"] = time.monotonic()
                rx_at = pair.get("diag_first_voice_rx_at")
                session_at = pair.get("diag_session_rx_at")
                rx_to_dl_ms = (pair["diag_first_voice_dl_accept_at"] - rx_at) * 1000.0 if rx_at else -1.0
                session_to_dl_ms = (pair["diag_first_voice_dl_accept_at"] - session_at) * 1000.0 if session_at else -1.0
                print(
                    f"[rBS LAT] {pair['name']} FIRST_VOICE_DL_ACCEPT | SEQ={val} | "
                    f"RX_TO_DL_ACCEPT={rx_to_dl_ms:.1f} ms | SESSION_RX_TO_DL_ACCEPT={session_to_dl_ms:.1f} ms"
                )
            print(f"[rBS V2C1 DL] {pair['name']} VOICE -> DU | SLOT={slot} SEQ={val}")
        elif kind == "fec":
            pair = st["pairs"][pi]
            pair["fec_relayed_groups"].add(int(val))
            pair["fec_pending"] = None
            print(f"[rBS V2C1 DL] {pair['name']} FEC -> DU | GROUP={val}")
        elif kind == "control":
            print(f"[rBS V2C1 DL CTRL] {val}")


def _v2c5_drop_stale_voice_session(st, pair, now, reason, age_s):
    """Thu hoi mot voice session stale ma khong doi packet/TDMA format.

    Muc dich chinh: neu AUDIO_END bi mat, session cu khong duoc phep giu rBS
    o DUAL hoac SINGLE sai pair vo han.  rBS queue mot END_AUDIO fail-safe cho
    DU roi xoa session stale.  RF friendly-jam van hard OFF.
    """
    sid = pair.get("session_id")
    if sid is None:
        return False

    # Xoa control cu cua dung session truoc, tranh READY/PLAY/HMI treo lai sau
    # khi slot da bi thu hoi. Sau do queue mot END_AUDIO fail-safe cho DU.
    kept = []
    for item in st.get("control_queue", []):
        key = item.get("key")
        if (
            isinstance(key, tuple)
            and len(key) >= 3
            and key[1] == pair["pair"]
            and key[2] == sid
        ):
            st["control_keys"].discard(key)
            continue
        kept.append(item)
    st["control_queue"] = kept

    end_raw = _v2c1_end_raw(pair)
    _v2c1_queue_control(
        st,
        ("stale_end", pair["pair"], sid),
        end_raw,
        f"{pair['name']} STALE END_AUDIO -> DU",
    )

    print(
        f"[rBS V2C5.1A VOICE LEASE EXPIRE] {pair['name']} | SESSION={sid:016X} | "
        f"NO_VOICE={age_s*1000.0:.0f}ms | REASON={reason} | "
        "RELEASE_SLOT=1 | RF_JAM=OFF"
    )

    pair["ended"] = True
    pair["du_ready"] = False
    pair["voice_active"] = False
    pair["join_pending"] = False
    pair["fast_setup_pending"] = False
    pair["session_id"] = None
    pair["session_inner"] = None
    pair["block"] = _v2c1_new_block()
    pair["fec_pending"] = None
    pair["fec_relayed_groups"] = set()
    pair["last_voice_rx_at"] = None
    pair["last_activity"] = now
    st["last_activity"] = now
    return True


def _v2c5_expire_stale_voice_sessions(st):
    """Nhanh chong DUAL -> SINGLE dung pair khi AUDIO_END bi mat.

    FAST path chi ap dung khi dang DUAL VA pair con lai vua co VOICE that,
    nen mot pair khong the bi thu hoi chi vi ca mang dang tam dung.
    HARD path don session voice bi treo lau hon de scheduler co the ve IDLE.
    """
    if not st.get("active"):
        return

    now = time.monotonic()
    mode = int(st.get("mode", V2C2_MODE_DUAL))

    # Snapshot truoc khi co the xoa pair de quyet dinh doi xung P1/P2.
    fresh = {}
    for pi, p in st["pairs"].items():
        lv = p.get("last_voice_rx_at")
        fresh[int(pi)] = bool(
            p.get("session_id") is not None
            and not p.get("ended")
            and p.get("du_ready")
            and p.get("voice_active")
            and lv is not None
            and (now - float(lv)) <= V2C5_OTHER_PAIR_FRESH_S
        )

    for pi, pair in st["pairs"].items():
        if (
            pair.get("session_id") is None
            or pair.get("ended")
            or not pair.get("du_ready")
            or not pair.get("voice_active")
        ):
            continue
        last_voice = pair.get("last_voice_rx_at")
        if last_voice is None:
            continue
        age = now - float(last_voice)
        other_pi = 2 if int(pi) == 1 else 1

        fast_release = (
            mode == V2C2_MODE_DUAL
            and fresh.get(other_pi, False)
            and age >= V2C5_DUAL_FAST_RELEASE_S
        )
        hard_release = age >= V2C5_VOICE_HARD_RELEASE_S

        if fast_release:
            _v2c5_drop_stale_voice_session(
                st, pair, now, "DUAL_OTHER_PAIR_STILL_ACTIVE", age
            )
        elif hard_release:
            _v2c5_drop_stale_voice_session(
                st, pair, now, "LOST_AUDIO_END_FAILSAFE", age
            )


def _v2c4_service_join_retries(st):
    """Retry a second-pair SESSION_START only in DUAL and fail back safely.

    The joining SU is intentionally silent until it receives SESSION_READY.
    Thus loss of one control packet must not leave the scheduler stuck in DUAL
    forever.  Retries are frame-scale, bounded, and never run in SINGLE where
    they would collide with the 2xHQ downlink budget.
    """
    if not st.get("active") or int(st.get("mode", V2C2_MODE_DUAL)) != V2C2_MODE_DUAL:
        return
    now = time.monotonic()
    for pair in st["pairs"].values():
        if not pair.get("join_pending") or pair.get("du_ready") or pair.get("ended"):
            continue
        sid = pair.get("session_id")
        raw = pair.get("session_inner")
        if sid is None or raw is None:
            continue
        attempts = int(pair.get("join_start_attempts", 0))
        last_tx = pair.get("join_last_tx_at")
        if last_tx is None:
            # Original JOIN control is already queued by SESSION_START handler.
            continue
        age = now - float(last_tx)
        if attempts < V2C4_JOIN_MAX_START_TX and age >= V2C4_JOIN_RETRY_S:
            relay = _v2c1_relay_raw(pair, raw)
            if _v2c1_queue_control(
                st, ("start", pair["pair"], sid), relay,
                f"{pair['name']} SESSION_START -> DU", session_start=True
            ):
                print(
                    f"[rBS V2C4.4 JOIN RETRY] {pair['name']} | "
                    f"NEXT_ATTEMPT={attempts+1}/{V2C4_JOIN_MAX_START_TX}"
                )
            continue
        if attempts >= V2C4_JOIN_MAX_START_TX and age >= V2C4_JOIN_RELEASE_S:
            print(
                f"[rBS V2C4.4 JOIN FAILSAFE] {pair['name']} | SESSION={sid:016X} | "
                "DU_NOT_READY -> RELEASE JOIN, RETURN OTHER PAIR TO SINGLE"
            )
            # Remove queued controls for this dead join session.
            kept=[]
            for item in st["control_queue"]:
                key=item.get("key")
                if isinstance(key, tuple) and len(key) >= 3 and key[1] == pair["pair"] and key[2] == sid:
                    st["control_keys"].discard(key)
                    continue
                kept.append(item)
            st["control_queue"] = kept
            pair["ended"] = True
            pair["du_ready"] = False
            pair["voice_active"] = False
            pair["join_pending"] = False
            pair["session_id"] = None
            pair["session_inner"] = None
            pair["block"] = _v2c1_new_block()
            pair["fec_pending"] = None
            st["last_activity"] = now


def _v2c3_expire_prevoice_sessions(st):
    now = time.monotonic()
    for pair in st["pairs"].values():
        if pair.get("session_id") is None or pair.get("ended") or pair.get("voice_active"):
            continue
        ready_at = pair.get("diag_du_ready_at")
        if ready_at is None:
            continue
        if (now - ready_at) < V2C3_PREVOICE_LEASE_S:
            continue
        sid = pair.get("session_id")
        print(
            f"[rBS V2C4 LEASE EXPIRE] {pair['name']} | SESSION={sid:016X} | "
            f"NO_FIRST_VOICE>{V2C3_PREVOICE_LEASE_S:.1f}s -> RELEASE SLOT"
        )
        pair["ended"] = True
        pair["du_ready"] = False
        pair["voice_active"] = False
        pair["join_pending"] = False
        pair["session_id"] = None
        pair["session_inner"] = None
        pair["block"] = _v2c1_new_block()
        pair["fec_pending"] = None
        st["last_activity"] = now


def v2c1_service_timing(radio, st):
    if not st["active"]:
        return
    _v2c4_service_join_retries(st)
    _v2c3_expire_prevoice_sessions(st)
    _v2c5_expire_stale_voice_sessions(st)
    now = time.monotonic()
    if st["downlink_due"] is not None and not st["downlink_done"] and now >= st["downlink_due"]:
        late = max(0.0, now - st["downlink_due"])
        st["late_max_s"] = max(st["late_max_s"], late)
        if late > 0.005:
            print(f"[rBS V2C1 WARN] DOWNLINK LATE={late*1000:.1f} ms")
        _v2c1_downlink(radio, st)
        st["downlink_done"] = True

    if st["next_beacon_at"] is not None and time.monotonic() >= st["next_beacon_at"]:
        scheduled = st["next_beacon_at"]
        _v2c1_send_beacon(radio, st, scheduled)

    # Keep schedule alive a few seconds for PLAY/HMI control, then return to telemetry-friendly idle.
    any_live = any(_v2c1_pair_has_work(p) for p in st["pairs"].values())
    if (
        not any_live
        and not st["control_queue"]
        and (time.monotonic() - st["last_activity"]) >= V2C1_IDLE_HOLD_S
    ):
        st["active"] = False
        print(
            f"[rBS V2C1] DEACTIVE | BEACON={st['beacon_count']} | LATE_MAX={st['late_max_s']*1000:.1f}ms"
        )
        if V2C5_JAM_DRYRUN:
            js = st.get("jam_dryrun_stats", {})
            print(
                f"[rBS JAM DRYRUN SUMMARY] FRAMES={int(js.get('frames',0))} | "
                f"UL_CAND={int(js.get('ul_candidates',0))} | DL_CAND={int(js.get('dl_candidates',0))} | "
                f"BLOCK_TRANSITION={int(js.get('blocked_transition',0))} | "
                f"BLOCK_CONTROL={int(js.get('blocked_control',0))} | RF_JAM=OFF"
            )


def _v2c1_pair_by_su(src, dst=None):
    pi = V2C1_SU_TO_PAIR.get(int(src))
    if pi is None:
        return None
    p = V2C1_PAIRS[pi]
    if dst is not None and int(dst) != int(p["du"]):
        return None
    return pi


def _v2c1_pair_by_du(src):
    return V2C1_DU_TO_PAIR.get(int(src))


def _v2c1_handle_session_start(st, pair, raw, radio):
    sid = _v2c1_u64be(raw[4:12])
    if sid == 0:
        return
    same = pair["session_id"] == sid
    if not same:
        other_work = any(
            _v2c1_pair_has_work(other)
            for pi, other in st["pairs"].items()
            if int(pi) != int(pair["pair"])
        )
        join_pending = bool(st.get("active") and other_work)
        pair.update({
            "session_id": sid,
            "session_inner": bytes(raw),
            "du_ready": False,
            "voice_active": False,
            "ended": False,
            "block": _v2c1_new_block(),
            "fec_pending": None,
            "fec_relayed_groups": set(),
            "last_activity": time.monotonic(),
            "last_voice_rx_at": None,
            # Session-local counters MUST reset here.  Older builds left these
            # cumulative across sessions, corrupting logs/AI training data.
            "voice_rx": 0,
            "voice_dup": 0,
            "voice_retry": 0,
            "fec_rx": 0,
            "diag_session_rx_at": time.monotonic(),
            "diag_du_ready_at": None,
            "diag_first_voice_rx_at": None,
            "diag_first_voice_dl_accept_at": None,
            "fast_setup_pending": False,
            "join_pending": join_pending,
            "join_start_attempts": 0,
            "join_last_tx_at": None,
            "join_requested_at": time.monotonic() if join_pending else None,
        })
        print(f"[PHIÊN THOẠI] BẮT ĐẦU | {pair['name']} | MÃ={sid:016X} | SF=7")
        print(
            f"[rBS LAT] {pair['name']} SESSION_RX | T={pair['diag_session_rx_at']:.6f} | "
            f"SESSION={sid:016X}"
        )
        if join_pending:
            print(
                f"[rBS V2C4.4 JOIN REQUEST] {pair['name']} | SESSION={sid:016X} | "
                "PROMOTE_TO_DUAL_BEFORE_HANDSHAKE=1"
            )
    st["last_activity"] = time.monotonic()

    if pair["du_ready"]:
        ready = _v2c1_control_raw(pair["su"], TYPE_SESSION_READY, 0, sid.to_bytes(8,"big"))
        if (not st["active"]) and _v2c2_send_control_now(
            radio, ready, f"{pair['name']} SESSION_READY -> SU"
        ):
            pair["fast_setup_pending"] = False
            _v2c1_activate(st)
        else:
            _v2c1_activate(st)
            _v2c1_queue_control(st, ("ready", pair["pair"], sid), ready, f"{pair['name']} SESSION_READY -> SU")
        return

    relay = _v2c1_relay_raw(pair, raw)

    # FAST_SETUP chi duoc phep khi scheduler dang IDLE. Khi da co pair khac
    # hoat dong, SESSION_START phai di JOIN/control slot de tranh va cham RF.
    if not st["active"]:
        if _v2c2_send_control_now(
            radio, relay, f"{pair['name']} SESSION_START -> DU"
        ):
            pair["fast_setup_pending"] = True
            return

    _v2c1_activate(st)
    _v2c1_queue_control(
        st, ("start", pair["pair"], sid), relay,
        f"{pair['name']} SESSION_START -> DU", session_start=True
    )


def _v2c1_handle_du_control(st, pair, raw, radio):
    typ = raw[2]
    sid = _v2c1_u64be(raw[4:12]) if len(raw) >= 12 else 0
    if sid == 0 or pair["session_id"] != sid:
        return
    st["last_activity"] = time.monotonic()
    pair["last_activity"] = st["last_activity"]
    if typ == TYPE_SESSION_READY:
        was_join = bool(pair.get("join_pending"))
        pair["du_ready"] = True
        pair["join_pending"] = False
        # voice_active chi duoc set khi VOICE dau tien that su den rBS.
        pair["voice_active"] = False
        if was_join:
            print(
                f"[rBS V2C4.4 JOIN READY] {pair['name']} | SESSION={sid:016X} | "
                "DU_READY=1 | KEEP_DUAL_UNTIL_FIRST_VOICE"
            )
        if pair.get("diag_du_ready_at") is None:
            pair["diag_du_ready_at"] = time.monotonic()
            session_at = pair.get("diag_session_rx_at")
            delta_ms = (pair["diag_du_ready_at"] - session_at) * 1000.0 if session_at else -1.0
            print(
                f"[rBS LAT] {pair['name']} DU_READY_RX | FROM_SESSION_RX={delta_ms:.1f} ms"
            )
        ready = _v2c1_control_raw(pair["su"], TYPE_SESSION_READY, 0, sid.to_bytes(8,"big"))
        if pair.get("fast_setup_pending") and not st["active"]:
            if _v2c2_send_control_now(
                radio, ready, f"{pair['name']} SESSION_READY -> SU"
            ):
                pair["fast_setup_pending"] = False
                print(f"[rBS V2C2 FAST SETUP] {pair['name']} HANDSHAKE COMPLETE | SESSION={sid:016X}")
                _v2c1_activate(st)
            else:
                pair["fast_setup_pending"] = False
                _v2c1_activate(st)
                _v2c1_queue_control(st, ("ready", pair["pair"], sid), ready, f"{pair['name']} SESSION_READY -> SU")
        else:
            _v2c1_queue_control(st, ("ready", pair["pair"], sid), ready, f"{pair['name']} SESSION_READY -> SU")
        print(f"[rBS V2C1 CTRL] {pair['name']} DU READY | SESSION={sid:016X}")
    elif typ == TYPE_PLAY_STARTED:
        fwd = _v2c1_control_raw(pair["su"], TYPE_PLAY_STARTED, 0, sid.to_bytes(8,"big"))
        _v2c1_queue_control(st, ("play", pair["pair"], sid), fwd, f"{pair['name']} PLAY_STARTED -> SU")
    elif typ == TYPE_USER_RESPONSE and raw[3] in (USER_RESPONSE_ACK, USER_RESPONSE_NACK):
        code = raw[3]
        fwd = _v2c1_control_raw(pair["su"], TYPE_USER_RESPONSE, code, sid.to_bytes(8,"big"))
        _v2c1_queue_control(st, ("user", pair["pair"], sid, code), fwd, f"{pair['name']} USER_RESPONSE -> SU")
        print(f"[PHẢN HỒI] {pair['name']} DU->rBS | CODE={code} | MÃ={sid:016X}")


def _v2c1_handle_su_confirm(st, pair, raw):
    if len(raw) != 12 or raw[2] != TYPE_USER_CONFIRM:
        return
    sid = _v2c1_u64be(raw[4:12])
    if sid == 0 or pair["session_id"] != sid:
        return
    code = raw[3]
    fwd = _v2c1_control_raw(pair["du"], TYPE_USER_CONFIRM, code, sid.to_bytes(8,"big"))
    _v2c1_queue_control(st, ("confirm", pair["pair"], sid, code), fwd, f"{pair['name']} USER_CONFIRM -> DU")


def _v2c1_handle_audio_end(st, pair):
    sid = pair["session_id"]
    if sid is None:
        return
    if not pair["ended"]:
        pair["ended"] = True
        pair["voice_active"] = False
        pair["join_pending"] = False
        end_raw = _v2c1_end_raw(pair)
        _v2c1_queue_control(st, ("end", pair["pair"], sid), end_raw, f"{pair['name']} END_AUDIO -> DU")
        print(
            f"[PHIÊN THOẠI] HOÀN TẤT | {pair['name']} | MÃ={sid:016X} | "
            f"VOICE_RX={pair['voice_rx']} | DUP={pair['voice_dup']}"
        )
        session_at = pair.get("diag_session_rx_at")
        first_rx = pair.get("diag_first_voice_rx_at")
        first_dl = pair.get("diag_first_voice_dl_accept_at")
        s_to_rx = (first_rx - session_at) * 1000.0 if session_at and first_rx else -1.0
        rx_to_dl = (first_dl - first_rx) * 1000.0 if first_rx and first_dl else -1.0
        print(
            f"[rBS LAT SUMMARY] {pair['name']} SESSION_TO_FIRST_VOICE_RX={s_to_rx:.1f} ms | "
            f"FIRST_RX_TO_DL_ACCEPT={rx_to_dl:.1f} ms | VOICE_RX={pair['voice_rx']} | DUP={pair['voice_dup']}"
        )
    st["last_activity"] = time.monotonic()



V2C1_TELEMETRY_CSV = Path(__file__).resolve().parent / "rbs_v2c1_telemetry.csv"
V2C1_CHANNEL_CSV = Path(__file__).resolve().parent / "rbs_v2c1_channel.csv"


def _v2c1_append_csv(path, row):
    path.parent.mkdir(parents=True, exist_ok=True)
    new = (not path.exists()) or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if new:
            w.writeheader()
        w.writerow(row)


def _v2c1_handle_telemetry(raw, radio):
    p = bytes(raw)
    if len(p) == 44 and p[0] == ID_TRAM_RBS and p[2] in (0x17, 0x18):
        src = p[1]
        label = {0x01:"SU1",0x02:"DU1",0x04:"SU2",0x05:"DU2"}.get(src)
        if label is None:
            return False
        flags = p[3]
        gps_valid = bool(flags & 0x01)
        lat = int.from_bytes(p[12:16],"big",signed=True)/1e7 if gps_valid else 0.0
        lon = int.from_bytes(p[16:20],"big",signed=True)/1e7 if gps_valid else 0.0
        row = {
            "UTC_rBS": datetime.now(timezone.utc).isoformat(),
            "Node": label,
            "Type": "GPS_SESSION" if p[2]==0x17 else "GPS_PERIODIC",
            "Ref": int.from_bytes(p[4:12],"big"),
            "GPS_valid": 1 if gps_valid else 0,
            "Lat": lat,
            "Lon": lon,
            "Alt_m": int.from_bytes(p[20:24],"big",signed=True)/100.0,
            "Speed_m_s": int.from_bytes(p[24:28],"big")/100.0,
            "Sat": p[28],
            "P_TX_dBm": int.from_bytes(p[29:30],"big",signed=True),
            "HDOP": int.from_bytes(p[30:32],"big")/100.0,
            "RSSI_rBS_dBm": radio.last_rssi,
            "SNR_rBS_dB": radio.last_snr,
        }
        _v2c1_append_csv(V2C1_TELEMETRY_CSV,row)
        return True

    if len(p) == 20 and p[0] == ID_TRAM_RBS and p[2] == TYPE_BAO_CAO_KENH_SU_DU:
        src = p[1]
        pi = V2C1_DU_TO_PAIR.get(src)
        if pi is None:
            return False
        rssi = int.from_bytes(p[12:14],"big",signed=True)/10.0
        snr = int.from_bytes(p[14:16],"big",signed=True)/10.0
        pt = int.from_bytes(p[16:17],"big",signed=True)
        _, _, h2, habs, hdb = tinh_kenh_tu_rssi_pt(rssi, pt)
        row = {
            "UTC_rBS": datetime.now(timezone.utc).isoformat(),
            "Pair": pi,
            "DU": f"DU{pi}",
            "SU_beacon_seq": int.from_bytes(p[4:12],"big"),
            "RSSI_SU_DU_dBm": rssi,
            "SNR_SU_DU_dB": snr,
            "P_TX_SU_dBm": pt,
            "SF_SU": p[17],
            "H2": h2,
            "H_abs": habs,
            "H_dB": hdb,
        }
        _v2c1_append_csv(V2C1_CHANNEL_CSV,row)
        return True
    return False


def _v2c1_handle_raw(st, radio, raw):
    p = bytes(raw)
    if len(p) < 4:
        return
    dst, src, typ_raw, flags = p[0], p[1], p[2], p[3]
    typ = typ_raw & TYPE_MASK

    # Ignore our own broadcast beacon/downlink echoes if RF environment loops them back.
    if src == ID_TRAM_RBS:
        return

    # SU voice/session/FEC/audio_end path.
    pi = _v2c1_pair_by_su(src, dst)
    if pi is not None:
        pair = st["pairs"][pi]
        if typ == TYPE_SESSION_START and len(p) == SIZE_SESSION and flags == 8:
            _v2c1_handle_session_start(st, pair, p, radio)
            return
        if typ == TYPE_VOICE and len(p) == SIZE_VOICE_PACKET:
            if pair["du_ready"] and not pair["ended"]:
                _v2c1_handle_voice(st, pair, p)
            return
        if typ == TYPE_FEC and len(p) == SIZE_FEC_PACKET:
            if pair["du_ready"] and not pair["ended"]:
                _v2c1_handle_fec(st, pair, p)
            return
        if typ == TYPE_AUDIO_END_SU and len(p) == SIZE_CONTROL_SU:
            _v2c1_handle_audio_end(st, pair)
            return

    # SU -> rBS HMI confirm (dst is rBS, not DU).
    pi = V2C1_SU_TO_PAIR.get(src)
    if pi is not None and dst == ID_TRAM_RBS and typ_raw == TYPE_USER_CONFIRM:
        _v2c1_handle_su_confirm(st, st["pairs"][pi], p)
        return

    # DU -> rBS control.
    pi = _v2c1_pair_by_du(src)
    if pi is not None and dst == ID_TRAM_RBS and len(p) == 12:
        if typ_raw in (TYPE_SESSION_READY, TYPE_PLAY_STARTED, TYPE_USER_RESPONSE):
            _v2c1_handle_du_control(st, st["pairs"][pi], p, radio)
            return

    # Telemetry remains best-effort; SU/DU suppress it while beacons are active.
    if dst == ID_TRAM_RBS and typ_raw in (0x17, 0x18, TYPE_BAO_CAO_KENH_SU_DU):
        _v2c1_handle_telemetry(p, radio)
        return


def main_v2c1():
    print(
        "[HỆ THỐNG] rBS V2C5 JAM DRY-RUN | 433MHz | BW=500kHz | CR=4/5 | SF7 | "
        "SU1=01/DU1=02 | SU2=04/DU2=05 | JAM=CHI_GIA_LAP/RF_OFF"
    )
    try:
        radio = khoi_tao_lora_rbs()
    except (RuntimeError, BridgeError, OSError) as error:
        print("[LỖI] Không kết nối được STM32/E22:", error)
        return

    print(
        f"[HỆ THỐNG] Pi->STM32 UART={radio.port} | baud={radio.baudrate} | STM32_BURST=ON"
    )
    print(
        f"[rBS BUILD] TAG={V2C1_BUILD_TAG} | VOICE={SIZE_VOICE_PACKET}B | "
        f"FEC={SIZE_FEC_PACKET}B | PROTECTED={VOICE_LENGTH}B | FEC_GROUP={V2C1_FEC_DATA_PER_GROUP}+1 | "
        f"UART={radio.baudrate}"
    )
    print(
        "[rBS V2C5.2] SCHEDULER READY | SCHEDULE_VER=7 | FAST_SETUP=ON | START_ALIGN=240ms | HOLDOVER=1_FRAME | PREPARE_COMMIT=ON | JOIN_PROMOTION=ON | "
        "SINGLE=300ms/DL170ms/HQ8+7 | DUAL=300ms/DL185ms/LQ15-per-pair | JOIN_RETRY=360msx3 | "
        f"DUAL_FAST_RELEASE={V2C5_DUAL_FAST_RELEASE_S*1000:.0f}ms | HARD_RELEASE={V2C5_VOICE_HARD_RELEASE_S*1000:.0f}ms | JAM_LEASE_MASK=ON | JAM_DRYRUN_LOG=FORCED_ON | RF_JAM=OFF"
    )
    print("[rBS V2C5.2] LEASE DISTRIBUTION = ON | BITS3..2=PAIR_MASK | PREPARE_COMMIT_MASK=0 | RF_JAM=OFF")
    print("[rBS V2C5.3B1] DL_PERMISSION_SIM=ON | ACTUAL_BURST_META | CONTROL_BLOCK=ON | TRANSITION_BLOCK=ON | RF_JAM=OFF")

    print("[rBS V2C5.3B2] DL_PERMISSION_DISTRIBUTION_SIM=ON | TYPE=0x1D | FIRST_IN_SAFE_BURST=ON | RF_JAM=OFF")
    systemd_wdt = SystemdServiceWatchdog()
    systemd_wdt.start()
    systemd_wdt.progress()
    systemd_wdt.ready()

    st = v2c1_new_state()
    last_rx = time.monotonic()

    while True:
        systemd_wdt.progress()
        try:
            v2c1_service_timing(radio, st)
        except (BridgeError, OSError) as exc:
            print(f"[LỖI] V2C1 timing: {exc}")
            try:
                radio.ensure_ready()
            except Exception:
                pass

        try:
            raw = radio.receive(timeout=0.002, with_header=True)
        except (BridgeError, OSError) as exc:
            print(f"[LỖI] V2C1 RX: {exc}")
            time.sleep(0.01)
            continue

        if raw is not None:
            last_rx = time.monotonic()
            try:
                _v2c1_handle_raw(st, radio, raw)
            except Exception as exc:  # noqa: BLE001
                print(f"[LỖI] V2C1 packet handler: {exc}")
            continue

        # Idle RX self-heal retained from baseline, but never reset just because air is quiet.
        if (time.monotonic() - last_rx) >= RX_WATCHDOG_IM_LANG_S:
            try:
                radio.listen()
                print(
                    f"[WATCHDOG] TỰ PHỤC HỒI RX | IM_LẶNG={time.monotonic()-last_rx:.1f}s | ACTION=RX_REARM"
                )
                last_rx = time.monotonic()
            except (BridgeError, OSError) as exc:
                print(f"[LỖI] V2C1 RX_REARM: {exc}")
        time.sleep(RX_IDLE_YIELD_S)


if __name__ == "__main__":
    if V2C1_ENABLED:
        main_v2c1()
    else:
        main()
