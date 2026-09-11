#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DIGITAL TWIN CAMPUS LIVE - Raspberry Pi

Chức năng:
- Dùng campus_live_background.png làm ảnh nền cố định.
- Đọc rbs_lien_ket_dinh_ky_v11.csv trực tiếp trên Raspberry Pi.
- rBS cố định.
- SU/DU tự di chuyển trên bản đồ khi có GPS mới.
- Chỉ DI CHUYỂN marker khi GPS_tin_cay_2D = 1.
- Nếu GPS yếu: GIỮ vị trí tốt gần nhất, nhưng báo "GPS YẾU".
- Không cần Blender chạy trên Raspberry Pi.
- Không cần Internet công cộng; Pi và Windows chỉ cần cùng mạng nội bộ.
- 3 kênh EVE lấy từ Sionna RT thật trên Windows qua HTTP nội bộ.

File nên đặt cùng thư mục:
    /home/pi5/rBS_AIOT/
        position_monitor_campus.py
        campus_live_background.png
        campus_live_map_config.json
        campus_geo_config.py
        rbs_lien_ket_dinh_ky_v11.csv
"""

import sys
import csv
import json
import math
import os
import time
import threading
import queue
import urllib.request
import urllib.error
from collections import deque
from pathlib import Path
from datetime import datetime, timezone

from PySide6.QtWidgets import QApplication, QWidget
from PySide6.QtGui import (
    QPainter,
    QPen,
    QBrush,
    QFont,
    QColor,
    QPixmap,
)
from PySide6.QtCore import Qt, QRectF, QPointF, QTimer

from channel_math import measured_channel_from_rssi


BASE_DIR = Path(__file__).resolve().parent

CSV_PATH = Path(
    os.environ.get(
        "RBS_GPS_CSV",
        str(BASE_DIR / "rbs_lien_ket_dinh_ky_v11.csv"),
    )
)

BACKGROUND_PATH = Path(
    os.environ.get(
        "RBS_CAMPUS_BG",
        str(BASE_DIR / "campus_live_background.png"),
    )
)

MAP_CONFIG_PATH = Path(
    os.environ.get(
        "RBS_CAMPUS_MAP_CONFIG",
        str(BASE_DIR / "campus_live_map_config.json"),
    )
)

SU_DU_CHANNEL_CSV = Path(
    os.environ.get(
        "RBS_SU_DU_CHANNEL_CSV",
        str(BASE_DIR / "rbs_kenh_su_du.csv"),
    )
)

# ============================================================
# EVE ẢO = SIONNA RT CHẠY TRÊN WINDOWS
# ============================================================
# Windows IP hiện tại suy ra từ phiên SSH tới Pi là 172.19.32.199.
# Có thể đổi mà không sửa code:
#   export RBS_SIONNA_EVE_URL="http://IP_WINDOWS:8765/eve-channel"
SIONNA_EVE_URL = os.environ.get(
    "RBS_SIONNA_EVE_URL",
    "http://172.19.32.199:8765/eve-channel",
)
SIONNA_HTTP_TIMEOUT_SEC = float(
    os.environ.get("RBS_SIONNA_TIMEOUT_SEC", "300")
)

# Không ray tracing mỗi 300 ms.
# Chỉ tính lại khi EVE được click hoặc SU/DU dịch chuyển đủ xa.
SIONNA_MOVE_THRESHOLD_M = float(
    os.environ.get("RBS_SIONNA_MOVE_THRESHOLD_M", "5.0")
)
SIONNA_MIN_INTERVAL_SEC = float(
    os.environ.get("RBS_SIONNA_MIN_INTERVAL_SEC", "15.0")
)

SU_HEIGHT_M = 2.0
DU_HEIGHT_M = 2.0
EVE_HEIGHT_M = 1.5

# Tần suất giao diện kiểm tra CSV có dòng mới.
CSV_POLL_INTERVAL_MS = 300

# Nếu quá thời gian này chưa nhận báo cáo mới thì báo mất/cũ dữ liệu.
STALE_AFTER_SEC = 15.0

# ============================================================
# LỌC GPS CHỐNG "NHẢY" MARKER
# ============================================================
# Lấy trung vị của N mẫu gần nhất để loại bỏ điểm GPS lệch đột ngột.
GPS_FILTER_WINDOW = int(
    os.environ.get("RBS_GPS_FILTER_WINDOW", "5")
)

# Nếu thiết bị gần như đứng yên, chỉ dịch marker khi vị trí lọc
# lệch quá ngưỡng này. Nhờ vậy marker không rung vài mét quanh chỗ đứng.
GPS_STATIONARY_DEADBAND_M = float(
    os.environ.get("RBS_GPS_STATIONARY_DEADBAND_M", "3.0")
)

# Khi đang di chuyển thật, cho marker phản ứng nhạy hơn.
GPS_MOVING_DEADBAND_M = float(
    os.environ.get("RBS_GPS_MOVING_DEADBAND_M", "0.8")
)

GPS_STATIONARY_SPEED_M_S = float(
    os.environ.get("RBS_GPS_STATIONARY_SPEED_M_S", "0.45")
)

# Hệ số làm mượt khi thiết bị đang đi.
GPS_MOVING_ALPHA = float(
    os.environ.get("RBS_GPS_MOVING_ALPHA", "0.65")
)

# ============================================================
# BỘ BẢO VỆ VỊ TRÍ CAMPUS
# ============================================================
# Nếu một vị trí mới nhảy quá xa so với vị trí đang hiển thị,
# không nhận ngay. Phải có nhiều mẫu liên tiếp xác nhận.
GPS_BIG_JUMP_M = float(
    os.environ.get("RBS_GPS_BIG_JUMP_M", "20.0")
)

GPS_BIG_JUMP_CONFIRM_SAMPLES = int(
    os.environ.get("RBS_GPS_BIG_JUMP_CONFIRM_SAMPLES", "3")
)

# Các mẫu xác nhận cùng một vị trí mới phải nằm gần nhau.
GPS_BIG_JUMP_CLUSTER_M = float(
    os.environ.get("RBS_GPS_BIG_JUMP_CLUSTER_M", "8.0")
)

# Vị trí tốt quá cũ thì không vẽ marker nữa.
GPS_GOOD_FIX_MAX_AGE_SEC = float(
    os.environ.get("RBS_GPS_GOOD_FIX_MAX_AGE_SEC", "30.0")
)

# Cho phép nới nhẹ ra ngoài biên mesh campus.
CAMPUS_BOUNDS_MARGIN_M = float(
    os.environ.get("RBS_CAMPUS_BOUNDS_MARGIN_M", "3.0")
)

# ============================================================
# CẤU HÌNH GPS -> SCENE
# ============================================================
try:
    from campus_geo_config import (
        RBS_LAT,
        RBS_LON,
        RBS_SCENE_X_M,
        RBS_SCENE_Y_M,
        MAP_ROTATION_DEG,
        FLIP_NORTH,
    )
    try:
        from campus_geo_config import RBS_SCENE_Z_M
    except ImportError:
        RBS_SCENE_Z_M = 18.419
except ImportError:
    # Fallback đúng baseline hiện tại của dự án.
    RBS_LAT = 20.9807467
    RBS_LON = 105.7962650
    RBS_SCENE_X_M = 49.79
    RBS_SCENE_Y_M = -10.83
    RBS_SCENE_Z_M = 18.419
    MAP_ROTATION_DEG = 0.0
    FLIP_NORTH = False


EARTH_R = 6_371_000.0


def gps_to_local_en(lat, lon, lat0, lon0):
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    east = EARTH_R * (lon_rad - lon0_rad) * math.cos(lat0_rad)
    north = EARTH_R * (lat_rad - lat0_rad)
    return east, north


def en_to_scene_xy(east_m, north_m):
    if FLIP_NORTH:
        north_m = -north_m

    theta = math.radians(MAP_ROTATION_DEG)

    scene_dx = (
        math.cos(theta) * east_m
        - math.sin(theta) * north_m
    )
    scene_dy = (
        math.sin(theta) * east_m
        + math.cos(theta) * north_m
    )

    return (
        RBS_SCENE_X_M + scene_dx,
        RBS_SCENE_Y_M + scene_dy,
    )


def gps_to_scene_xy(lat, lon):
    east, north = gps_to_local_en(
        lat, lon, RBS_LAT, RBS_LON
    )
    return en_to_scene_xy(east, north)


def safe_float(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value, default=None):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def valid_coordinate(lat, lon):
    return (
        lat is not None
        and lon is not None
        and -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
        and not (lat == 0.0 and lon == 0.0)
    )


def parse_time_iso(text):
    try:
        return datetime.fromisoformat(
            (text or "").strip().replace("Z", "+00:00")
        )
    except Exception:
        return None


class NodeState:
    def __init__(self, name):
        self.name = name

        # Vị trí tốt gần nhất
        self.scene_x = None
        self.scene_y = None
        self.lat = None
        self.lon = None

        # Thông tin mới nhất, kể cả bản ghi GPS yếu
        self.gps_hop_le = False
        self.gps_tin_cay_2d = False
        self.satellites = None
        self.hdop = None
        self.rssi = None
        self.snr = None
        self.distance_m = None
        self.tx_power_dbm = None
        self.sf = None
        self.seq = None
        self.last_report_time = None
        self.last_good_time = None

        # Bộ lọc GPS hiển thị.
        self.speed_m_s = None
        self.position_samples = deque(maxlen=max(3, GPS_FILTER_WINDOW))

        # Xác nhận nhảy lớn: không cho marker bay hàng chục mét
        # chỉ vì 1-2 mẫu GPS sai.
        self.pending_jump_center = None
        self.pending_jump_count = 0

    def update_filtered_position(self, raw_x, raw_y, lat, lon):
        """
        Lọc GPS cho mục đích hiển thị:
        1) trung vị N mẫu gần nhất;
        2) deadband khi đứng yên;
        3) làm mượt khi đang di chuyển;
        4) nhảy > GPS_BIG_JUMP_M phải có N mẫu liên tiếp xác nhận.

        Trả về True nếu marker thực sự thay đổi vị trí.
        """
        self.position_samples.append((float(raw_x), float(raw_y)))

        xs = sorted(p[0] for p in self.position_samples)
        ys = sorted(p[1] for p in self.position_samples)
        mid = len(xs) // 2

        if len(xs) % 2:
            fx = xs[mid]
            fy = ys[mid]
        else:
            fx = 0.5 * (xs[mid - 1] + xs[mid])
            fy = 0.5 * (ys[mid - 1] + ys[mid])

        # Lần đầu: nhận vị trí ngay.
        if self.scene_x is None or self.scene_y is None:
            self.scene_x = fx
            self.scene_y = fy
            self.lat = lat
            self.lon = lon
            self.pending_jump_center = None
            self.pending_jump_count = 0
            return True

        dx = fx - self.scene_x
        dy = fy - self.scene_y
        displacement = math.hypot(dx, dy)

        # ----------------------------------------------------
        # BẢO VỆ NHẢY LỚN
        # ----------------------------------------------------
        if displacement >= GPS_BIG_JUMP_M:
            candidate = (fx, fy)

            if self.pending_jump_center is None:
                self.pending_jump_center = candidate
                self.pending_jump_count = 1
                return False

            pdx = candidate[0] - self.pending_jump_center[0]
            pdy = candidate[1] - self.pending_jump_center[1]

            if math.hypot(pdx, pdy) <= GPS_BIG_JUMP_CLUSTER_M:
                # Các mẫu mới cùng xác nhận một khu vực.
                n = self.pending_jump_count
                self.pending_jump_center = (
                    (self.pending_jump_center[0] * n + candidate[0]) / (n + 1),
                    (self.pending_jump_center[1] * n + candidate[1]) / (n + 1),
                )
                self.pending_jump_count += 1
            else:
                # Mẫu mới không cùng cụm -> bắt đầu xác nhận lại.
                self.pending_jump_center = candidate
                self.pending_jump_count = 1

            if self.pending_jump_count < GPS_BIG_JUMP_CONFIRM_SAMPLES:
                return False

            # Đủ số mẫu liên tiếp -> đây có khả năng là di chuyển thật.
            fx, fy = self.pending_jump_center
            self.pending_jump_center = None
            self.pending_jump_count = 0

            self.scene_x = fx
            self.scene_y = fy
            self.lat = lat
            self.lon = lon
            return True

        # Không còn nhảy lớn -> xóa ứng viên đang chờ.
        self.pending_jump_center = None
        self.pending_jump_count = 0

        speed = self.speed_m_s if self.speed_m_s is not None else 0.0
        stationary = speed < GPS_STATIONARY_SPEED_M_S

        threshold = (
            GPS_STATIONARY_DEADBAND_M
            if stationary
            else GPS_MOVING_DEADBAND_M
        )

        if displacement < threshold:
            return False

        if stationary:
            new_x = fx
            new_y = fy
        else:
            alpha = min(1.0, max(0.05, GPS_MOVING_ALPHA))
            new_x = self.scene_x + alpha * dx
            new_y = self.scene_y + alpha * dy

        self.scene_x = new_x
        self.scene_y = new_y
        self.lat = lat
        self.lon = lon
        return True

    @property
    def has_position(self):
        return self.scene_x is not None and self.scene_y is not None

    def is_stale(self):
        if self.last_report_time is None:
            return True
        now = datetime.now(timezone.utc)
        ts = self.last_report_time
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)
        return (now - ts).total_seconds() > STALE_AFTER_SEC

    def status_text(self):
        if self.last_report_time is None:
            return "CHƯA CÓ DỮ LIỆU"
        if self.is_stale():
            return "DỮ LIỆU CŨ - GIỮ VỊ TRÍ TỐT CUỐI"
        if not self.gps_hop_le:
            return "CHƯA CÓ GPS - GIỮ VỊ TRÍ TỐT CUỐI"
        if not self.gps_tin_cay_2d:
            return "GPS YẾU - GIỮ VỊ TRÍ TỐT CUỐI"
        return "GPS TỐT"


class CampusLiveMonitor(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("AIoT Campus Digital Twin - Live SU / rBS / DU")
        self.resize(1024, 600)
        self.setMinimumSize(800, 480)

        self.background = QPixmap(str(BACKGROUND_PATH))
        if self.background.isNull():
            raise RuntimeError(
                f"Không mở được ảnh nền campus:\n{BACKGROUND_PATH}"
            )

        if not MAP_CONFIG_PATH.exists():
            raise FileNotFoundError(
                f"Không tìm thấy map config:\n{MAP_CONFIG_PATH}"
            )

        with MAP_CONFIG_PATH.open("r", encoding="utf-8") as f:
            self.map_cfg = json.load(f)

        self.image_width = float(
            self.map_cfg["image_width_px"]
        )
        self.image_height = float(
            self.map_cfg["image_height_px"]
        )
        self.camera_center_x = float(
            self.map_cfg["camera_center_x_m"]
        )
        self.camera_center_y = float(
            self.map_cfg["camera_center_y_m"]
        )
        self.visible_width_m = float(
            self.map_cfg["visible_width_m"]
        )
        self.visible_height_m = float(
            self.map_cfg["visible_height_m"]
        )

        # rBS cố định
        self.rbs_scene_x = float(RBS_SCENE_X_M)
        self.rbs_scene_y = float(RBS_SCENE_Y_M)

        self.nodes = {
            "SU": NodeState("SU"),
            "DU": NodeState("DU"),
        }

        # EVE ảo: click chuột trực tiếp trên map để đặt/di chuyển.
        self.eve_scene_x = None
        self.eve_scene_y = None

        # Backend Sionna RT chạy trên Windows.
        self.eve_sionna_channels = {}
        self.eve_sionna_status = "CHƯA ĐẶT EVE"
        self.sionna_request_inflight = False
        self.sionna_pending = False
        self.sionna_last_request_time = 0.0
        self.sionna_last_requested_positions = None
        self.sionna_result_queue = queue.Queue()

        self.sionna_debounce_timer = QTimer(self)
        self.sionna_debounce_timer.setSingleShot(True)
        self.sionna_debounce_timer.timeout.connect(
            self.request_sionna_eve_channels
        )

        self.sionna_result_timer = QTimer(self)
        self.sionna_result_timer.timeout.connect(
            self.poll_sionna_result_queue
        )
        self.sionna_result_timer.start(200)

        # Kênh thật SU-DU được DU đo bằng cách nghe beacon SU.
        self.su_du_channel = None
        self.su_du_csv_offset = 0
        self.su_du_fieldnames = None
        self.initialize_su_du_channel_reader()

        # Incremental CSV reader
        self.csv_fieldnames = None
        self.csv_offset = 0
        self.csv_initialized = False
        self.csv_last_size = 0

        self.initialize_csv_reader()

        self.csv_timer = QTimer(self)
        self.csv_timer.timeout.connect(self.poll_live_csv)
        self.csv_timer.start(CSV_POLL_INTERVAL_MS)

        self.channel_timer = QTimer(self)
        self.channel_timer.timeout.connect(self.poll_su_du_channel_csv)
        self.channel_timer.start(CSV_POLL_INTERVAL_MS)

        # Repaint timer để trạng thái "cũ" tự đổi dù CSV không tăng.
        self.repaint_timer = QTimer(self)
        self.repaint_timer.timeout.connect(self.update)
        self.repaint_timer.start(1000)

        print("=" * 72)
        print("[DIGITAL TWIN CAMPUS LIVE]")
        print(f"[CSV] {CSV_PATH}")
        print(f"[BACKGROUND] {BACKGROUND_PATH}")
        print(f"[MAP CONFIG] {MAP_CONFIG_PATH}")
        print(f"[SU-DU CHANNEL CSV] {SU_DU_CHANNEL_CSV}")
        print(f"[SIONNA EVE SERVER] {SIONNA_EVE_URL}")
        print(
            f"[GPS FILTER] window={GPS_FILTER_WINDOW} "
            f"| deadband_stationary={GPS_STATIONARY_DEADBAND_M:.1f}m "
            f"| deadband_moving={GPS_MOVING_DEADBAND_M:.1f}m"
        )
        print(
            f"[GPS GUARD] campus_bounds +{CAMPUS_BOUNDS_MARGIN_M:.1f}m "
            f"| big_jump={GPS_BIG_JUMP_M:.1f}m "
            f"x{GPS_BIG_JUMP_CONFIRM_SAMPLES} "
            f"| good_fix_max_age={GPS_GOOD_FIX_MAX_AGE_SEC:.0f}s"
        )
        print(
            "[GPS DISPLAY] GIỮ marker ở vị trí tốt cuối cùng; "
            "Sionna chỉ dùng vị trí tốt còn mới."
        )
        print(
            f"[rBS GPS] {RBS_LAT:.7f}, {RBS_LON:.7f}"
        )
        print(
            f"[rBS SCENE] X={RBS_SCENE_X_M:.3f}, "
            f"Y={RBS_SCENE_Y_M:.3f}"
        )
        print("=" * 72)

    # --------------------------------------------------------
    # POSITION GUARDS
    # --------------------------------------------------------
    def scene_point_inside_campus(self, scene_x, scene_y):
        min_x = float(self.map_cfg.get("scene_min_x_m", -1e9))
        max_x = float(self.map_cfg.get("scene_max_x_m", 1e9))
        min_y = float(self.map_cfg.get("scene_min_y_m", -1e9))
        max_y = float(self.map_cfg.get("scene_max_y_m", 1e9))

        return (
            (min_x - CAMPUS_BOUNDS_MARGIN_M)
            <= scene_x
            <= (max_x + CAMPUS_BOUNDS_MARGIN_M)
            and
            (min_y - CAMPUS_BOUNDS_MARGIN_M)
            <= scene_y
            <= (max_y + CAMPUS_BOUNDS_MARGIN_M)
        )

    @staticmethod
    def good_fix_is_fresh(node):
        if node.last_good_time is None:
            return False

        ts = node.last_good_time
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)

        age = (
            datetime.now(timezone.utc) - ts
        ).total_seconds()

        return age <= GPS_GOOD_FIX_MAX_AGE_SEC

    def node_should_be_drawn(self, node):
        """
        HIỂN THỊ:
        - Nếu từng có vị trí tốt và vị trí đó nằm trong campus thì vẫn vẽ.
        - Không ẩn marker chỉ vì GPS hiện tại yếu/cũ.
        """
        return (
            node.has_position
            and self.scene_point_inside_campus(
                node.scene_x,
                node.scene_y,
            )
        )

    def node_is_safe_for_sionna(self, node):
        """
        TÍNH SIONNA:
        - Chỉ dùng vị trí tốt còn mới.
        - Nhờ vậy map vẫn giữ marker, nhưng Sionna không dùng tọa độ cũ.
        """
        return (
            node.has_position
            and self.good_fix_is_fresh(node)
            and self.scene_point_inside_campus(
                node.scene_x,
                node.scene_y,
            )
            and node.gps_hop_le
            and node.gps_tin_cay_2d
        )

    # --------------------------------------------------------
    # CSV
    # --------------------------------------------------------
    @staticmethod
    def parse_csv_line(line, fieldnames):
        try:
            values = next(csv.reader([line]))
        except (csv.Error, StopIteration):
            return None

        if len(values) != len(fieldnames):
            return None

        return dict(zip(fieldnames, values))

    def process_row(self, row):
        if not row:
            return

        device = (row.get("Thiet_bi") or "").strip().upper()
        if device not in self.nodes:
            return

        node = self.nodes[device]

        node.gps_hop_le = (
            (row.get("GPS_hop_le") or "").strip() == "1"
        )
        node.gps_tin_cay_2d = (
            (row.get("GPS_tin_cay_2D") or "").strip() == "1"
        )

        node.satellites = safe_int(row.get("So_ve_tinh"))
        node.hdop = safe_float(row.get("HDOP"))
        node.speed_m_s = safe_float(row.get("Toc_do_m_s"))
        node.rssi = safe_float(row.get("RSSI_dBm"))
        node.snr = safe_float(row.get("SNR_dB"))
        distance_candidate = safe_float(
            row.get("Khoang_cach_rBS_node_m")
        )
        # Khi GPS yếu, CSV thường để trống khoảng cách.
        # Không xóa khoảng cách tốt gần nhất chỉ vì một bản ghi yếu.
        if distance_candidate is not None:
            node.distance_m = distance_candidate
        node.tx_power_dbm = safe_float(
            row.get("Cong_suat_phat_dBm")
        )
        node.sf = safe_int(row.get("SF"))
        node.seq = safe_int(row.get("So_thu_tu_bao_cao"))
        node.last_report_time = parse_time_iso(
            row.get("Thoi_gian_rBS_nhan_UTC")
        )

        lat = safe_float(row.get("Vi_do_tho"))
        lon = safe_float(row.get("Kinh_do_tho"))

        # CHỈ di chuyển marker bằng một GPS đủ tin cậy.
        if (
            node.gps_hop_le
            and node.gps_tin_cay_2d
            and valid_coordinate(lat, lon)
        ):
            raw_sx, raw_sy = gps_to_scene_xy(lat, lon)

            # Một fix có thể được module GPS đánh dấu "tin cậy" nhưng
            # vẫn nằm ngoài campus do multipath / sai số lớn.
            if not self.scene_point_inside_campus(raw_sx, raw_sy):
                print(
                    f"[{device} GPS REJECT] NGOAI CAMPUS "
                    f"| SCENE=({raw_sx:.2f},{raw_sy:.2f}) "
                    f"| SAT={node.satellites} "
                    f"| HDOP={node.hdop}"
                )
                self.update()
                return

            changed = node.update_filtered_position(
                raw_sx,
                raw_sy,
                lat,
                lon,
            )
            node.last_good_time = node.last_report_time

            if changed:
                print(
                    f"[{device} LIVE FILTERED] "
                    f"RAW=({raw_sx:.2f},{raw_sy:.2f}) "
                    f"-> MAP=({node.scene_x:.2f},{node.scene_y:.2f}) "
                    f"| SPEED={node.speed_m_s if node.speed_m_s is not None else 0.0:.2f}m/s "
                    f"| SAT={node.satellites} "
                    f"| HDOP={node.hdop}"
                )

                # Chỉ yêu cầu Sionna khi vị trí ĐÃ LỌC thực sự thay đổi.
                self.schedule_sionna_update(
                    force=False,
                    reason=f"{device}_MOVE",
                )

        self.update()

    def initialize_csv_reader(self):
        if not CSV_PATH.exists():
            print(f"[GUI] Chưa có CSV: {CSV_PATH}")
            return

        try:
            with CSV_PATH.open(
                "r",
                encoding="utf-8-sig",
                newline="",
            ) as f:
                header_line = f.readline()
                if not header_line:
                    return

                self.csv_fieldnames = next(
                    csv.reader([header_line])
                )

                while True:
                    line = f.readline()
                    if not line:
                        break

                    row = self.parse_csv_line(
                        line,
                        self.csv_fieldnames,
                    )
                    self.process_row(row)

                self.csv_offset = f.tell()
                self.csv_initialized = True
                self.csv_last_size = CSV_PATH.stat().st_size

        except (OSError, csv.Error) as error:
            print(f"[GUI] Không đọc được CSV: {error}")

    def poll_live_csv(self):
        if not CSV_PATH.exists():
            return

        try:
            current_size = CSV_PATH.stat().st_size

            # CSV bị reset / truncate
            if (
                self.csv_initialized
                and current_size < self.csv_last_size
            ):
                self.csv_fieldnames = None
                self.csv_offset = 0
                self.csv_initialized = False

            self.csv_last_size = current_size

            if not self.csv_initialized:
                self.initialize_csv_reader()
                return

            if current_size <= self.csv_offset:
                return

            with CSV_PATH.open(
                "r",
                encoding="utf-8-sig",
                newline="",
            ) as f:
                f.seek(self.csv_offset)

                while True:
                    line_start = f.tell()
                    line = f.readline()

                    if not line:
                        break

                    if not line.endswith("\n"):
                        f.seek(line_start)
                        break

                    row = self.parse_csv_line(
                        line,
                        self.csv_fieldnames,
                    )
                    self.process_row(row)
                    self.csv_offset = f.tell()

        except OSError as error:
            print(f"[GUI] Lỗi đọc CSV live: {error}")

    # --------------------------------------------------------
    # KENH THAT SU-DU CSV
    # --------------------------------------------------------
    def _process_su_du_channel_row(self, row):
        if not row:
            return
        try:
            self.su_du_channel = {
                "time": row.get("Thoi_gian_rBS_nhan_UTC", ""),
                "rssi_dbm": safe_float(row.get("RSSI_SU_DU_dBm")),
                "pt_dbm": safe_float(row.get("P_TX_SU_dBm")),
                "pr_mw": safe_float(row.get("P_r_mW")),
                "h2": safe_float(row.get("H_abs_binh_phuong")),
                "h_abs": safe_float(row.get("H_abs")),
                "h_db": safe_float(row.get("H_dB")),
            }
        except Exception:
            return
        self.update()

    def initialize_su_du_channel_reader(self):
        if not SU_DU_CHANNEL_CSV.exists():
            return
        try:
            with SU_DU_CHANNEL_CSV.open("r", encoding="utf-8-sig", newline="") as f:
                header = f.readline()
                if not header:
                    return
                self.su_du_fieldnames = next(csv.reader([header]))
                while True:
                    line = f.readline()
                    if not line:
                        break
                    row = self.parse_csv_line(line, self.su_du_fieldnames)
                    self._process_su_du_channel_row(row)
                self.su_du_csv_offset = f.tell()
        except (OSError, csv.Error):
            pass

    def poll_su_du_channel_csv(self):
        if not SU_DU_CHANNEL_CSV.exists():
            return
        try:
            size = SU_DU_CHANNEL_CSV.stat().st_size
            if size < self.su_du_csv_offset:
                self.su_du_csv_offset = 0
                self.su_du_fieldnames = None
                self.initialize_su_du_channel_reader()
                return
            if size <= self.su_du_csv_offset:
                return
            with SU_DU_CHANNEL_CSV.open("r", encoding="utf-8-sig", newline="") as f:
                if self.su_du_fieldnames is None:
                    header = f.readline()
                    if not header:
                        return
                    self.su_du_fieldnames = next(csv.reader([header]))
                    self.su_du_csv_offset = f.tell()
                else:
                    f.seek(self.su_du_csv_offset)
                while True:
                    pos = f.tell()
                    line = f.readline()
                    if not line:
                        break
                    if not line.endswith("\n"):
                        f.seek(pos)
                        break
                    row = self.parse_csv_line(line, self.su_du_fieldnames)
                    self._process_su_du_channel_row(row)
                    self.su_du_csv_offset = f.tell()
        except OSError:
            return

    # --------------------------------------------------------
    # CHANNEL ENGINE
    # --------------------------------------------------------
    def real_channel_node_rbs(self, node_name):
        node = self.nodes[node_name]
        return measured_channel_from_rssi(node.rssi, node.tx_power_dbm)

    def _current_sionna_positions(self):
        if self.eve_scene_x is None or self.eve_scene_y is None:
            return None

        su = self.nodes["SU"]
        du = self.nodes["DU"]

        return {
            "eve": (
                float(self.eve_scene_x),
                float(self.eve_scene_y),
                float(EVE_HEIGHT_M),
            ),
            "rbs": (
                float(self.rbs_scene_x),
                float(self.rbs_scene_y),
                float(RBS_SCENE_Z_M),
            ),
            "su": (
                float(su.scene_x),
                float(su.scene_y),
                float(SU_HEIGHT_M),
            ) if self.node_is_safe_for_sionna(su) else None,
            "du": (
                float(du.scene_x),
                float(du.scene_y),
                float(DU_HEIGHT_M),
            ) if self.node_is_safe_for_sionna(du) else None,
        }

    @staticmethod
    def _positions_moved(a, b, threshold_m):
        if a is None or b is None:
            return True

        for key in ("eve", "rbs", "su", "du"):
            pa = a.get(key)
            pb = b.get(key)

            if pa is None and pb is None:
                continue
            if (pa is None) != (pb is None):
                return True

            dx = float(pa[0]) - float(pb[0])
            dy = float(pa[1]) - float(pb[1])
            dz = float(pa[2]) - float(pb[2])

            if math.sqrt(dx*dx + dy*dy + dz*dz) >= threshold_m:
                return True

        return False

    def schedule_sionna_update(self, force=False, reason="UPDATE"):
        positions = self._current_sionna_positions()
        if positions is None:
            return

        # Nếu một lần ray tracing đang chạy, chỉ ghi nhận rằng
        # vị trí đã thay đổi. Không đổi status khỏi "ĐANG TÍNH".
        if self.sionna_request_inflight:
            self.sionna_pending = True
            # Giữ nguyên "SIONNA ĐANG TÍNH..." trên màn hình.
            return

        now = time.monotonic()

        if not force:
            if not self._positions_moved(
                positions,
                self.sionna_last_requested_positions,
                SIONNA_MOVE_THRESHOLD_M,
            ):
                return

            if (
                now - self.sionna_last_request_time
                < SIONNA_MIN_INTERVAL_SEC
            ):
                return

        self.sionna_pending = True
        self.eve_sionna_status = f"CHỜ SIONNA ({reason})"

        # Debounce để SU và DU có thể cập nhật gần nhau trước khi gửi.
        self.sionna_debounce_timer.start(700)
        self.update()

    def request_sionna_eve_channels(self):
        if self.eve_scene_x is None or self.eve_scene_y is None:
            return

        if self.sionna_request_inflight:
            self.sionna_pending = True
            return

        positions = self._current_sionna_positions()
        if positions is None:
            return

        def point_dict(p):
            if p is None:
                return None
            return {"x": p[0], "y": p[1], "z": p[2]}

        payload = {
            "eve": point_dict(positions["eve"]),
            "rbs": point_dict(positions["rbs"]),
            "su": point_dict(positions["su"]),
            "du": point_dict(positions["du"]),
        }

        self.sionna_request_inflight = True
        self.sionna_pending = False
        self.sionna_last_request_time = time.monotonic()
        self.sionna_last_requested_positions = positions
        self.eve_sionna_status = "SIONNA ĐANG TÍNH..."
        self.update()

        print(
            f"[EVE SIONNA] GỬI YÊU CẦU -> {SIONNA_EVE_URL} "
            f"| EVE=({self.eve_scene_x:.2f},{self.eve_scene_y:.2f})"
        )

        threading.Thread(
            target=self._sionna_http_worker,
            args=(payload,),
            daemon=True,
        ).start()

    def _sionna_http_worker(self, payload):
        try:
            raw = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                SIONNA_EVE_URL,
                data=raw,
                headers={"Content-Type": "application/json"},
                method="POST",
            )

            with urllib.request.urlopen(
                req,
                timeout=SIONNA_HTTP_TIMEOUT_SEC,
            ) as resp:
                data = json.loads(
                    resp.read().decode("utf-8")
                )

            self.sionna_result_queue.put(("OK", data))

        except Exception as exc:
            self.sionna_result_queue.put(
                ("ERR", f"{type(exc).__name__}: {exc}")
            )

    def poll_sionna_result_queue(self):
        changed = False

        while True:
            try:
                kind, data = self.sionna_result_queue.get_nowait()
            except queue.Empty:
                break

            changed = True
            self.sionna_request_inflight = False

            if kind == "OK" and data.get("ok"):
                self.eve_sionna_channels = data.get("channels", {})
                self.eve_sionna_status = "SIONNA OK"

                def htext(key):
                    d = self.eve_sionna_channels.get(key)
                    if not d or not d.get("ok") or d.get("h_db") is None:
                        return "--"
                    return f"{d['h_db']:.1f} dB"

                print(
                    "[EVE SIONNA] OK | "
                    f"rBS-EVE={htext('RBS_EVE')} | "
                    f"SU-EVE={htext('SU_EVE')} | "
                    f"DU-EVE={htext('DU_EVE')}"
                )
            else:
                if kind == "OK":
                    message = data.get("message") or data.get("error") or "UNKNOWN"
                else:
                    message = str(data)

                msg_lower = message.lower()

                if "timed out" in msg_lower or "timeout" in msg_lower:
                    short_status = "TIMEOUT - SIONNA CHẬM"
                elif "connection refused" in msg_lower or "unable to connect" in msg_lower:
                    short_status = "SERVER WINDOWS OFFLINE"
                elif "http error 500" in msg_lower:
                    short_status = "SERVER SIONNA LỖI 500"
                elif "urlopen error" in msg_lower:
                    short_status = "LỖI KẾT NỐI WINDOWS"
                else:
                    short_status = "LỖI SIONNA"

                self.eve_sionna_status = short_status
                print(
                    f"[EVE SIONNA] {short_status} | CHI TIẾT: {message}"
                )

            # Nếu trong lúc Sionna đang tính mà vị trí lọc thay đổi,
            # kiểm tra lại theo ngưỡng 5 m. Không ray-tracing theo jitter GPS.
            if self.sionna_pending:
                self.sionna_pending = False
                self.schedule_sionna_update(
                    force=False,
                    reason="POS_UPDATE",
                )

        if changed:
            self.update()

    def virtual_eve_channels(self):
        # Giữ tên hàm cũ để phần vẽ không phải thay đổi nhiều.
        # Nhưng dữ liệu bây giờ hoàn toàn đến từ Sionna RT trên Windows.
        if self.eve_scene_x is None or self.eve_scene_y is None:
            return {}
        return self.eve_sionna_channels

    @staticmethod
    def channel_line(name, data, source):
        # Bản hiển thị gọn trên màn Pi: ưu tiên H_dB để chữ không bị khuất.
        # |H|, |H|^2 và P_r vẫn được hệ thống tính trong Channel Engine.
        if not data or data.get("h_db") is None:
            return f"{name}: -- dB  [{source}]"
        return f"{name}: {data['h_db']:.1f} dB  [{source}]"

    # --------------------------------------------------------
    # MAP COORDINATE
    # --------------------------------------------------------
    def scene_to_image_pixel(self, scene_x, scene_y):
        min_x = (
            self.camera_center_x
            - self.visible_width_m / 2.0
        )
        max_y = (
            self.camera_center_y
            + self.visible_height_m / 2.0
        )

        u = (
            (scene_x - min_x)
            / self.visible_width_m
            * self.image_width
        )

        # +Y scene là hướng lên trên ảnh
        v = (
            (max_y - scene_y)
            / self.visible_height_m
            * self.image_height
        )

        return u, v

    def layout_rects(self):
        # V8 ULTRAWIDE:
        # ưu tiên campus chiếm gần hết chiều ngang màn Pi.
        # Với màn 1024 px, panel còn khoảng 190-200 px.
        margin = 4.0
        panel_width = max(188.0, self.width() * 0.19)
        panel_width = min(panel_width, 205.0)

        map_outer = QRectF(
            margin,
            margin,
            max(420.0, self.width() - panel_width - 3 * margin),
            self.height() - 2 * margin,
        )

        panel = QRectF(
            map_outer.right() + margin,
            margin,
            panel_width,
            self.height() - 2 * margin,
        )

        # Fit ảnh nền trong map rect mà không làm méo.
        img_aspect = (
            self.background.width()
            / self.background.height()
        )
        box_aspect = (
            map_outer.width()
            / max(1.0, map_outer.height())
        )

        if box_aspect > img_aspect:
            draw_h = map_outer.height()
            draw_w = draw_h * img_aspect
        else:
            draw_w = map_outer.width()
            draw_h = draw_w / img_aspect

        image_rect = QRectF(
            map_outer.center().x() - draw_w / 2.0,
            map_outer.center().y() - draw_h / 2.0,
            draw_w,
            draw_h,
        )

        return map_outer, image_rect, panel

    def scene_to_screen(self, scene_x, scene_y, image_rect):
        u, v = self.scene_to_image_pixel(scene_x, scene_y)

        x = (
            image_rect.left()
            + (u / self.image_width) * image_rect.width()
        )
        y = (
            image_rect.top()
            + (v / self.image_height) * image_rect.height()
        )

        return QPointF(x, y)

    def screen_to_scene(self, point, image_rect):
        if image_rect.width() <= 0 or image_rect.height() <= 0:
            return None
        u = (point.x() - image_rect.left()) / image_rect.width() * self.image_width
        v = (point.y() - image_rect.top()) / image_rect.height() * self.image_height
        min_x = self.camera_center_x - self.visible_width_m / 2.0
        max_y = self.camera_center_y + self.visible_height_m / 2.0
        scene_x = min_x + (u / self.image_width) * self.visible_width_m
        scene_y = max_y - (v / self.image_height) * self.visible_height_m
        return scene_x, scene_y

    # --------------------------------------------------------
    # DRAW
    # --------------------------------------------------------
    @staticmethod
    def draw_marker(
        painter,
        point,
        name,
        fill_color,
        status_text=None,
        fixed=False,
    ):
        radius = 12.0 if not fixed else 13.0

        # vòng ngoài trắng để nhìn rõ trên mọi nền
        painter.setPen(QPen(QColor(255, 255, 255), 4))
        painter.setBrush(QBrush(fill_color))
        painter.drawEllipse(point, radius, radius)

        painter.setPen(QPen(QColor(20, 20, 20), 1))
        painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))

        label = name
        if status_text and "GPS TỐT" not in status_text:
            label += " !"

        painter.drawText(
            int(point.x() + 16),
            int(point.y() - 10),
            label,
        )

    @staticmethod
    def fmt(value, suffix="", decimals=1):
        if value is None:
            return "--"
        return f"{value:.{decimals}f}{suffix}"

    def draw_rbs_info(
        self,
        painter,
        x,
        y,
        width,
    ):
        painter.setPen(QPen(QColor(255, 215, 40), 1))
        painter.setFont(
            QFont("Arial", 10, QFont.Weight.Bold)
        )
        painter.drawText(int(x), int(y), "rBS")

        painter.setPen(QPen(QColor(230, 230, 230), 1))
        painter.setFont(QFont("Arial", 8))

        lines = [
            "Trạng thái: CỐ ĐỊNH",
            f"GPS: {RBS_LAT:.6f}, {RBS_LON:.6f}",
            f"Scene: X={RBS_SCENE_X_M:.1f}, Y={RBS_SCENE_Y_M:.1f}",
            f"Z: {RBS_SCENE_Z_M:.1f} m",
        ]

        yy = y + 22
        for line in lines:
            painter.drawText(int(x), int(yy), line)
            yy += 18

        return yy

    def display_distance_to_rbs(self, node):
        """
        Khoảng cách hiển thị:
        - ưu tiên khoảng cách tốt gần nhất rBS đã ghi;
        - nếu chưa có thì tính từ vị trí tốt cuối cùng trên scene.
        GPS hiện tại yếu/cũ không làm mất dòng khoảng cách.
        """
        if node.distance_m is not None:
            return node.distance_m

        if node.has_position:
            dx = float(node.scene_x) - float(self.rbs_scene_x)
            dy = float(node.scene_y) - float(self.rbs_scene_y)
            return math.hypot(dx, dy)

        return None

    def draw_node_info(
        self,
        painter,
        x,
        y,
        width,
        title,
        node,
        title_color,
    ):
        painter.setPen(QPen(title_color, 1))
        painter.setFont(
            QFont("Arial", 10, QFont.Weight.Bold)
        )
        painter.drawText(int(x), int(y), title)

        painter.setPen(QPen(QColor(230, 230, 230), 1))
        painter.setFont(QFont("Arial", 8))

        lines = [
            f"Trạng thái: {node.status_text()}",
            f"Vệ tinh: {node.satellites if node.satellites is not None else '--'}",
            f"d-rBS: {self.fmt(self.display_distance_to_rbs(node), ' m', 1)}",
            f"TX: {self.fmt(node.tx_power_dbm, ' dBm', 0)} | SF{node.sf if node.sf is not None else '--'}",
        ]

        yy = y + 22
        for line in lines:
            painter.drawText(int(x), int(yy), line)
            yy += 18

        return yy

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(
            QPainter.RenderHint.Antialiasing
        )

        # Nền cửa sổ
        painter.fillRect(
            self.rect(),
            QColor(18, 20, 24),
        )

        map_outer, image_rect, panel = self.layout_rects()

        # Khung bản đồ
        painter.setPen(QPen(QColor(90, 95, 105), 1))
        painter.setBrush(QBrush(QColor(30, 32, 36)))
        painter.drawRoundedRect(map_outer, 8, 8)

        # Ảnh campus
        painter.drawPixmap(
            image_rect.toRect(),
            self.background,
        )

        # Điểm rBS
        rbs_point = self.scene_to_screen(
            self.rbs_scene_x,
            self.rbs_scene_y,
            image_rect,
        )

        su_point = None
        du_point = None

        if self.node_should_be_drawn(self.nodes["SU"]):
            su_point = self.scene_to_screen(
                self.nodes["SU"].scene_x,
                self.nodes["SU"].scene_y,
                image_rect,
            )

        if self.node_should_be_drawn(self.nodes["DU"]):
            du_point = self.scene_to_screen(
                self.nodes["DU"].scene_x,
                self.nodes["DU"].scene_y,
                image_rect,
            )

        # Đường liên kết
        painter.setPen(
            QPen(QColor(255, 215, 40), 3)
        )

        if su_point is not None:
            painter.drawLine(su_point, rbs_point)

        if du_point is not None:
            painter.drawLine(rbs_point, du_point)

        # Markers
        if su_point is not None:
            self.draw_marker(
                painter,
                su_point,
                "SU",
                QColor(35, 220, 90),
                self.nodes["SU"].status_text(),
            )

        self.draw_marker(
            painter,
            rbs_point,
            "rBS",
            QColor(255, 205, 30),
            fixed=True,
        )

        if du_point is not None:
            self.draw_marker(
                painter,
                du_point,
                "DU",
                QColor(60, 135, 255),
                self.nodes["DU"].status_text(),
            )

        # EVE ảo: click để đặt. Đường đỏ nét đứt là kênh ảo.
        if self.eve_scene_x is not None and self.eve_scene_y is not None:
            eve_point = self.scene_to_screen(self.eve_scene_x, self.eve_scene_y, image_rect)
            eve_pen = QPen(QColor(255, 80, 80), 2)
            eve_pen.setStyle(Qt.PenStyle.DashLine)
            painter.setPen(eve_pen)
            painter.drawLine(eve_point, rbs_point)
            if su_point is not None:
                painter.drawLine(eve_point, su_point)
            if du_point is not None:
                painter.drawLine(eve_point, du_point)
            self.draw_marker(
                painter, eve_point, "EVE", QColor(235, 45, 55), "AO", fixed=False
            )

        # Panel
        painter.setPen(QPen(QColor(80, 85, 95), 1))
        painter.setBrush(QBrush(QColor(27, 29, 34)))
        painter.drawRoundedRect(panel, 8, 8)

        px = panel.left() + 16
        py = panel.top() + 30

        painter.setPen(QPen(QColor(245, 245, 245), 1))
        painter.setFont(
            QFont("Arial", 12, QFont.Weight.Bold)
        )
        painter.drawText(
            int(px),
            int(py),
            "DIGITAL TWIN - LIVE",
        )

        painter.setFont(QFont("Arial", 8))
        painter.setPen(QPen(QColor(190, 195, 205), 1))
        painter.drawText(
            int(px),
            int(py + 22),
            "SU / rBS / DU / EVE",
        )

        py += 62
        py = self.draw_rbs_info(
            painter,
            px,
            py,
            panel.width() - 32,
        )

        py += 24
        py = self.draw_node_info(
            painter,
            px,
            py,
            panel.width() - 32,
            "SU",
            self.nodes["SU"],
            QColor(80, 240, 120),
        )

        py += 24
        py = self.draw_node_info(
            painter,
            px,
            py,
            panel.width() - 32,
            "DU",
            self.nodes["DU"],
            QColor(100, 165, 255),
        )

        # 6 HE SO KENH
        py += 24
        painter.setPen(QPen(QColor(245, 245, 245), 1))
        painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        painter.drawText(int(px), int(py), "6 HỆ SỐ KÊNH")
        py += 20

        su_rbs = self.real_channel_node_rbs("SU")
        du_rbs = self.real_channel_node_rbs("DU")
        su_du = self.su_du_channel
        eve = self.virtual_eve_channels()

        painter.setFont(QFont("Arial", 8))
        lines = [
            self.channel_line("SU-rBS", su_rbs, "THẬT"),
            self.channel_line("DU-rBS", du_rbs, "THẬT"),
            self.channel_line("SU-DU", su_du, "THẬT"),
            self.channel_line("EVE-rBS", eve.get("RBS_EVE"), "SIONNA"),
            self.channel_line("EVE-SU", eve.get("SU_EVE"), "SIONNA"),
            self.channel_line("EVE-DU", eve.get("DU_EVE"), "SIONNA"),
        ]
        painter.setPen(QPen(QColor(225, 225, 230), 1))
        for line in lines:
            painter.drawText(int(px), int(py), line)
            py += 18

        painter.setPen(QPen(QColor(255, 210, 90), 1))
        painter.setFont(QFont("Arial", 7))
        painter.drawText(
            int(px),
            int(py),
            f"EVE: {self.eve_sionna_status}",
        )

        # Chú thích cuối
        painter.setPen(QPen(QColor(165, 170, 180), 1))
        painter.setFont(QFont("Arial", 8))
        painter.drawText(
            int(px),
            int(panel.bottom() - 42),
            "Click trái trên bản đồ: đặt/di chuyển EVE.",
        )
        painter.drawText(
            int(px),
            int(panel.bottom() - 24),
            "Click phải trên bản đồ: xóa EVE.",
        )

    def mousePressEvent(self, event):
        _map_outer, image_rect, _panel = self.layout_rects()
        p = event.position()
        if image_rect.contains(p):
            if event.button() == Qt.MouseButton.LeftButton:
                scene = self.screen_to_scene(p, image_rect)
                if scene is not None:
                    self.eve_scene_x, self.eve_scene_y = scene
                    print(
                        f"[EVE AO] DAT TAI SCENE X={self.eve_scene_x:.2f}, "
                        f"Y={self.eve_scene_y:.2f}"
                    )

                    # Xóa kết quả cũ để không hiển thị số của vị trí EVE trước.
                    self.eve_sionna_channels = {}
                    self.eve_sionna_status = "CHỜ SIONNA"
                    self.schedule_sionna_update(
                        force=True,
                        reason="EVE_CLICK",
                    )
                    self.update()
                    return
            elif event.button() == Qt.MouseButton.RightButton:
                self.eve_scene_x = None
                self.eve_scene_y = None
                self.eve_sionna_channels = {}
                self.eve_sionna_status = "CHƯA ĐẶT EVE"
                self.sionna_pending = False
                print("[EVE AO] DA XOA")
                self.update()
                return
        super().mousePressEvent(event)

    def keyPressEvent(self, event):
        # F11: fullscreen / normal
        if event.key() == Qt.Key.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            return

        # ESC thoát fullscreen, không đóng app
        if event.key() == Qt.Key.Key_Escape:
            if self.isFullScreen():
                self.showNormal()
                return

        super().keyPressEvent(event)


def main():
    app = QApplication(sys.argv)
    window = CampusLiveMonitor()

    # Có thể bật fullscreen bằng biến môi trường:
    # RBS_UI_FULLSCREEN=1
    if os.environ.get("RBS_UI_FULLSCREEN", "0") == "1":
        window.showFullScreen()
    else:
        window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
