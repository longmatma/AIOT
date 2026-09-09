import sys
import math
import csv
import os
from pathlib import Path

from PySide6.QtWidgets import QApplication, QWidget
from PySide6.QtGui import QPainter, QPen, QFont
from PySide6.QtCore import Qt, QPointF, QTimer

# ==========================================================
# LẤY CẤU HÌNH GPS/rBS TRỰC TIẾP TỪ PROJECT
# File này nên đặt cùng thư mục với gps_rbs.py trên Raspberry Pi.
# ==========================================================
try:
    from gps_rbs import (
        VI_DO_RBS,
        KINH_DO_RBS,
        DUONG_DAN_CSV_DINH_KY,
    )
except ImportError:
    # Chỉ để code có thể mở trên Windows/VS Code khi chưa copy gps_rbs.py.
    # Khi chạy thật trên Pi, nên import được gps_rbs.py ở phía trên.
    VI_DO_RBS = 20.9807467
    KINH_DO_RBS = 105.7962650
    DUONG_DAN_CSV_DINH_KY = Path("rbs_lien_ket_dinh_ky_v11.csv")

RBS_LAT = float(VI_DO_RBS)
RBS_LON = float(KINH_DO_RBS)

# Có thể override đường dẫn khi test trên Windows bằng biến môi trường RBS_GPS_CSV.
GPS_CSV_PATH = Path(
    os.environ.get("RBS_GPS_CSV", str(DUONG_DAN_CSV_DINH_KY))
)

# ==========================================================
# CẤU HÌNH GIAO DIỆN
# ==========================================================
INFO_PANEL_WIDTH = 280
MAP_MARGIN = 40
MIN_RANGE_M = 100
CSV_POLL_INTERVAL_MS = 300


# ==========================================================
# GPS -> X,Y CỤC BỘ (MÉT)
# rBS = (0,0)
# X > 0: Đông | X < 0: Tây
# Y > 0: Bắc | Y < 0: Nam
# ==========================================================
def gps_to_xy(lat, lon, lat0, lon0):
    r = 6_371_000.0

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    x = r * (lon_rad - lon0_rad) * math.cos(lat0_rad)
    y = r * (lat_rad - lat0_rad)

    return x, y


# ==========================================================
# KHOẢNG CÁCH HAVERSINE (MÉT)
# ==========================================================
def haversine_distance(lat1, lon1, lat2, lon2):
    r = 6_371_000.0

    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1_rad)
        * math.cos(lat2_rad)
        * math.sin(dlon / 2) ** 2
    )
    a = max(0.0, min(1.0, a))

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return r * c


# ==========================================================
# GIAO DIỆN CHÍNH
# ==========================================================
class PositionMonitor(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("SU - rBS - DU Live Position Monitor")
        self.resize(1200, 750)

        # rBS cố định
        self.rbs_x = 0.0
        self.rbs_y = 0.0

        # SU/DU chưa có dữ liệu thật khi vừa mở chương trình
        self.su_lat = None
        self.su_lon = None
        self.su_x = None
        self.su_y = None
        self.distance_rbs_su = None

        self.du_lat = None
        self.du_lon = None
        self.du_x = None
        self.du_y = None
        self.distance_rbs_du = None

        # Thông tin GPS gần nhất
        self.su_satellites = None
        self.su_hdop = None
        self.du_satellites = None
        self.du_hdop = None

        # Theo dõi file CSV do rbs_gateway/gps_rbs ghi
        self.csv_fieldnames = None
        self.csv_offset = 0
        self.csv_initialized = False
        self.csv_last_size = 0

        # Đọc dữ liệu đã có sẵn một lần khi khởi động
        self.initialize_csv_reader()

        # Sau đó chỉ đọc các dòng mới append vào CSV
        self.csv_timer = QTimer(self)
        self.csv_timer.timeout.connect(self.poll_live_gps)
        self.csv_timer.start(CSV_POLL_INTERVAL_MS)

        print(f"[GUI] rBS = {RBS_LAT:.7f}, {RBS_LON:.7f}")
        print(f"[GUI] Theo dõi GPS thật từ: {GPS_CSV_PATH}")

    # ======================================================
    # VALIDATE GPS
    # ======================================================
    @staticmethod
    def valid_coordinate(lat, lon):
        return (
            lat is not None
            and lon is not None
            and -90.0 <= lat <= 90.0
            and -180.0 <= lon <= 180.0
            and not (lat == 0.0 and lon == 0.0)
        )

    # ======================================================
    # FORMAT KHOẢNG CÁCH
    # ======================================================
    @staticmethod
    def format_distance(distance):
        if distance is None:
            return "--"
        if distance < 1000:
            return f"{distance:.1f} m"
        return f"{distance / 1000:.2f} km"

    # ======================================================
    # CẬP NHẬT SU TỪ GPS THẬT
    # ======================================================
    def update_su(self, lat, lon, satellites=None, hdop=None):
        if not self.valid_coordinate(lat, lon):
            return

        self.su_lat = float(lat)
        self.su_lon = float(lon)
        self.su_satellites = satellites
        self.su_hdop = hdop

        self.su_x, self.su_y = gps_to_xy(
            self.su_lat,
            self.su_lon,
            RBS_LAT,
            RBS_LON,
        )

        self.distance_rbs_su = haversine_distance(
            RBS_LAT,
            RBS_LON,
            self.su_lat,
            self.su_lon,
        )

        print(
            f"[SU LIVE] Lat={self.su_lat:.7f} | "
            f"Lon={self.su_lon:.7f} | "
            f"Distance={self.format_distance(self.distance_rbs_su)}"
        )

        self.update()

    # ======================================================
    # CẬP NHẬT DU TỪ GPS THẬT
    # ======================================================
    def update_du(self, lat, lon, satellites=None, hdop=None):
        if not self.valid_coordinate(lat, lon):
            return

        self.du_lat = float(lat)
        self.du_lon = float(lon)
        self.du_satellites = satellites
        self.du_hdop = hdop

        self.du_x, self.du_y = gps_to_xy(
            self.du_lat,
            self.du_lon,
            RBS_LAT,
            RBS_LON,
        )

        self.distance_rbs_du = haversine_distance(
            RBS_LAT,
            RBS_LON,
            self.du_lat,
            self.du_lon,
        )

        print(
            f"[DU LIVE] Lat={self.du_lat:.7f} | "
            f"Lon={self.du_lon:.7f} | "
            f"Distance={self.format_distance(self.distance_rbs_du)}"
        )

        self.update()

    # ======================================================
    # CSV -> DICT
    # ======================================================
    @staticmethod
    def parse_csv_line(line, fieldnames):
        try:
            values = next(csv.reader([line]))
        except (csv.Error, StopIteration):
            return None

        if len(values) != len(fieldnames):
            return None

        return dict(zip(fieldnames, values))

    # ======================================================
    # XỬ LÝ 1 DÒNG GPS THẬT TỪ rBS
    # ======================================================
    def process_gps_row(self, row):
        if not row:
            return

        device = (row.get("Thiet_bi") or "").strip().upper()
        if device not in ("SU", "DU"):
            return

        # Hiển thị tọa độ khi bản tin có GPS hợp lệ; GPS_tin_cay_2D
        # chỉ là mức chất lượng dùng để công bố khoảng cách tin cậy.
        gps_valid_text = (row.get("GPS_hop_le") or "").strip()

        if gps_valid_text != "1":
            return

        try:
            lat = float(row["Vi_do_tho"])
            lon = float(row["Kinh_do_tho"])
        except (KeyError, TypeError, ValueError):
            return

        satellites = None
        hdop = None

        try:
            satellites = int(float(row.get("So_ve_tinh", "")))
        except (TypeError, ValueError):
            pass

        try:
            hdop = float(row.get("HDOP", ""))
        except (TypeError, ValueError):
            pass

        if device == "SU":
            self.update_su(lat, lon, satellites, hdop)
        else:
            self.update_du(lat, lon, satellites, hdop)

    # ======================================================
    # ĐỌC DỮ LIỆU ĐÃ CÓ SẴN KHI GUI VỪA KHỞI ĐỘNG
    # ======================================================
    def initialize_csv_reader(self):
        if not GPS_CSV_PATH.exists():
            return

        try:
            with GPS_CSV_PATH.open("r", encoding="utf-8", newline="") as f:
                header_line = f.readline()
                if not header_line:
                    return

                self.csv_fieldnames = next(csv.reader([header_line]))

                while True:
                    line = f.readline()
                    if not line:
                        break

                    row = self.parse_csv_line(line, self.csv_fieldnames)
                    self.process_gps_row(row)

                self.csv_offset = f.tell()
                self.csv_initialized = True
                self.csv_last_size = GPS_CSV_PATH.stat().st_size

        except (OSError, csv.Error) as error:
            print(f"[GUI] Không đọc được CSV GPS: {error}")

    # ======================================================
    # ĐỌC CÁC GPS MỚI ĐƯỢC rBS APPEND VÀO CSV
    # ======================================================
    def poll_live_gps(self):
        if not GPS_CSV_PATH.exists():
            return

        try:
            current_size = GPS_CSV_PATH.stat().st_size

            # File bị xóa/tạo lại hoặc truncate -> khởi tạo lại reader.
            if self.csv_initialized and current_size < self.csv_last_size:
                self.csv_fieldnames = None
                self.csv_offset = 0
                self.csv_initialized = False

            self.csv_last_size = current_size

            if not self.csv_initialized:
                self.initialize_csv_reader()
                return

            if current_size == 0 or current_size <= self.csv_offset:
                return

            with GPS_CSV_PATH.open("r", encoding="utf-8", newline="") as f:
                f.seek(self.csv_offset)

                while True:
                    line_start = f.tell()
                    line = f.readline()

                    if not line:
                        break

                    # Nếu đang đúng lúc writer chưa ghi xong dòng, chờ vòng poll sau.
                    if not line.endswith("\n"):
                        f.seek(line_start)
                        break

                    row = self.parse_csv_line(line, self.csv_fieldnames)
                    self.process_gps_row(row)
                    self.csv_offset = f.tell()

        except OSError as error:
            print(f"[GUI] Lỗi đọc GPS live: {error}")

    # ======================================================
    # TÂM KHU VỰC VẼ
    # ======================================================
    def get_map_center(self):
        map_left = INFO_PANEL_WIDTH + MAP_MARGIN
        map_right = self.width() - MAP_MARGIN
        map_top = MAP_MARGIN
        map_bottom = self.height() - MAP_MARGIN

        center_x = (map_left + map_right) / 2
        center_y = (map_top + map_bottom) / 2

        return center_x, center_y

    # ======================================================
    # AUTO SCALE
    # ======================================================
    def calculate_scale(self):
        values = [MIN_RANGE_M]

        if self.su_x is not None and self.su_y is not None:
            values.extend((abs(self.su_x), abs(self.su_y)))

        if self.du_x is not None and self.du_y is not None:
            values.extend((abs(self.du_x), abs(self.du_y)))

        max_range = max(values)

        map_width = self.width() - INFO_PANEL_WIDTH - 2 * MAP_MARGIN
        map_height = self.height() - 2 * MAP_MARGIN

        pixel_radius = min(map_width / 2, map_height / 2) * 0.80

        return pixel_radius / max_range

    # ======================================================
    # X,Y -> PIXEL
    # ======================================================
    def world_to_screen(self, x, y):
        center_x, center_y = self.get_map_center()
        scale = self.calculate_scale()

        screen_x = center_x + x * scale
        screen_y = center_y - y * scale

        return QPointF(screen_x, screen_y)

    # ======================================================
    # VẼ NODE
    # ======================================================
    def draw_node(self, painter, x, y, name):
        if x is None or y is None:
            return

        point = self.world_to_screen(x, y)
        radius = 10

        painter.setPen(QPen(Qt.GlobalColor.black, 1))
        painter.setBrush(Qt.GlobalColor.black)
        painter.drawEllipse(point, radius, radius)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        painter.setPen(QPen(Qt.GlobalColor.black, 1))
        painter.setFont(QFont("Arial", 11))
        painter.drawText(
            int(point.x() + 15),
            int(point.y() - 10),
            name,
        )

    # ======================================================
    # HIỂN THỊ GPS TRONG PANEL
    # ======================================================
    @staticmethod
    def format_coordinate(value):
        return "--" if value is None else f"{value:.7f}"

    # ======================================================
    # PAINT EVENT
    # ======================================================
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width = self.width()
        height = self.height()

        center_x, center_y = self.get_map_center()

        map_left = INFO_PANEL_WIDTH + MAP_MARGIN
        map_right = width - MAP_MARGIN
        map_top = MAP_MARGIN
        map_bottom = height - MAP_MARGIN

        # --------------------------------------------------
        # ĐƯỜNG NGĂN INFO / MAP
        # --------------------------------------------------
        painter.setPen(QPen(Qt.GlobalColor.lightGray, 1))
        painter.drawLine(
            INFO_PANEL_WIDTH,
            0,
            INFO_PANEL_WIDTH,
            height,
        )

        # --------------------------------------------------
        # TRỤC TỌA ĐỘ
        # --------------------------------------------------
        painter.setPen(QPen(Qt.GlobalColor.gray, 1))

        painter.drawLine(
            int(map_left),
            int(center_y),
            int(map_right),
            int(center_y),
        )

        painter.drawLine(
            int(center_x),
            int(map_top),
            int(center_x),
            int(map_bottom),
        )

        # --------------------------------------------------
        # HƯỚNG
        # --------------------------------------------------
        painter.setFont(QFont("Arial", 11))

        painter.drawText(int(center_x - 15), 28, "BẮC")
        painter.drawText(int(center_x - 15), height - 15, "NAM")
        painter.drawText(int(map_left), int(center_y - 10), "TÂY")
        painter.drawText(int(map_right - 40), int(center_y - 10), "ĐÔNG")

        # --------------------------------------------------
        # rBS luôn có vị trí
        # --------------------------------------------------
        rbs = self.world_to_screen(self.rbs_x, self.rbs_y)

        # --------------------------------------------------
        # ĐƯỜNG NỐI CHỈ VẼ KHI CÓ GPS THẬT
        # --------------------------------------------------
        painter.setPen(QPen(Qt.GlobalColor.darkGray, 2))

        if self.su_x is not None and self.su_y is not None:
            su = self.world_to_screen(self.su_x, self.su_y)
            painter.drawLine(su, rbs)

        if self.du_x is not None and self.du_y is not None:
            du = self.world_to_screen(self.du_x, self.du_y)
            painter.drawLine(rbs, du)

        # --------------------------------------------------
        # NODE
        # --------------------------------------------------
        self.draw_node(painter, self.rbs_x, self.rbs_y, "rBS")
        self.draw_node(painter, self.su_x, self.su_y, "SU")
        self.draw_node(painter, self.du_x, self.du_y, "DU")

        # --------------------------------------------------
        # BẢNG THÔNG TIN
        # --------------------------------------------------
        painter.setPen(QPen(Qt.GlobalColor.black, 1))
        painter.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        painter.drawText(20, 35, "THÔNG TIN VỊ TRÍ")

        painter.setFont(QFont("Arial", 10))

        # rBS
        painter.drawText(20, 70, "rBS")
        painter.drawText(35, 95, f"Lat: {RBS_LAT:.7f}")
        painter.drawText(35, 120, f"Lon: {RBS_LON:.7f}")

        # SU
        painter.drawText(20, 165, "SU")
        painter.drawText(35, 190, f"Lat: {self.format_coordinate(self.su_lat)}")
        painter.drawText(35, 215, f"Lon: {self.format_coordinate(self.su_lon)}")
        painter.drawText(
            35,
            240,
            f"Distance: {self.format_distance(self.distance_rbs_su)}",
        )

        # DU
        painter.drawText(20, 290, "DU")
        painter.drawText(35, 315, f"Lat: {self.format_coordinate(self.du_lat)}")
        painter.drawText(35, 340, f"Lon: {self.format_coordinate(self.du_lon)}")
        painter.drawText(
            35,
            365,
            f"Distance: {self.format_distance(self.distance_rbs_du)}",
        )


# ==========================================================
# MAIN
# ==========================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PositionMonitor()
    window.show()
    sys.exit(app.exec())