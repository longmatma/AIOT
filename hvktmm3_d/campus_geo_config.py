# -*- coding: utf-8 -*-
"""
CẤU HÌNH ÁNH XẠ GPS -> TỌA ĐỘ SCENE SIONNA

Baseline hiện tại:
- GPS rBS lấy theo gps_rbs.py / position_monitor.py của hệ thống chính.
- Vị trí rBS trong Blender đã được chốt:
    X = 49.79 m
    Y = -10.83 m
    Z = 18.419 m

Quy ước:
    X/Y: mặt bằng scene
    Z: độ cao trong scene
"""

# ============================================================
# GPS CỐ ĐỊNH CỦA rBS
# ============================================================
# Đồng bộ với gps_rbs.py / position_monitor.py hiện tại.
# Đây vẫn là tọa độ tạm của rBS trong code chính; khi đo GPS rBS
# ngoài trời chính xác hơn, chỉ cần thay hai giá trị này.
RBS_LAT = 20.9807467
RBS_LON = 105.7962650

# ============================================================
# VỊ TRÍ rBS TRONG DIGITAL TWIN / BLENDER
# ============================================================
RBS_SCENE_X_M = 49.79
RBS_SCENE_Y_M = -10.83

# Z của rBS trong scene. rBS thực tế đang đặt trên cao.
RBS_HEIGHT_M = 18.419

# Chiều cao node SU/DU trong scene
SU_HEIGHT_M = 2.0
DU_HEIGHT_M = 2.0

# ============================================================
# HƯỚNG MAP
# ============================================================
# Tạm giữ:
#   East  -> +X
#   North -> +Y
#
# Sau khi marker GPS nằm đúng vị trí tương đối nhưng sai hướng,
# mới hiệu chỉnh hai tham số này.
MAP_ROTATION_DEG = 0.0
FLIP_NORTH = False
