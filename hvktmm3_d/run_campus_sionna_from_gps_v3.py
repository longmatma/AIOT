#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GPS -> SIONNA RT BRIDGE CHO CAMPUS 433 MHz - V3

File này ghép 2 phần của project:
1) Logic GPS/rBS/SU/DU từ position_monitor.py
2) Pipeline Sionna campus mới từ run_campus_sionna_433mhz_FIXED_v2.py

Cách hoạt động:
- Đọc dòng GPS hợp lệ mới nhất của SU và/hoặc DU từ CSV.
- rBS là mốc GPS cố định.
- Đổi GPS -> East/North theo mét.
- Ánh xạ East/North -> X/Y của scene 3D.
- Đặt rBS/SU/DU thành TX/RX thật trong Sionna.
- Chạy ray tracing và xuất CSV multipath.

LƯU Ý:
- Không chạy ray tracing mỗi 300 ms như GUI position_monitor.py.
  Sionna nặng hơn nhiều. File này chạy "snapshot" khi bạn gọi lệnh.
- Muốn kiểm tra tọa độ trước mà chưa ray tracing, dùng --dry-run.

Ví dụ:
    python run_campus_sionna_from_gps.py ^
        --gps-csv "D:\\duong_dan\\rbs_lien_ket_dinh_ky_v11.csv" ^
        --device SU ^
        --dry-run

Chạy thật rBS -> SU:
    python run_campus_sionna_from_gps.py ^
        --gps-csv "D:\\duong_dan\\rbs_lien_ket_dinh_ky_v11.csv" ^
        --device SU ^
        --direction downlink ^
        --samples 20000 ^
        --max-depth 3

Chạy rBS -> cả SU và DU:
    python run_campus_sionna_from_gps.py ^
        --gps-csv "D:\\duong_dan\\rbs_lien_ket_dinh_ky_v11.csv" ^
        --device BOTH ^
        --samples 20000 ^
        --max-depth 3
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from datetime import datetime
from bisect import bisect_left
from pathlib import Path

import numpy as np

# Cấu hình địa lý của campus
try:
    from campus_geo_config import (
        RBS_LAT,
        RBS_LON,
        RBS_SCENE_X_M,
        RBS_SCENE_Y_M,
        RBS_HEIGHT_M,
        SU_HEIGHT_M,
        DU_HEIGHT_M,
        MAP_ROTATION_DEG,
        FLIP_NORTH,
    )
except ImportError:
    raise ImportError(
        "Không tìm thấy campus_geo_config.py. "
        "Hãy đặt file này cùng thư mục với script."
    )

# Tái sử dụng pipeline Sionna campus đã chạy thành công
try:
    from run_campus_sionna_433mhz_FIXED_v2 import (
        build_scene,
        configure_radio_devices,
        solve_paths,
        analyze_paths,
    )
except ImportError:
    raise ImportError(
        "Không tìm thấy run_campus_sionna_433mhz_FIXED_v2.py. "
        "Hãy đặt file đó cùng thư mục với script."
    )


EARTH_R = 6_371_000.0


# ============================================================
# GPS -> East/North cục bộ (m)
# Giống logic position_monitor.py
# ============================================================

def gps_to_local_en(lat: float, lon: float, lat0: float, lon0: float):
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    east = EARTH_R * (lon_rad - lon0_rad) * math.cos(lat0_rad)
    north = EARTH_R * (lat_rad - lat0_rad)
    return east, north


def haversine_distance(lat1, lon1, lat2, lon2):
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
    return EARTH_R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ============================================================
# East/North -> X/Y scene
# ============================================================

def en_to_scene_xy(east_m: float, north_m: float):
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

    x = RBS_SCENE_X_M + scene_dx
    y = RBS_SCENE_Y_M + scene_dy
    return x, y


def gps_to_scene(lat: float, lon: float, height_m: float):
    east, north = gps_to_local_en(
        lat,
        lon,
        RBS_LAT,
        RBS_LON,
    )
    x, y = en_to_scene_xy(east, north)
    return np.array([x, y, height_m], dtype=np.float64), east, north


# ============================================================
# ĐỌC CSV GPS + ĐỒNG BỘ THỜI GIAN
# ============================================================

def valid_coordinate(lat, lon):
    return (
        -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
        and not (lat == 0.0 and lon == 0.0)
    )


def parse_time_iso(text):
    try:
        return datetime.fromisoformat((text or "").strip().replace("Z", "+00:00"))
    except Exception:
        return None


def _safe_float(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _safe_int(value, default=None):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def read_valid_gps_rows(csv_path: Path):
    """
    Đọc toàn bộ bản ghi GPS hợp lệ của SU/DU.

    Trả về:
        {
            "SU": [record, ...],
            "DU": [record, ...],
        }

    Mỗi record giữ cả timestamp và tọa độ rBS được ghi trong CSV
    để phát hiện CSV cũ dùng anchor rBS khác với config hiện tại.
    """
    if not csv_path.exists():
        raise FileNotFoundError(f"Không thấy CSV GPS:\n  {csv_path}")

    rows = {"SU": [], "DU": []}

    with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)

        if not reader.fieldnames:
            raise RuntimeError("CSV GPS không có header.")

        required = {
            "Thiet_bi",
            "GPS_hop_le",
            "Vi_do_tho",
            "Kinh_do_tho",
            "Thoi_gian_rBS_nhan_UTC",
        }
        missing = required - set(reader.fieldnames)
        if missing:
            raise RuntimeError(
                "CSV thiếu các cột cần thiết: "
                + ", ".join(sorted(missing))
            )

        for row_number, row in enumerate(reader, start=2):
            device = (row.get("Thiet_bi") or "").strip().upper()
            if device not in ("SU", "DU"):
                continue

            if (row.get("GPS_hop_le") or "").strip() != "1":
                continue

            lat = _safe_float(row.get("Vi_do_tho"))
            lon = _safe_float(row.get("Kinh_do_tho"))
            ts = parse_time_iso(row.get("Thoi_gian_rBS_nhan_UTC"))

            if lat is None or lon is None or ts is None:
                continue

            if not valid_coordinate(lat, lon):
                continue

            rows[device].append({
                "device": device,
                "lat": lat,
                "lon": lon,
                "satellites": _safe_int(row.get("So_ve_tinh")),
                "hdop": _safe_float(row.get("HDOP")),
                "row_number": row_number,
                "time": (row.get("Thoi_gian_rBS_nhan_UTC") or "").strip(),
                "time_obj": ts,
                "csv_rbs_lat": _safe_float(row.get("Vi_do_rBS")),
                "csv_rbs_lon": _safe_float(row.get("Kinh_do_rBS")),
            })

    for device in ("SU", "DU"):
        rows[device].sort(key=lambda item: item["time_obj"])

    return rows


def latest_record(rows_by_device, device):
    records = rows_by_device.get(device, [])
    return records[-1] if records else None


def latest_synced_pair(rows_by_device, max_delta_sec=10.0):
    """
    Chọn cặp SU/DU MỚI NHẤT có độ lệch timestamp <= max_delta_sec.

    Khác với việc chỉ tìm cặp có delta nhỏ nhất trong toàn bộ lịch sử:
    hàm này ưu tiên snapshot mới nhất nhưng vẫn bắt buộc đồng bộ thời gian.
    """
    su_rows = rows_by_device.get("SU", [])
    du_rows = rows_by_device.get("DU", [])

    if not su_rows or not du_rows:
        return None, None, None

    su_times = [item["time_obj"] for item in su_rows]
    candidates = []

    for du in du_rows:
        idx = bisect_left(su_times, du["time_obj"])

        for j in (idx - 1, idx):
            if j < 0 or j >= len(su_rows):
                continue

            su = su_rows[j]
            delta = abs((su["time_obj"] - du["time_obj"]).total_seconds())

            if delta <= max_delta_sec:
                latest_time = max(su["time_obj"], du["time_obj"])
                candidates.append((latest_time, -delta, su, du, delta))

    if not candidates:
        return None, None, None

    # Snapshot mới nhất trước; nếu trùng thời điểm thì ưu tiên delta nhỏ hơn.
    candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
    _, _, su, du, delta = candidates[0]
    return su, du, delta


def warn_if_csv_anchor_differs(data):
    """
    CSV lịch sử có thể được ghi bằng một tọa độ rBS cũ.
    Chỉ cảnh báo; GPS thô SU/DU vẫn có thể được dùng với anchor hiện tại.
    """
    if not data:
        return

    csv_lat = data.get("csv_rbs_lat")
    csv_lon = data.get("csv_rbs_lon")

    if csv_lat is None or csv_lon is None:
        return

    diff_m = haversine_distance(
        RBS_LAT, RBS_LON,
        csv_lat, csv_lon,
    )

    if diff_m > 5.0:
        print(
            f"[CẢNH BÁO CSV] Bản ghi này được tạo với rBS cũ "
            f"({csv_lat:.7f}, {csv_lon:.7f}), "
            f"lệch anchor hiện tại khoảng {diff_m:.1f} m."
        )
        print(
            "[CẢNH BÁO CSV] Không dùng cột Khoang_cach_rBS_node_m cũ "
            "để kiểm chứng khoảng cách hiện tại; script sẽ tính lại từ GPS thô."
        )


# ============================================================
# HIỂN THỊ / KIỂM TRA VỊ TRÍ
# ============================================================

def print_position(device: str, data: dict):
    height = SU_HEIGHT_M if device == "SU" else DU_HEIGHT_M
    scene_pos, east, north = gps_to_scene(
        data["lat"],
        data["lon"],
        height,
    )

    dist = haversine_distance(
        RBS_LAT,
        RBS_LON,
        data["lat"],
        data["lon"],
    )

    print()
    print(f"[{device}] GPS")
    if data.get("time"):
        print(f"  Time UTC     = {data['time']}")
    if data.get("row_number") is not None:
        print(f"  CSV row      = {data['row_number']}")
    print(f"  Lat/Lon      = {data['lat']:.7f}, {data['lon']:.7f}")
    print(f"  Vệ tinh      = {data['satellites']}")
    print(f"  HDOP         = {data['hdop']}")
    print(f"  East         = {east:.3f} m")
    print(f"  North        = {north:.3f} m")
    print(f"  d(rBS,{device}) GPS = {dist:.3f} m")
    print(
        f"  Scene XYZ    = "
        f"({scene_pos[0]:.3f}, "
        f"{scene_pos[1]:.3f}, "
        f"{scene_pos[2]:.3f})"
    )

    warn_if_csv_anchor_differs(data)
    return scene_pos, dist


def inside_xy_bounds(pos, bmin, bmax):
    return (
        bmin[0] <= pos[0] <= bmax[0]
        and bmin[1] <= pos[1] <= bmax[1]
    )


# ============================================================
# CHẠY 1 LINK
# ============================================================

def run_one_link(
    base: Path,
    glb_path: Path,
    cache_dir: Path,
    device: str,
    device_pos: np.ndarray,
    direction: str,
    samples: int,
    max_depth: int,
    seed: int,
    tx_power_dbm: float,
    bandwidth_hz: float,
    noise_figure_db: float,
):
    print()
    print("=" * 92)
    print(f"SIONNA GPS LINK: {device} | direction={direction}")
    print("=" * 92)

    # Mỗi link build scene riêng để tránh trùng tên tx/rx
    scene, bmin, bmax, center = build_scene(glb_path, cache_dir)

    rbs_pos = np.array(
        [RBS_SCENE_X_M, RBS_SCENE_Y_M, RBS_HEIGHT_M],
        dtype=np.float64,
    )

    if direction == "downlink":
        tx_pos = rbs_pos
        rx_pos = device_pos
        link_name = f"rBS_to_{device}"
    else:
        tx_pos = device_pos
        rx_pos = rbs_pos
        link_name = f"{device}_to_rBS"

    if not inside_xy_bounds(rbs_pos, bmin, bmax):
        print(
            f"[CẢNH BÁO] rBS nằm ngoài X/Y scene: {rbs_pos[:2]}"
        )

    if not inside_xy_bounds(device_pos, bmin, bmax):
        print(
            f"[CẢNH BÁO] {device} nằm ngoài X/Y scene: "
            f"{device_pos[:2]}"
        )
        print(
            "[GỢI Ý] Kiểm tra RBS_SCENE_X/Y, "
            "MAP_ROTATION_DEG hoặc FLIP_NORTH "
            "trong campus_geo_config.py."
        )

    _, _, _, _, distance_m = configure_radio_devices(
        scene=scene,
        bmin=bmin,
        bmax=bmax,
        center=center,
        distance_m=float(np.linalg.norm(rx_pos - tx_pos)),
        height_m=0.0,
        tx_override=tx_pos.tolist(),
        rx_override=rx_pos.tolist(),
    )

    paths = solve_paths(
        scene=scene,
        samples=samples,
        max_depth=max_depth,
        seed=seed,
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = base / "gps_sionna_results"
    out_dir.mkdir(parents=True, exist_ok=True)

    csv_out = out_dir / (
        f"{link_name}_433mhz_{timestamp}.csv"
    )

    analyze_paths(
        scene=scene,
        paths=paths,
        distance_m=distance_m,
        tx_power_dbm=tx_power_dbm,
        bandwidth_hz=bandwidth_hz,
        noise_figure_db=noise_figure_db,
        output_csv=csv_out,
    )

    return csv_out


# ============================================================
# CLI
# ============================================================

def parse_args():
    p = argparse.ArgumentParser(
        description="GPS thật SU/DU -> tọa độ campus -> Sionna RT 433 MHz"
    )

    p.add_argument(
        "--gps-csv",
        default=os.environ.get(
            "RBS_GPS_CSV",
            "rbs_lien_ket_dinh_ky_v11.csv",
        ),
        help="CSV GPS do rBS/gateway ghi",
    )

    p.add_argument(
        "--glb",
        default="campus_chinh_chieu_cao.glb",
    )

    p.add_argument(
        "--device",
        choices=["SU", "DU", "BOTH"],
        default="SU",
    )

    p.add_argument(
        "--direction",
        choices=["downlink", "uplink"],
        default="downlink",
        help="downlink: rBS phát; uplink: SU/DU phát",
    )

    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Chỉ đọc GPS và in tọa độ scene, chưa chạy Sionna",
    )

    # Override GPS để test khi chưa có CSV live
    p.add_argument(
        "--su",
        type=float,
        nargs=2,
        metavar=("LAT", "LON"),
        default=None,
    )
    p.add_argument(
        "--du",
        type=float,
        nargs=2,
        metavar=("LAT", "LON"),
        default=None,
    )

    p.add_argument("--samples", type=int, default=20_000)
    p.add_argument("--max-depth", type=int, default=3)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--tx-power-dbm", type=float, default=30.0)
    p.add_argument("--bandwidth-hz", type=float, default=500_000.0)
    p.add_argument("--noise-figure-db", type=float, default=6.0)

    p.add_argument(
        "--sync-window-sec",
        type=float,
        default=10.0,
        help="Khi --device BOTH: độ lệch timestamp SU/DU tối đa cho phép",
    )

    return p.parse_args()


def main():
    args = parse_args()
    base = Path(__file__).resolve().parent

    glb_path = Path(args.glb)
    if not glb_path.is_absolute():
        glb_path = base / glb_path

    gps_csv = Path(args.gps_csv)
    if not gps_csv.is_absolute():
        gps_csv = base / gps_csv

    # Đọc CSV nếu cần
    rows_by_device = {"SU": [], "DU": []}
    if gps_csv.exists():
        rows_by_device = read_valid_gps_rows(gps_csv)
    elif args.su is None and args.du is None:
        raise FileNotFoundError(
            f"Không thấy CSV GPS:\n  {gps_csv}\n"
            "Có thể dùng --gps-csv đường_dẫn_đúng "
            "hoặc --su LAT LON / --du LAT LON để test."
        )

    selected = {}

    # Với BOTH: nếu không override cả hai bằng CLI thì ưu tiên cặp mới nhất
    # đã đồng bộ thời gian.
    if args.device == "BOTH" and not (args.su is not None and args.du is not None):
        su_sync, du_sync, pair_delta = latest_synced_pair(
            rows_by_device,
            max_delta_sec=args.sync_window_sec,
        )

        if su_sync is None or du_sync is None:
            raise RuntimeError(
                "Không tìm được cặp SU/DU đồng bộ trong "
                f"{args.sync_window_sec:.1f} giây. "
                "Hãy chạy SU/DU riêng hoặc tăng --sync-window-sec."
            )

        selected["SU"] = su_sync
        selected["DU"] = du_sync
        print(
            f"[TIME SYNC] Cặp SU/DU mới nhất: "
            f"delta={pair_delta:.3f} s"
        )
    else:
        if args.device in ("SU", "BOTH"):
            rec = latest_record(rows_by_device, "SU")
            if rec is not None:
                selected["SU"] = rec

        if args.device in ("DU", "BOTH"):
            rec = latest_record(rows_by_device, "DU")
            if rec is not None:
                selected["DU"] = rec

    # Override bằng CLI nếu có
    if args.su is not None:
        selected["SU"] = {
            "device": "SU",
            "lat": args.su[0],
            "lon": args.su[1],
            "satellites": None,
            "hdop": None,
            "row_number": None,
            "time": None,
            "time_obj": None,
            "csv_rbs_lat": None,
            "csv_rbs_lon": None,
        }

    if args.du is not None:
        selected["DU"] = {
            "device": "DU",
            "lat": args.du[0],
            "lon": args.du[1],
            "satellites": None,
            "hdop": None,
            "row_number": None,
            "time": None,
            "time_obj": None,
            "csv_rbs_lat": None,
            "csv_rbs_lon": None,
        }

    print("=" * 92)
    print("GPS -> CAMPUS SCENE -> SIONNA RT 433 MHz")
    print("=" * 92)
    print(
        f"[rBS GPS]   {RBS_LAT:.7f}, {RBS_LON:.7f}"
    )
    print(
        f"[rBS scene] X={RBS_SCENE_X_M:.3f}, "
        f"Y={RBS_SCENE_Y_M:.3f}, "
        f"Z={RBS_HEIGHT_M:.3f}"
    )
    print(f"[MAP] rotation = {MAP_ROTATION_DEG:.3f} deg")
    print(f"[MAP] flip north = {FLIP_NORTH}")
    if gps_csv.exists():
        print(f"[GPS CSV] {gps_csv}")

    requested = (
        ["SU", "DU"]
        if args.device == "BOTH"
        else [args.device]
    )

    positions = {}

    for device in requested:
        data = selected.get(device)
        if data is None:
            print(
                f"\n[CẢNH BÁO] Chưa tìm thấy GPS hợp lệ mới nhất "
                f"của {device}."
            )
            continue

        pos, gps_dist = print_position(device, data)
        positions[device] = pos

    if not positions:
        raise RuntimeError(
            "Không có thiết bị nào có GPS hợp lệ để chạy."
        )

    if args.dry_run:
        print()
        print("[DRY RUN] Chưa chạy ray tracing.")
        print(
            "Nếu các tọa độ Scene XYZ hợp lý, "
            "bỏ --dry-run để chạy Sionna."
        )
        return

    for device, pos in positions.items():
        cache_dir = base / f"sionna_meshes_cache_{device.lower()}"

        csv_out = run_one_link(
            base=base,
            glb_path=glb_path,
            cache_dir=cache_dir,
            device=device,
            device_pos=pos,
            direction=args.direction,
            samples=args.samples,
            max_depth=args.max_depth,
            seed=args.seed,
            tx_power_dbm=args.tx_power_dbm,
            bandwidth_hz=args.bandwidth_hz,
            noise_figure_db=args.noise_figure_db,
        )

        print()
        print(f"[DONE] {device}: {csv_out}")


if __name__ == "__main__":
    main()
