#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SIONNA EVE CHANNEL SERVER - CFR API - WINDOWS

Chạy trên máy Windows đang có Sionna RT + campus 3D.

Nhiệm vụ:
- Nhận vị trí Scene XYZ của EVE, SU, DU, rBS từ Raspberry Pi qua HTTP.
- Dùng pipeline Sionna campus CHÍNH:
      run_campus_sionna_433mhz_FIXED_v2.py
- Tính 3 kênh ảo:
      rBS -> EVE
      SU  -> EVE
      DU  -> EVE
- Gọi trực tiếp API Sionna RT:
      paths.cfr(...)
  để lấy Channel Frequency Response.
- Không tự cộng paths.a và paths.tau bằng công thức tay nữa.
- Tính tại baseband offset 0 Hz, tức đáp ứng kênh tại sóng mang hiện tại.

Trả về:
    |H|
    |H|^2
    H_dB = 10 log10(|H|^2)
    số path hợp lệ
    delay spread cơ bản

Không thay thế 3 kênh đo thật SU-rBS, DU-rBS, SU-DU.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import traceback
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any

import numpy as np

BASE_DIR = Path(__file__).resolve().parent

# Pipeline Sionna campus đã PASS của project.
try:
    from run_campus_sionna_433mhz_FIXED_v2 import (
        build_scene,
        configure_radio_devices,
        solve_paths,
    )
except ImportError as exc:
    raise ImportError(
        "Không import được run_campus_sionna_433mhz_FIXED_v2.py.\n"
        "Hãy đặt sionna_eve_server_v3_cfr.py cùng thư mục D:\\hvktmm3_d với file pipeline."
    ) from exc

# Pipeline đã set Mitsuba variant = llvm_ad_mono_polarized.
# Import sau pipeline để dùng đúng variant CPU LLVM.
import mitsuba as mi


DEFAULT_GLB = BASE_DIR / "campus_chinh_chieu_cao.glb"
DEFAULT_CACHE = BASE_DIR / "sionna_meshes_cache"

C0 = 299_792_458.0


def tensor_numpy(x) -> np.ndarray:
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def vector_last_axis(x) -> np.ndarray:
    """Giữ link đầu tiên, trục cuối là path."""
    a = tensor_numpy(x)
    if a.ndim == 1:
        return a
    return a.reshape(-1, a.shape[-1])[0]


def safe_db10(x: float):
    if x <= 0.0 or not np.isfinite(x):
        return None
    return 10.0 * math.log10(x)


def validate_point(name: str, point: Any) -> np.ndarray | None:
    if point is None:
        return None
    if not isinstance(point, dict):
        raise ValueError(f"{name} phải là object JSON {{x,y,z}}")
    vals = [point.get("x"), point.get("y"), point.get("z")]
    if any(v is None for v in vals):
        raise ValueError(f"{name} thiếu x/y/z")
    arr = np.asarray(vals, dtype=np.float64)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} có tọa độ không hợp lệ: {vals}")
    return arr


def _cfr_to_complex_numpy(cfr) -> np.ndarray:
    """
    Chuẩn hóa output của paths.cfr() về ndarray complex.

    Sionna RT có thể trả:
    - tuple/list (real, imag), hoặc
    - ndarray complex, tùy version / out_type.
    """
    if isinstance(cfr, (tuple, list)) and len(cfr) == 2:
        real = np.asarray(cfr[0], dtype=np.float64)
        imag = np.asarray(cfr[1], dtype=np.float64)
        return real + 1j * imag

    arr = np.asarray(cfr)

    if np.iscomplexobj(arr):
        return arr.astype(np.complex128, copy=False)

    # Một số backend có thể stack real/imag trên trục đầu.
    if arr.ndim >= 1 and arr.shape[0] == 2:
        return (
            np.asarray(arr[0], dtype=np.float64)
            + 1j * np.asarray(arr[1], dtype=np.float64)
        )

    raise TypeError(
        f"Không nhận dạng được output paths.cfr(): "
        f"dtype={arr.dtype}, shape={arr.shape}"
    )


def extract_channel_from_paths(paths, distance_m: float) -> dict:
    """
    Lấy hệ số kênh trực tiếp bằng API Sionna RT Paths.cfr().

    Sionna định nghĩa CFR baseband tương đương:
        h_hat(f,t) = sum_i a_i^b(t) exp(-j 2π f tau_i)

    Trong đó cir() đã đưa carrier phase vào a_i^b.
    Vì vậy frequencies=[0 Hz] cho đáp ứng tại sóng mang hiện tại.

    Quan trọng:
    - normalize_delays=False: giữ delay vật lý.
    - normalize=False: KHÔNG chuẩn hóa năng lượng kênh.
      Nếu normalize=True thì mất ý nghĩa gain/path-loss thực tế.
    """
    # Kiểm tra có ít nhất một path hợp lệ để báo lỗi rõ ràng.
    tau_all = vector_last_axis(paths.tau).astype(np.float64)
    valid_all = vector_last_axis(paths.valid).astype(bool)

    valid_mask = valid_all & np.isfinite(tau_all) & (tau_all >= 0.0)
    idx = np.where(valid_mask)[0]

    if len(idx) == 0:
        return {
            "ok": False,
            "reason": "NO_VALID_PATH",
            "distance_m": float(distance_m),
            "path_count": 0,
            "h_abs": None,
            "h2": None,
            "h_db": None,
        }

    # Một điểm tần số duy nhất: offset 0 Hz quanh carrier của scene.
    frequencies = mi.Float([0.0])

    cfr = paths.cfr(
        frequencies=frequencies,
        sampling_frequency=1.0,
        num_time_steps=1,
        normalize_delays=False,
        normalize=False,
        reverse_direction=False,
        out_type="numpy",
    )

    h_np = _cfr_to_complex_numpy(cfr)
    h_flat = np.asarray(h_np).reshape(-1)

    if h_flat.size == 0:
        return {
            "ok": False,
            "reason": "EMPTY_CFR",
            "distance_m": float(distance_m),
            "path_count": int(len(idx)),
            "h_abs": None,
            "h2": None,
            "h_db": None,
        }

    # Scene hiện tại chỉ có 1 TX, 1 RX, mỗi bên 1 anten V,
    # 1 frequency và 1 time step => phần tử hữu ích là duy nhất.
    h_complex = complex(h_flat[0])

    h_abs = float(abs(h_complex))
    h2 = float(h_abs * h_abs)
    h_db = safe_db10(h2)

    # Delay statistics vẫn lấy từ paths.tau để tiện kiểm chứng.
    tau = tau_all[idx]

    return {
        "ok": True,
        "mode": "SIONNA_RT_CFR",
        "cfr_api": "paths.cfr",
        "frequency_offset_hz": 0.0,
        "distance_m": float(distance_m),
        "path_count": int(len(idx)),
        "h_real": float(h_complex.real),
        "h_imag": float(h_complex.imag),
        "h_abs": h_abs,
        "h2": h2,
        "h_db": h_db,
        "min_delay_s": float(np.min(tau)),
        "max_delay_s": float(np.max(tau)),
    }


class SionnaEveEngine:
    def __init__(
        self,
        glb_path: Path,
        cache_dir: Path,
        samples: int,
        max_depth: int,
        seed: int,
    ):
        self.glb_path = glb_path
        self.cache_dir = cache_dir
        self.samples = int(samples)
        self.max_depth = int(max_depth)
        self.seed = int(seed)

        if not self.glb_path.exists():
            raise FileNotFoundError(f"Không tìm thấy GLB: {self.glb_path}")

        self.cache_dir.mkdir(parents=True, exist_ok=True)

    def solve_one(self, link_name: str, tx_pos: np.ndarray, rx_pos: np.ndarray) -> dict:
        print()
        print("=" * 88)
        print(f"[SIONNA EVE] {link_name}")
        print(f"[TX] {tx_pos.tolist()}")
        print(f"[RX] {rx_pos.tolist()}")
        print("=" * 88)

        # Giữ đúng cách bridge hiện tại: mỗi link tạo scene riêng
        # để không trùng tên tx/rx trong Sionna.
        scene, bmin, bmax, center = build_scene(self.glb_path, self.cache_dir)

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
            samples=self.samples,
            max_depth=self.max_depth,
            seed=self.seed,
        )

        result = extract_channel_from_paths(paths, distance_m)
        result["link"] = link_name
        result["tx_xyz"] = [float(v) for v in tx_pos]
        result["rx_xyz"] = [float(v) for v in rx_pos]

        if result.get("ok"):
            print(
                f"[KẾT QUẢ CFR] {link_name}: "
                f"H={result['h_db']:.3f} dB | "
                f"|H|={result['h_abs']:.6e} | "
                f"paths={result['path_count']} | "
                f"API=paths.cfr()"
            )
        else:
            print(f"[KẾT QUẢ] {link_name}: KHÔNG CÓ PATH HỢP LỆ")

        return result

    def solve_request(self, payload: dict) -> dict:
        eve = validate_point("eve", payload.get("eve"))
        if eve is None:
            raise ValueError("Thiếu vị trí EVE")

        rbs = validate_point("rbs", payload.get("rbs"))
        su = validate_point("su", payload.get("su"))
        du = validate_point("du", payload.get("du"))

        channels = {}

        # Đúng bài toán nghe lén: node hợp pháp là TX, EVE là RX.
        if rbs is not None:
            channels["RBS_EVE"] = self.solve_one("rBS->EVE", rbs, eve)

        if su is not None:
            channels["SU_EVE"] = self.solve_one("SU->EVE", su, eve)

        if du is not None:
            channels["DU_EVE"] = self.solve_one("DU->EVE", du, eve)

        return {
            "ok": True,
            "backend": "SIONNA_RT_CFR",
            "frequency_hz": 433_000_000.0,
            "samples": self.samples,
            "max_depth": self.max_depth,
            "channels": channels,
        }


ENGINE: SionnaEveEngine | None = None


class Handler(BaseHTTPRequestHandler):
    server_version = "SionnaEveHTTP/1.0"

    def log_message(self, fmt, *args):
        print("[HTTP]", fmt % args)

    def send_json(self, status: int, data: dict):
        raw = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        if self.path.rstrip("/") == "/health":
            self.send_json(
                200,
                {
                    "ok": True,
                    "service": "sionna-eve-channel",
                    "backend": "SIONNA_RT_CFR",
                },
            )
            return
        self.send_json(404, {"ok": False, "error": "NOT_FOUND"})

    def do_POST(self):
        if self.path.rstrip("/") != "/eve-channel":
            self.send_json(404, {"ok": False, "error": "NOT_FOUND"})
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 1_000_000:
                raise ValueError("Content-Length không hợp lệ")

            payload = json.loads(self.rfile.read(length).decode("utf-8"))

            if ENGINE is None:
                raise RuntimeError("ENGINE chưa được khởi tạo")

            result = ENGINE.solve_request(payload)
            self.send_json(200, result)

        except Exception as exc:
            traceback.print_exc()
            self.send_json(
                500,
                {
                    "ok": False,
                    "error": type(exc).__name__,
                    "message": str(exc),
                },
            )


def parse_args():
    p = argparse.ArgumentParser(
        description="HTTP server tính 3 kênh EVE bằng Sionna RT Paths.cfr()"
    )
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--glb", default=str(DEFAULT_GLB))
    p.add_argument("--cache-dir", default=str(DEFAULT_CACHE))
    p.add_argument("--samples", type=int, default=20_000)
    p.add_argument("--max-depth", type=int, default=3)
    p.add_argument("--seed", type=int, default=42)
    return p.parse_args()


def main():
    global ENGINE

    args = parse_args()

    glb = Path(args.glb)
    if not glb.is_absolute():
        glb = BASE_DIR / glb

    cache_dir = Path(args.cache_dir)
    if not cache_dir.is_absolute():
        cache_dir = BASE_DIR / cache_dir

    ENGINE = SionnaEveEngine(
        glb_path=glb,
        cache_dir=cache_dir,
        samples=args.samples,
        max_depth=args.max_depth,
        seed=args.seed,
    )

    httpd = HTTPServer((args.host, args.port), Handler)

    print("=" * 88)
    print("SIONNA EVE CHANNEL SERVER - CFR API")
    print("=" * 88)
    print(f"[GLB]       {glb}")
    print(f"[CACHE]     {cache_dir}")
    print(f"[SAMPLES]   {args.samples}")
    print(f"[MAX DEPTH] {args.max_depth}")
    print(f"[LISTEN]    http://{args.host}:{args.port}")
    print(f"[HEALTH]    http://127.0.0.1:{args.port}/health")
    print("Để dừng: Ctrl+C")
    print("=" * 88)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[SERVER] Dừng.")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
