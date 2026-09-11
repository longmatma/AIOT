#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SIONNA EVE CHANNEL SERVER - GPU 2.1.0 / CUDA / SCENE ONCE / ADAPTIVE 20K - WINDOWS

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
import importlib.metadata as importlib_metadata
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any

import numpy as np

BASE_DIR = Path(__file__).resolve().parent

# ============================================================
# GPU VARIANT PHẢI ĐƯỢC CHỌN TRƯỚC KHI IMPORT SIONNA RT
# ============================================================
# Pipeline campus cũ của project từng ép:
#     mi.set_variant("llvm_ad_mono_polarized")
# và DRJIT_LIBLLVM_PATH.
#
# Nếu import pipeline đó trước rồi mới đổi sang CUDA thì các lớp
# RadioMaterial / Mitsuba có thể bị tạo ở variant LLVM, sau đó scene
# lại được load ở CUDA -> lỗi kiểu:
#     Found 1 unreferenced property ... "bsdf": RadioMaterial
#
# Vì vậy V10:
#   1) chọn CUDA trước;
#   2) đọc source pipeline campus;
#   3) vô hiệu hóa dòng ép LLVM trong bản source chạy trong RAM;
#   4) exec pipeline sau khi CUDA đã active.
import re
import types

import mitsuba as mi
import drjit as dr

MITSUBA_VARIANT = os.environ.get(
    "SIONNA_MITSUBA_VARIANT",
    "cuda_ad_mono_polarized",
)

if MITSUBA_VARIANT not in mi.variants():
    raise RuntimeError(
        f"Mitsuba không có variant yêu cầu: {MITSUBA_VARIANT}. "
        f"Variants hiện có: {mi.variants()}"
    )

if MITSUBA_VARIANT.startswith("cuda"):
    if not dr.has_backend(dr.JitBackend.CUDA):
        raise RuntimeError(
            "Dr.Jit CUDA backend không khả dụng. "
            "Hãy kiểm tra NVIDIA driver/CUDA."
        )

# QUAN TRỌNG: set CUDA trước khi bất kỳ import sionna.rt nào xảy ra.
mi.set_variant(MITSUBA_VARIANT)

PIPELINE_FILE = BASE_DIR / "run_campus_sionna_433mhz_FIXED_v2.py"

if not PIPELINE_FILE.exists():
    raise FileNotFoundError(
        f"Không tìm thấy pipeline campus: {PIPELINE_FILE}"
    )

pipeline_source = PIPELINE_FILE.read_text(
    encoding="utf-8",
    errors="replace",
)

# Vô hiệu hóa DRJIT_LIBLLVM_PATH trong bản pipeline chạy trong RAM.
pipeline_source = re.sub(
    r'(?m)^\s*os\.environ\[\s*["\']DRJIT_LIBLLVM_PATH["\']\s*\]\s*=.*$',
    '# [V10 GPU] DRJIT_LIBLLVM_PATH disabled',
    pipeline_source,
)

# Thay mọi lệnh ép LLVM bằng CUDA. Không sửa file gốc trên ổ đĩa.
pipeline_source = pipeline_source.replace(
    'mi.set_variant("llvm_ad_mono_polarized")',
    'mi.set_variant("cuda_ad_mono_polarized")',
)
pipeline_source = pipeline_source.replace(
    "mi.set_variant('llvm_ad_mono_polarized')",
    "mi.set_variant('cuda_ad_mono_polarized')",
)

pipeline_module = types.ModuleType("_campus_pipeline_cuda_runtime")
pipeline_module.__file__ = str(PIPELINE_FILE)
pipeline_module.__package__ = ""
sys.modules[pipeline_module.__name__] = pipeline_module

exec(
    compile(
        pipeline_source,
        str(PIPELINE_FILE),
        "exec",
    ),
    pipeline_module.__dict__,
)

build_scene = pipeline_module.build_scene
configure_radio_devices = pipeline_module.configure_radio_devices
solve_paths = pipeline_module.solve_paths

if mi.variant() != MITSUBA_VARIANT:
    raise RuntimeError(
        "Pipeline campus đã đổi Mitsuba variant ngoài ý muốn: "
        f"{mi.variant()} != {MITSUBA_VARIANT}"
    )

print(
    f"[GPU IMPORT ORDER] PASS | Mitsuba={mi.variant()} "
    "| pipeline campus được nạp sau CUDA"
)
try:
    print(
        "[GPU STACK] "
        f"sionna-rt={importlib_metadata.version('sionna-rt')} | "
        f"mitsuba={importlib_metadata.version('mitsuba')} | "
        f"drjit={importlib_metadata.version('drjit')}"
    )
except Exception:
    pass


DEFAULT_GLB = BASE_DIR / "campus_chinh_chieu_cao.glb"
DEFAULT_CACHE = BASE_DIR / "sionna_meshes_cache"

C0 = 299_792_458.0


def tensor_numpy(x) -> np.ndarray:
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def vector_last_axis(x) -> np.ndarray:
    """
    Giữ link đầu tiên, trục cuối là path.

    Quan trọng:
    Sionna có thể trả tensor rỗng khi PathSolver không tìm thấy
    propagation path hợp lệ. Không được reshape(..., 0).
    """
    a = tensor_numpy(x)

    if a.size == 0:
        return np.asarray([], dtype=a.dtype)

    if a.ndim == 0:
        return a.reshape(1)

    if a.ndim == 1:
        return a

    if a.shape[-1] == 0:
        return np.asarray([], dtype=a.dtype)

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

    # PathSolver có thể hoàn tất bình thường nhưng không tìm thấy path.
    # Đây KHÔNG phải lỗi server.
    if tau_all.size == 0 or valid_all.size == 0:
        return {
            "ok": False,
            "reason": "NO_VALID_PATH",
            "distance_m": float(distance_m),
            "path_count": 0,
            "h_abs": None,
            "h2": None,
            "h_db": None,
        }

    # Phòng trường hợp backend trả shape khác nhau.
    n = min(tau_all.size, valid_all.size)
    tau_all = tau_all[:n]
    valid_all = valid_all[:n]

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
        retry_samples_1: int,
        retry_depth_1: int,
        retry_samples_2: int,
        retry_depth_2: int,
    ):
        self.glb_path = glb_path
        self.cache_dir = cache_dir

        # Mức LIVE nhanh
        self.samples = int(samples)
        self.max_depth = int(max_depth)
        self.seed = int(seed)

        # Hai mức retry chỉ dùng khi NO_VALID_PATH
        self.retry_samples_1 = int(retry_samples_1)
        self.retry_depth_1 = int(retry_depth_1)
        self.retry_samples_2 = int(retry_samples_2)
        self.retry_depth_2 = int(retry_depth_2)

        if not self.glb_path.exists():
            raise FileNotFoundError(f"Không tìm thấy GLB: {self.glb_path}")

        self.cache_dir.mkdir(parents=True, exist_ok=True)

        # ====================================================
        # TỐI ƯU QUAN TRỌNG: LOAD CAMPUS ĐÚNG 1 LẦN
        # ====================================================
        # V8 build lại toàn bộ GLB cho rBS->EVE, SU->EVE, DU->EVE.
        # V9 build scene khi server khởi động, sau đó chỉ di chuyển
        # cùng một cặp TX/RX để tính từng link.
        if mi.variant() != MITSUBA_VARIANT:
            mi.set_variant(MITSUBA_VARIANT)

        print()
        print("=" * 88)
        print("[SCENE ONCE] ĐANG NẠP CAMPUS 3D MỘT LẦN KHI KHỞI ĐỘNG SERVER")
        print("=" * 88)

        t0 = time.perf_counter()
        self.scene, self.bmin, self.bmax, self.center = build_scene(
            self.glb_path,
            self.cache_dir,
        )
        self.scene_load_seconds = time.perf_counter() - t0

        # TX/RX chỉ được tạo một lần ở request đầu tiên.
        # Từ link thứ hai trở đi và các request sau chỉ đổi position.
        self.tx = None
        self.rx = None
        self.device_configured = False

        print(
            f"[SCENE ONCE] CAMPUS ĐÃ NẠP XONG "
            f"| {self.scene_load_seconds:.2f}s"
        )
        print(
            "[SCENE ONCE] Từ giờ KHÔNG build lại GLB cho từng link/click."
        )

    def set_link_positions(
        self,
        link_name: str,
        tx_pos: np.ndarray,
        rx_pos: np.ndarray,
    ) -> float:
        """
        Tạo TX/RX đúng 1 lần, sau đó chỉ cập nhật position.

        Đây là điểm giúp tránh:
            build_scene() x 3 link
        và cũng tránh build lại scene cho mỗi lần click EVE.
        """
        tx_pos = np.asarray(tx_pos, dtype=np.float64)
        rx_pos = np.asarray(rx_pos, dtype=np.float64)
        distance_m = float(np.linalg.norm(rx_pos - tx_pos))

        if not self.device_configured:
            self.tx, self.rx, _, _, _ = configure_radio_devices(
                scene=self.scene,
                bmin=self.bmin,
                bmax=self.bmax,
                center=self.center,
                distance_m=distance_m,
                height_m=0.0,
                tx_override=tx_pos.tolist(),
                rx_override=rx_pos.tolist(),
            )
            self.device_configured = True
            print("[SCENE ONCE] TX/RX đã tạo một lần và sẽ được tái sử dụng.")
        else:
            # Sionna RT cho phép cập nhật vị trí radio device.
            self.tx.position = tx_pos.tolist()
            self.rx.position = rx_pos.tolist()

            try:
                self.tx.look_at(self.rx)
            except Exception:
                pass

            try:
                self.rx.look_at(self.tx)
            except Exception:
                pass

            print()
            print("=" * 88)
            print(f"[REUSE SCENE] {link_name}")
            print(f"[TX MOVE] {tx_pos.tolist()}")
            print(f"[RX MOVE] {rx_pos.tolist()}")
            print(f"[LINK] Khoảng cách Euclid = {distance_m:.3f} m")
            print("=" * 88)

        return distance_m

    def solve_one(self, link_name: str, tx_pos: np.ndarray, rx_pos: np.ndarray) -> dict:
        print()
        print("=" * 88)
        print(f"[SIONNA EVE] {link_name}")
        print(f"[TX] {tx_pos.tolist()}")
        print(f"[RX] {rx_pos.tolist()}")
        print("=" * 88)

        # Scene đã được load một lần trong __init__.
        # Ở đây chỉ cập nhật vị trí TX/RX.
        if mi.variant() != MITSUBA_VARIANT:
            mi.set_variant(MITSUBA_VARIANT)

        distance_m = self.set_link_positions(
            link_name,
            tx_pos,
            rx_pos,
        )

        attempts = [
            {
                "name": "LIVE",
                "samples": self.samples,
                "max_depth": self.max_depth,
                "seed": self.seed,
            },
            {
                "name": "RETRY_1",
                "samples": self.retry_samples_1,
                "max_depth": self.retry_depth_1,
                "seed": self.seed + 101,
            },
            {
                "name": "RETRY_2",
                "samples": self.retry_samples_2,
                "max_depth": self.retry_depth_2,
                "seed": self.seed + 202,
            },
        ]

        last_result = None
        attempt_history = []

        for attempt_no, cfg in enumerate(attempts, start=1):
            print(
                f"[ADAPTIVE] {link_name} | {cfg['name']} "
                f"| samples={cfg['samples']:,} "
                f"| depth={cfg['max_depth']} "
                f"| seed={cfg['seed']}"
            )

            paths = solve_paths(
                scene=self.scene,
                samples=cfg["samples"],
                max_depth=cfg["max_depth"],
                seed=cfg["seed"],
            )

            result = extract_channel_from_paths(paths, distance_m)

            attempt_history.append(
                {
                    "attempt": attempt_no,
                    "name": cfg["name"],
                    "samples": int(cfg["samples"]),
                    "max_depth": int(cfg["max_depth"]),
                    "seed": int(cfg["seed"]),
                    "ok": bool(result.get("ok")),
                    "reason": result.get("reason"),
                    "path_count": int(result.get("path_count") or 0),
                }
            )

            last_result = result

            if result.get("ok"):
                result["link"] = link_name
                result["tx_xyz"] = [float(v) for v in tx_pos]
                result["rx_xyz"] = [float(v) for v in rx_pos]
                result["adaptive_retry"] = attempt_no > 1
                result["attempt_used"] = attempt_no
                result["samples_used"] = int(cfg["samples"])
                result["max_depth_used"] = int(cfg["max_depth"])
                result["seed_used"] = int(cfg["seed"])
                result["attempt_history"] = attempt_history

                print(
                    f"[KẾT QUẢ CFR] {link_name}: "
                    f"H={result['h_db']:.3f} dB | "
                    f"|H|={result['h_abs']:.6e} | "
                    f"paths={result['path_count']} | "
                    f"attempt={attempt_no} | "
                    f"samples={cfg['samples']:,} | "
                    f"depth={cfg['max_depth']} | "
                    f"API=paths.cfr()"
                )
                return result

            reason = result.get("reason")
            if reason not in ("NO_VALID_PATH", "EMPTY_CFR"):
                # Không retry các lỗi logic khác.
                break

            if attempt_no < len(attempts):
                next_cfg = attempts[attempt_no]
                print(
                    f"[ADAPTIVE RETRY] {link_name}: {reason} "
                    f"-> thử {next_cfg['samples']:,} samples / "
                    f"depth={next_cfg['max_depth']}"
                )

        # Hết tất cả mức retry vẫn không có path
        result = dict(last_result or {})
        result["link"] = link_name
        result["tx_xyz"] = [float(v) for v in tx_pos]
        result["rx_xyz"] = [float(v) for v in rx_pos]
        result["adaptive_retry"] = True
        result["attempt_used"] = len(attempt_history)
        result["samples_used"] = attempt_history[-1]["samples"] if attempt_history else self.samples
        result["max_depth_used"] = attempt_history[-1]["max_depth"] if attempt_history else self.max_depth
        result["seed_used"] = attempt_history[-1]["seed"] if attempt_history else self.seed
        result["attempt_history"] = attempt_history

        print(
            f"[KẾT QUẢ CFR] {link_name}: NO_VALID_PATH "
            f"sau {len(attempt_history)} lần thử | "
            f"không làm lỗi toàn request"
        )
        return result

    def solve_request(self, payload: dict) -> dict:
        eve = validate_point("eve", payload.get("eve"))
        if eve is None:
            raise ValueError("Thiếu vị trí EVE")

        rbs = validate_point("rbs", payload.get("rbs"))
        su = validate_point("su", payload.get("su"))
        du = validate_point("du", payload.get("du"))

        channels = {}

        def run_link(key, label, tx_pos):
            if tx_pos is None:
                return
            try:
                channels[key] = self.solve_one(label, tx_pos, eve)
            except Exception as exc:
                # Một link lỗi/không có path không được làm mất kết quả
                # của hai link còn lại.
                traceback.print_exc()
                channels[key] = {
                    "ok": False,
                    "reason": "LINK_EXCEPTION",
                    "message": f"{type(exc).__name__}: {exc}",
                    "h_abs": None,
                    "h2": None,
                    "h_db": None,
                }
                print(
                    f"[KẾT QUẢ CFR] {label}: LINK_EXCEPTION "
                    f"| {type(exc).__name__}: {exc}"
                )

        # Đúng bài toán nghe lén: node hợp pháp là TX, EVE là RX.
        run_link("RBS_EVE", "rBS->EVE", rbs)
        run_link("SU_EVE", "SU->EVE", su)
        run_link("DU_EVE", "DU->EVE", du)

        solved_count = sum(
            1 for ch in channels.values()
            if isinstance(ch, dict) and ch.get("ok")
        )
        total_count = len(channels)

        if total_count > 0 and solved_count == total_count:
            channel_status = "FULL"
        elif solved_count > 0:
            channel_status = "PARTIAL"
        else:
            channel_status = "NO_PATH"

        return {
            "ok": True,
            "backend": "SIONNA_RT_2_1_CFR_CUDA_SCENE_ONCE",
            "mitsuba_variant": mi.variant(),
            "scene_reuse": True,
            "gpu_import_order_safe": True,
            "scene_load_seconds": self.scene_load_seconds,
            "adaptive_retry": True,
            "channel_status": channel_status,
            "solved_links": solved_count,
            "total_links": total_count,
            "frequency_hz": 433_000_000.0,
            "samples": self.samples,
            "max_depth": self.max_depth,
            "retry_1": {
                "samples": self.retry_samples_1,
                "max_depth": self.retry_depth_1,
            },
            "retry_2": {
                "samples": self.retry_samples_2,
                "max_depth": self.retry_depth_2,
            },
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
                    "backend": "SIONNA_RT_2_1_CFR_CUDA_SCENE_ONCE",
                    "mitsuba_variant": mi.variant(),
                    "cuda_available": bool(
                        dr.has_backend(dr.JitBackend.CUDA)
                    ),
                    "scene_reuse": True,
                    "scene_loaded": bool(
                        ENGINE is not None
                        and getattr(ENGINE, "scene", None) is not None
                    ),
                    "scene_load_seconds": (
                        getattr(ENGINE, "scene_load_seconds", None)
                        if ENGINE is not None
                        else None
                    ),
                    "adaptive_retry": True,
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
    p.add_argument("--samples", type=int, default=2_000)
    p.add_argument("--max-depth", type=int, default=2)
    p.add_argument("--seed", type=int, default=42)

    p.add_argument("--retry-samples-1", type=int, default=5_000)
    p.add_argument("--retry-depth-1", type=int, default=3)

    p.add_argument("--retry-samples-2", type=int, default=20_000)
    p.add_argument("--retry-depth-2", type=int, default=4)

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
        retry_samples_1=args.retry_samples_1,
        retry_depth_1=args.retry_depth_1,
        retry_samples_2=args.retry_samples_2,
        retry_depth_2=args.retry_depth_2,
    )

    httpd = HTTPServer((args.host, args.port), Handler)

    print("=" * 88)
    print("SIONNA EVE CHANNEL SERVER - GPU 2.1.0 / CUDA / SCENE ONCE / ADAPTIVE 20K")
    print("=" * 88)
    print(f"[GLB]       {glb}")
    print(f"[CACHE]     {cache_dir}")
    try:
        print(
            "[STACK]     "
            f"sionna-rt={importlib_metadata.version('sionna-rt')} | "
            f"mitsuba={importlib_metadata.version('mitsuba')} | "
            f"drjit={importlib_metadata.version('drjit')}"
        )
    except Exception:
        pass
    print(f"[MITSUBA]   {mi.variant()}")
    print(
        f"[CUDA]      available="
        f"{bool(dr.has_backend(dr.JitBackend.CUDA))}"
    )
    print("[SCENE]     load 1 lần khi server start, tái sử dụng cho mọi link/click")
    print(f"[LIVE]      samples={args.samples:,} | depth={args.max_depth}")
    print(
        f"[RETRY 1]   samples={args.retry_samples_1:,} "
        f"| depth={args.retry_depth_1}"
    )
    print(
        f"[RETRY 2]   samples={args.retry_samples_2:,} "
        f"| depth={args.retry_depth_2}"
    )
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
