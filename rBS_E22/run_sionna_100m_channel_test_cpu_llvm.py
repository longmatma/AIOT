#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEST KÊNH SIONNA RT ~100 m TRÊN MESH OSM/BLENDER

Mặc định:
- Mesh mặc định: map_triangles.obj (đặt cùng thư mục với file này)
- f_c = 433 MHz
- TX/RX cách nhau 100 m theo trục X, đặt quanh tâm mesh
- Anten đẳng hướng, phân cực V
- Bật:
    + LOS
    + phản xạ gương (specular reflection)
    + phản xạ khuếch tán (diffuse reflection)
    + khúc xạ/truyền xuyên (refraction/transmission)
    + nhiễu xạ (diffraction)
    + edge diffraction

LƯU Ý QUAN TRỌNG
-----------------
File map_triangles.obj hiện là MỘT mesh đã gộp. Vì vậy bản test này gán MỘT
vật liệu vô tuyến "urban_433" cho toàn mesh để kiểm thử thuật toán.
Muốn mô phỏng đúng đất/nhà/đường/cây/nước, cần export chúng thành các
SceneObject/mesh riêng.

Sionna RT 2.0.x mô hình "refraction" như truyền xuyên qua một slab mỏng.
Đường ray không bị bẻ góc như mô hình quang học của vật thể dày.

Cài:
    pip install sionna-rt numpy

Chạy:
    python run_sionna_100m_channel_test.py

Ví dụ TX 30 dBm:
    python run_sionna_100m_channel_test.py --tx-power-dbm 30

Tăng độ chính xác ray tracing:
    python run_sionna_100m_channel_test.py --samples 2000000 --max-depth 5

Tự chọn TX/RX:
    python run_sionna_100m_channel_test.py \
        --tx -50 0 2 \
        --rx  50 0 2
"""

from __future__ import annotations

import os
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

# ============================================================
# WINDOWS: ÉP DR.JIT / MITSUBA CHẠY BẰNG CPU LLVM
# ============================================================
LLVM_C_DLL = r"C:\Program Files\LLVM\bin\LLVM-C.dll"

if sys.platform.startswith("win"):
    if not Path(LLVM_C_DLL).exists():
        raise FileNotFoundError(
            "Không tìm thấy LLVM-C.dll tại:\n"
            f"  {LLVM_C_DLL}\n"
            "Hãy kiểm tra lại thư mục cài LLVM."
        )
    os.environ["DRJIT_LIBLLVM_PATH"] = LLVM_C_DLL

import mitsuba as mi
mi.set_variant("llvm_ad_mono_polarized")

try:
    import sionna.rt as rt
    from sionna.rt import (
        load_scene,
        PlanarArray,
        Transmitter,
        Receiver,
        PathSolver,
        RadioMaterial,
        SceneObject,
    )
    from sionna.rt.constants import InteractionType
except ImportError as exc:
    print("[LỖI] Chưa cài Sionna RT.")
    print("      Chạy: pip install sionna-rt numpy")
    raise

C0 = 299_792_458.0


# ============================================================
# HÀM TIỆN ÍCH
# ============================================================

def db10(x: float) -> float:
    if x <= 0 or not np.isfinite(x):
        return -np.inf
    return 10.0 * math.log10(x)


def db20(x: float) -> float:
    if x <= 0 or not np.isfinite(x):
        return -np.inf
    return 20.0 * math.log10(x)


def scalar_float(x) -> float:
    """Ép Mitsuba/Dr.Jit scalar -> float."""
    try:
        return float(x)
    except Exception:
        a = np.asarray(x)
        return float(a.reshape(-1)[0])


def point3_to_np(p) -> np.ndarray:
    return np.array(
        [scalar_float(p.x), scalar_float(p.y), scalar_float(p.z)],
        dtype=np.float64,
    )


def tensor_numpy(x) -> np.ndarray:
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def vector_last_axis(x) -> np.ndarray:
    """Lấy link đầu tiên và giữ trục cuối = path."""
    a = tensor_numpy(x)
    if a.ndim == 1:
        return a
    return a.reshape(-1, a.shape[-1])[0]


def interaction_matrix(paths) -> np.ndarray:
    """
    Shape chuẩn:
    [max_depth, num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]
    -> [max_depth, num_paths]
    """
    a = tensor_numpy(paths.interactions)
    if a.ndim < 2:
        raise RuntimeError(f"Shape interactions không hợp lệ: {a.shape}")
    depth = a.shape[0]
    num_paths = a.shape[-1]
    return a.reshape(depth, -1, num_paths)[:, 0, :]


def vertices_matrix(paths, num_paths: int) -> np.ndarray | None:
    """
    -> [max_depth, num_paths, 3]
    """
    if not hasattr(paths, "vertices"):
        return None
    a = tensor_numpy(paths.vertices)
    if a.ndim < 3 or a.shape[-1] != 3:
        return None
    depth = a.shape[0]
    return a.reshape(depth, -1, num_paths, 3)[:, 0, :, :]


def objects_matrix(paths, num_paths: int) -> np.ndarray | None:
    """
    -> [max_depth, num_paths]
    """
    if not hasattr(paths, "objects"):
        return None
    a = tensor_numpy(paths.objects)
    if a.ndim < 2:
        return None
    depth = a.shape[0]
    return a.reshape(depth, -1, num_paths)[:, 0, :]


def const_int(x, fallback: int) -> int:
    try:
        return int(x)
    except Exception:
        try:
            a = tensor_numpy(x)
            return int(a.reshape(-1)[0])
        except Exception:
            return fallback


TYPE_NONE = const_int(getattr(InteractionType, "NONE", 0), 0)
TYPE_SPECULAR = const_int(getattr(InteractionType, "SPECULAR", 1), 1)
TYPE_DIFFUSE = const_int(getattr(InteractionType, "DIFFUSE", 2), 2)
TYPE_REFRACTION = const_int(getattr(InteractionType, "REFRACTION", 4), 4)
TYPE_DIFFRACTION = const_int(getattr(InteractionType, "DIFFRACTION", 8), 8)

INTERACTION_NAMES = {
    TYPE_NONE: "NONE",
    TYPE_SPECULAR: "PHAN_XA_GUONG",
    TYPE_DIFFUSE: "PHAN_XA_KHUECH_TAN",
    TYPE_REFRACTION: "KHUC_XA_TRUYEN_XUYEN",
    TYPE_DIFFRACTION: "NHIEU_XA",
}


def chain_to_text(chain: np.ndarray) -> str:
    vals = [int(v) for v in chain if int(v) != TYPE_NONE]
    if not vals:
        return "LOS"
    return " -> ".join(INTERACTION_NAMES.get(v, f"TYPE_{v}") for v in vals)


def chain_contains(chain: np.ndarray, interaction_type: int) -> bool:
    return bool(np.any(chain.astype(np.int64) == interaction_type))


def print_title(s: str) -> None:
    print("\n" + "=" * 92)
    print(s)
    print("=" * 92)


# ============================================================
# TẠO SCENE TỪ PLY
# ============================================================

def build_scene(ply_path: Path):
    print_title("1. NẠP MÔ HÌNH 3D")

    if not ply_path.exists():
        raise FileNotFoundError(
            f"Không thấy {ply_path}\n"
            "Hãy đặt file script và mesh map_triangles.obj cùng thư mục, hoặc dùng --mesh."
        )

    # Scene rỗng
    scene = load_scene()

    # Vật liệu baseline 433 MHz cho mesh đô thị đã gộp.
    # eps_r và sigma là giá trị baseline để TEST, không phải phép đo hiện trường.
    urban_mat = RadioMaterial(
        name="urban_433",
        relative_permittivity=5.24,
        conductivity=0.0240,
        thickness=0.20,
        scattering_coefficient=0.35,
        xpd_coefficient=0.10,
    )

    urban_obj = SceneObject(
        fname=str(ply_path),
        name="urban_mesh",
        radio_material=urban_mat,
        remove_duplicate_vertices=True,
    )

    scene.edit(add=urban_obj)
    scene.frequency = 433e6

    bbox = urban_obj.mi_mesh.bbox()
    bmin = point3_to_np(bbox.min)
    bmax = point3_to_np(bbox.max)
    center = (bmin + bmax) / 2.0

    print(f"[SCENE] Mesh       : {ply_path}")
    print(f"[SCENE] BBox min   : {bmin}")
    print(f"[SCENE] BBox max   : {bmax}")
    print(f"[SCENE] Kích thước : {bmax - bmin} m")
    print(f"[SCENE] Tâm mesh   : {center}")
    print(f"[RF]    f_c        : {scalar_float(scene.frequency)/1e6:.3f} MHz")
    print(f"[RF]    lambda     : {scalar_float(scene.wavelength):.6f} m")
    print("[RF]    material   : urban_433")
    print("[RF]    eps_r      : 5.24")
    print("[RF]    sigma      : 0.0240 S/m")
    print("[RF]    thickness  : 0.20 m")
    print("[RF]    scattering : 0.35")

    return scene, urban_obj, bmin, bmax, center


# ============================================================
# ĐẶT TX/RX
# ============================================================

def configure_radio_devices(
    scene,
    bmin: np.ndarray,
    bmax: np.ndarray,
    center: np.ndarray,
    distance_m: float,
    height_m: float,
    tx_override,
    rx_override,
):
    print_title("2. ĐẶT TX / RX")

    if tx_override is not None and rx_override is not None:
        tx_pos = np.asarray(tx_override, dtype=float)
        rx_pos = np.asarray(rx_override, dtype=float)
    else:
        # Đặt quanh tâm scene, cách nhau đúng distance_m theo trục X
        z = bmin[2] + height_m
        tx_pos = np.array([center[0] - distance_m / 2.0, center[1], z])
        rx_pos = np.array([center[0] + distance_m / 2.0, center[1], z])

        # Nếu scene hẹp hơn yêu cầu thì báo rõ.
        if tx_pos[0] < bmin[0] or rx_pos[0] > bmax[0]:
            raise RuntimeError(
                "Mesh không đủ rộng để tự đặt TX/RX theo khoảng cách yêu cầu. "
                "Hãy truyền --tx và --rx thủ công."
            )

    actual_distance = float(np.linalg.norm(rx_pos - tx_pos))

    scene.tx_array = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=0.5,
        horizontal_spacing=0.5,
        pattern="iso",
        polarization="V",
    )
    scene.rx_array = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=0.5,
        horizontal_spacing=0.5,
        pattern="iso",
        polarization="V",
    )

    tx = Transmitter(name="tx", position=tx_pos.tolist(), display_radius=2)
    rx = Receiver(name="rx", position=rx_pos.tolist(), display_radius=2)

    scene.add(tx)
    scene.add(rx)

    try:
        tx.look_at(rx)
    except Exception:
        pass
    try:
        rx.look_at(tx)
    except Exception:
        pass

    print(f"[TX] {tx_pos}")
    print(f"[RX] {rx_pos}")
    print(f"[LINK] Khoảng cách Euclid = {actual_distance:.3f} m")

    return tx, rx, tx_pos, rx_pos, actual_distance


# ============================================================
# RAY TRACING
# ============================================================

def solve_paths(scene, samples: int, max_depth: int, seed: int):
    print_title("3. RAY TRACING")

    print("[SOLVER] LOS                  = ON")
    print("[SOLVER] Specular reflection  = ON")
    print("[SOLVER] Diffuse reflection   = ON")
    print("[SOLVER] Refraction/Tx-through= ON")
    print("[SOLVER] Diffraction          = ON")
    print("[SOLVER] Edge diffraction     = ON")
    print(f"[SOLVER] max_depth            = {max_depth}")
    print(f"[SOLVER] samples_per_src      = {samples:,}")
    print(f"[SOLVER] seed                 = {seed}")
    print("[SOLVER] Đang tính...")

    solver = PathSolver()

    paths = solver(
        scene=scene,
        max_depth=max_depth,
        max_num_paths_per_src=1_000_000,
        samples_per_src=samples,
        synthetic_array=True,
        los=True,
        specular_reflection=True,
        diffuse_reflection=True,
        refraction=True,
        diffraction=True,
        edge_diffraction=True,
        diffraction_lit_region=True,
        seed=seed,
    )

    print("[SOLVER] Hoàn tất.")
    return paths


# ============================================================
# PHÂN TÍCH KÊNH
# ============================================================

def analyze_paths(
    scene,
    paths,
    distance_m: float,
    tx_power_dbm: float,
    bandwidth_hz: float,
    noise_figure_db: float,
    output_csv: Path,
):
    print_title("4. THỐNG KÊ KÊNH TRUYỀN")

    a_real = vector_last_axis(paths.a[0]).astype(np.float64)
    a_imag = vector_last_axis(paths.a[1]).astype(np.float64)
    a_all = a_real + 1j * a_imag

    tau_all = vector_last_axis(paths.tau).astype(np.float64)
    valid_all = vector_last_axis(paths.valid).astype(bool)

    theta_t_all = vector_last_axis(paths.theta_t).astype(np.float64)
    phi_t_all = vector_last_axis(paths.phi_t).astype(np.float64)
    theta_r_all = vector_last_axis(paths.theta_r).astype(np.float64)
    phi_r_all = vector_last_axis(paths.phi_r).astype(np.float64)
    doppler_all = vector_last_axis(paths.doppler).astype(np.float64)

    interactions_all = interaction_matrix(paths)
    total_slots = len(a_all)

    # Chỉ giữ path hợp lệ
    valid_mask = (
        valid_all
        & np.isfinite(tau_all)
        & (tau_all >= 0)
        & np.isfinite(a_real)
        & np.isfinite(a_imag)
    )

    valid_idx = np.where(valid_mask)[0]

    if len(valid_idx) == 0:
        print("[KẾT QUẢ] Không tìm thấy path hợp lệ.")
        print("Có thể TX/RX nằm bên trong mesh, samples quá thấp, hoặc mesh có vấn đề topology.")
        return

    a = a_all[valid_idx]
    tau = tau_all[valid_idx]
    theta_t = theta_t_all[valid_idx]
    phi_t = phi_t_all[valid_idx]
    theta_r = theta_r_all[valid_idx]
    phi_r = phi_r_all[valid_idx]
    doppler = doppler_all[valid_idx]
    interactions = interactions_all[:, valid_idx]

    power = np.abs(a) ** 2
    gain_db_each = np.array([db10(p) for p in power])
    phase_deg = np.rad2deg(np.angle(a))
    path_len = C0 * tau

    min_tau = float(np.min(tau))
    excess_tau = tau - min_tau

    p_sum = float(np.sum(power))
    w = power / p_sum if p_sum > 0 else np.ones_like(power) / len(power)

    mean_delay = float(np.sum(w * tau))
    rms_delay = float(np.sqrt(np.sum(w * (tau - mean_delay) ** 2)))
    max_excess_delay = float(np.max(excess_tau))

    # Hai cách cộng công suất:
    # - incoherent: tổng công suất từng MPC
    # - coherent: tổng phasor tại tần số trung tâm
    h_coherent = np.sum(a)
    coherent_power_gain = float(np.abs(h_coherent) ** 2)
    incoherent_power_gain = p_sum

    coherent_gain_db = db10(coherent_power_gain)
    incoherent_gain_db = db10(incoherent_power_gain)

    strongest_local = int(np.argmax(power))
    strongest_global_idx = int(valid_idx[strongest_local])
    strongest_gain_db = float(gain_db_each[strongest_local])

    # Rice-like dominant-path ratio (không phải K-factor đo chuẩn nếu strongest không phải LOS)
    rest_power = p_sum - float(power[strongest_local])
    if rest_power > 0:
        dominant_k_db = db10(float(power[strongest_local]) / rest_power)
    else:
        dominant_k_db = np.inf

    fc = scalar_float(scene.frequency)
    wavelength = scalar_float(scene.wavelength)
    fspl_db = 20.0 * math.log10(4.0 * math.pi * distance_m / wavelength)

    rx_power_coh_dbm = tx_power_dbm + coherent_gain_db
    rx_power_inc_dbm = tx_power_dbm + incoherent_gain_db

    noise_dbm = (
        -174.0
        + 10.0 * math.log10(bandwidth_hz)
        + noise_figure_db
    )
    snr_coh_db = rx_power_coh_dbm - noise_dbm
    snr_inc_db = rx_power_inc_dbm - noise_dbm

    # Approx. coherence bandwidth
    if rms_delay > 0:
        bc_50 = 1.0 / (5.0 * rms_delay)
        bc_90 = 1.0 / (50.0 * rms_delay)
    else:
        bc_50 = np.inf
        bc_90 = np.inf

    # Frequency response across requested bandwidth
    f_off = np.linspace(-bandwidth_hz / 2.0, bandwidth_hz / 2.0, 101)
    Hf = np.sum(
        a[:, None] * np.exp(-1j * 2.0 * np.pi * tau[:, None] * f_off[None, :]),
        axis=0,
    )
    Hf_gain_db = 20.0 * np.log10(np.maximum(np.abs(Hf), 1e-30))
    freq_ripple_db = float(np.max(Hf_gain_db) - np.min(Hf_gain_db))

    # Đếm hiện tượng
    count_los = 0
    count_spec = 0
    count_diffuse = 0
    count_refract = 0
    count_diffract = 0

    for j in range(interactions.shape[1]):
        ch = interactions[:, j]
        nonzero = ch[ch != TYPE_NONE]
        if len(nonzero) == 0:
            count_los += 1
        if chain_contains(ch, TYPE_SPECULAR):
            count_spec += 1
        if chain_contains(ch, TYPE_DIFFUSE):
            count_diffuse += 1
        if chain_contains(ch, TYPE_REFRACTION):
            count_refract += 1
        if chain_contains(ch, TYPE_DIFFRACTION):
            count_diffract += 1

    print(f"[PATH] Slots solver          : {total_slots}")
    print(f"[PATH] Path hợp lệ          : {len(valid_idx)}")
    print(f"[PATH] LOS                  : {count_los}")
    print(f"[PATH] Có phản xạ gương     : {count_spec}")
    print(f"[PATH] Có phản xạ khuếch tán: {count_diffuse}")
    print(f"[PATH] Có khúc xạ/truyền xuyên: {count_refract}")
    print(f"[PATH] Có nhiễu xạ          : {count_diffract}")

    print("\n--- THÔNG SỐ TỔNG HỢP ---")
    print(f"f_c                         = {fc/1e6:.3f} MHz")
    print(f"lambda                      = {wavelength:.6f} m")
    print(f"d(TX,RX)                    = {distance_m:.3f} m")
    print(f"FSPL tham chiếu             = {fspl_db:.3f} dB")
    print(f"Gain coherent @fc           = {coherent_gain_db:.3f} dB")
    print(f"Gain incoherent Σ|a|²       = {incoherent_gain_db:.3f} dB")
    print(f"Path loss coherent          = {-coherent_gain_db:.3f} dB")
    print(f"Path loss incoherent        = {-incoherent_gain_db:.3f} dB")
    print(f"Strongest path index        = {strongest_global_idx}")
    print(f"Strongest path gain         = {strongest_gain_db:.3f} dB")
    print(f"Dominant/other ratio        = {dominant_k_db:.3f} dB")
    print(f"Min delay                   = {min_tau*1e6:.6f} us")
    print(f"Mean delay (power-weighted) = {mean_delay*1e6:.6f} us")
    print(f"RMS delay spread            = {rms_delay*1e9:.3f} ns")
    print(f"Max excess delay            = {max_excess_delay*1e6:.6f} us")
    print(f"Bc ~50% (xấp xỉ)            = {bc_50/1e6:.3f} MHz")
    print(f"Bc ~90% (xấp xỉ)            = {bc_90/1e6:.3f} MHz")
    print(f"Doppler min/max             = {np.min(doppler):.3f} / {np.max(doppler):.3f} Hz")
    print(f"BW kiểm tra CFR             = {bandwidth_hz/1e3:.1f} kHz")
    print(f"CFR min/max gain            = {np.min(Hf_gain_db):.3f} / {np.max(Hf_gain_db):.3f} dB")
    print(f"CFR ripple                  = {freq_ripple_db:.3f} dB")

    print("\n--- LINK BUDGET THAM KHẢO ---")
    print(f"TX power                    = {tx_power_dbm:.2f} dBm")
    print(f"Noise figure giả định       = {noise_figure_db:.2f} dB")
    print(f"Thermal noise + NF          = {noise_dbm:.2f} dBm")
    print(f"RX power coherent           = {rx_power_coh_dbm:.2f} dBm")
    print(f"RX power incoherent         = {rx_power_inc_dbm:.2f} dBm")
    print(f"SNR coherent                = {snr_coh_db:.2f} dB")
    print(f"SNR incoherent              = {snr_inc_db:.2f} dB")

    # Chuẩn bị vertices/object IDs
    verts = vertices_matrix(paths, total_slots)
    objs = objects_matrix(paths, total_slots)

    print_title("5. CHI TIẾT TỪNG MULTIPATH COMPONENT")

    header = (
        "IDX | TYPE/CHAIN | |a| | PHASE(deg) | GAIN(dB) | "
        "DELAY(us) | LEN(m) | EXCESS(us) | "
        "AoD(theta,phi deg) | AoA(theta,phi deg) | DOPPLER(Hz)"
    )
    print(header)
    print("-" * len(header))

    rows = []

    for local_i, global_i in enumerate(valid_idx):
        ch = interactions[:, local_i]
        chain_text = chain_to_text(ch)

        row = {
            "path_index": int(global_i),
            "interaction_chain": chain_text,
            "a_real": float(np.real(a[local_i])),
            "a_imag": float(np.imag(a[local_i])),
            "a_abs": float(np.abs(a[local_i])),
            "phase_deg": float(phase_deg[local_i]),
            "path_gain_db": float(gain_db_each[local_i]),
            "delay_us": float(tau[local_i] * 1e6),
            "path_length_m": float(path_len[local_i]),
            "excess_delay_us": float(excess_tau[local_i] * 1e6),
            "aod_theta_deg": float(np.rad2deg(theta_t[local_i])),
            "aod_phi_deg": float(np.rad2deg(phi_t[local_i])),
            "aoa_theta_deg": float(np.rad2deg(theta_r[local_i])),
            "aoa_phi_deg": float(np.rad2deg(phi_r[local_i])),
            "doppler_hz": float(doppler[local_i]),
        }
        rows.append(row)

        print(
            f"{global_i:3d} | {chain_text:32.32s} | "
            f"{row['a_abs']:.3e} | {row['phase_deg']:9.3f} | "
            f"{row['path_gain_db']:8.3f} | {row['delay_us']:9.6f} | "
            f"{row['path_length_m']:8.3f} | {row['excess_delay_us']:9.6f} | "
            f"({row['aod_theta_deg']:7.2f},{row['aod_phi_deg']:7.2f}) | "
            f"({row['aoa_theta_deg']:7.2f},{row['aoa_phi_deg']:7.2f}) | "
            f"{row['doppler_hz']:9.3f}"
        )

        if verts is not None:
            used_depth = [
                d for d in range(interactions.shape[0])
                if int(ch[d]) != TYPE_NONE
            ]
            if used_depth:
                for d in used_depth:
                    v = verts[d, global_i, :]
                    obj_id = None
                    if objs is not None:
                        obj_id = int(objs[d, global_i])
                    print(
                        f"      ↳ depth={d+1} "
                        f"type={INTERACTION_NAMES.get(int(ch[d]), int(ch[d]))} "
                        f"vertex=({v[0]:.3f},{v[1]:.3f},{v[2]:.3f}) "
                        f"object_id={obj_id}"
                    )

    # CSV
    with output_csv.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print_title("6. KIỂM TRA HIỆN TƯỢNG")
    print(f"Phản xạ gương      : {'CÓ' if count_spec else 'KHÔNG TÌM THẤY'}")
    print(f"Phản xạ khuếch tán : {'CÓ' if count_diffuse else 'KHÔNG TÌM THẤY'}")
    print(f"Khúc xạ/truyền xuyên: {'CÓ' if count_refract else 'KHÔNG TÌM THẤY'}")
    print(f"Nhiễu xạ           : {'CÓ' if count_diffract else 'KHÔNG TÌM THẤY'}")

    if count_spec == 0 or count_refract == 0 or count_diffract == 0:
        print(
            "\n[LƯU Ý] Solver đã BẬT các hiện tượng trên, nhưng việc một loại path "
            "có xuất hiện hay không phụ thuộc geometry, vị trí TX/RX, topology mesh "
            "và số ray samples. 'Bật' không đồng nghĩa chắc chắn sẽ tìm được path."
        )

    print(f"\n[CSV] Đã ghi chi tiết path: {output_csv}")


# ============================================================
# MAIN
# ============================================================

def parse_args():
    p = argparse.ArgumentParser(
        description="Sionna RT 433 MHz - 100 m full channel test"
    )
    p.add_argument(
        "--mesh",
        "--ply",
        dest="mesh",
        default="map_triangles.obj",
        help=(
            "Mesh 3D .obj/.ply "
            "(mặc định: map_triangles.obj cùng thư mục script). "
            "--ply vẫn được giữ làm alias để tương thích lệnh cũ."
        ),
    )
    p.add_argument("--distance", type=float, default=100.0, help="Khoảng cách TX-RX [m]")
    p.add_argument("--height", type=float, default=2.0, help="Độ cao so với z_min của mesh [m]")
    p.add_argument("--samples", type=int, default=500_000, help="samples_per_src")
    p.add_argument("--max-depth", type=int, default=5, help="Số tương tác tối đa")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--tx-power-dbm", type=float, default=20.0)
    p.add_argument("--bandwidth-hz", type=float, default=500_000.0)
    p.add_argument("--noise-figure-db", type=float, default=6.0)
    p.add_argument(
        "--tx",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="Tọa độ TX thủ công [m]",
    )
    p.add_argument(
        "--rx",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="Tọa độ RX thủ công [m]",
    )
    p.add_argument(
        "--csv",
        default="channel_paths_433mhz.csv",
        help="File CSV kết quả từng path",
    )
    return p.parse_args()


def main():
    args = parse_args()

    if (args.tx is None) != (args.rx is None):
        raise ValueError("Nếu dùng tọa độ thủ công, phải truyền cả --tx và --rx.")

    base = Path(__file__).resolve().parent
    mesh_path = Path(args.mesh)
    if not mesh_path.is_absolute():
        mesh_path = base / mesh_path

    csv_path = Path(args.csv)
    if not csv_path.is_absolute():
        csv_path = base / csv_path

    print_title("SIONNA RT - TEST KÊNH 433 MHz / ~100 m")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Sionna RT module: {rt.__file__}")
    print(f"Mitsuba variant: {mi.variant()}")
    if sys.platform.startswith("win"):
        print(f"LLVM-C.dll: {os.environ.get('DRJIT_LIBLLVM_PATH')}")

    scene, obj, bmin, bmax, center = build_scene(mesh_path)

    tx, rx, tx_pos, rx_pos, distance_m = configure_radio_devices(
        scene=scene,
        bmin=bmin,
        bmax=bmax,
        center=center,
        distance_m=args.distance,
        height_m=args.height,
        tx_override=args.tx,
        rx_override=args.rx,
    )

    paths = solve_paths(
        scene=scene,
        samples=args.samples,
        max_depth=args.max_depth,
        seed=args.seed,
    )

    analyze_paths(
        scene=scene,
        paths=paths,
        distance_m=distance_m,
        tx_power_dbm=args.tx_power_dbm,
        bandwidth_hz=args.bandwidth_hz,
        noise_figure_db=args.noise_figure_db,
        output_csv=csv_path,
    )


if __name__ == "__main__":
    main()
