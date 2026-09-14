#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Công thức hệ số kênh dùng chung cho đo THẬT và EVE ẢO.

Đo thật:
    RSSI[dBm] = 10 log10(Pr[mW])
    Pr = Pt |H|^2
    |H|^2 = Pr/Pt
    |H| = sqrt(Pr/Pt)

EVE ảo V1 (mô hình hai đường):
    H(f) = sum_i a_i exp(-j 2 pi f tau_i)
    i=1 đường trực tiếp, i=2 đường phản xạ mặt đất.

Lưu ý: EVE ảo V1 chưa xét tường/cây của campus. Có thể thay backend bằng
Sionna RT sau mà không đổi giao diện.
"""
import cmath
import math

C = 299_792_458.0
DEFAULT_FREQUENCY_HZ = 433_000_000.0
DEFAULT_GROUND_REFLECTION = -0.55


def dbm_to_mw(dbm: float) -> float:
    return 10.0 ** (float(dbm) / 10.0)


def mw_to_dbm(mw: float):
    mw = float(mw)
    if mw <= 0.0:
        return None
    return 10.0 * math.log10(mw)


def measured_channel_from_rssi(rssi_dbm, pt_dbm):
    """Tính kênh đo thật từ RSSI và công suất phát theo đúng chuỗi công thức."""
    if rssi_dbm is None or pt_dbm is None:
        return None
    pr_mw = dbm_to_mw(float(rssi_dbm))
    pt_mw = dbm_to_mw(float(pt_dbm))
    if pt_mw <= 0.0:
        return None
    h2 = pr_mw / pt_mw
    h_abs = math.sqrt(max(h2, 0.0))
    h_db = 10.0 * math.log10(h2) if h2 > 0.0 else None
    return {
        'mode': 'THAT',
        'rssi_dbm': float(rssi_dbm),
        'pr_mw': pr_mw,
        'pt_dbm': float(pt_dbm),
        'pt_mw': pt_mw,
        'h2': h2,
        'h_abs': h_abs,
        'h_db': h_db,
    }


def virtual_two_ray_channel(
    tx_x_m, tx_y_m, tx_h_m,
    rx_x_m, rx_y_m, rx_h_m,
    pt_dbm,
    frequency_hz=DEFAULT_FREQUENCY_HZ,
    reflection_coeff=DEFAULT_GROUND_REFLECTION,
):
    """Kênh ảo V1 theo H(f)=Σa_i exp(-j2πfτ_i) với 2 đường.

    a1 = λ/(4πd1)
    a2 = Γ λ/(4πd2)
    tau_i = d_i/c

    Đây là mô hình nhanh chạy trực tiếp trên Raspberry Pi, chưa xét geometry tòa nhà.
    """
    f = float(frequency_hz)
    if f <= 0.0:
        return None
    dx = float(tx_x_m) - float(rx_x_m)
    dy = float(tx_y_m) - float(rx_y_m)
    horizontal = math.hypot(dx, dy)

    # Tránh singularity nếu click trùng đúng transmitter.
    d1 = max(1.0, math.sqrt(horizontal**2 + (float(tx_h_m)-float(rx_h_m))**2))
    d2 = max(1.0, math.sqrt(horizontal**2 + (float(tx_h_m)+float(rx_h_m))**2))

    wavelength = C / f
    a1 = wavelength / (4.0 * math.pi * d1)
    a2 = complex(reflection_coeff) * wavelength / (4.0 * math.pi * d2)
    tau1 = d1 / C
    tau2 = d2 / C

    h_complex = (
        a1 * cmath.exp(-1j * 2.0 * math.pi * f * tau1)
        + a2 * cmath.exp(-1j * 2.0 * math.pi * f * tau2)
    )
    h_abs = abs(h_complex)
    h2 = h_abs * h_abs
    h_db = 10.0 * math.log10(h2) if h2 > 0.0 else None

    pt_mw = dbm_to_mw(float(pt_dbm))
    pr_mw = pt_mw * h2
    rssi_dbm = mw_to_dbm(pr_mw)

    return {
        'mode': 'AO_2_DUONG',
        'frequency_hz': f,
        'horizontal_m': horizontal,
        'd_direct_m': d1,
        'd_reflected_m': d2,
        'a1': a1,
        'a2': a2,
        'tau1_s': tau1,
        'tau2_s': tau2,
        'h_complex': h_complex,
        'h_abs': h_abs,
        'h2': h2,
        'h_db': h_db,
        'pt_dbm': float(pt_dbm),
        'pt_mw': pt_mw,
        'pr_mw': pr_mw,
        'rssi_dbm': rssi_dbm,
    }
