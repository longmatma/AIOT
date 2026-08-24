#ifndef MAN_HINH_H
#define MAN_HINH_H

#include <Arduino.h>

void KhoiTao_OLED();

void HienThi_DU_KhoiDong(
    const char *trang_thai
);

void HienThi_DU_ChoNhan();

void HienThi_DU_DangNhan(
    uint32_t so_voice_expected,
    uint32_t so_goi_mat_raw,
    uint32_t so_goi_fec_cuu
);

void HienThi_DU_KetQua(
    uint32_t so_voice_expected,
    uint32_t so_goi_mat_raw,
    uint32_t so_goi_fec_cuu,
    uint32_t so_goi_con_mat
);

#endif
