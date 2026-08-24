#ifndef HIEN_THI_H
#define HIEN_THI_H

#include <Arduino.h>
#include <U8g2lib.h>

// Giao dien mic/record hien tai.
void KhoiTao_OLED();
int Tinh_BienDo_Mic(uint8_t *buffer_am_thanh_8bit, int so_byte);
void Ve_GiaoDien_OLED(int bien_do_mic, bool dang_phat);

// Giao dien kiem tra link Hop1 SU -> rBS.
void HienThi_SU_DangGui(
    uint32_t so_voice_packet,
    uint32_t so_lan_retry,
    uint32_t so_packet_fail
);

void HienThi_SU_KetQua(
    uint32_t so_voice_packet,
    uint32_t so_lan_retry,
    uint32_t so_packet_fail,
    bool e2e_hop_le,
    uint32_t e2e_ms
);

#endif
