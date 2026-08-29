#ifndef HMI_DU_H
#define HMI_DU_H

#include <Arduino.h>

// ============================================================
// HMI DU - 1 NUT NACK + 1 LED TRANG THAI
//
// ACK: DU tu dong gui sau khi phat xong audio.
// NACK: nguoi dung bam GPIO6 neu nghe khong ro.
// LED: GPIO16.
//
// Module nay cung cung cap trang thai "radio dang ban" de beacon GPS
// dinh ky KHONG chen vao SESSION/VOICE/FEC/HMI.
// ============================================================

void KhoiTao_HMI_DU();

// Goi ngay khi RX task vua nhan packet quan trong.
// Chi khoa beacon tam thoi, khong tu y thay doi session.
void HMI_DU_TamDung_Beacon(uint32_t thoi_gian_ms);

// Goi khi decoder chap nhan SESSION_START moi.
void HMI_DU_Reset_Cho_Session_Moi();

// Bao LED vua nhan mot VOICE/FEC hop le.
void HMI_DU_Bao_Nhan_Data();

// Goi khi END_AUDIO da toi: ket thuc LED session/data.
void HMI_DU_Bao_END_Audio();

// Goi sau khi speaker phat xong toan bo cau.
// Ham nay tao yeu cau AUTO_ACK cho task HMI.
void HMI_DU_Bao_Phat_Xong(uint64_t session_id);

// Xu ly USER_CONFIRM tu SU. Tra true neu confirm khop giao dich dang cho.
bool HMI_DU_XuLy_User_Confirm(uint64_t session_id, uint8_t code);

// Beacon dinh ky phai hoan khi ham nay tra true.
bool HMI_DU_Radio_Dang_Ban();

#endif
