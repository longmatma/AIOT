#ifndef HMI_SU_H
#define HMI_SU_H

#include <Arduino.h>

// ============================================================
// HMI SU - 1 NUT PTT + 1 LED TRANG THAI
//
// Muc tieu cua module nay:
// - Tach toan bo xu ly nut nhan/LED khoi main.cpp.
// - main.cpp chi goi cac ham co y nghia nghiep vu.
// - Khong thay doi giao thuc LoRa/VOICE/FEC/AES/ARQ.
//
// Phan cung:
//   GPIO6  = nut PTT, active LOW, INPUT_PULLUP.
//   GPIO16 = LED trang thai.
// ============================================================

void KhoiTao_HMI_SU();

// Doc nut PTT. Tra ve true khi nguoi dung dang giu nut.
bool Nut_PTT_Dang_Bam_SU();

// Dieu khien LED truc tiep cho cac pha SESSION/TX packet.
void Dat_LED_SU(bool sang);

// Goi khi bat dau mot cau PTT moi: xoa ACK/NACK/session cu.
void HMI_SU_BatDau_CauMoi();

// Luu/lay session gan nhat de xu ly ACK/NACK tu DU.
void HMI_SU_Dat_Session(uint64_t session_id);
uint64_t HMI_SU_Lay_Session();

// True khi SU da co session vua gui xong nhung chua nhan feedback DU.
// Dung de hoan beacon GPS dinh ky, uu tien nghe control packet.
bool HMI_SU_Dang_Cho_PhanHoi();

// Cap nhat feedback cua DU: ACK/NACK.
void Dat_PhanHoi_SU(uint8_t code);

// Cap nhat mau chop LED theo feedback hien tai.
void CapNhat_LED_PhanHoi_SU();

// Bao loi SESSION bang 3 chop cham.
void Bao_Loi_Session_SU();

#endif
