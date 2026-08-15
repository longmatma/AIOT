#ifndef HIEN_THI_H
#define HIEN_THI_H

#include <Arduino.h>
#include <U8g2lib.h>

// Khai báo các hàm sẽ dùng ở bên ngoài
void KhoiTao_OLED();
int Tinh_BienDo_Mic(uint8_t *buffer_am_thanh_8bit, int so_byte);
void Ve_GiaoDien_OLED(int bien_do_mic, bool dang_phat);

#endif