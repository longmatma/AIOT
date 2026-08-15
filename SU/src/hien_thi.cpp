#include "hien_thi.h"

// Khởi tạo màn hình OLED 1.3 inch (Chip SH1106)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void KhoiTao_OLED() {
    u8g2.begin();
}

int Tinh_BienDo_Mic(uint8_t *buffer_am_thanh_8bit, int so_byte) {
    int max_val = -32768;
    int min_val = 32767;
    int16_t *buffer_16bit = (int16_t *)buffer_am_thanh_8bit;
    int so_mau = so_byte / 2; 

    // Lọc tìm đỉnh cao nhất và thấp nhất của sóng âm
    for(int i = 0; i < so_mau; i++) {
        if(buffer_16bit[i] > max_val) max_val = buffer_16bit[i];
        if(buffer_16bit[i] < min_val) min_val = buffer_16bit[i];
    }
    
    int bien_do = max_val - min_val;
    
    // Khử nhiễu nền tĩnh của cảm biến MAX9814 (nếu biến động quá nhỏ thì cho về 0)
    if (bien_do < 50) bien_do = 0; 
    
    return bien_do;
}

void Ve_GiaoDien_OLED(int bien_do_mic, bool dang_phat) {
    u8g2.clearBuffer();

    // -- HEADER: Trạng thái --
    u8g2.setFont(u8g2_font_ncenB08_tr); 
    u8g2.drawStr(0, 10, "LoRa: ");
    
    u8g2.setDrawColor(1);
    u8g2.drawBox(35, 1, 24, 11);
    u8g2.setDrawColor(0);
    u8g2.drawStr(37, 10, " OK ");
    u8g2.setDrawColor(1); 

    // -- MAIN: Chế độ --
    u8g2.setFont(u8g2_font_ncenB14_tr); 
    if (dang_phat) {
        // Khi đang bấm PTT
        u8g2.drawStr(12, 35, "PHAT AM"); 
    } else {
        // Khi nhả PTT
        u8g2.drawStr(10, 35, "SAN SANG"); 
    }

    // -- FOOTER: Thanh Mic --
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 52);
    
    if (dang_phat) {
        // Hiển thị số ADC
        u8g2.print("ADC: ");
        u8g2.print(bien_do_mic);
        
        // Kéo giãn thang đo lên 4096
        int chieu_dai_thanh = map(bien_do_mic, 0, 4096, 0, 124);
        
        if(chieu_dai_thanh > 124) chieu_dai_thanh = 124; 

        u8g2.drawFrame(0, 55, 128, 9);
        if(chieu_dai_thanh > 0) {
            u8g2.drawBox(2, 57, chieu_dai_thanh, 5);
        }
    } else {
        // Khi không bấm nút
        u8g2.print("Mic: Tam dung (Mute)");
        u8g2.drawFrame(0, 55, 128, 9); 
    }

    u8g2.sendBuffer();
} 