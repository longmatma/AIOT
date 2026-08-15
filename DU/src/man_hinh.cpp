#include "man_hinh.h"
#include <U8g2lib.h>
#include <Wire.h>

// Đổi sang 2 chân rảnh bất kỳ (tránh các chân LoRa và chân 1 của Audio)
#define OLED_SDA 4 
#define OLED_SCL 5 

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void KhoiTao_OLED() {
    // Ép I2C phần cứng chuyển sang dùng chân 4 và 5
    Wire.begin(OLED_SDA, OLED_SCL); 
    
    u8g2.begin();
    u8g2.enableUTF8Print(); 
    
    // Màn hình khởi động
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(15, 30, "TRAM NHAN (DU)");
    u8g2.drawStr(15, 50, "Dang khoi dong...");
    u8g2.sendBuffer();
    
    Serial.println("Khoi tao OLED THANH CONG!");
}

void CapNhat_TrangThai_OLED(String trang_thai, int rssi, int so_goi_tin) {
    u8g2.clearBuffer();
    
    // Vẽ khung giao diện
    u8g2.drawFrame(0, 0, 128, 64);
    
    // Hiển thị trạng thái
    u8g2.setFont(u8g2_font_helvB08_te);
    u8g2.setCursor(5, 15);
    u8g2.print("Status: ");
    u8g2.print(trang_thai);
    
    // Hiển thị cường độ sóng RSSI
    u8g2.setCursor(5, 35);
    u8g2.print("LoRa RSSI: ");
    u8g2.print(rssi);
    u8g2.print(" dBm");
    
    // Hiển thị bộ đếm gói tin
    u8g2.setCursor(5, 55);
    u8g2.print("Goi tin: ");
    u8g2.print(so_goi_tin);
    
    u8g2.sendBuffer();
}