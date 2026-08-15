#include "dong_goi.h"
#include "ma_hoa.h"

// Biến đếm số thứ tự gói tin (Sequence Number) để trạm DU không bị nhận lộn xộn
uint16_t so_thu_tu_goi = 0;

void Tao_GoiTin_LoRa(
    uint8_t* khung_1,
    uint8_t* khung_2,
    uint8_t so_frame,
    uint8_t* goi_tin_ra)
{
    // 1. Lưu lại số thứ tự hiện tại để làm chìa khóa IV trước khi tăng biến đếm
    uint16_t so_thu_tu_hien_tai = so_thu_tu_goi;
    so_thu_tu_goi++;

    // 2. Viết Header (6 byte đầu)
    goi_tin_ra[0] = ID_TRAM_DU;
    goi_tin_ra[1] = ID_TRAM_SU;
    goi_tin_ra[2] =
    0x01 | ((so_frame - 1) << 7);

    // Chiều dài thực tế bây giờ = 40 byte âm thanh + 8 byte Auth Tag = 48
    goi_tin_ra[3] = 48; 

    goi_tin_ra[4] = (so_thu_tu_hien_tai >> 8) & 0xFF;
    goi_tin_ra[5] = so_thu_tu_hien_tai & 0xFF;

    // 3. Chuẩn bị 40 byte Payload thô (chưa mã hóa)
    uint8_t payload_tho[40];
    
    // Copy khung 1
    memcpy(&payload_tho[0], khung_1, 20);

    // Copy khung 2 hoặc nhồi số 0 nếu chỉ có 1 khung
    if (so_frame == 2)
    {
        memcpy(&payload_tho[20], khung_2, 20);
    }
    else
    {
        memset(&payload_tho[20], 0, 20);
    }

    // 4. MÃ HÓA VÀ ĐÓNG TEM GCM
    // - Input Header (6 byte): goi_tin_ra (AAD - Không mã hóa nhưng được bảo vệ)
    // - Input Dữ liệu (40 byte): payload_tho
    // - Output Bản mã (40 byte): &goi_tin_ra[6]
    // - Output Tem xác thực (8 byte): &goi_tin_ra[46]
    MaHoa_GCM(goi_tin_ra, payload_tho, &goi_tin_ra[6], &goi_tin_ra[46], so_thu_tu_hien_tai);
}