#include <Arduino.h> 
#include "giai_ma_speex.h"
#include <speex/speex.h>

void *trang_thai_giai_ma;
SpeexBits cac_bit_speex;

void KhoiTao_GiaiMa_Speex() {
    speex_bits_init(&cac_bit_speex);
    
    // Khởi tạo decoder khớp với encoder bên SU
    trang_thai_giai_ma = speex_decoder_init(&speex_nb_mode);
    
    int bat_enhancer = 1;
    speex_decoder_ctl(trang_thai_giai_ma, SPEEX_SET_ENH, &bat_enhancer);
    
    Serial.println("Khoi tao bo giai ma Speex THANH CONG!");
}

void GiaiMa_KhungThoai(uint8_t* khung_vao_20b, uint8_t* pcm_ra_320b) {
    spx_int16_t am_thanh_giai_ma[160]; 
    
    // Dọn rác bộ nhớ trước khi đọc
    speex_bits_reset(&cac_bit_speex);
    
    // Đọc chính xác 20 byte dữ liệu
    speex_bits_read_from(&cac_bit_speex, (char*)khung_vao_20b, 20);
    
    // Giải mã
    speex_decode_int(trang_thai_giai_ma, &cac_bit_speex, am_thanh_giai_ma);
    
    memcpy(pcm_ra_320b, am_thanh_giai_ma, 320);
}

// =====================================================
// PACKET LOSS CONCEALMENT (PLC)
//
// Khi packet-level FEC không cứu được (mất >1 data hoặc parity mất),
// gọi Speex decoder với bits = NULL để decoder sinh 20ms concealment.
// Quan trọng hơn việc chèn zero PCM: decoder state vẫn được tiến
// theo đúng số frame bị mất.
// =====================================================

void GiaiMa_KhungMat(
    uint8_t* pcm_ra_320b)
{
    spx_int16_t am_thanh_giai_ma[160];

    speex_decode_int(
        trang_thai_giai_ma,
        NULL,
        am_thanh_giai_ma
    );

    memcpy(
        pcm_ra_320b,
        am_thanh_giai_ma,
        320
    );
}
