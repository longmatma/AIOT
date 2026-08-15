#include "nen_speex.h"
#include <speex/speex.h>
extern "C" {
    __attribute__((weak)) void * _impure_ptr = nullptr;
}

void *trang_thai_speex;
SpeexBits cac_bit_speex;

static int32_t dc_offset_tich_luy = 1551 * 2048; 
static int16_t mau_am_thanh_truoc_do = 0; 

void KhoiTao_MayEp_Speex() {
    speex_bits_init(&cac_bit_speex);
    trang_thai_speex = speex_encoder_init(&speex_nb_mode);
    int bitrate = 8000; 
    speex_encoder_ctl(trang_thai_speex, SPEEX_SET_BITRATE, &bitrate);
    int vad = 0;
    speex_encoder_ctl(trang_thai_speex, SPEEX_SET_VAD, &vad);
    Serial.println("Khoi tao May Ep Speex (CBR 8000bps) THANH CONG!");
}

bool Nen_Thanh_KhungThoai(uint8_t* pcm_vao, uint8_t* khung_thoai_ra) {
    speex_bits_reset(&cac_bit_speex);
    int16_t* mau_am_thanh = (int16_t*)pcm_vao;

   for (int i = 0; i < 160; i++) {
        uint16_t mau_goc = mau_am_thanh[i] & 0x0FFF; 
        
        dc_offset_tich_luy = dc_offset_tich_luy - (dc_offset_tich_luy / 2048) + mau_goc;
        int16_t dc_offset_hien_tai = dc_offset_tich_luy / 2048;

        int32_t val = mau_goc - dc_offset_hien_tai; 
        mau_am_thanh[i] = (int16_t)(val * 8); 
    }
    speex_encode_int(trang_thai_speex, mau_am_thanh, &cac_bit_speex);
    int so_byte_da_nen = speex_bits_write(&cac_bit_speex, (char*)khung_thoai_ra, 20);

    return (so_byte_da_nen > 0 && so_byte_da_nen <= 20);
}