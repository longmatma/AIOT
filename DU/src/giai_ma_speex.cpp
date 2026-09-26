#include <Arduino.h>
#include "giai_ma_speex.h"
#include <speex/speex.h>
static void *trang_thai_giai_ma=nullptr;
static SpeexBits cac_bit_speex;
static constexpr int32_t DU_PCM_LIMIT=15000;
static inline void Limit(spx_int16_t *pcm){for(int i=0;i<160;i++){int32_t v=pcm[i];if(v>DU_PCM_LIMIT)v=DU_PCM_LIMIT;else if(v<-DU_PCM_LIMIT)v=-DU_PCM_LIMIT;pcm[i]=(int16_t)v;}}
void GiaiMa_ResetAudioProfile(){}
void KhoiTao_GiaiMa_Speex(){speex_bits_init(&cac_bit_speex);trang_thai_giai_ma=speex_decoder_init(&speex_nb_mode);int enh=0;speex_decoder_ctl(trang_thai_giai_ma,SPEEX_SET_ENH,&enh);Serial.println("Khoi tao Speex decoder ADAPTIVE | HQ10B/LQ6B | ENH=OFF | PCM_LIMIT=15000");}
void GiaiMa_KhungThoai_Adaptive(const uint8_t* data,uint8_t frame_bytes,uint8_t* pcm_ra){spx_int16_t pcm[160];speex_bits_reset(&cac_bit_speex);speex_bits_read_from(&cac_bit_speex,(char*)data,frame_bytes);int rc=speex_decode_int(trang_thai_giai_ma,&cac_bit_speex,pcm);if(rc<0)speex_decode_int(trang_thai_giai_ma,NULL,pcm);Limit(pcm);memcpy(pcm_ra,pcm,320);}
void GiaiMa_KhungMat(uint8_t* pcm_ra){spx_int16_t pcm[160];speex_decode_int(trang_thai_giai_ma,NULL,pcm);Limit(pcm);memcpy(pcm_ra,pcm,320);}

void GiaiMa_KhungThoai(uint8_t* data6, uint8_t* pcm_ra)
{
    GiaiMa_KhungThoai_Adaptive(data6, 6U, pcm_ra);
}
