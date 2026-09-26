#ifndef GIAI_MA_SPEEX_H
#define GIAI_MA_SPEEX_H
#include <Arduino.h>
void KhoiTao_GiaiMa_Speex();
void GiaiMa_ResetAudioProfile();
void GiaiMa_KhungThoai_Adaptive(const uint8_t* data, uint8_t frame_bytes, uint8_t* pcm_ra_320b);
void GiaiMa_KhungThoai(uint8_t* data6, uint8_t* pcm_ra_320b); // legacy=LQ
void GiaiMa_KhungMat(uint8_t* pcm_ra_320b);
#endif
