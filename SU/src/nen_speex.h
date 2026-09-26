#ifndef NEN_SPEEX_H
#define NEN_SPEEX_H

#include <Arduino.h>

struct SpeexAudioDiagSU
{
    uint32_t sample_count;
    uint32_t clip_count;
    int32_t peak_pre_gain;
    int32_t peak_post_gain;
    uint32_t frame_size_error_count;
};

void KhoiTao_MayEp_Speex();
enum SpeexProfileSU : uint8_t { SPEEX_PROFILE_LQ = 0, SPEEX_PROFILE_HQ = 1 };
uint8_t Speex_SU_FrameBytes(SpeexProfileSU profile);
bool Nen_Thanh_KhungThoai_Profile(uint8_t* pcm_vao, uint8_t* khung_thoai_ra_10b, SpeexProfileSU profile);
bool Nen_Thanh_KhungThoai(uint8_t* pcm_vao, uint8_t* khung_thoai_ra); // legacy=LQ
void Speex_SU_ResetDiag();
SpeexAudioDiagSU Speex_SU_LayDiag();

#endif