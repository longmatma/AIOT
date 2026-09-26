#include "nen_speex.h"
#include <speex/speex.h>

extern "C" { __attribute__((weak)) void * _impure_ptr = nullptr; }

static void *trang_thai_speex = nullptr;
static SpeexBits cac_bit_speex;
static SpeexProfileSU profile_hien_tai = SPEEX_PROFILE_HQ;

// V2C4 AUDIO CLEAR: zero-lookahead front-end.
// HPF ~150 Hz de bot low-mid/boom; pre-emphasis 0.08 de ro phu am nhung
// khong day treble manh nhu probe cu. Khong them look-ahead/latency.
static int32_t dc_offset_tich_luy = 1551 * 2048;
static float hpf_x_truoc = 0.0f;
static float hpf_y_truoc = 0.0f;
static constexpr float HPF_ALPHA = 0.8946f;
static float clarity_prev = 0.0f;
static constexpr float CLARITY_PREEMPH = 0.08f;
static constexpr float AUDIO_GAIN = 3.2f;
static constexpr int32_t AUDIO_LIMIT = 15000;

static volatile uint32_t su_audio_diag_sample_count = 0;
static volatile uint32_t su_audio_diag_clip_count = 0;
static volatile int32_t su_audio_diag_peak_pre_gain = 0;
static volatile int32_t su_audio_diag_peak_post_gain = 0;
static volatile uint32_t su_audio_diag_frame_size_error_count = 0;

static void Dat_Profile(SpeexProfileSU profile)
{
    if (!trang_thai_speex || profile == profile_hien_tai) return;
    int bitrate = (profile == SPEEX_PROFILE_HQ) ? 3950 : 2150;
    speex_encoder_ctl(trang_thai_speex, SPEEX_SET_BITRATE, &bitrate);
    profile_hien_tai = profile;
}

uint8_t Speex_SU_FrameBytes(SpeexProfileSU profile)
{
    return profile == SPEEX_PROFILE_HQ ? 10U : 6U;
}

void Speex_SU_ResetDiag()
{
    hpf_x_truoc = hpf_y_truoc = 0.0f;
    clarity_prev = 0.0f;
    su_audio_diag_sample_count = 0;
    su_audio_diag_clip_count = 0;
    su_audio_diag_peak_pre_gain = 0;
    su_audio_diag_peak_post_gain = 0;
    su_audio_diag_frame_size_error_count = 0;
}

SpeexAudioDiagSU Speex_SU_LayDiag()
{
    SpeexAudioDiagSU d = {};
    d.sample_count = su_audio_diag_sample_count;
    d.clip_count = su_audio_diag_clip_count;
    d.peak_pre_gain = su_audio_diag_peak_pre_gain;
    d.peak_post_gain = su_audio_diag_peak_post_gain;
    d.frame_size_error_count = su_audio_diag_frame_size_error_count;
    return d;
}

void KhoiTao_MayEp_Speex()
{
    speex_bits_init(&cac_bit_speex);
    trang_thai_speex = speex_encoder_init(&speex_nb_mode);
    int complexity = 10, tat = 0, vad = 0;
    speex_encoder_ctl(trang_thai_speex, SPEEX_SET_COMPLEXITY, &complexity);
    speex_encoder_ctl(trang_thai_speex, SPEEX_SET_VBR, &tat);
    speex_encoder_ctl(trang_thai_speex, SPEEX_SET_DTX, &tat);
    speex_encoder_ctl(trang_thai_speex, SPEEX_SET_VAD, &vad);
    profile_hien_tai = SPEEX_PROFILE_LQ; // force first switch
    Dat_Profile(SPEEX_PROFILE_HQ);
    Speex_SU_ResetDiag();
    Serial.println("Khoi tao Speex ADAPTIVE V2C4 | HQ=3950/10B | LQ=2150/6B | complexity=10 | HPF~150Hz | PREEMPH=0.08 | gain=3.2");
}

bool Nen_Thanh_KhungThoai_Profile(uint8_t *pcm_vao, uint8_t *khung_ra, SpeexProfileSU profile)
{
    Dat_Profile(profile);
    speex_bits_reset(&cac_bit_speex);
    int16_t *mau = (int16_t *)pcm_vao;
    for (int i=0;i<160;i++) {
        uint16_t raw = mau[i] & 0x0FFF;
        dc_offset_tich_luy = dc_offset_tich_luy - dc_offset_tich_luy/2048 + raw;
        int16_t dc = dc_offset_tich_luy/2048;
        float x=(float)((int32_t)raw-(int32_t)dc);
        float y=HPF_ALPHA*(hpf_y_truoc + x - hpf_x_truoc);
        hpf_x_truoc=x; hpf_y_truoc=y;
        int32_t ap=(int32_t)(x>=0?x:-x); if(ap>su_audio_diag_peak_pre_gain) su_audio_diag_peak_pre_gain=ap;
        float clear = y - CLARITY_PREEMPH * clarity_prev;
        clarity_prev = y;
        int32_t v=(int32_t)(clear*AUDIO_GAIN);
        int32_t av=v>=0?v:-v; if(av>su_audio_diag_peak_post_gain) su_audio_diag_peak_post_gain=av;
        if(v>AUDIO_LIMIT){v=AUDIO_LIMIT;su_audio_diag_clip_count++;}
        else if(v<-AUDIO_LIMIT){v=-AUDIO_LIMIT;su_audio_diag_clip_count++;}
        su_audio_diag_sample_count++;
        mau[i]=(int16_t)v;
    }
    speex_encode_int(trang_thai_speex, mau, &cac_bit_speex);
    const uint8_t expect=Speex_SU_FrameBytes(profile);
    int n=speex_bits_write(&cac_bit_speex,(char*)khung_ra,10);
    if(n!=expect){
        su_audio_diag_frame_size_error_count++;
        Serial.printf("[SU SPEEX SIZE ERROR] PROFILE=%s GOT=%d EXPECT=%u\n", profile==SPEEX_PROFILE_HQ?"HQ":"LQ",n,(unsigned)expect);
        return false;
    }
    return true;
}


bool Nen_Thanh_KhungThoai(uint8_t* pcm_vao, uint8_t* khung_thoai_ra)
{
    return Nen_Thanh_KhungThoai_Profile(pcm_vao, khung_thoai_ra, SPEEX_PROFILE_LQ);
}
