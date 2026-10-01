#include "nen_speex.h"
#include <speex/speex.h>
#include <math.h>

extern "C" { __attribute__((weak)) void * _impure_ptr = nullptr; }

static void *trang_thai_speex = nullptr;
static SpeexBits cac_bit_speex;
static SpeexProfileSU profile_hien_tai = SPEEX_PROFILE_HQ;

// =====================================================
// V2C4.12 NATURAL CLEAR DSP4 - SU ONLY / AI=OFF
//
// Muc tieu:
// - Lam sach noise nen them mot bac bang spectral subtraction/Wiener-like gain.
// - Giu spectral suppressor nhung ha mix khi co speech de giong tu nhien hon.
// - Lam trong hon nhe: giam low-mid 250-500 Hz va tang articulation 1.5-3 kHz.
// - KHONG doi TDMA/session/JOIN/PREPARE-COMMIT/AES/FEC/packet/rBS/DU/STM32.
// - KHONG doi Speex HQ3950/LQ2150, byte/frame, sample rate 8 kHz.
// - KHONG them queue/look-ahead/frame delay. Xu ly dung frame 160 mau hien tai.
// - Khong AI/TinyML. Day la DSP co dien trong mien tan so.
//
// De tranh musical-noise va an phu am, spectral gain duoc:
// - gioi han floor khac nhau speech/noise,
// - smooth theo tan so + theo thoi gian,
// - blend voi signal goc da HPF/pre-emphasis thay vi thay 100%.
// =====================================================

static constexpr int AUDIO_FRAME_N = 160;
static constexpr int FFT_N = 256;
static constexpr int FFT_HALF = FFT_N / 2;
static constexpr float SAMPLE_RATE_HZ = 8000.0f;
static constexpr float PI_F = 3.14159265358979323846f;

// Front-end giu cung tinh than DSP2, chi dieu chinh clarity rat nhe.
static int32_t dc_offset_tich_luy = 1551 * 2048;
static float hpf_x_truoc = 0.0f;
static float hpf_y_truoc = 0.0f;
static constexpr float HPF_ALPHA = 0.8878f;       // ~160 Hz @ 8 kHz
static float clarity_prev = 0.0f;
static constexpr float CLARITY_PREEMPH = 0.095f; // DSP2=0.08; tang rat nhe de ro phu am
static constexpr float AUDIO_GAIN = 3.05f;        // ha nhe de bu presence EQ, tranh harsh/clipping
static constexpr int32_t AUDIO_LIMIT = 15000;


// =====================================================
// V2C5.3 AUDIO-NOISE-FIX1
// Speech-band FIR + adaptive SOFT gate, before Speex.
//
// FIR: 63 taps, Hamming, Fs=8 kHz, passband ~180..3300 Hz.
// Group delay = 31 samples = 3.875 ms.
// Gate: adaptive noise floor, minimum gain 0.18 (never hard mute).
// No packet/TDMA/codec bitrate/RF-jam change.
// =====================================================
static constexpr const char *SU_AUDIO_NOISE_FIX1_TAG =
    "V2C53_AUDIO_NOISE_FIX1_FIR63_GATE";

static constexpr int SU_NF1_TAPS = 63;
static const float su_nf1_fir[SU_NF1_TAPS] =
{
    -0.000021627f,  0.001398831f,  0.000574800f,  0.000483434f,
     0.001918992f, -0.000808826f,  0.002706880f, -0.000860282f,
     0.000581956f,  0.001582680f, -0.004614954f,  0.003716795f,
    -0.008247514f, -0.000813146f, -0.005244680f, -0.013985467f,
     0.000852962f, -0.026455468f, -0.003281040f, -0.024190871f,
    -0.026545440f, -0.007007488f, -0.056118655f,  0.001578824f,
    -0.060142438f, -0.028946170f, -0.016027404f, -0.103198702f,
     0.060902613f, -0.184760156f,  0.121378358f,  0.781279806f,
     0.121378358f, -0.184760156f,  0.060902613f, -0.103198702f,
    -0.016027404f, -0.028946170f, -0.060142438f,  0.001578824f,
    -0.056118655f, -0.007007488f, -0.026545440f, -0.024190871f,
    -0.003281040f, -0.026455468f,  0.000852962f, -0.013985467f,
    -0.005244680f, -0.000813146f, -0.008247514f,  0.003716795f,
    -0.004614954f,  0.001582680f,  0.000581956f, -0.000860282f,
     0.002706880f, -0.000808826f,  0.001918992f,  0.000483434f,
     0.000574800f,  0.001398831f, -0.000021627f
};

static float su_nf1_hist[SU_NF1_TAPS] = {};
static int su_nf1_pos = 0;
static float su_nf1_noise_abs = 80.0f;
static float su_nf1_gate_gain = 1.0f;

static void SU_NoiseFix1_Reset()
{
    for (int i = 0; i < SU_NF1_TAPS; ++i)
        su_nf1_hist[i] = 0.0f;

    su_nf1_pos = 0;
    su_nf1_noise_abs = 80.0f;
    su_nf1_gate_gain = 1.0f;
}

static void SU_NoiseFix1_ProcessFrame(int16_t *pcm)
{
    if (pcm == nullptr)
        return;

    float filtered[160];
    float sum_abs = 0.0f;

    for (int n = 0; n < 160; ++n)
    {
        su_nf1_hist[su_nf1_pos] = (float)pcm[n];

        float acc = 0.0f;
        int idx = su_nf1_pos;
        for (int k = 0; k < SU_NF1_TAPS; ++k)
        {
            acc += su_nf1_fir[k] * su_nf1_hist[idx];
            --idx;
            if (idx < 0)
                idx = SU_NF1_TAPS - 1;
        }

        ++su_nf1_pos;
        if (su_nf1_pos >= SU_NF1_TAPS)
            su_nf1_pos = 0;

        filtered[n] = acc;
        sum_abs += (acc >= 0.0f) ? acc : -acc;
    }

    const float mean_abs = sum_abs / 160.0f;

    const float learn_limit =
        (su_nf1_noise_abs * 2.2f > 320.0f)
        ? su_nf1_noise_abs * 2.2f
        : 320.0f;

    if (mean_abs <= learn_limit)
        su_nf1_noise_abs = 0.970f * su_nf1_noise_abs + 0.030f * mean_abs;
    else
        su_nf1_noise_abs = 0.9995f * su_nf1_noise_abs + 0.0005f * mean_abs;

    if (su_nf1_noise_abs < 35.0f)
        su_nf1_noise_abs = 35.0f;
    if (su_nf1_noise_abs > 700.0f)
        su_nf1_noise_abs = 700.0f;

    float open_level = su_nf1_noise_abs * 2.35f;
    if (open_level < 150.0f)
        open_level = 150.0f;

    float full_level = su_nf1_noise_abs * 5.0f;
    if (full_level < open_level + 220.0f)
        full_level = open_level + 220.0f;

    float target_gain = 0.18f;

    if (mean_abs >= full_level)
    {
        target_gain = 1.0f;
    }
    else if (mean_abs > open_level)
    {
        const float u =
            (mean_abs - open_level) / (full_level - open_level);
        target_gain = 0.18f + 0.82f * u;
    }

    const float alpha =
        (target_gain > su_nf1_gate_gain) ? 0.65f : 0.08f;

    su_nf1_gate_gain +=
        alpha * (target_gain - su_nf1_gate_gain);

    for (int n = 0; n < 160; ++n)
    {
        float v = filtered[n] * su_nf1_gate_gain;

        if (v > 15000.0f)
            v = 15000.0f;
        else if (v < -15000.0f)
            v = -15000.0f;

        pcm[n] = (int16_t)v;
    }
}


// Frame-level VAD/noise floor. Khong mute cung.
static constexpr float NS_NOISE_INIT_RMS = 100.0f;
static constexpr float NS_NOISE_MIN_RMS = 18.0f;
static constexpr float NS_NOISE_MAX_RMS = 450.0f;
static constexpr float NS_FRAME_MIN_GAIN = 0.50f; // ~-6 dB chi o frame noise-like
static constexpr float NS_RATIO_NOISE = 1.12f;
static constexpr float NS_RATIO_VOICE = 1.68f;
static constexpr uint8_t NS_HANGOVER_FRAMES = 8U; // 160 ms
static constexpr uint8_t NS_WARMUP_FRAMES = 8U;   // 160 ms pass-through spectral

// Spectral suppressor.
// SPEECH floor cao hon de giu phu am; NOISE floor thap hon de yen hon khi nghi.
static constexpr float SPEC_FLOOR_SPEECH = 0.40f; // ~-8 dB
static constexpr float SPEC_FLOOR_NOISE  = 0.25f; // ~-12 dB
static constexpr float SPEC_OVERSUB_SPEECH = 0.90f;
static constexpr float SPEC_OVERSUB_NOISE  = 1.20f;
static constexpr float SPEC_MIX_SPEECH = 0.60f;
static constexpr float SPEC_MIX_NOISE  = 0.82f;
static constexpr float SPEC_GAIN_UP_ALPHA = 0.62f;   // mo nhanh
static constexpr float SPEC_GAIN_DOWN_ALPHA = 0.22f; // dong cham, giam musical noise

static float ns_noise_rms = NS_NOISE_INIT_RMS;
static float ns_gain_state = 1.0f;
static uint8_t ns_hangover = 0U;
static uint32_t ns_frame_count = 0U;

// FFT scratch/state. Static de khong an stack cua task audio.
static float fft_re[FFT_N];
static float fft_im[FFT_N];
static float fft_tw_cos[FFT_HALF];
static float fft_tw_sin[FFT_HALF];
static bool fft_twiddle_ready = false;

static float spec_noise_power[FFT_HALF + 1];
static float spec_gain_state[FFT_HALF + 1];
static float spec_gain_target[FFT_HALF + 1];
static float spec_gain_freq[FFT_HALF + 1];
static bool spec_noise_ready = false;

static volatile uint32_t su_audio_diag_sample_count = 0;
static volatile uint32_t su_audio_diag_clip_count = 0;
static volatile int32_t su_audio_diag_peak_pre_gain = 0;
static volatile int32_t su_audio_diag_peak_post_gain = 0;
static volatile uint32_t su_audio_diag_frame_size_error_count = 0;

static inline float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float absf_local(float v)
{
    return v >= 0.0f ? v : -v;
}

static void FFT_InitTwiddle()
{
    if (fft_twiddle_ready) return;
    for (int k = 0; k < FFT_HALF; ++k)
    {
        const float a = 2.0f * PI_F * (float)k / (float)FFT_N;
        fft_tw_cos[k] = cosf(a);
        fft_tw_sin[k] = sinf(a);
    }
    fft_twiddle_ready = true;
}

static void FFT_256(float *re, float *im, bool inverse)
{
    // Bit reversal.
    unsigned int j = 0U;
    for (unsigned int i = 1U; i < (unsigned int)FFT_N; ++i)
    {
        unsigned int bit = (unsigned int)FFT_N >> 1U;
        while (j & bit)
        {
            j ^= bit;
            bit >>= 1U;
        }
        j ^= bit;
        if (i < j)
        {
            const float tr = re[i]; re[i] = re[j]; re[j] = tr;
            const float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }

    for (unsigned int len = 2U; len <= (unsigned int)FFT_N; len <<= 1U)
    {
        const unsigned int half = len >> 1U;
        const unsigned int step = (unsigned int)FFT_N / len;
        for (unsigned int base = 0U; base < (unsigned int)FFT_N; base += len)
        {
            for (unsigned int k = 0U; k < half; ++k)
            {
                const unsigned int wi = k * step;
                const float wr = fft_tw_cos[wi];
                const float ws = inverse ? fft_tw_sin[wi] : -fft_tw_sin[wi];

                const unsigned int ia = base + k;
                const unsigned int ib = ia + half;

                const float br = re[ib];
                const float bi = im[ib];
                const float vr = br * wr - bi * ws;
                const float vi = br * ws + bi * wr;
                const float ar = re[ia];
                const float ai = im[ia];

                re[ia] = ar + vr;
                im[ia] = ai + vi;
                re[ib] = ar - vr;
                im[ib] = ai - vi;
            }
        }
    }

    if (inverse)
    {
        const float inv_n = 1.0f / (float)FFT_N;
        for (int i = 0; i < FFT_N; ++i)
        {
            re[i] *= inv_n;
            im[i] *= inv_n;
        }
    }
}

// EQ rat nhe de "trong" hon ma khong bien thanh giong mong/xi.
// Thuc hien trong spectral domain nen khong them filter state/latency.
static float Spectral_ClarityEQ(float freq_hz)
{
    if (freq_hz < 120.0f) return 0.22f;
    if (freq_hz < 220.0f)
    {
        const float t = (freq_hz - 120.0f) / 100.0f;
        return 0.22f + 0.70f * t;
    }
    if (freq_hz < 500.0f) return 0.84f;  // low-mid ~-1.5 dB, bot om nhe
    if (freq_hz < 1000.0f) return 0.96f;
    if (freq_hz < 1500.0f) return 1.00f;
    if (freq_hz < 2800.0f) return 1.12f; // presence ~+1.0 dB, ro phu am hon
    if (freq_hz < 3350.0f) return 1.06f;
    if (freq_hz < 3600.0f)
    {
        const float t = (freq_hz - 3350.0f) / 250.0f;
        return 1.06f - 0.30f * t;
    }
    // Gan Nyquist cat mem hiss/EMI, khong hard-zero de tranh ringing.
    const float t = clampf_local((freq_hz - 3600.0f) / 400.0f, 0.0f, 1.0f);
    return 0.76f - 0.51f * t;
}

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
        SU_NoiseFix1_Reset();
hpf_x_truoc = 0.0f;
    hpf_y_truoc = 0.0f;
    clarity_prev = 0.0f;

    ns_noise_rms = NS_NOISE_INIT_RMS;
    ns_gain_state = 1.0f;
    ns_hangover = 0U;
    ns_frame_count = 0U;

    spec_noise_ready = false;
    for (int k = 0; k <= FFT_HALF; ++k)
    {
        spec_noise_power[k] = 1.0f;
        spec_gain_state[k] = 1.0f;
        spec_gain_target[k] = 1.0f;
        spec_gain_freq[k] = 1.0f;
    }

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
    FFT_InitTwiddle();

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

    Serial.println(
        "Khoi tao Speex ADAPTIVE + AUDIO_DSP=V2C4.12_NATURAL_CLEAR4 | "
        "HQ=3950/10B | LQ=2150/6B | HPF~160Hz | PREEMPH=0.095 | "
        "FFT=256 | SPEC_NS=ON | LOWMID=-1.5dB | PRESENCE=+1.0dB | SPEC_MIX_SPEECH=0.60 | "
        "GAIN=3.05 | LOOKAHEAD=0 | AI=OFF"
    );
}

bool Nen_Thanh_KhungThoai_Profile(
    uint8_t *pcm_vao,
    uint8_t *khung_ra,
    SpeexProfileSU profile)
{
    Dat_Profile(profile);
    speex_bits_reset(&cac_bit_speex);

    int16_t *mau = (int16_t *)pcm_vao;
    float clean_frame[AUDIO_FRAME_N];
    float energy_sum = 0.0f;

    // -------------------------------------------------
    // PASS 1: DC -> HPF -> pre-emphasis. Khong them look-ahead.
    // -------------------------------------------------
    for (int i = 0; i < AUDIO_FRAME_N; ++i)
    {
        const uint16_t raw = (uint16_t)(mau[i] & 0x0FFF);

        dc_offset_tich_luy =
            dc_offset_tich_luy
            - (dc_offset_tich_luy / 2048)
            + raw;

        const int16_t dc = (int16_t)(dc_offset_tich_luy / 2048);
        const float x = (float)((int32_t)raw - (int32_t)dc);

        const float y =
            HPF_ALPHA
            * (hpf_y_truoc + x - hpf_x_truoc);

        hpf_x_truoc = x;
        hpf_y_truoc = y;

        const int32_t ap = (int32_t)absf_local(x);
        if (ap > su_audio_diag_peak_pre_gain)
            su_audio_diag_peak_pre_gain = ap;

        energy_sum += y * y;

        const float clear = y - CLARITY_PREEMPH * clarity_prev;
        clarity_prev = y;
        clean_frame[i] = clear;
    }

    const float frame_rms = sqrtf(energy_sum / (float)AUDIO_FRAME_N + 1.0e-6f);

    // -------------------------------------------------
    // FRAME VAD / NOISE FLOOR: chi dung de quyet dinh toc do hoc noise va floor.
    // -------------------------------------------------
    bool speech_hold = true;
    float frame_target_gain = 1.0f;

    if (ns_frame_count < NS_WARMUP_FRAMES)
    {
        if (frame_rms < ns_noise_rms)
            ns_noise_rms += 0.18f * (frame_rms - ns_noise_rms);
        speech_hold = true;
        frame_target_gain = 1.0f;
    }
    else
    {
        const float ratio_before = frame_rms / (ns_noise_rms + 1.0f);
        const bool strong_voice = ratio_before >= NS_RATIO_VOICE;

        if (strong_voice)
            ns_hangover = NS_HANGOVER_FRAMES;
        else if (ns_hangover > 0U)
            ns_hangover--;

        speech_hold = strong_voice || (ns_hangover > 0U);

        if (!speech_hold)
        {
            const float alpha = (frame_rms < ns_noise_rms) ? 0.12f : 0.018f;
            ns_noise_rms += alpha * (frame_rms - ns_noise_rms);
            ns_noise_rms = clampf_local(ns_noise_rms, NS_NOISE_MIN_RMS, NS_NOISE_MAX_RMS);
        }
        else if (frame_rms < ns_noise_rms)
        {
            ns_noise_rms += 0.02f * (frame_rms - ns_noise_rms);
        }

        const float ratio = frame_rms / (ns_noise_rms + 1.0f);
        if (speech_hold)
        {
            frame_target_gain = 1.0f;
        }
        else
        {
            const float t = clampf_local(
                (ratio - NS_RATIO_NOISE) / (NS_RATIO_VOICE - NS_RATIO_NOISE),
                0.0f,
                1.0f
            );
            frame_target_gain = NS_FRAME_MIN_GAIN + (1.0f - NS_FRAME_MIN_GAIN) * t;
        }
    }

    // -------------------------------------------------
    // FFT 256: 160 sample hien tai + zero pad. KHONG doi frame duration.
    // -------------------------------------------------
    for (int i = 0; i < FFT_N; ++i)
    {
        fft_re[i] = (i < AUDIO_FRAME_N) ? clean_frame[i] : 0.0f;
        fft_im[i] = 0.0f;
    }

    FFT_256(fft_re, fft_im, false);

    // Khoi tao noise spectrum bao thu: 12% power frame dau, de khong hoc speech thanh noise.
    if (!spec_noise_ready)
    {
        for (int k = 0; k <= FFT_HALF; ++k)
        {
            const float p = fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k];
            spec_noise_power[k] = 0.12f * p + 1.0f;
            spec_gain_state[k] = 1.0f;
        }
        spec_noise_ready = true;
    }

    const bool spectral_warm = ns_frame_count < NS_WARMUP_FRAMES;
    const float spec_floor = speech_hold ? SPEC_FLOOR_SPEECH : SPEC_FLOOR_NOISE;
    const float oversub = speech_hold ? SPEC_OVERSUB_SPEECH : SPEC_OVERSUB_NOISE;

    // -------------------------------------------------
    // Spectral noise estimate + raw gain.
    // Noise PSD chi hoc dang ke khi KHONG co speech; trong speech hoc rat cham.
    // -------------------------------------------------
    for (int k = 0; k <= FFT_HALF; ++k)
    {
        const float p = fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k] + 1.0e-6f;

        if (!spectral_warm)
        {
            float alpha_noise;
            if (!speech_hold)
            {
                alpha_noise = (p < spec_noise_power[k]) ? 0.16f : 0.035f;
            }
            else
            {
                alpha_noise = (p < spec_noise_power[k]) ? 0.010f : 0.0015f;
            }
            spec_noise_power[k] += alpha_noise * (p - spec_noise_power[k]);
            if (spec_noise_power[k] < 1.0e-3f) spec_noise_power[k] = 1.0e-3f;
        }

        float g = 1.0f;
        if (!spectral_warm)
        {
            const float noise_ratio = spec_noise_power[k] / p;
            const float remain = 1.0f - oversub * noise_ratio;
            g = remain > 0.0f ? sqrtf(remain) : 0.0f;
            if (g < spec_floor) g = spec_floor;
        }

        const float freq = (float)k * SAMPLE_RATE_HZ / (float)FFT_N;
        g *= Spectral_ClarityEQ(freq);
        spec_gain_target[k] = clampf_local(g, 0.12f, 1.12f);
    }

    // Frequency smoothing 3-bin de giam musical noise / ringing.
    spec_gain_freq[0] = 0.75f * spec_gain_target[0] + 0.25f * spec_gain_target[1];
    for (int k = 1; k < FFT_HALF; ++k)
    {
        spec_gain_freq[k] =
            0.25f * spec_gain_target[k - 1]
            + 0.50f * spec_gain_target[k]
            + 0.25f * spec_gain_target[k + 1];
    }
    spec_gain_freq[FFT_HALF] =
        0.25f * spec_gain_target[FFT_HALF - 1]
        + 0.75f * spec_gain_target[FFT_HALF];

    // Time smoothing + apply doi xung cho real signal.
    for (int k = 0; k <= FFT_HALF; ++k)
    {
        const float target = spec_gain_freq[k];
        const float alpha = (target > spec_gain_state[k])
            ? SPEC_GAIN_UP_ALPHA
            : SPEC_GAIN_DOWN_ALPHA;

        spec_gain_state[k] += alpha * (target - spec_gain_state[k]);
        const float g = spec_gain_state[k];

        fft_re[k] *= g;
        fft_im[k] *= g;

        if (k > 0 && k < FFT_HALF)
        {
            const int mirror = FFT_N - k;
            fft_re[mirror] *= g;
            fft_im[mirror] *= g;
        }
    }

    FFT_256(fft_re, fft_im, true);

    // Frame gain mo nhanh/dong cham; nhe hon DSP2 vi spectral da lam phan chinh.
    const float frame_alpha = (frame_target_gain > ns_gain_state) ? 0.72f : 0.16f;
    const float frame_gain_start = ns_gain_state;
    const float frame_gain_end = ns_gain_state + frame_alpha * (frame_target_gain - ns_gain_state);
    ns_gain_state = frame_gain_end;

    // -------------------------------------------------
    // OUTPUT: blend spectral + original de giu tu nhien, sau do gain/limiter.
    // -------------------------------------------------
    const float spec_mix = spectral_warm
        ? 0.0f
        : (speech_hold ? SPEC_MIX_SPEECH : SPEC_MIX_NOISE);

    for (int i = 0; i < AUDIO_FRAME_N; ++i)
    {
        const float pos = (float)(i + 1) / (float)AUDIO_FRAME_N;
        const float frame_gain = frame_gain_start + (frame_gain_end - frame_gain_start) * pos;

        const float spectral = fft_re[i];
        const float natural = clean_frame[i];
        const float mixed = natural + spec_mix * (spectral - natural);

        int32_t v = (int32_t)(mixed * frame_gain * AUDIO_GAIN);

        const int32_t av = (v >= 0) ? v : -v;
        if (av > su_audio_diag_peak_post_gain)
            su_audio_diag_peak_post_gain = av;

        if (v > AUDIO_LIMIT)
        {
            v = AUDIO_LIMIT;
            su_audio_diag_clip_count++;
        }
        else if (v < -AUDIO_LIMIT)
        {
            v = -AUDIO_LIMIT;
            su_audio_diag_clip_count++;
        }

        su_audio_diag_sample_count++;
        mau[i] = (int16_t)v;
    }

    ns_frame_count++;

    // Speex va packet size GIU NGUYEN.
    // V2C5.3 AUDIO-NOISE-FIX1: speech-band FIR + adaptive soft gate.
    SU_NoiseFix1_ProcessFrame(mau);
    speex_encode_int(trang_thai_speex, mau, &cac_bit_speex);

    const uint8_t expect = Speex_SU_FrameBytes(profile);
    const int n = speex_bits_write(&cac_bit_speex, (char *)khung_ra, 10);

    if (n != expect)
    {
        su_audio_diag_frame_size_error_count++;
        Serial.printf(
            "[SU SPEEX SIZE ERROR] PROFILE=%s GOT=%d EXPECT=%u\n",
            profile == SPEEX_PROFILE_HQ ? "HQ" : "LQ",
            n,
            (unsigned int)expect
        );
        return false;
    }

    return true;
}

bool Nen_Thanh_KhungThoai(uint8_t *pcm_vao, uint8_t *khung_thoai_ra)
{
    return Nen_Thanh_KhungThoai_Profile(
        pcm_vao,
        khung_thoai_ra,
        SPEEX_PROFILE_LQ
    );
}
