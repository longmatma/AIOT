#ifndef KHOA_GOC_LOCAL_H
#define KHOA_GOC_LOCAL_H

#include <Arduino.h>

// =====================================================
// VEDC ROOT KEY - PRIVATE LOCAL KEY
//
// TUYET DOI KHONG commit file nay len Git/GitHub.
// SU va DU phai dung CUNG mot root key.
// rBS khong can biet root key vi hien tai chi relay ciphertext.
// =====================================================

#define VEDC_ROOT_KEY_LEN 32

static const uint8_t VEDC_ROOT_KEY[VEDC_ROOT_KEY_LEN] =
{
    0x30, 0x71, 0x24, 0x08, 0xFA, 0xB0, 0x4B, 0x66,
    0x67, 0x62, 0xD2, 0xFC, 0xDD, 0xF4, 0xEA, 0x20,
    0x22, 0x07, 0x6C, 0xC6, 0xC3, 0xC1, 0x04, 0x05,
    0x06, 0x9D, 0x1B, 0x62, 0xAF, 0xFF, 0x28, 0x46
};

#endif
