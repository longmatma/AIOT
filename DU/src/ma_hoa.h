#ifndef MA_HOA_H
#define MA_HOA_H

#include <Arduino.h>
#include "mbedtls/gcm.h"

// CHÌA KHÓA BÍ MẬT 128-bit (16 byte)
const unsigned char KHOA_BIMAT_AES[16] = {
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

// ----------------------------------------------------
// HÀM 1: MÃ HÓA VÀ ĐÓNG TEM (Dành cho trạm phát SU)
// ----------------------------------------------------
inline void MaHoa_GCM(uint8_t* header, uint8_t* payload_vao, uint8_t* payload_ra, uint8_t* auth_tag, uint16_t so_thu_tu) {
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, KHOA_BIMAT_AES, 128);

    // Tạo IV 12 byte (Chuẩn bắt buộc của GCM). Ghép Số thứ tự vào cuối.
    unsigned char iv[12] = {0};
    iv[10] = (so_thu_tu >> 8) & 0xFF;
    iv[11] = so_thu_tu & 0xFF;

    // Quăng vào Hardware Crypto Accelerator của ESP32
    mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, 
                              40, // Chiều dài âm thanh thô
                              iv, 12, 
                              header, 6, // AAD (Header không mã hóa nhưng được bảo vệ)
                              payload_vao, payload_ra, 
                              8, auth_tag); // Sinh ra 8 byte Auth Tag
    
    mbedtls_gcm_free(&gcm);
}

// ----------------------------------------------------
// HÀM 2: KIỂM TRA TEM VÀ GIẢI MÃ (Dành cho trạm nhận DU)
// ----------------------------------------------------
inline bool GiaiMa_GCM(uint8_t* header, uint8_t* payload_vao, uint8_t* payload_ra, uint8_t* auth_tag, uint16_t so_thu_tu) {
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, KHOA_BIMAT_AES, 128);

    unsigned char iv[12] = {0};
    iv[10] = (so_thu_tu >> 8) & 0xFF;
    iv[11] = so_thu_tu & 0xFF;

    // Hàm trả về 0 nếu tem xịn và giải mã thành công, khác 0 là tem rách/dữ liệu giả
    int ket_qua = mbedtls_gcm_auth_decrypt(&gcm, 
                                           40, 
                                           iv, 12, 
                                           header, 6, 
                                           auth_tag, 8, 
                                           payload_vao, payload_ra);
    
    mbedtls_gcm_free(&gcm);
    return (ket_qua == 0); 
}

#endif