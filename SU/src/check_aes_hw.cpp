#include "sdkconfig.h"

#ifdef CONFIG_MBEDTLS_HARDWARE_AES
#pragma message("VEDC_AES_HW=ON")
#else
#pragma message("VEDC_AES_HW=OFF")
#endif

#ifdef CONFIG_MBEDTLS_HARDWARE_GCM
#pragma message("VEDC_GCM_HW=ON")
#else
#pragma message("VEDC_GCM_HW=OFF")
#endif
