#ifndef SPEEX_CONFIG_TYPES_H
#define SPEEX_CONFIG_TYPES_H

#include <stdint.h>

typedef int16_t spx_int16_t;
typedef uint16_t spx_uint16_t;
typedef int32_t spx_int32_t;
typedef uint32_t spx_uint32_t;

#define OVERRIDE_SPEEX_WARNING
#define speex_warning(str)

#define OVERRIDE_SPEEX_WARNING_INT
#define speex_warning_int(str, val)

#define OVERRIDE_SPEEX_NOTIFY
#define speex_notify(str)

#define OVERRIDE_SPEEX_FATAL
#define speex_fatal(str)
#define _speex_fatal(str, file, line)

#define OVERRIDE_SPEEX_ERROR
#define speex_error(str)

#endif