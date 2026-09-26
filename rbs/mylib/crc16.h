#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stddef.h>

uint16_t CRC16_CCITT_FALSE(const uint8_t *data, size_t length);

#endif /* CRC16_H */
