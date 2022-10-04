#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t calc_crc32(const uint8_t *buffer,
    unsigned int length);

#ifdef __cplusplus
}
#endif