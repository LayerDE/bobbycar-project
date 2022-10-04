#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void init_adc(int pin);
uint32_t read_voltage(int pin);

#ifdef __cplusplus
}
#endif