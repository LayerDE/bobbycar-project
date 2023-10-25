#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_crsf();
void crsf_task();

void dump_channels(c_data *out);

#ifdef __cplusplus
}
#endif