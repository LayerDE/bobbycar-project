#pragma once
#include "c_data.h"

#ifdef __cplusplus
extern "C" {
#endif
extern const char endl4ptr;
extern const char newl4ptr;
extern const char* inputs[];

    bool exec(const char* command, c_data* out);
#ifdef __cplusplus
}
#endif