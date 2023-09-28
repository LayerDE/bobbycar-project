#pragma once
#include <stdbool.h>
#include "c_data.h"

#ifdef __cplusplus
extern "C"
{
#endif
bool reset_cmd(const char* argv, c_data* out);
#ifdef __cplusplus
}
#endif