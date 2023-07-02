#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_crsf();
void crsf_task();
bool rc_get_steering_pid_active();

#ifdef __cplusplus
}
#endif