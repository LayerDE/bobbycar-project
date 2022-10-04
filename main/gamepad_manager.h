#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void init_gpm();
void gpm_read(int *throttle,int *steering, int *active);


#ifdef __cplusplus
}
#endif