#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void init_gpm();
void gpm_read(int *throttle,int *steering, int *active);
bool get_gamepad_connected();
void reset_gamepads();

#ifdef __cplusplus
}
#endif