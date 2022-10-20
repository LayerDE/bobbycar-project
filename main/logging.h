#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void start_log();
int dump_log();
void add_log(double po, unsigned long time, float steer_real, float desired_steering);
#ifdef __cplusplus
}
#endif