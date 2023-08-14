#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void set_log_active(bool in);
int dump_log();
void add_log(double po, unsigned long time, float steer_real, float desired_steering, float trailer);
bool is_dumping();
#ifdef __cplusplus
}
#endif