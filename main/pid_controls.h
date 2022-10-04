#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
void pid_update();
void init_pid();
void set_pid_kp(double kp);
void set_pid_ki(double ki);
void set_pid_kd(double kd);

double get_pid_kp();
double get_pid_ki();
double get_pid_kd();

void set_pid_refresh(int time_ms);
void set_pid_limit(double max);
#ifdef __cplusplus
}
#endif