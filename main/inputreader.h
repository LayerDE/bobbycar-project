#pragma once

#ifdef __cplusplus
extern "C" {
#endif
//extern volatile float adc_steering;
//extern volatile float desired_steering;

bool get_steering_pid_active();

unsigned int get_input_src();
void set_input_src(unsigned int src);

int get_throttle();
float get_steering();
float get_des_steering();
float get_pid_steer();
void set_pid_steer(float in);
void set_des_steering(float steering, unsigned int src);
void set_ext_throttle(int throttle, unsigned int src);

bool get_steering_pid_active();

bool get_trailer_connected();
float get_trailer();


void init_gamepad(void* ignore);
void init_adc_task(void* ignore);

void init_rc_in(void* ignore);
void rc_in_task(void* ignore);

void gamepad_task(void* ignore);
void adc_task(void* ignore);
#ifdef __cplusplus
}
#endif