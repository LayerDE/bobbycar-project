#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
    float rad2deg(float in);
    float deg2rad(float in);
    void init_buffer();
    uint32_t value_buffer(uint32_t in, int val);
    // bobbycar
    int sign(float in);
    int clean_adc_full(uint32_t inval);
    int clean_adc_steering(uint32_t inval);
    unsigned int clean_adc_half(uint32_t inval);
    int throttle_calc(int cleaned_adc);
    int calc_torque(int throttle, int breaks);
    float calc_steering_eagle(int inval);
    void calc_torque_per_wheel(int throttle, float steering_eagle,int torque_regulated, int *torque);
    int calc_median(int x[], int cnt);
    
#ifdef __cplusplus
}
#endif