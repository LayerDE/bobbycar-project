#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

//#include "adc1_read.h"
#include "math_functions.h"
#include "gamepad_manager.h"
#include "rc_manager.h"
#include "config.h"
#include <Arduino.h>
#include <stddef.h>

volatile static int adc_throttle;
volatile static int adc_steering;
volatile static int adc_follower;
volatile static int external_throttle[INPUT_COUNT-1];
volatile static float desired_steering[INPUT_COUNT-1];
volatile static int input_src = DEFAULT_INPUT;
volatile static float regulated_steering_factor = 0;

float get_pid_steer(){
    return regulated_steering_factor;
}
void set_pid_steer(float in){
    regulated_steering_factor = in;
}

unsigned int get_input_src(){
    return input_src;
}

float get_steering(){
    return calc_steering_eagle(adc_steering);
}

float get_des_steering(){
    if(input_src == INPUT_ADC)
        return get_steering();
    else
        return desired_steering[input_src-1];
}

void set_des_steering(float steering, unsigned int src){
    if(src == INPUT_ADC)
        return;
    if(src < INPUT_COUNT)
        desired_steering[src-1] = steering;
}

void set_input_src(unsigned int src){
    if(src >= INPUT_COUNT || src == get_input_src())
        return;
    if(src != 0)
        set_des_steering(get_steering(), src);
    input_src = src;
}

int get_throttle(){
    if(input_src == 0)
        return throttle_calc(adc_throttle);
    else
        return external_throttle[input_src-1];
}

static inline bool check_trottle(int throttle){
    if(throttle < THROTTLE_MAX && throttle > -THROTTLE_REVERSE_MAX)
        return true;
    else
        return false;
}

void set_ext_throttle(int throttle, unsigned int src){
    if(check_trottle(throttle) && src < INPUT_COUNT && src != 0)
        external_throttle[src - 1] = throttle;
}

bool get_contoller_active(){
    return input_src == INPUT_GAMEPAD;
}

void init_gamepad(void* ignore){
    init_gpm();
    vTaskDelete(NULL);
}

void init_adc_task(void* ignore){
    pinMode(STEERING_PIN, INPUT);
    #if(THROTTLE0_PIN != THROTTLE1_PIN)
        pinMode(THROTTLE1_PIN, INPUT);
    #endif
    pinMode(THROTTLE0_PIN, INPUT);
    vTaskDelete(NULL);
}

void init_auto_reveser(void* ignore){
    vTaskDelete(NULL);
}

void gamepad_task(void* ignore){
    int pad_data[6];
    int tmp;
    while(true){
        tmp = input_src;
        gpm_read(&pad_data[0],&pad_data[1],&tmp);
        //external_throttle[1] = pad_data[0];
        set_ext_throttle(pad_data[0],INPUT_GAMEPAD);
        set_des_steering(pad_data[1],INPUT_GAMEPAD);
        //desired_steering[1] = pad_data[1];
        input_src = tmp;
        vTaskDelay(20);
    }
    vTaskDelete(NULL);
}

void adc_task(void* ignore){
    while(true){
        adc_steering = clean_adc_steering(value_buffer(analogRead(STEERING_PIN),0));
        //if(input_src == 0){
            #if(THROTTLE0_PIN != THROTTLE1_PIN)
                adc_throttle = clean_adc_half(value_buffer(analogRead(THROTTLE0_PIN),1)) - clean_adc_half(value_buffer(analogRead(THROTTLE1_PIN),2));
            #else
                adc_throttle = clean_adc_full(value_buffer(analogRead(THROTTLE0_PIN),1));
            #endif
        //}
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

void init_rc_in(void* ignore){
    init_crsf();
    //printf("end init crsf\n");
    //vTaskDelay(1000);
    vTaskDelete(NULL);
}

void rc_in_task(void* ignore){
    while(true){
        crsf_task();
        vTaskDelay(10);
    }
    vTaskDelete(NULL);
}

void auto_reverser_task(void* ignore){
    vTaskDelete(NULL);
}