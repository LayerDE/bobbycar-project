#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <stdbool.h>

#include <CRSF.hpp>
#include "inputreader.h"
#include "config.h"
#include "defines.h"
#include <math.h>
#include "math_functions.h"
#include <c_data.h>
CRSF crsf;

#define CHANNEL_MODE 6
#define CHANNEL_INPUT_SRC 4
#define CHANNEL_THROTTLE 2
#define CHANNEL_STEERING 0
#define CHANNEL_REVERSE 5
#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_MID  992
#define CRSF_CHANNEL_VALUE_MAX  1811
#define MODE_COUNT_MAIN 6

static int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int typecast_throttle(unsigned int us_in, int mode){
    int throttle = map(us_in, CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX,0,THROTTLE_MAX);
    switch(mode){
        case 0: // switch in reverse
            return throttle_calc(-throttle); // expo from adc
        case 1: //switch in the middle
        default:
            return 0;
        case 2: // switch in forward
            return throttle_calc(throttle); // expo from adc
    }
}

static float typecast_steering(unsigned int us_in){
    return ((float)us_in-(float)CRSF_CHANNEL_VALUE_MID) * 0.0007f; //  pi mal daumen
}


extern "C" void init_crsf()
{
    crsf.begin();
}

const int rc_ch_range = CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN;
int decode_mode(int ch_in, int mode_count){
    ch_in -= CRSF_CHANNEL_VALUE_MIN;
    for(int x = 0; x < mode_count-1; x++)
        if(ch_in < (rc_ch_range / mode_count)*(x+1))
            return x;
    return mode_count-1;
}

extern "C" void crsf_task()
{
    crsf.GetCrsfPacket();
    crsf.UpdateChannels();
    if(crsf.failsafe_status == CRSF_SIGNAL_OK){
        int forward_reverse = decode_mode(crsf.channels[CHANNEL_REVERSE],3);
        int mode = decode_mode(crsf.channels[CHANNEL_MODE],MODE_COUNT_MAIN);
        int throttle = typecast_throttle(crsf.channels[CHANNEL_THROTTLE],forward_reverse);
        float rc_steer = typecast_steering(crsf.channels[CHANNEL_STEERING]);
        if(get_input_src() == INPUT_RC){
            if(get_mode()!= mode)
                set_mode(mode);
        }
        set_des_steering(throttle,INPUT_RC);
        set_ext_throttle(rc_steer,INPUT_RC);
    }
    else{
        set_des_steering(0,INPUT_RC);
        set_ext_throttle(0,INPUT_RC);
    }
    switch(decode_mode(crsf.channels[CHANNEL_INPUT_SRC],3)){ // select input
        case 0:
            if(get_input_src() == INPUT_RC)
                set_input_src(DEFAULT_INPUT);
            break;
        case 1:
            break;// do nothing no change (middle position)
        case 2:
            set_input_src(INPUT_RC);
            break;
    }
}

extern "C" void dump_channels(c_data *out){
    char buffer[20];
    sprintf(buffer, "RC=");
    c_data_extend_raw(out, buffer, strlen(buffer));
    memset(buffer,'\0',20);
    for(int i = 0; i < 16; i++){
        sprintf(buffer, "%i: %i;%c",i,crsf.channels[i], i == 15 ? '\n':'\t');
        c_data_extend_raw(out, buffer, strlen(buffer));
        memset(buffer,'\0',20);
    }  
}