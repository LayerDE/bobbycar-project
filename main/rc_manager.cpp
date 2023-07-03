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
CRSF crsf;

static bool active_steering;
static bool trailer_connected;
static bool trailer_control;
static int last_mode;

static int typecast_throttle(unsigned int us_in, bool reverse){
    int throttle = ((float)us_in-1000.0f)*1.23f;
    if(throttle < 0)
        return CLAMP(throttle *3 / 10,-300,0);
    else
        return CLAMP(throttle,0,1000);
}

static float typecast_steering(unsigned int us_in){
    return ((float)us_in-1000.0f) * 0.0007f; //  pi mal daumen
}


extern "C" void init_crsf()
{
    crsf.begin();
}

const int mode_count_main = 6;
const int rc_ch_range = 2000;
int decode_mode(int ch_in, int mode_count){
    for(int x = 0; x < mode_count-1; x++)
        if(ch_in < (rc_ch_range / mode_count)*(x+1))
            return x;
    return mode_count-1;
}

bool crsf_get_active_steering(){
    return active_steering;
}

extern "C" bool rc_get_steering_pid_active(){
    return active_steering;
}

extern "C" void crsf_task()
{
    crsf.GetCrsfPacket();
    crsf.UpdateChannels();
    if(crsf.failsafe_status == CRSF_SIGNAL_OK){
        bool reverse = decode_mode(crsf.channels[7],2);
        int mode = decode_mode(crsf.channels[6],mode_count_main);
        if(last_mode != mode)
            switch(last_mode = mode){
                case 0: // rc no trailer
                    active_steering = true;
                    trailer_connected = false;
                    trailer_control = false;
                    break;
                case 1: // rc trailer protection
                    active_steering = true;
                    trailer_connected = true;
                    trailer_control = false;
                    break;
                case 2: //rc trailer control and protection
                    active_steering = true;
                    trailer_connected = true;
                    trailer_control = true;
                    break;
                case 3: //rc kids mode (no steering)
                    active_steering = false;
                    trailer_connected = false;
                    trailer_control = false;
                    break;
            }
        set_des_steering(typecast_steering(crsf.channels[0]),INPUT_RC);
        set_ext_throttle(typecast_throttle(crsf.channels[2],reverse),INPUT_RC);
        //printf("T: %i,%i; S: %i,%f; Switch: %i\n",crsf.channels[2],get_throttle(),crsf.channels[0],rad2deg(get_des_steering()), crsf.channels[5]);
    }
    else{
        if(get_input_src() == INPUT_RC)
            set_input_src(INPUT_ADC);
        set_des_steering(0,INPUT_RC);
        set_ext_throttle(0,INPUT_RC);
        active_steering = false;
        trailer_connected = false;
        trailer_control = false;
    }
    if(crsf.channels[4]>1500 && get_input_src() != INPUT_RC)
        set_input_src(INPUT_RC);
    else if(get_input_src() == INPUT_RC && crsf.channels[5]<500)
        set_input_src(INPUT_ADC);
        // Must call CrsfSerial.loop() in loop() to process data
        //crsf->loop();
    //printf("CH: %i %i %i %i %i %i\n",crsf.channels[0],crsf.channels[1],crsf.channels[2],crsf.channels[3],crsf.channels[4],crsf.channels[5]);
}