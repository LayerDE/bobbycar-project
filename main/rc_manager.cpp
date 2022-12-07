#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
    
#include <CRSF.hpp>
#include "inputreader.h"
#include "config.h"
#include "defines.h"
#include <math.h>
#include "math_functions.h"
CRSF crsf;

static int typecast_throttle(unsigned int us_in){
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

extern "C" void crsf_task()
{
    crsf.GetCrsfPacket();
    crsf.UpdateChannels();
    if(crsf.failsafe_status == CRSF_SIGNAL_OK){
        set_des_steering(typecast_steering(crsf.channels[0]),INPUT_RC);
        set_ext_throttle(typecast_throttle(crsf.channels[2]),INPUT_RC);
        //printf("T: %i,%i; S: %i,%f; Switch: %i\n",crsf.channels[2],get_throttle(),crsf.channels[0],rad2deg(get_des_steering()), crsf.channels[5]);
    }
    else{
        //if(get_input_src() == INPUT_RC)
        //    set_input_src(INPUT_ADC);
        //set_des_steering(0,INPUT_RC);
        //set_ext_throttle(0,INPUT_RC);
    }
    if(crsf.channels[5]>1500 && get_input_src() != INPUT_RC)
        set_input_src(INPUT_RC);
    else if(get_input_src() == INPUT_RC && crsf.channels[5]<500)
        set_input_src(INPUT_ADC);
        // Must call CrsfSerial.loop() in loop() to process data
        //crsf->loop();
    //printf("CH: %i %i %i %i %i %i\n",crsf.channels[0],crsf.channels[1],crsf.channels[2],crsf.channels[3],crsf.channels[4],crsf.channels[5]);
}