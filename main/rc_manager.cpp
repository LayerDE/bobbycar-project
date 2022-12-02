#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
    
#include <CRSF.hpp>
#include "inputreader.h"
#include "config.h"
#include "defines.h"
#include <math.h>
CRSF crsf;



static int typecast_throttle(unsigned int us_in){
    return ((int)us_in-1000)*2;
}

static float typecast_steering(unsigned int us_in){
    return ((float)us_in-1000.0f) * 0.0002f; //  pi mal daumen
}


extern "C" void init_crsf()
{
    crsf.begin();
}

extern "C" void crsf_task()
{
    crsf.GetCrsfPacket();
    if(crsf.failsafe_status == 0){
    }
    crsf.UpdateChannels();
    set_des_steering(typecast_steering(crsf.channels[0]),INPUT_RC);
    set_ext_throttle(typecast_throttle(crsf.channels[2]),INPUT_RC);
    if(crsf.channels[5]>1500)
        set_input_src(INPUT_RC);
        // Must call CrsfSerial.loop() in loop() to process data
        //crsf->loop();
}