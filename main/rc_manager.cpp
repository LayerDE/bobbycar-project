#include <CrsfSerial.h>
#include "inputreader.h"
#include "config.h"
CrsfSerial *crsf;


static int typecast_throttle(unsigned int us_in){
    return ((int)us_in-1500)*2;
}

static float typecast_steering(unsigned int us_in){
    return (float)us_in-1500.0f * 0.0002f; //  pi mal daumen
}

static void packetChannels(){
    set_des_steering(typecast_steering(crsf->getChannel(1)),INPUT_RC);
    set_ext_throttle(typecast_throttle(crsf->getChannel(2)),INPUT_RC);
    if(crsf->getChannel(5)>1500)
        set_input_src(INPUT_RC);

}

static void LinkDownHandle(){
    set_des_steering(get_steering(),INPUT_RC); // stop pid
    set_ext_throttle(0,INPUT_RC); // stop throttle
}

void init_crsf()
{
    crsf = new CrsfSerial(Serial1, CRSF_BAUDRATE);
    // If something other than changing the baud of the UART needs to be done, do it here
    // Serial1.end(); Serial1.begin(500000, SERIAL_8N1, 16, 17);

    // Attach the channels callback
    crsf->onLinkDown = &LinkDownHandle;
    crsf->onPacketChannels = &packetChannels;
}

void crsf_task()
{
    // Must call CrsfSerial.loop() in loop() to process data
    crsf->loop();

}