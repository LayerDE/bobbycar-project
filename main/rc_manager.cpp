#include <CrsfSerial.h>

CrsfSerial *crsf;

static void packetChannels()
{
    //Serial.print("CH1=");
    //Serial.println(crsf.getChannel(1));
}

void init_crsf()
{
    crsf = new CrsfSerial(Serial1, CRSF_BAUDRATE);
    // If something other than changing the baud of the UART needs to be done, do it here
    // Serial1.end(); Serial1.begin(500000, SERIAL_8N1, 16, 17);

    // Attach the channels callback
    crsf->onPacketChannels = &packetChannels;
}

void crsf_task()
{
    // Must call CrsfSerial.loop() in loop() to process data
    crsf->loop();
}