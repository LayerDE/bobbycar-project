#include <stdbool.h>
#include "Arduino.h"
#include "c_data.h"
#include "data.hpp"
#include "display.hpp"

bool dsp_connected;

display *lcd;

extern "C" bool reset_cmd(const char* argv, c_data* out){
    ESP.restart();
    return true;
}