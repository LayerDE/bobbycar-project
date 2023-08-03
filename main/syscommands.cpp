#include <stdbool.h>
#include "Arduino.h"
#include "c_data.h"
#include "data.hpp"

extern "C" bool reset_cmd(const char* argv, c_data* out){
    ESP.restart();
    return true;
}