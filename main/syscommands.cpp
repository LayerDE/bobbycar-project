#include <stdbool.h>
#include "Arduino.h"
#include "c_data.h"
#include "data.hpp"
#include "display.hpp"

display *lcd;

extern "C" void display_log_line(char* string){
    if(lcd->get_state() == CONSOLE)
        lcd->draw_console_line(string);
}

extern "C" bool reset_cmd(const char* argv, c_data* out){
    ESP.restart();
    return true;
}