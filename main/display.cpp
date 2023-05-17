#include <stdint.h>
#include <stdio.h>

#include "display.hpp"

display::display(TwoWire *bus, char adr){
    i2c_bus = bus;
    address = adr;
}

display::~display(){
    
}

void display::set_power(bool on){
    if(on){
        if(get_state() == OFF)
            set_state(IDLE);
    }
    else{
        if(get_state() != OFF)
            set_state(OFF);
    }
}

void display::draw_screen(int throttle, float steering, float steering_desired, bool trailer_en, float trailer_angle, int *torgue, int torgue_regulated, int speed, int voltage, int input_src){
    set_state(USERINTERFACE);
}

STATES_OF_DISPLAY display::get_state(){
    return state;
}

bool display::set_state(STATES_OF_DISPLAY hstate){
    if(state != hstate){
        clear();
        state = hstate;
        return true;
    }
    else{
        return false;
    }
}

int scan_i2c(c_data *valid, TwoWire *i2c_bus) {
    uint8_t error, address;
    int nDevices;
    c_data_set_size(valid, 0);
    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            c_data_extend_raw(valid,&address,sizeof(address));
            nDevices++;
        } else if (error == 4) {
            printf("Unknow error at address 0x%02hhX\n", address);
        }
    }
    return nDevices;
}