#include <stdint.h>
#include <stdio.h>

#include "config.h"
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

void display::draw_screen(int throttle, float steering, float steering_desired, bool trailer_en, float trailer_angle, int *torgue, int torgue_regulated,bool front, bool rear, int speed, int voltage, int input_src){
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

int display_address_check(uint8_t address){
    return DSP_NONE_INDEX; // remove after display fix
    switch(address){
        case 0x3D:
        case 0x3C:
            return DSP_OLED_INDEX;
        case 0x27:
        case 0x3F:
            return DSP_2004_INDEX;
        default:
            return DSP_NONE_INDEX;
    }
}

int scan_i2c(c_data *valid, c_data *invalid, TwoWire *i2c_bus) {
    uint8_t error, address;
    c_data_set_size(valid, 0);
    if(invalid != NULL)
        c_data_set_size(invalid, 0);
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            c_data_extend_raw(valid,&address,sizeof(address));
        } else if (error == 4) {
            if(invalid != NULL)
                c_data_extend_raw(invalid,&address,sizeof(address));
            //printf("Unknow error at address 0x%02hhX\n", address);
        }
    }
    return valid->size/sizeof(address);
}