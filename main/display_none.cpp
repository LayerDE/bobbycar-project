#include <Wire.h>

#include "display_none.hpp"

display_none::display_none(TwoWire *bus) : display(bus,0){
}

void display_none::draw_console_line(char* line){

}

void display_none::draw_line(const char* in, int y) {
}

display_none :: ~display_none(){

}
void display_none::clear(){}
void display_none::draw_screen(int throttle, float steering, float steering_desired, bool trailer_en, float trailer_angle, int *torgue, int torgue_regulated,bool front, bool rear, int speed, int voltage, int input_src){
}

bool display_none::set_state(STATES_OF_DISPLAY hstate){
    return display::set_state(hstate);
}

void display_none::draw_menu(int options, char* option_name[], int highlight){

}

void display_none::draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection){

}

void display_none::draw_confirmation(char* text, int options, char* option_name[], int highlight){

}