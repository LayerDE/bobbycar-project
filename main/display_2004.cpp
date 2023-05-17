#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <stdlib.h>
#include <math.h>
#include "defines.h"
#include "math_functions.h"
#include "display_2004.hpp"

display_2004::display_2004(TwoWire *bus, char adr) : display(bus,adr){
    lcd = new LiquidCrystal_I2C(adr,20,4);
    buffer = (char**)malloc(sizeof(char*)*4);
    for(int x = 0; x < 4; x++)
        buffer[x] = (char*)malloc(sizeof(char)*20);
    lcd->begin(20,4);
    lcd->noBlink();
    lcd->noCursor();
    lcd->backlight();
    clear();
}

void display_2004::_draw_line(const char* in, int y) {

    lcd->setCursor(0, y);              // Start at top-left corner
    // Not all the characters will fit on the display. This is normal.
    // Library will draw what it can and the rest will be clipped.
    for (int16_t i = 0; in[i] && i < 20; i++)
        lcd->write(in[i]);
    lcd->display();
}

void display_2004::print_init(){

}
void display_2004::_draw_buffer(){
    clear();
    for(int x = 0; x < 4; x++)
        _draw_line(buffer[x],x);
}

static inline void swp_char_ptr(char **x, char **y)
{
  char *tmp = *x;
  *x = *y;
  *y = tmp;
}

void display_2004::draw_console_line(char* line){
    for(int x=0; x < 3; x++)
        swp_char_ptr(&buffer[x+0],&buffer[x+1]);
    for(int x = 0, y = 1; x < 20; x++)
        if(y){
            if(line[x]){
                buffer[3][x] = line[x];
            }
            else{
                buffer[3][x] = ' ';
                y = 0;
            }
        }
        else{
            buffer[3][x] = ' ';
        }
    _draw_buffer();
}

display_2004::~display_2004(){
    for(int x = 0; x < 4; x++)
        free(buffer[x]);
    free(buffer);
    delete lcd;
}

void display_2004::clear(){
    for(int x = 0; x < 4; x++)
        for(int y = 0; y < 20; y++)
            buffer[x][y] = ' ';
    lcd->clear();
}

bool display_2004::set_state(STATES_OF_DISPLAY hstate){
    if(display::set_state(hstate)){
        switch (hstate)
        {
        case OFF:
            //clear();
            lcd->noBacklight();
            break;

        case IDLE:
            {
                lcd->backlight();
                 char sprint_buffer[256];
                sprintf(sprint_buffer, "LCD Address: 0x%02hhX\n", address);
                _draw_line(sprint_buffer, 0);
            }
            break;
        
        case CONSOLE:
            lcd->backlight();
            //clear();
            break;
        case USERINTERFACE:
            lcd->backlight();
            //clear();
            break;
        default:
            break;
        }
    return true;
  }
  else{
    return false;
  }
}

void display_2004::draw_screen(int throttle, float steering, float steering_desired, bool trailer_en, float trailer_angle, int *torgue, int torgue_regulated, int speed, int voltage, int input_src){
    display::draw_screen(throttle, steering, steering_desired, ,trailer_en. trailer_angle, torgue, torgue_regulated, speed, voltage, input_src);
    char line_buffer[512];
    snprintf(line_buffer, 512, "T%i S%.1f SD%.1f", throttle,rad2deg(steering),rad2deg(steering_desired));
    _draw_line(line_buffer,0);

    snprintf(line_buffer, 512, "%i%c%i %i%c%i", torgue[0], torgue_regulated<0 ? '+' : '-' , ABS(torgue_regulated), torgue[1], torgue_regulated>0 ? '+' : '-' , ABS(torgue_regulated));
    _draw_line(line_buffer,1);

    snprintf(line_buffer, 512, "%i %i", torgue[2], torgue[3]);
    _draw_line(line_buffer,2);

    snprintf(line_buffer, 512, "Input:%i  S:%i", input_src, speed);
    _draw_line(line_buffer,3);
}

void display_2004::draw_menu(int options, char* option_name[], int highlight){

}

void display_2004::draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection){

}

void display_2004::draw_confirmation(char* text, int options, char* option_name[], int highlight){

}
