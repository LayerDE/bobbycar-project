#include <Wire.h>
#include <stdlib.h>
#include <math.h>
#include "defines.h"
#include "math_functions.h"

#include "display_oled.hpp"

display_oled::display_oled(TwoWire *bus, char adr, uint8_t width, uint8_t height) : display(bus,adr){
    oled = new Adafruit_SSD1306(width, height, bus);
    if(!oled->begin(SSD1306_SWITCHCAPVCC, adr))
        printf("SSD1306 allocation failed");
    else
    {
        oled->display();
    }
}

void display_oled::draw_console_line(char* line){

}

void display_oled::draw_line(const char* in, int y) {
    oled->setCursor(0, y);     // Start at top-left corner

    // Not all the characters will fit on the display. This is normal.
    // Library will draw what it can and the rest will be clipped.
    for(int16_t i=0; in[i]; i++)
        oled->write(in[i]);
    oled->display();
}

display_oled :: ~display_oled(){
    delete oled;
}
void display_oled::clear(){
    oled->clearDisplay();
    oled->display();
}
void display_oled::draw_screen(int throttle, float steering, float steering_desired, bool trailer_en, float trailer_angle, int *torgue, int torgue_regulated,bool front, bool rear, int speed, int voltage, int input_src){
    display::draw_screen(throttle, steering, steering_desired, trailer_en, trailer_angle, torgue, torgue_regulated,front, rear, speed, voltage, input_src);
    char sprint_buffer[256];
    char steer_buf[32];
    char torgue_buf0[54];
    char torgue_buf1[32];
    if(trailer_en)
        sprintf(steer_buf, "S: %.2f->%.2f T: %.2f",rad2deg(steering), rad2deg(steering_desired), rad2deg(trailer_angle));
    else
        sprintf(steer_buf, "Steering: %.2f->%.2f", rad2deg(steering), rad2deg(steering_desired));
    if(front)
        sprintf(torgue_buf0, "%i %c %i  \t  %i %c %i", torgue[0], torgue_regulated<0 ? '+' : '-' , ABS(torgue_regulated), torgue[1], torgue_regulated>0 ? '+' : '-' , ABS(torgue_regulated));
    else
        sprintf(torgue_buf0, "front not connected");
    if(rear)
        sprintf(torgue_buf1,"%i      \t      %i",torgue[2], torgue[3]);
    else
        sprintf(torgue_buf1, "rear not connected");
    sprintf(sprint_buffer, "Throttle: %i\n%s\n%s\n%s\n%i: S%i B%i T%i\nV:%i    SPEED: %i", throttle,
        steer_buf, torgue_buf0, torgue_buf1, input_src , 0, 0,
        throttle, voltage, speed);
    oled->clearDisplay();
    draw_line(sprint_buffer, 0);
}

bool display_oled::set_state(STATES_OF_DISPLAY hstate){
    if(display::set_state(hstate)){
        switch (hstate)
        {
        case OFF:
            //clear();
            break;

        case IDLE:
            {
                oled->setTextSize(1);      // Normal 1:1 pixel scale
                oled->setTextColor(SSD1306_WHITE); // Draw white text
                oled->cp437(true);         // Use full 256 char 'Code Page 437' font
                char sprint_buffer[256];
                sprintf(sprint_buffer, "OLED Address: 0x%02hhX\n", address);
                draw_line(sprint_buffer, 0);
            }
            break;
        
        case CONSOLE:
            //clear();
            break;
        case USERINTERFACE:
            oled->setTextSize(1);      // Normal 1:1 pixel scale
            oled->setTextColor(SSD1306_WHITE); // Draw white text
            oled->cp437(true);         // Use full 256 char 'Code Page 437' font
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

void display_oled::draw_menu(int options, char* option_name[], int highlight){

}

void display_oled::draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection){

}

void display_oled::draw_confirmation(char* text, int options, char* option_name[], int highlight){

}