#pragma once

#include "display.hpp"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <inttypes.h>

class display_2004 : display{
    private:
        LiquidCrystal_I2C *lcd;
        char** buffer;
        void print_init();
        void _draw_line(const char* in, int y);
        void _draw_buffer();
        bool set_state(STATES_OF_DISPLAY hstate);
    public:
        void draw_console_line(char* line);
        display_2004(TwoWire *bus, char adr);
        ~display_2004();
        void clear();
        void draw_screen(int throttle, float steering, float steering_desired, bool trailer_en, float trailer_angle, int *torgue, int torgue_regulated,bool front, bool rear, int speed, int voltage, int input_src);
        void draw_menu(int options, char* option_name[], int highlight);
        void draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection);
        void draw_confirmation(char* text, int options, char* option_name[], int highlight);
};