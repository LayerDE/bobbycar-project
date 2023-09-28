#pragma once

#include <Wire.h>
#include "display.hpp"

class display_none : public display{
    private:
        bool set_state(STATES_OF_DISPLAY hstate);
        void draw_line(const char* in, int y);
    public:
        void draw_console_line(char* line);
        display_none(TwoWire *bus);
        ~display_none();
        void clear();
        void draw_screen(int throttle, float steering, float steering_desired, bool trailer_en, float trailer_angle, int *torgue, int torgue_regulated,bool front, bool rear, int speed, int voltage, int input_src);
        void draw_menu(int options, char* option_name[], int highlight);
        void draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection);
        void draw_confirmation(char* text, int options, char* option_name[], int highlight);
};
