#pragma once

#include <Wire.h>
#include <stdbool.h>
#include <c_data.h>
typedef enum{IDLE, OFF, USERINTERFACE, CONSOLE} STATES_OF_DISPLAY;

int scan_i2c(c_data *valid, c_data *invalid, TwoWire *i2c_bus);

class display{
    private:
        STATES_OF_DISPLAY state;
        char* buffer;
    protected:
        virtual bool set_state(STATES_OF_DISPLAY hstate);
        TwoWire *i2c_bus;
        char address;
    public:
        virtual void set_power(bool on);
        virtual STATES_OF_DISPLAY get_state();
        virtual void draw_console_line(char* line) = 0;
        display(TwoWire *bus, char adr);
        virtual ~display();
        virtual void clear() = 0;
        virtual void draw_screen(int throttle, float steering, float steering_desired, bool trailer_en, float trailer_angle, int *torgue, int torgue_regulated,bool front, bool rear, int speed, int voltage, int input_src) = 0;
        virtual void draw_menu(int options, char* option_name[], int highlight) = 0;
        virtual void draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection) = 0;
        virtual void draw_confirmation(char* text, int options, char* option_name[], int highlight) = 0;
};
