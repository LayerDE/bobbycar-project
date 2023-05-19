#pragma once
#include "position.hpp"

class car : public position{
    private:
        float wheelbase;
        float rear2hitch;
    public:
        float get_r2h();
        car(float wb, float rh, float hx, float hy, float hdirection, float halpha);
        float alpha;
        void calc_curve(float lenght, float alpha_steer, float &x, float &y, float &angle);
        void move(float move_lenght);
};