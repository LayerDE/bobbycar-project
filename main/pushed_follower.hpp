#pragma once
#include "PID_v1.h"


typedef float (*get_float)();
typedef int (*get_int)();


class follower{
    public:
        follower(int rc_axle2hitch, int hitch2car_axle);
        ~follower();
        double calculate(int des_speed, float des_steering);
    private:
        PID *alpha_calc;
        float calc_beta_const(float alpha_steer);
        float calc_alpha_const(float beta);
        double setPoint;
        double isPoint;
        double output;
        get_float get_steering;
        get_float get_hitch_angle;
        get_int get_speed;
        int hitch2axle;
        int car2hitch;
        int car_wheelbase;

};