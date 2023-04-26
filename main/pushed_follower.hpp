#pragma once
#include "PID_v1.h"

class follower{
    public:
        follower(int to_car_axle, int to_follower_axle);
        int get_speed();
        float get_steering();
        double calculate(int des_speed, float des_steering);
    private:
        PID alpha_calc;
        double setPoint;
        double isPoint;
        double output;
        void math_model();
        int lenght;
        int axle;

};