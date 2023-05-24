#include "defines.h"
#include "pushed_follower.hpp"

pushed_follower::pushed_follower(int c_wheelbase, int rc_axle2hitch, int hitch2car_axle,
            get_float steering_ptr, get_float hitch_angle_ptr, get_int speed_ptr,
            double ki, double kp, double kd) : simulation = simulator(){
    alpha_calc = new PID(&isPoint, &output, &setPoint, ki, kp, kd, 0);
}

pushed_follower::~pushed_follower(){
    delete alpha_calc;
}

double pushed_follower::calculate(int des_speed, float des_steering){
    isPoint = get_hitch_angle();
    setPoint = des_steering;
    output = get_steering();
    alpha_calc->Compute();
    return output;
}
