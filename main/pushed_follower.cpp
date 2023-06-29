#include "defines.h"
#include "pushed_follower.hpp"

pushed_follower::pushed_follower(int c_wheelbase, int rc_axle2hitch, int hitch2car_axle,
            get_float steering_ptr, get_float hitch_angle_ptr, get_int speed_ptr,
            double ki, double kp, double kd) : simulation(0,0,(float)c_wheelbase/100.0,(float)hitch2car_axle/100.0,0,steering_ptr(),(float)rc_axle2hitch/100.0, hitch_angle_ptr(),0.01){
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
