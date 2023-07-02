#include "defines.h"
#include "pushed_follower.hpp"
#include <cmath>

static inline float pow2(float x){
  return x*x;
}

static void trail_point_out(point x0, float direction){
    //_point_out("Trailer", x0.x, x0.y, direction);
}

static void car_point_out(point x0, float direction){
    //_point_out("Car", x0.x, x0.y, direction);
}

static bool isNear_points(point x0, point x1, float range){
    point x2 = {.x = x0.x -x1.x, .y = x0.y - x1.y};
    float _straight = sqrt(pow2(x2.x) + pow2(x2.y));
    if(_straight<range)
        return true;
    else
        return false;
}

pushed_follower::pushed_follower(int c_wheelbase, int rc_axle2hitch, int hitch2car_axle,
            get_float steering_ptr, get_float hitch_angle_ptr, get_int speed_ptr,
            double ki, double kp, double kd) : simulation(0,0,(float)c_wheelbase/100.0,(float)hitch2car_axle/100.0,0,steering_ptr(),(float)rc_axle2hitch/100.0, hitch_angle_ptr(),0.01){
    simulation.set_output(car_point_out,trail_point_out, false);
    alpha_calc = new PID(&isPoint, &output, &setPoint, ki, kp, kd, 0);
}

pushed_follower::~pushed_follower(){
    delete alpha_calc;
}

double pushed_follower::calculate(int des_speed, float des_steering){ // simple pid
    isPoint = get_hitch_angle();
    setPoint = des_steering;
    output = get_steering();
    alpha_calc->Compute();
    return output;
}
