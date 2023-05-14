#include <math.h>
#include <stdint.h>
#include "defines.h"
#include "math_functions.h"
#include "pushed_follower.hpp"

follower::follower(int c_wheelbase, int rc_axle2hitch, int hitch2car_axle,
            get_float steering_ptr, get_float hitch_angle_ptr, get_int speed_ptr,
            double ki, double kp, double kd){
    alpha_calc = new PID(&isPoint, &output, &setPoint, ki, kp, kd, 0);
}

follower::~follower(){
    delete alpha_calc;
}

double follower::calculate(int des_speed, float des_steering){
    isPoint = get_hitch_angle();
    setPoint = des_steering;
    output = get_steering();
    alpha_calc->Compute();
    return output;
}

float follower::calc_beta_const(float alpha_steer){
    if(alpha_steer == 0.0)
        return 0.0;
    float V_bw = car_wheelbase/tan(fabs(alpha_steer));
    return tan(car2hitch/V_bw);
    //float delta_2 = sin(hitch2axle/sqrt(pow2(V_bw)+pow2(car2hitch)));
    //return delta_1;
}

float follower::calc_alpha_const(float beta){
    float V_bw = hitch2axle/tan(beta);
    return atan(car_wheelbase/V_bw);
}

const float bb_sample_steps
const float f_sample_steps

float calc_distance(float alpha, float beta_old, float beta_new){
    if(beta_old == beta_new)
    {
        return 0.0;
    }
    else if(calc_beta_const(alpha)==beta_new)
    {
        return INFINITY;
    }
    else{
        float distance = 0;
        float test_beta;
        for(int x = 0;x < 1; x++){

        }
    }
        
}
float calc_alpha(float distance, float beta_old, float beta_new){
    if(beta_old == beta_new)
    {
        return calc_alpha_const(beta_new);
    }
    else if(beta_old>)
    {

    }

}