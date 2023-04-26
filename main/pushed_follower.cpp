#include <math.h>
#include <stdint.h>
#include "defines.h"
#include "config.h"
#include "math_functions.h"
#include "pushed_follower.hpp"

follower::follower(int to_car_axle, int to_follower_axle){
    alpha_calc = PID(&isPoint, &output, &setPoint,0.0,0.0,0.0,0);
}

double follower::calculate(int des_speed, float des_steering){
    isPoint = get_steering();
    setPoint = des_steering;
    alpha_calc.Compute();
    return output;
}

float calc_beta_const(float alpha_steer){
    if(alpha_steer == 0.0)
        return 0.0;
    float V_bw = L_WHEELBASE/tan(fabs(alpha_steer));
    return tan(L_REAR_TO_HITCH/V_bw);
    float delta_2 = sin(L_HITCH_TO_FOLLOWER_AXLE/sqrt(pow2(V_bw)+pow2(L_REAR_TO_HITCH)));
    //return delta_1;
}

float calc_alpha_const(float beta){
    float V_bw = L_HITCH_TO_FOLLOWER_AXLE/tan(beta);
    return atan(L_WHEELBASE/V_bw);
}

float calc_distance(float alpha, float beta_old, float beta_new){
    if(beta_old == beta_new)
    {
        return 0.0;
    }
    else if(calc_beta_const(alpha)==beta_new)
    {
        return FLOAT_MAX;
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