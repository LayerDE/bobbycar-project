#include <math.h>
#include <stdint.h>
#include "defines.h"
#include "config.h"

float calc_beta_const(float alpha_steer){
    if(alpha_steer == 0.0)
        return 0.0;
    float V_bw = L_WHEELBASE/tan(fabs(alpha_steer));
    float delta_1 = tan(L_REAR_TO_HITCH/V_bw);
    float delta_2 = sin(L_HITCH_TO_FOLLOWER_AXLE/sqrt(V_bw*V_bw+L_REAR_TO_HITCH*L_REAR_TO_HITCH));
    return delta_1;
}

float calc_alpha_const(float beta){
    float V_bw = L_HITCH_TO_FOLLOWER_AXLE/tan(beta);
    return atan(L_WHEELBASE/V_bw);
}

float calc_distance(float alpha, float beta_old, float beta_new){
    if(beta_old == beta_new)
        return 0.0;
    else
        return 0;
}
float calc_alpha(float distance, float beta_old, float beta_new){

}