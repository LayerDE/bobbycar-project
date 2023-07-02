#include "defines.h"
#include "pushed_follower.hpp"
#include <cmath>


typedef struct {float beta_start; float beta_end; float steering_angle; float distance;} lookup_steering;
typedef struct {float beta_start; float beta_end; lookup_steering* possibilies; int possibilies_len;} looup_possible;

static inline float pow2(float x){
  return x*x;
}

static void trail_point_out(void* context, point x0, float direction){

}

static void car_point_out(void* context, point x0, float direction){

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
            double ki, double kp, double kd) : simulation(this, 0, 0, (float)c_wheelbase/100.0,(float)hitch2car_axle/100.0,0,steering_ptr(),(float)rc_axle2hitch/100.0, hitch_angle_ptr(),0.01){
    simulation.set_output(car_point_out,trail_point_out, false);
    alpha_calc = new PID(&isPoint, &output, &setPoint, ki, kp, kd, 0);
}

pushed_follower::~pushed_follower(){
    delete alpha_calc;
}

        float get_des_steering(float real_beta, float des_beta);
        float get_stable_steering(float des_beta);

double pushed_follower::calculate(int des_speed, float des_steering){ // simple pid
    isPoint = get_hitch_angle();
    setPoint = des_steering;
    output = get_steering();
    alpha_calc->Compute();
    return output;
}

float pushed_follower::calc_alpha_const(float beta){ // todo
    if(beta == 0.0)
        return 0.0;
    float V_fbw = rc_axle2hitch/
    float V_bw = hitch2car_axle/tan(beta);
    return atan(c_wheelbase/V_bw);
}

float pushed_follower::calc_beta_const(float alpha_steer){ // todo
    if(alpha_steer == 0.0)
        return 0.0;
    float V_bw = c_wheelbase/tan(alpha_steer);
    float V_fbw = sqrt(pow2(V_bw)+pow2(car2hitch));
    
    return atan(hitch2car_axle/V_bw);
    //float delta_2 = sin(hitch2axle/sqrt(pow2(V_bw)+pow2(car2hitch)));
    //return delta_1;
}

