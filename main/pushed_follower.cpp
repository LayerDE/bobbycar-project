#include "defines.h"
#include "pushed_follower.hpp"
#include <cmath>
#include "math_functions.h"


#include <stdio.h>


typedef struct {float beta_start; float beta_end; float steering_angle;} lookup_steering;

static inline float pow2(float x){
  return x*x;
}

static void trail_point_out(void* context, point x0, float direction){
    printf("trailer: %f,%f <-> %f\n", x0.x,x0.y,direction);
}

static void car_point_out(void* context, point x0, float direction){
    printf("car: %f,%f <-> %f\n", x0.x,x0.y,direction);
}

static bool isNear_points(point x0, point x1, float range) {
    point x2;
    x2.x = x0.x - x1.x;
    x2.y = x0.y - x1.y;
    float _straight = sqrt(pow2(x2.x) + pow2(x2.y));
    if(_straight<range)
        return true;
    else
        return false;
}

pushed_follower::pushed_follower(int c_wheelbase, int rc_axle2hitch, int hitch2trail_axle,float alpha_max, float beta_protect, unsigned int lookup_alpha_size, float sim_distance,
            get_float steering_ptr, get_float hitch_angle_ptr, get_int speed_ptr, double ki, double kp, double kd)
            : simulation(this, 0, 0, (float)c_wheelbase/100.0,(float)rc_axle2hitch/100.0,0,steering_ptr(),(float)hitch2trail_axle/100.0, hitch_angle_ptr(),0.00001){
    hitch2axle = hitch2trail_axle;
    car2hitch = rc_axle2hitch;
    car_wheelbase = c_wheelbase;
    alpha_lookup_size = lookup_alpha_size;
    alpha_max_steer = alpha_max;
    // allocating alpha sim lookup
    alpha_sim_lookup = new float*[alpha_lookup_size/2];
    for(int i = 0; i < alpha_lookup_size/2; i++)
        alpha_sim_lookup[i] = new float[alpha_lookup_size];

    beta_max = beta_protect;
    simulator_distance = sim_distance;
    create_alpha_lookup();
    create_alpha_sim_lookup(simulator_distance);
    export_lookuptalbe();
    simulation.set_output(car_point_out,trail_point_out, false);
    alpha_calc = new PID(&isPoint, &output, &setPoint, ki, kp, kd, 0);
}

pushed_follower::~pushed_follower(){
    delete alpha_calc;
    for(int i = 0; i < alpha_lookup_size/2; i++)
        delete [] alpha_sim_lookup[i];
    delete [] alpha_sim_lookup;
}

double pushed_follower::calculate(int des_speed, float des_steering){ // simple pid
    isPoint = get_hitch_angle();
    setPoint = des_steering;
    output = get_steering();
    alpha_calc->Compute();
    return output;
}

float pushed_follower::calc_alpha_const(float beta){ // todo
    return -beta/c_alpha_beta_factor;
}

float pushed_follower::calc_beta_const(float alpha_steer){ // todo
    return -alpha_steer * c_alpha_beta_factor;
}

static int get_lookup(float in, float max, unsigned int size, bool full_range){
    return CLAMP(round(full_range ? in :fabs(in) / (max/(float)(size/ (1+full_range)))) + size / 2 * full_range, 0, size-1);
}

static float get_lookup_reverse(unsigned int in, float max, unsigned int size, bool full_range){
    return CLAMP((float)((int)in - size / 2 * full_range) * (max/(float)(size/(1+full_range))),full_range ? -max : 0,max);
}


float pushed_follower::calc_alpha(float beta_old, float beta_new){
    unsigned int indexA = get_lookup(beta_old, beta_max, alpha_lookup_size/2, false);
    unsigned int indexB = get_lookup(beta_new, beta_max, alpha_lookup_size, true);
    return alpha_sim_lookup[indexA][indexB];
}

float pushed_follower::calc_beta(float alpha, float beta_old, float distance){
    // so simple but so complex
    simulation.set_values(0,0,0,alpha,beta_old);
    simulation.simulate(distance);
    return simulation.output();
}

float pushed_follower::create_alpha_sim(float beta_old, float beta_new, float precicion, float distance){
    //check possibility
    if(fabs(beta_old)>beta_max || fabs(beta_new)>beta_max)
        return 0;
    float test_val;
    float low;
    float high;
    float hl_delta;
    float step;
    if(beta_new == beta_old)
        return calc_alpha_const(beta_new);
    else if(beta_new > beta_old){
        printf("c1\n");
        low = -alpha_max_steer;
        high = calc_alpha_const(beta_old);
        if(calc_beta(-alpha_max_steer, beta_old, distance) < beta_new)
            return -CPP_M_PI;
    }
    else /* beta_new < beta_old*/{
        printf("c2\n");
        low = calc_alpha_const(beta_old);
        high = alpha_max_steer;
        if(calc_beta(alpha_max_steer, beta_old, distance) > beta_new)
            return CPP_M_PI;
    }
    test_val = (high-low) / 2;
    step = test_val / 2;
    float result;
    do{
        printf("c: %f\n",rad2deg(test_val));
        result = calc_beta(test_val + low, beta_old, distance);
        if(result > beta_new)
            test_val += step;
        else
            test_val -= step;
        step /= 2;
    }while(!isNear(result, beta_new, precicion) &&  step > (precicion/c_alpha_beta_factor));
    return result;
}

void pushed_follower::create_alpha_sim_lookup(float distance){
    for(int x = 0; x < alpha_lookup_size/2;x++)
        for(int y = 0; y < alpha_lookup_size; y++){
            alpha_sim_lookup[x][y] = create_alpha_sim(get_lookup_reverse(x,beta_max, alpha_lookup_size / 2, false),get_lookup_reverse(y, beta_max, alpha_lookup_size, true),deg2rad(0.25),0.4);
        }
    // todo
}

void pushed_follower::export_lookuptalbe(){
    printf("lookup-tables.c\n\nconst unsigned int UNREACHABLE = 0x%X;\n\nconst float* unreachable = (float*)&UNREACHABLE;\n\n", 0x7FBFFFFF);
    printf("const float lookup_alpha_max = %f;\n", alpha_max);
    printf("const int lookup_index0_max = %i;\n", alpha_lookup_size/2);
    printf("const int lookup_index1_max = %i;\n", alpha_lookup_size);
    printf("const float beta_max = %f;\n", beta_max);
    printf("const float alpha_max = %f;\n", alpha_max_steer);
    printf("const float linear_alpha_beta_faktor = %f;\n", c_alpha_beta_factor);
    for (int x = 0; x < alpha_lookup_size / 2; x++) {
        printf("static const float lookup_ab_%i[] = {",x);
        for (int y = 0; y < alpha_lookup_size; y++) {
            float alpha = alpha_sim_lookup[x][y];
            printf("%f,", alpha);
        }
        printf("};\n");
    }
    printf("const float* lookup_alpha_by_beta[] = {");
    for (int x = 0; x < alpha_lookup_size / 2; x++) {
        printf("(const float*)&lookup_ab_%i, ", x);
    }
    printf("};\n");
}

//linear
float pushed_follower::create_beta_const(float alpha){
    float V_bw = car_wheelbase / tan(alpha_steer);
    float V_fbw = sqrt(pow2(V_bw) + pow2(car2hitch));
    float delta_2 = asin(car2hitch / V_fbw);
    float delta_1 = asin(hitch2axle / V_fbw);
    return delta_1 + delta_2;
}
void pushed_follower::create_alpha_lookup(){
    float alpha_steer = deg2rad(10);
    c_alpha_beta_factor = create_beta_const(alpha_steer)/alpha_steer;
    alpha_max = calc_alpha_const(beta_max);
}