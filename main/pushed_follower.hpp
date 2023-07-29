#pragma once
#include "PID_v1.h"
#include "simulator.hpp"


typedef float (*get_float)();
typedef int (*get_int)();

class pushed_follower{
    public:
        pushed_follower(int c_wheelbase, int rc_axle2hitch, int hitch2car_axle, float alpha_max, float beta_protect, unsigned int lookup_alpha_size, float sim_distance,
            get_float steering_ptr, get_float hitch_angle_ptr, get_int speed_ptr,
            double ki, double kp, double kd);
        ~pushed_follower();
        double calculate(int des_speed, float des_steering);
        float get_des_steering(float real_beta, float des_beta);
        float get_stable_steering(float des_beta);
        float calc_alpha_const(float beta);
        float calc_beta_const(float alpha_steer);
        float calc_alpha(float beta_old, float beta_new);
        float calc_beta(float alpha, float beta_old, float distance);
        float create_alpha_sim(float beta_old, float beta_new, float precicion, float distance);
        float create_beta_const(float alpha);
        bool protection();
    private:
        float c_alpha_beta_factor;
        float simulator_distance;
        float beta_max;
        unsigned int alpha_lookup_size;
        float alpha_max;
        float alpha_max_steer;
        float** alpha_sim_lookup;
        PID *alpha_calc;
        simulator simulation;
        float lenght;
        double setPoint;
        double isPoint;
        double output;
        get_float get_steering;
        get_float get_hitch_angle;
        get_int get_speed;
        int hitch2axle;
        int car2hitch;
        int car_wheelbase;
        void export_lookuptalbe();
        void create_alpha_lookup();
        void create_alpha_sim_lookup(float distance);
};