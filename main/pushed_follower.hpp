#pragma once
#include "PID_v1.h"
#include "simulator.hpp"
#include "lookup-tables.h"


typedef float (*get_float)();
typedef int (*get_int)();

class pushed_follower{
    public:
        pushed_follower(int c_wheelbase, int rc_axle2hitch, int hitch2car_axle, float alpha_max, unsigned int lookup_alpha_size, int sim_distance);
        ~pushed_follower();
        float calc_alpha_const(float beta);
        float calc_beta_const(float alpha_steer);
        float calc_alpha(float beta_old, float beta_new);
        float calc_beta(float alpha, float beta_old);
        float calc_alpha_linear(float beta_old, float beta_new);
        float create_alpha_sim(float beta_old, float beta_new, float precicion, float distance);
        float create_beta_sim(float alpha, float beta_old, float distance);
        float create_beta_const(float alpha);
        bool protection(float alpha, float beta, int speed);
        void load_lookup(lookup_loader function);
    private:
        lookup_table data_table;
        float simulator_distance;
        unsigned int alpha_lookup_size;
        float alpha_max;
        float alpha_max_steer;
        simulator simulation;
        float lenght;
        int hitch2axle;
        int car2hitch;
        int car_wheelbase;
        void export_lookuptalbe_c();
        void create_alpha_lookup();
        void create_alpha_beta_sim_lookup(float distance);

        void allocate_lookup_table(int index0, int index1);
        void deallocate_lookup_table();
};