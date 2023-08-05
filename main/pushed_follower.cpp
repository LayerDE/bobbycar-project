#include "defines.h"
#include "pushed_follower.hpp"
#include <cmath>
#include "math_functions.h"


#include <stdio.h>


typedef struct {float beta_start; float beta_end; float steering_angle;} lookup_steering;

static inline float pow2(float x){
  return x*x;
}

static void trail_point_out(void* context, point x0, double direction){
    printf("trailer: %f,%f <-> %f\n", x0.x,x0.y,direction);
}

static void car_point_out(void* context, point x0, double direction){
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

static int get_lookup(float in, float max, unsigned int size, bool full_range) {
    return CLAMP(round((full_range ? in : fabs(in)) / (max / (float)(size / (1 + full_range)))) + size / 2 * full_range, 0, size - 1);
}

static float get_lookup_reverse(unsigned int in, float max, unsigned int size, bool full_range) {
    return CLAMP((float)((int)in - (int)size / 2 * full_range ) * (max / (float)(size / (1 + full_range))), full_range ? -max : 0, max);
}


pushed_follower::pushed_follower(int c_wheelbase, int rc_axle2hitch, int hitch2trail_axle,float alpha_max, unsigned int lookup_alpha_size, float sim_distance,
            get_float steering_ptr, get_float hitch_angle_ptr, get_int speed_ptr, double ki, double kp, double kd)
            : simulation(this, 0, 0, (float)c_wheelbase/100.0,(float)rc_axle2hitch/100.0,0,steering_ptr(),(float)hitch2trail_axle/100.0, hitch_angle_ptr(),0.001){
    hitch2axle = hitch2trail_axle;
    car2hitch = rc_axle2hitch;
    printf("lookup: %f full: %i %f\n",rad2deg(get_lookup_reverse(get_lookup(deg2rad(20.0), alpha_max, 60, false), alpha_max, 60, false)), get_lookup(deg2rad(-1), alpha_max, 60, true), rad2deg(get_lookup_reverse(get_lookup(deg2rad(-1.0), alpha_max, 60, true), alpha_max, 60, true)));
    
    car_wheelbase = c_wheelbase;
    alpha_lookup_size = lookup_alpha_size;
    alpha_max_steer = alpha_max;
    simulator_distance = sim_distance;
    data_table.beta_max = deg2rad(20);
    allocate_lookup_table(lookup_alpha_size / 2, lookup_alpha_size);
    create_alpha_lookup();
    create_alpha_beta_sim_lookup(simulator_distance);
    export_lookuptalbe_c();
    simulation.set_output(car_point_out,trail_point_out, false);
    alpha_calc = new PID(&isPoint, &output, &setPoint, ki, kp, kd, 0);
}

pushed_follower::~pushed_follower(){
    deallocate_lookup_table();
}

void pushed_follower::allocate_lookup_table(int index0, int index1){
    data_table.constant_table = false;
    data_table.lookup_index0_max = index0;
    data_table.lookup_index1_max = index1;
    //alpha by beta
    data_table.lookup_alpha_by_beta = new float* [data_table.lookup_index0_max];
    for (int i = 0; i < data_table.lookup_index0_max; i++)
        data_table.lookup_alpha_by_beta[i] = new float[data_table.lookup_index1_max];
    //beta by alpha
    data_table.lookup_beta_by_alpha = new float* [data_table.lookup_index0_max];
    for (int i = 0; i < data_table.lookup_index0_max; i++)
        data_table.lookup_beta_by_alpha[i] = new float[data_table.lookup_index1_max];
}

void pushed_follower::deallocate_lookup_table() {
    if (!data_table.constant_table) {
        delete alpha_calc;
        for (int i = 0; i < data_table.lookup_index0_max; i++)
            delete[] data_table.lookup_alpha_by_beta[i];
        delete[] data_table.lookup_alpha_by_beta;
    }
}

double pushed_follower::calculate(int des_speed, float des_steering){ // simple pid
    isPoint = get_hitch_angle();
    setPoint = des_steering;
    output = get_steering();
    alpha_calc->Compute();
    return output;
}

float pushed_follower::calc_alpha_const(float beta){ // todo
    return -beta/data_table.linear_alpha_beta_faktor;
}

float pushed_follower::calc_beta_const(float alpha_steer){ // todo
    return -alpha_steer * data_table.linear_alpha_beta_faktor;
}

float pushed_follower::calc_alpha(float beta_old, float beta_new){
    unsigned int indexA = get_lookup(beta_old, data_table.beta_max, data_table.lookup_index0_max, false);
    unsigned int indexB = get_lookup(beta_new, data_table.beta_max, data_table.lookup_index1_max, true);
    return data_table.lookup_alpha_by_beta[indexA][indexB];
}

float pushed_follower::calc_beta(float alpha, float beta_old) {
    unsigned int indexA = get_lookup(alpha, data_table.alpha_max, data_table.lookup_index0_max, false);
    unsigned int indexB = get_lookup(beta_old, data_table.beta_max, data_table.lookup_index1_max, true);
    return data_table.lookup_beta_by_alpha[indexA][indexB];
}

float pushed_follower::create_beta_sim(float alpha, float beta_old, float distance){
    // so simple but so complex
    simulation.set_values(0,0,0,alpha,beta_old);
    simulation.simulate(distance);
    return simulation.output();
}

void pushed_follower::load_lookup(lookup_loader function) {
    function(&data_table);
}

float pushed_follower::create_alpha_sim(float beta_old, float beta_new, float precicion, float distance){
    //check possibility
    if(fabs(beta_old)>data_table.beta_max || fabs(beta_new)>data_table.beta_max)
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
        if(create_beta_sim(-alpha_max_steer, beta_old, distance) < beta_new)
            return -CPP_M_PI;
    }
    else /* beta_new < beta_old*/{
        printf("c2\n");
        low = calc_alpha_const(beta_old);
        high = alpha_max_steer;
        if(create_beta_sim(alpha_max_steer, beta_old, distance) > beta_new)
            return CPP_M_PI;
    }
    test_val = (high-low) / 2;
    step = test_val / 2;
    float result;
    do{
        printf("c: %f\n",rad2deg(test_val));
        result = create_beta_sim(test_val + low, beta_old, distance);
        if(result > beta_new)
            test_val += step;
        else
            test_val -= step;
        step /= 2;
    }while(!isNear(result, beta_new, precicion)
        &&  step > (precicion/data_table.linear_alpha_beta_faktor));
    return result;
}

void pushed_follower::create_alpha_beta_sim_lookup(float distance){
    for(int x = 0; x < data_table.lookup_index0_max;x++)
        for(int y = 0; y < data_table.lookup_index1_max; y++){
            data_table.lookup_alpha_by_beta[x][y] = create_alpha_sim(
                get_lookup_reverse(x,data_table.beta_max, data_table.lookup_index0_max, false),
                    get_lookup_reverse(y, data_table.beta_max, data_table.lookup_index1_max, true),
                        get_lookup_reverse(1,data_table.beta_max,data_table.lookup_index1_max,false)/2.0f, // precision
                        distance);
        }
    // todo
    for (int x = 0; x < data_table.lookup_index0_max; x++)
        for (int y = 0; y < data_table.lookup_index1_max; y++) {
            data_table.lookup_beta_by_alpha[x][y] = create_beta_sim(
                get_lookup_reverse(x, data_table.alpha_max, data_table.lookup_index0_max, false),
                get_lookup_reverse(y, data_table.beta_max, data_table.lookup_index1_max, true),
                distance);
        }
}

void pushed_follower::export_lookuptalbe_c(){
    printf("lookup-tables.c\n\n\
        #include \"lookup-tables.h\"\n\n\
        const unsigned int UNREACHABLE = 0x%X;\n\n\
        const float* unreachable = (float*)&UNREACHABLE;\n\n", 0x7FBFFFFF);
    printf("const float lookup_alpha_max = %f;\n", data_table.alpha_max);
    printf("const int lookup_index0_max = %i;\n", data_table.lookup_index0_max);
    printf("const int lookup_index1_max = %i;\n", data_table.lookup_index1_max);
    printf("const float beta_max = %f;\n", data_table.beta_max);
    printf("const float alpha_max = %f;\n", alpha_max_steer);
    printf("const float linear_alpha_beta_faktor = %f;\n\n", data_table.linear_alpha_beta_faktor);
    printf("// Lookup Alpha by Beta\n");
    for (int x = 0; x < data_table.lookup_index0_max; x++) {
        printf("static const float lookup_ab_%i[] = {", x);
        for (int y = 0; y < data_table.lookup_index1_max; y++) {
            float alpha = data_table.lookup_alpha_by_beta[x][y];
            printf("%f,", alpha);
        }
        printf("};\n");
    }
    printf("const float* lookup_alpha_by_beta[] = {");
    for (int x = 0; x < data_table.lookup_index0_max; x++) {
        printf("(const float*)&lookup_ab_%i, ", x);
    }
    printf("};\n");
    printf("// Lookup Beta by Alpha\n");
    for (int x = 0; x < data_table.lookup_index0_max; x++) {
        printf("static const float lookup_ba_%i[] = {", x);
        for (int y = 0; y < data_table.lookup_index1_max; y++) {
            float beta = data_table.lookup_beta_by_alpha[x][y];
            printf("%f,", beta);
        }
        printf("};\n");
    }
    printf("const float* lookup_beta_by_alpha[] = {");
    for (int x = 0; x < data_table.lookup_index0_max; x++) {
        printf("(const float*)&lookup_ba_%i, ", x);
    }
    printf("};\n");
    printf("void export_lookup(lookup_table* inval) {\n\
        inval->constant_table = true;\n\
        inval->alpha_max = alpha_max;\n\
        inval->beta_max = beta_max;\n\
        inval->lookup_alpha_by_beta = lookup_alpha_by_beta;\n\
        inval->lookup_beta_by_alpha = lookup_beta_by_alpha;\n\
        inval->linear_alpha_beta_faktor = linear_alpha_beta_faktor;\n\
        inval->lookup_index0_max = lookup_index0_max;\n\
        inval->lookup_index1_max = lookup_index1_max;\n}\n");
}

//linear
float pushed_follower::create_beta_const(float alpha){
    float V_bw = car_wheelbase / tan(alpha);
    float V_fbw = sqrt(pow2(V_bw) + pow2(car2hitch));
    float delta_2 = asin(car2hitch / V_fbw);
    float delta_1 = asin(hitch2axle / V_fbw);
    return delta_1 + delta_2;
}


void pushed_follower::create_alpha_lookup(){
    float alpha_steer = deg2rad(10);
    data_table.linear_alpha_beta_faktor = create_beta_const(alpha_steer)/alpha_steer;
    alpha_max = calc_alpha_const(data_table.beta_max);
}

bool pushed_follower::protection(){
    float alpha = get_steering();
    float beta = get_hitch_angle();
    int speed = get_speed();
    float stable_beta = calc_beta_const(alpha);
    if(beta< data_table.beta_max)
        return true;
    switch (sign(speed))
    {
    case 1: // forward
        if(fabs(stable_beta)<data_table.beta_max)
            return true;
        else if(sign(beta) != sign(stable_beta))
            return true;
        else
            return false;
    case -1: // reverse
        if(sign(stable_beta) == 0)
            return false;
        else if(sign(beta) != sign(stable_beta))
            return false;
        else if(fabs(stable_beta)>fabs(beta))
            return true;
        else
            return false;
    default:
        return true; // not relevant because if speed is 0 the car doesn't move and the output will do nothing
    }
}