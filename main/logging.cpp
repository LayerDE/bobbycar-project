#include <stdio.h>
#include <stdbool.h>
#include "timecritical_function.h"

typedef struct {
    unsigned long time;
    double pout_value;
    float steer_real;
    float steer_des;
    float trailer;
} pid_log __attribute__((packed));


pid_log pid_logging[1024];
static int pid_logging_ptr;
static bool log_running;
static bool dumping = false;
extern "C" void set_log_active(bool in){
    if(in == log_running)
        return;
    if(in){
        pid_logging_ptr = 0;
        log_running = true;
    }
    else{
        log_running = false;
    }
}

extern "C" int dump_log(){
    bool tmp = log_running;
    int ret = pid_logging_ptr;
    int count = 0;
    log_running = false;
    dumping = true;
    printf("\n-:\n");
    count += printf("time,steering_pid_out,real_steering,target_steering,trailer_value\n");
        for(int i = 0, temp; i < ret;i++){
            count += temp = printf("%lu,%f,%f,%f,%f",pid_logging[i].time,pid_logging[i].pout_value,pid_logging[i].steer_real,pid_logging[i].steer_des, pid_logging[i].trailer);
            count += printf(":%i\n",temp);
        }
    printf(":-\n\n");
    printf("%i characters printed\n", count);
    dumping = false;
    pid_logging_ptr = 0;
    log_running = true;
    return ret;
}

extern "C" void add_log(double po, unsigned long time, float steer_real, float desired_steering, float trailer){
    if(!log_running)
        return;
    if(pid_logging_ptr >= 1024){
        log_running = false;
    }
    if(pid_logging_ptr == 0)
        pid_logging[pid_logging_ptr++] = pid_log {.time = time, .pout_value = po, .steer_real = steer_real, .steer_des = desired_steering, .trailer = trailer};
    else if(pid_logging[pid_logging_ptr-1].pout_value != po ||
            pid_logging[pid_logging_ptr-1].steer_des != desired_steering ||
            pid_logging[pid_logging_ptr-1].steer_real != steer_real)
        pid_logging[pid_logging_ptr++] = pid_log {.time = time, .pout_value = po, .steer_real = steer_real, .steer_des = desired_steering, .trailer = trailer};
}

extern "C" bool is_dumping(){
    return dumping;
}