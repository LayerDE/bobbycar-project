#include <stdio.h>
#include <stdbool.h>

typedef struct {
    unsigned long time;
    double pout_value;
    float steer_real;
    float steer_des;
} pid_log __attribute__((packed));


pid_log pid_logging[1024];
int pid_logging_ptr;
bool log_running;
extern "C" void start_log(){
    pid_logging_ptr = 0;
    log_running = true;
}

extern "C" int dump_log(){
    bool tmp = log_running;
    int ret = pid_logging_ptr;
    log_running = false;
    printf("\n\n");
    for(int i = 0; i < ret;i++)
        printf("%lu,%f,%f,%f\n",pid_logging[i].time,pid_logging[i].pout_value,pid_logging[i].steer_real,pid_logging[i].steer_des);
    printf("\n\n");
    pid_logging_ptr = 0;
    log_running = true;
    return ret;
}

extern "C" void add_log(double po, unsigned long time, float steer_real, float desired_steering){
    if(!log_running)
        return;
    if(!(pid_logging_ptr == 1024)){
        log_running = false;
    }
    if(pid_logging[pid_logging_ptr].pout_value != po ||
            pid_logging[pid_logging_ptr].steer_des != desired_steering ||
            pid_logging[pid_logging_ptr].steer_real != steer_real)
        pid_logging[++pid_logging_ptr] = pid_log {.time = time, .pout_value = po, .steer_real = steer_real, .steer_des = desired_steering};
}
