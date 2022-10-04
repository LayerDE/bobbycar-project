#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include "inputreader.h"

PID* steering_controls = nullptr;
PID_ATune* pid_tuner = nullptr;
static double setPoint;
static double isPoint;
static double output;

extern "C" void pid_update(){
    if(get_input_src() == 0)
        return;
    unsigned long time = millis();
    setPoint = get_des_steering();
    isPoint = get_steering();
    steering_controls->Compute();
    set_pid_steer(output);
}

extern "C" void init_pid(){
    if(steering_controls != nullptr)
        delete steering_controls;
    double kp = 0,ki = 0, kd = 0;
    bool on = true;
    pid_tuner = new PID_ATune(&isPoint, &output);
    steering_controls = new PID(&isPoint, &output, &setPoint, kp, ki, kd, on);
    steering_controls->SetOutputLimits(-1,1);
}

extern "C" void set_pid_kp(double kp){
    steering_controls->SetTunings(kp, steering_controls->GetKi(),steering_controls->GetKd());
}

extern "C" void set_pid_ki(double ki){
    steering_controls->SetTunings(steering_controls->GetKp(),ki ,steering_controls->GetKd());
}

extern "C" void set_pid_kd(double kd){
    steering_controls->SetTunings(steering_controls->GetKp(),steering_controls->GetKi(), kd);
}

extern "C" double get_pid_kp(){
    return steering_controls->GetKp();
}

extern "C" double get_pid_ki(){
    return steering_controls->GetKi();
}

extern "C" double get_pid_kd(){
    return steering_controls->GetKd();
}

extern "C" void set_pid_refresh(int time_ms){
    steering_controls->SetSampleTime(time_ms);
}

extern "C" void set_pid_limit(double max){
    if(max >= 1.0)
        steering_controls->SetOutputLimits(-max,max);
}