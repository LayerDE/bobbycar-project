#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include "inputreader.h"

PID* steering_controls = nullptr;
PID_ATune* pid_tuner = nullptr;
static double setPoint;
static double isPoint;
static double output;

bool start,tuning;
int ATuneModeRemember;
static void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = steering_controls->GetMode();
  else
    steering_controls->SetMode(ATuneModeRemember);
}

static void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output = 0;
    pid_tuner->SetNoiseBand(0);
    pid_tuner->SetOutputStep(0.01);
    pid_tuner->SetLookbackSec((int)1);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    pid_tuner->Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

extern "C" void pid_update(){
  if(tuning)
  {
    byte val = (pid_tuner->Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      double kp = pid_tuner->GetKp();
      double ki = pid_tuner->GetKi();
      double kd = pid_tuner->GetKd();
      steering_controls->SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else steering_controls->Compute();
  



    if(get_input_src() == 0)
        return;
    unsigned long time = millis();
    setPoint = get_des_steering();
    isPoint = get_steering();
    steering_controls->Compute();
    set_pid_steer(output);
}

extern "C" void init_pid(){
      pid_tuner = new PID_ATune(&isPoint, &output);
    pid_tuner->SetControlType(1); // set to pid
    pid_tuner->SetLookbackSec(10);
    pid_tuner->SetOutputStep(0.05);
    pid_tuner->SetNoiseBand(0);
        if(steering_controls != nullptr)
        delete steering_controls;
    double kp = 0,ki = 0, kd = 0;
    bool on = true;
    steering_controls = new PID(&isPoint, &output, &setPoint, kp, ki, kd, on);
    steering_controls->SetOutputLimits(-1,1);

    steering_controls->SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
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