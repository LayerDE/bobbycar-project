#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "c_data.h"
#include "math_functions.h"
#include "inputreader.h"
#include "command_interpreter.h"
#include "pid_controls.h"

const char endl4ptr = '\0';
const char newl4ptr = '\n';

typedef bool (*func_ptr)(const char*, c_data*);
typedef struct 
{
    const char* name;
    func_ptr exec;
} command;

const char* inputs[] = {"ADC","CONSOLE", "GAMEPAD"};

void clean_argv_cdata(c_data* argv){
    for (int i=0; ((char*)argv->content)[i] && i < argv->size; i++)
        if(((char*)argv->content)[i]==' ' || ((char*)argv->content)[i]=='\n' || ((char*)argv->content)[i]=='\t'){
            ((char*)argv->content)[i] = '\0';
            break;
        }
        else
            ((char*)argv->content)[i] = toupper((unsigned char)((char*)argv->content)[i]);
}

static bool internal_set_int(const char* argv, int *out){
    c_data parameter = c_data_spawn();
    c_data_set(&parameter,argv,strlen(argv));
    clean_argv_cdata(&parameter);
    int tmp = (1<<31)-1;
    sscanf(((char*)parameter.content),"%i",&tmp);
    c_data_delete(parameter);
    if(tmp == (1<<31)-1)
        return false;
    *out = tmp;
    return true;
}

static bool internal_set_float(const char* argv, float *out){
    c_data parameter = c_data_spawn();
    c_data_set(&parameter,argv,strlen(argv));
    clean_argv_cdata(&parameter);
    float tmp = 180.0;
    sscanf(((char*)parameter.content),"%f",&tmp);
    c_data_delete(parameter);
    if(tmp == 180.0)
        return false;
    *out = tmp;
    return true;
}

static bool echo(const char* argv, c_data* out){
    c_data_extend_raw(out, argv, strlen(argv));
    c_data_extend_raw(out, &newl4ptr, sizeof(newl4ptr));
    return true;
}
static bool print_help_of(const char* argv, c_data* out);

static bool cmd_set_steering(const char* argv, c_data* out){
    float tmp = 0;
    if(!internal_set_float(argv,&tmp))
        return false;
    {
        char buffer[20];
        sprintf(buffer, "setS:%f\n",deg2rad(tmp));
        c_data_extend_raw(out, buffer, strlen(buffer));
    }
    set_des_steering(deg2rad(tmp));
    return true;
}

static bool cmd_set_throttle(const char* argv, c_data* out){
    int tmp = 0;
    if(!internal_set_int(argv,&tmp))
        return false;
    {
        char buffer[20];
        sprintf(buffer, "setT:%i\n",tmp);
        c_data_extend_raw(out, buffer, strlen(buffer));
    }
    set_ext_throttle(tmp);
    return true;
}


static bool cmd_get_steering(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "S:%f\n",rad2deg(get_steering()));
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}

static bool cmd_get_des_steering(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "dS:%f\n",rad2deg(get_des_steering()));
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}

static bool cmd_get_throttle(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "T:%i\n",get_throttle());
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}

static bool cmd_set_pid_kp(const char* argv, c_data* out){
    float tmp = 0;
    if(!internal_set_float(argv,&tmp))
        return false;
    {
        char buffer[20];
        sprintf(buffer, "setKp:%f\n",tmp);
        c_data_extend_raw(out, buffer, strlen(buffer));
    }
    set_pid_kp(tmp);
    return true;
}
static bool cmd_set_pid_ki(const char* argv, c_data* out){
    float tmp = 0;
    if(!internal_set_float(argv,&tmp))
        return false;
    {
        char buffer[20];
        sprintf(buffer, "setKi:%f\n",tmp);
        c_data_extend_raw(out, buffer, strlen(buffer));
    }
    set_pid_ki(tmp);
    return true;
}
static bool cmd_set_pid_kd(const char* argv, c_data* out){
    float tmp = 0;
    if(!internal_set_float(argv,&tmp))
        return false;
    {
        char buffer[20];
        sprintf(buffer, "setKd:%f\n",tmp);
        c_data_extend_raw(out, buffer, strlen(buffer));
    }
    set_pid_kd(tmp);
    return true;
}

static bool cmd_autolevel(const char* argv, c_data* out){
    if(get_input_src()==0){
        char buffer[20];
        sprintf(buffer, "Failed I:%s\n",inputs[get_input_src()]);
        c_data_extend_raw(out, buffer, strlen(buffer));
        return false;
    }
    else{
        char buffer[20];
        sprintf(buffer, "Valid I:%s\n",inputs[get_input_src()]);
        c_data_extend_raw(out, buffer, strlen(buffer));
        return true;
    }
    
}

static bool cmd_set_input(const char* argv, c_data* out){
    char buffer[20];
    c_data parameter = c_data_spawn();
    c_data_set(&parameter,argv,strlen(argv));
    clean_argv_cdata(&parameter);
    for(int i = 0;i < (sizeof(inputs)/sizeof(char*));i++)
        if(strcmp(inputs[i],((char*)parameter.content)) == 0){
            set_input_src(i);
            c_data_delete(parameter);
            return true;
        }
    c_data_delete(parameter);
    return false;
}

static bool cmd_get_input(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "I:%s\n",inputs[get_input_src()]);
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}

static bool cmd_get_pid_kp(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "Kp:%f\n",get_pid_kp());
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}
static bool cmd_get_pid_ki(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "Ki:%f\n",get_pid_ki());
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}
static bool cmd_get_pid_kd(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "Kd:%f\n",get_pid_kd());
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}

static bool cmd_get_pid_out(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "PID_Out:%f\n",get_pid_steer());
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}

static const command commands[] = {
    {"help",print_help_of},
    {"echo", echo},
    {"sets",cmd_set_steering},
    {"sett",cmd_set_throttle},
    {"seti",cmd_set_input},
    {"gets",cmd_get_steering},
    {"getds",cmd_get_des_steering},
    {"gett",cmd_get_throttle},
    {"geti",cmd_get_input},
    {"getkp",cmd_get_pid_kp},
    {"getki",cmd_get_pid_ki},
    {"getkd",cmd_get_pid_kd},
    {"setkp",cmd_set_pid_kp},
    {"setki",cmd_set_pid_ki},
    {"setkd",cmd_set_pid_kd},
    {"getpo",cmd_get_pid_out},
    {"autolvl",cmd_autolevel},
    {"exec",exec}
};

static bool print_help_of(const char* argv, c_data* out){
    printf("help:\n");
    for(int i = 0;i < (sizeof(commands)/sizeof(command));i++)
        printf("%s\n",commands[i].name);
    return true;
}

void exec(const char* exec, c_data* out){
    char* tmp = malloc(strlen(exec));
    strcpy(tmp,exec);
    char* argv = &tmp[strlen(exec)];
    for (int i=0; tmp[i]; i++)
        if(tmp[i]==' ' || tmp[i]=='\n' || tmp[i]=='\t'){
            tmp[i] = '\0';
            if(tmp[i+1])
                argv = &tmp[i+1];
            else
                argv = &tmp[i];
            //printf("\nint %i,%i\n",i,strlen(argv));
            break;
        }
    bool valid_command = false;
    for(int i = 0;i < (sizeof(commands)/sizeof(command));i++)
        if(strcmp(commands[i].name,tmp) == 0){
            printf("execute %i: %s (%s) with %s\n",i,tmp,commands[i].name,argv);
            if(!commands[i].exec(argv, out))
                printf("fail\n");
            valid_command = true;
            break;
        }
    if(!valid_command)
        printf("%s %i is unknown\n", tmp, strlen(tmp));
    free(tmp);
}