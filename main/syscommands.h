#pragma once
#include <stdbool.h>
#include "c_data.h"
#include <stdio.h>

#define DISPLAY_LOG(...) {\
    char buffer[256];\
    snprintf(buffer,256,__VA_ARGS__);\
    display_log_line(buffer);\
    printf("%s",buffer);\
}

#ifdef __cplusplus
extern "C"
{
#endif
void display_log_line(char* string);
bool reset_cmd(const char* argv, c_data* out);
#ifdef __cplusplus
}
#endif