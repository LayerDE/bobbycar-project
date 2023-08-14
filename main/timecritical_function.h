#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#define timecritical(x) {\
    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;\
    taskENTER_CRITICAL(&myMutex);\
    x\
    taskEXIT_CRITICAL(&myMutex);\
}