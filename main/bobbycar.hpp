#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <stdint.h>

typedef struct {
    uint16_t start;
    int16_t speed0;
    int16_t speed1;
    uint16_t checksumL;
    uint16_t checksumH;
} SerialCommand;

typedef struct {
    uint16_t start;
    uint16_t steps0;
    uint16_t steps1;
    int16_t speedR_meas;
    int16_t speedL_meas;
    int16_t batVoltage;
    int16_t boardTemp;
    uint16_t cmdLed;
    uint16_t checksumL;
    uint16_t checksumH;
} SerialFeedback;

typedef struct {
    uint8_t idx;  // index_buff_vals for new data pointer
    uint8_t buffer[sizeof(SerialFeedback)];
    unsigned long lastUpdate;
    unsigned long lastValidPack;
} SerialVariables;

void Send(EspSoftwareSerial::UART* board, int16_t speed0, int16_t speed1);
bool Receive(EspSoftwareSerial::UART* board, SerialFeedback* out, SerialVariables *vars, unsigned long time);
