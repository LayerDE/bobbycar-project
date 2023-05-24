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
    int16_t steps0;
    int16_t steps1;
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
    byte* p;          // Pointer declaration for the new received data
    byte incomingByte;
    byte incomingBytePrev;
    unsigned long lastUpdate;
} SerialVariables;

void Send(SoftwareSerial* board, int16_t speed0, int16_t speed1);
bool Receive(SoftwareSerial* board, SerialFeedback* out, SerialVariables *vars,SerialFeedback *NewFeedback, unsigned long time);