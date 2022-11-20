#pragma once

#define ADC_MIN 0                  // min ADC1-value while poti at minimum-position (0 - 1023)
#define ADC_MAX ((1 << 12) -1)               // max ADC1-value while poti at maximum-position (0 - 1023)
#define ADC_MID (ADC_MAX / 2)
#define DEAD_ZONE 64

#define STR_MID (ADC_MID-100)
#define STR_RANGE (ADC_MID*10/9)

#define STEERING_EAGLE_FACTOR (M_PI_4/1000.0f)
#define L_WHEELBASE 35.0
#define L_WIDTH 30.0
#define L_STEERING_WIDTH 20.0
#define L_STEERING_TO_WHEEL 5.0

#define THROTTLE_MAX 1000
#define THROTTLE_REVERSE_MAX (THROTTLE_MAX * 3 / 10)
#define STR_MAX THROTTLE_MAX

#define THROTTLE0_PIN 32
#define THROTTLE1_PIN 32
#define STEERING_PIN 33
#define I2C_SDA 19
#define I2C_SCL 23
#define TX0 18
#define RX0 5
#define TX1 17
#define RX1 16
#define CRSF_RX 9
#define CRSF_TX 10
#define LED_BUILTIN 22
// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD 57600 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME 0x7A7A      // [-] Start frame definition for reliable serial communication
#define TIME_SEND 20             // [ms] Sending time interval
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define BUFFERSIZE 128
#define VAL_CNT 3

#define INPUT_ADC 0
#define INPUT_CONSOLE (INPUT_ADC+1)
#define INPUT_GAMEPAD (INPUT_CONSOLE+1)
#define INPUT_RC (INPUT_GAMEPAD+1)
