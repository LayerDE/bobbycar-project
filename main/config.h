#pragma once

#define ADC_MIN 0                  // min ADC1-value while poti at minimum-position (0 - 1023)
#define ADC_MAX ((1 << 12) -1)               // max ADC1-value while poti at maximum-position (0 - 1023)
#define ADC_MID (ADC_MAX / 2)
#define DEAD_ZONE 64

#define STR_MID (ADC_MID-100)
#define STR_RANGE (ADC_MID*10/9)

#define FLW_MID (ADC_MID)

#define STEERING_ANGLE_FACTOR (M_PI_4/1000.0f)
#define L_WHEELBASE 35.0
#define L_WIDTH 30.0
#define L_STEERING_WIDTH 20.0
#define L_STEERING_TO_WHEEL 5.0

#define L_REAR_TO_HITCH 10.0
#define L_HITCH_TO_FOLLOWER_AXLE 60.0


#define THROTTLE_MAX 1000
#define THROTTLE_REVERSE_MAX (THROTTLE_MAX * 3 / 10)
#define STR_MAX THROTTLE_MAX

#define THROTTLE0_PIN 32
#define THROTTLE1_PIN 32
#define STEERING_PIN 33
#define TRAILER_PIN 35
#define I2C_SDA 19
#define I2C_SCL 23
#define TX0 18
#define RX0 5
#define TX1 15
#define RX1 13
#define CRSF_RX 16
#define CRSF_TX 17
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


#if(THROTTLE0_PIN != THROTTLE1_PIN)
#define VAL_CNT 4
#else
#define VAL_CNT 3
#endif

#define INPUT_ADC 0
#define INPUT_CONSOLE (INPUT_ADC+1)
#define INPUT_GAMEPAD (INPUT_CONSOLE+1)
#define INPUT_RC (INPUT_GAMEPAD+1)
#define DEFAULT_INPUT INPUT_ADC
#define INPUT_COUNT 4
