/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <Arduino.h>
#include <SoftwareSerial.h>

#include <math_functions.h>
#include "config.h"
#include "defines.h"
#include "inputreader.h"
#include "console_task.h"
#include "pid_controls.h"
#include "display.hpp"
#include "display_oled.hpp"
#include "display_2004.hpp"
#include <crc32.h>
#include <Wire.h>

// unserem Fall „lcd“. Die Adresse des I²C Displays kann je nach Modul variieren.
SoftwareSerial HoverSerial_front(RX0, TX0);  // RX, TX
SoftwareSerial HoverSerial_rear(RX1, TX1);   // RX, TX
// BluetoothSerial ESP_BT; //Object for Bluetooth
// PID steering_calculator;
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...

bool dsp_connected;
typedef struct {
    uint16_t start;
    int16_t steer;
    int16_t speed_per_wheel;
    uint16_t checksumL;
    uint16_t checksumH;
} SerialCommand;

typedef struct {
    uint16_t start;
    int16_t cmd1;
    int16_t cmd2;
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

display *lcd;
SerialFeedback NewFeedback_front;
SerialFeedback NewFeedback_rear;

SerialFeedback SerialFeedback_front;
SerialFeedback SerialFeedback_rear;

SerialVariables SerialVar_front;
SerialVariables SerialVar_rear;

volatile int speed_per_wheel[4];
volatile int speed = 0;
volatile long last_time;
volatile int voltage =0;

volatile long rec_cnt = 0;

void Send(SoftwareSerial* board, int16_t speed0, int16_t speed1) {
    SerialCommand Command;
    // Create command
    Command.start = (uint16_t)START_FRAME;
    Command.steer = (int16_t)speed0;
    Command.speed_per_wheel = (int16_t)speed1;
    uint32_t checksum = calc_crc32((uint8_t*)&Command, sizeof(SerialCommand) - sizeof(uint16_t)*2);
    Command.checksumL = checksum & 0xFFFF;
    Command.checksumH = checksum >> 16;
    //printf("%x ? %x\n",Command.checksumL,Command.checksumH);
    // Write to Serial
    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&myMutex);
    board->write((uint8_t*)&Command, sizeof(SerialCommand));
    taskEXIT_CRITICAL(&myMutex);
}

// ########################## RECEIVE ##########################

bool Receive(SoftwareSerial* board, SerialFeedback* out, SerialVariables *vars,SerialFeedback *NewFeedback, unsigned long time) {
    uint16_t bufStartFrame;  // Buffer Start Frame
    // byte buffer[sizeof(SerialFeedback)];
    //  Check for new data availability in the Serial buffer
    bool data_complete = false;
    while(board->available()){
        rec_cnt++;
        vars->incomingByte = board->read();                                       // Read the incoming byte
        bufStartFrame = ((uint16_t)(vars->incomingByte) << 8) | vars->incomingBytePrev;  // Construct the start frame

            // Copy received data
        if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
            vars->p = (byte*)NewFeedback;
            *(vars->p)++ = vars->incomingBytePrev;
            *(vars->p)++ = vars->incomingByte;
            vars->idx = 2;
        } else if (vars->idx >= 2 && vars->idx < sizeof(SerialFeedback)) {  // Save the new received data
            *(vars->p)++ = vars->incomingByte;
            vars->idx++;
        }
        // Update previous states
        vars->incomingBytePrev = vars->incomingByte;
        // Check if we reached the end of the package
        if (vars->idx == sizeof(SerialFeedback)) {
            uint32_t checksum = calc_crc32((uint8_t*)NewFeedback,sizeof(SerialFeedback)-sizeof(uint16_t)*2);
            vars->idx = 0;  // Reset the index_buff_vals (it prevents to enter in this if condition in the next cycle)
            uint32_t checksum_package = (uint32_t)NewFeedback->checksumL | ((uint32_t)NewFeedback->checksumH << 16);
            // printf("%x == %x\n",checksum_package,checksum);
            // Check validity of the new data
            if (NewFeedback->start == START_FRAME && checksum == checksum_package) {
                // Copy the new data
                memcpy(out, NewFeedback, sizeof(SerialFeedback));
                // Print data to built-in Serial
                vars->lastUpdate = time;
                data_complete = true;
            }
        }
    }
    return data_complete;

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
    Serial.println(incomingByte, HEX);
#endif
}

static inline void feedback_update(){
    last_time = MAX(SerialVar_front.lastUpdate, SerialVar_rear.lastUpdate);
    speed = 0;
    speed += speed_per_wheel[0] = SerialFeedback_front.speedL_meas;
    speed += speed_per_wheel[1] = SerialFeedback_front.speedR_meas;
    speed += speed_per_wheel[2] = SerialFeedback_rear.speedL_meas;
    speed += speed_per_wheel[3] = SerialFeedback_rear.speedR_meas;
    voltage = SerialFeedback_front.batVoltage + SerialFeedback_rear.batVoltage / 2;
    speed /= 4;
}

// Arduino setup function. Runs in CPU 1
void setup() {
    const int task_count = 3;
    printf("Hoverboard Serial v1.1\n");
    TaskHandle_t tasks[task_count] = {NULL, NULL, NULL};
    xTaskCreate(&init_gamepad, "init_gamepad", 2048 * 3, NULL, 5, &tasks[0]);
    xTaskCreate(&init_adc_task, "init_adc_task", 2048 * 2, NULL, 5, &tasks[1]);
    xTaskCreate(&usb_console_init, "usb_console_init", 2048 * 2, NULL, 5, &tasks[2]);
    for(int y = task_count; y > 0;){
        y = task_count;
        for(int x = 0; x < task_count; x++)
            if(eTaskGetState(tasks[x]))
                y--;
        vTaskDelay(10);
        printf("next %i\n",y);
    }
    Wire.begin(I2C_SDA,I2C_SCL);
    // ESP_BT.begin("ESP32_BobbyCon"); //Name of your Bluetooth Signal
    // lcd.init(); //Im Setup wird der LCD gestartet
    // lcd.backlight(); //Hintergrundbeleuchtung einschalten (0 schaltet die Beleuchtung aus).
    xTaskCreate(&adc_task, "adc", 2048 * 2, NULL, 4, NULL);
    xTaskCreate(&tast_usb_console, "tast_usb_console", 2048 * 2, NULL, 2, NULL);
    // xTaskCreate(&tast_bt_console, "tast_bt_console", 2048 * 2, NULL, 2, NULL);
    xTaskCreate(&gamepad_task, "tast_bt_gamepad", 2048 * 2, NULL, 3, NULL);
      SerialFeedback_front.speedL_meas = SerialFeedback_front.speedR_meas = SerialFeedback_rear.speedL_meas = SerialFeedback_rear.speedR_meas = 0;
    HoverSerial_front.begin(HOVER_SERIAL_BAUD);
    HoverSerial_rear.begin(HOVER_SERIAL_BAUD);

    pinMode(LED_BUILTIN, OUTPUT);
    // init_debug_screen();
    init_pid();
    init_buffer();
    {
        c_data empty = c_data_spawn();
        int devices = scan_i2c(&empty, &Wire);
        printf("%i I2C device%s found\n", devices, devices > 1 ? "s" : "");
        for(int x = 0; x < empty.size; x++)
            printf("Device at 0x%02hhX\n", ((char*)empty.content)[x]);
        c_data_delete(empty);
    }
    lcd = new display_oled(&Wire, SCREEN_ADDRESS, SCREEN_WIDTH, SCREEN_HEIGHT);
    // Setup the Bluepad32 callbacks
    // addr:9C:AA:1B:E6:7D:13
}

unsigned long iTimeSend = 0;

bool controller = false;
int torgue[4];

int send_cnt = 0;
// Arduino loop function. Runs in CPU 1
char sprint_buffer[256];

int last_throttle;
float last_steering;
float last_des_steering;

bool isNear(float a,float b, float range){
    if(ABS(a-b)<range)
        return true;
    else
        return false;
}

void loop() {

    unsigned long timeNow = millis();
    int torgue_regulated = 0;
    //int throttle = throttle_calc(clean_adc_full(value_buffer(analogRead(THROTTLE0_PIN),1)));
    //float steering = calc_steering_eagle(clean_adc_steering(value_buffer(analogRead(STEERING_PIN),0)));
    int throttle = get_throttle();
    float steering =  get_steering();
    float des_steering = get_des_steering();
    if(!(isNear(last_throttle,throttle,100) && isNear(last_steering,steering,deg2rad(0.5)) && isNear(last_des_steering,des_steering, deg2rad(0.5)))){
        printf("update time %li %li\n",last_time, timeNow);
        last_time = timeNow;
        last_throttle = throttle;
        last_steering = steering;
        last_des_steering = des_steering;
    }
    if(Receive(&HoverSerial_front, &SerialFeedback_front, &SerialVar_front, &NewFeedback_front, timeNow)
        || Receive(&HoverSerial_rear, &SerialFeedback_rear, &SerialVar_rear, &NewFeedback_rear, timeNow))
        feedback_update();
    if (iTimeSend <= timeNow){
        iTimeSend = timeNow + TIME_SEND;
        if (get_input_src()==0){
            calc_torque_per_wheel(throttle, steering,0 , torgue);
        }
        else{
            pid_update();
            calc_torque_per_wheel(throttle, des_steering,torgue_regulated = round(get_pid_steer() * THROTTLE_MAX) , torgue);
        }
        Send(&HoverSerial_front, torgue[0], torgue[1]);
        Send(&HoverSerial_rear, torgue[2], torgue[3]);
        //printf("heartbeat %li %li\n",timeNow, rec_cnt);
        if (!((send_cnt++) % 7)) {
            if( last_time + 20000 < timeNow){
                lcd->set_power(0);
            }else{
                lcd->draw_screen(throttle, steering, des_steering, torgue, torgue_regulated, speed, voltage, get_input_src());
            }
        }
    }
    digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
    vTaskDelay(1);
}
