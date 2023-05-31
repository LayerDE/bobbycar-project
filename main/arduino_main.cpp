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
#include "bobbycar.hpp"

#include <Wire.h>
#include "logging.h"

// unserem Fall „lcd“. Die Adresse des I²C Displays kann je nach Modul variieren.
EspSoftwareSerial::UART HoverSerial_front(RX0, TX0);  // RX, TX
EspSoftwareSerial::UART HoverSerial_rear(RX1, TX1);   // RX, TX
// BluetoothSerial ESP_BT; //Object for Bluetooth
// PID steering_calculator;
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...

bool dsp_connected;

display *lcd;
SerialFeedback NewFeedback_front;
SerialFeedback NewFeedback_rear;

SerialFeedback SerialFeedback_front;
SerialFeedback SerialFeedback_rear;

SerialVariables SerialVar_front;
SerialVariables SerialVar_rear;

volatile int speed_per_wheel[4];
volatile int64_t distance_per_wheel[4];
volatile uint16_t last_distance[4];
volatile int speed = 0;
volatile long last_time;
volatile int voltage =0;

bool front_active = false, rear_active = false;

static inline void feedback_update(){
    last_time = MAX(SerialVar_front.lastUpdate, SerialVar_rear.lastUpdate);
    speed_per_wheel[0] = SerialFeedback_front.speedL_meas;
    speed_per_wheel[1] = SerialFeedback_front.speedR_meas;
    speed_per_wheel[2] = SerialFeedback_rear.speedL_meas;
    speed_per_wheel[3] = SerialFeedback_rear.speedR_meas;
    distance_per_wheel[0] += (SerialFeedback_front.steps0 - last_distance[0]) * sign(speed_per_wheel[0]);
    distance_per_wheel[1] += (SerialFeedback_front.steps1 - last_distance[1]) * sign(speed_per_wheel[1]);
    distance_per_wheel[2] += (SerialFeedback_rear.steps0 - last_distance[2]) * sign(speed_per_wheel[2]);
    distance_per_wheel[3] += (SerialFeedback_rear.steps1 - last_distance[3]) * sign(speed_per_wheel[3]);
    if(last_time - SerialVar_front.lastUpdate > 1000){
        speed = calc_average((const int*)&(speed_per_wheel[2]),2);
        voltage = SerialFeedback_rear.batVoltage;
    }
    else if(last_time - SerialVar_rear.lastUpdate > 1000){
        speed = calc_average((const int*)&(speed_per_wheel[0]),2);
        voltage = SerialFeedback_front.batVoltage;
    }
    else{
        speed = calc_average((const int*)speed_per_wheel,4);
        voltage = SerialFeedback_front.batVoltage + SerialFeedback_rear.batVoltage / 2;
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    const int task_count = INPUT_COUNT;
    printf("Hoverboard Serial v1.1\n");
    TaskHandle_t tasks[task_count] = {NULL, NULL, NULL, NULL};
    xTaskCreate(&init_gamepad, "init_gamepad", 2048 * 3, NULL, 5, &tasks[3]);
    xTaskCreate(&init_adc_task, "init_adc_task", 2048 * 2, NULL, 5, &tasks[0]);
    xTaskCreate(&usb_console_init, "usb_console_init", 2048 * 2, NULL, 5, &tasks[1]);
    xTaskCreate(&init_rc_in, "rc_in_init", 2048 * 2, NULL, 5, &tasks[2]);
    for(int y = task_count; y > 0;){
        y = task_count;
        for(int x = 0; x < task_count; x++)
            if(eTaskGetState(tasks[x]))
                y--;
        vTaskDelay(10);
        //printf("next %i\n",y);
    }
    printf("next\n");
    Wire.begin(I2C_SDA,I2C_SCL);
    // ESP_BT.begin("ESP32_BobbyCon"); //Name of your Bluetooth Signal
    // lcd.init(); //Im Setup wird der LCD gestartet
    // lcd.backlight(); //Hintergrundbeleuchtung einschalten (0 schaltet die Beleuchtung aus).
    xTaskCreate(&adc_task, "adc", 2048 * 2, NULL, 4, NULL);
    xTaskCreate(&tast_usb_console, "tast_usb_console", 2048 * 2, NULL, 2, NULL);
    // xTaskCreate(&tast_bt_console, "tast_bt_console", 2048 * 2, NULL, 2, NULL);
    //xTaskCreate(&gamepad_task, "tast_bt_gamepad", 2048 * 2, NULL, 3, NULL);

    xTaskCreate(&rc_in_task, "tast_rc_in", 2048 * 2, NULL, 3, NULL);
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
    int throttle = get_throttle();
    float steering =  get_steering();
    float des_steering = get_des_steering();
    if(!(isNear(last_throttle,throttle,100) && isNear(last_steering,steering,deg2rad(0.5)) && isNear(last_des_steering,des_steering, deg2rad(0.5)))){
        last_time = timeNow;
        last_throttle = throttle;
        last_steering = steering;
        last_des_steering = des_steering;
    }
    if(Receive(&HoverSerial_front, &SerialFeedback_front, &SerialVar_front, &NewFeedback_front, timeNow)){
        feedback_update();
        last_distance[0] = SerialFeedback_front.steps0;
        last_distance[1] = SerialFeedback_front.steps1;
    }
    front_active = timeNow - SerialVar_front.lastUpdate < 1000;
    if(Receive(&HoverSerial_rear, &SerialFeedback_rear, &SerialVar_rear, &NewFeedback_rear, timeNow)){
        feedback_update();
        last_distance[2] = SerialFeedback_rear.steps0;
        last_distance[3] = SerialFeedback_rear.steps1;
    }
    rear_active = timeNow - SerialVar_front.lastUpdate < 1000;
    if (iTimeSend <= timeNow){
        iTimeSend = timeNow + TIME_SEND;
        if (get_input_src()==0){
            calc_torque_per_wheel(throttle, steering,0 , torgue);
        }
        else{
            pid_update();
            double tmp_out = get_pid_steer();
            add_log(tmp_out, timeNow,rad2deg(steering),rad2deg(des_steering));
            if(ABS(tmp_out) > 0.01)
                tmp_out += SIGN(tmp_out)* CLAMP((0.05-(double)speed/100.0),0,0.05);  
            calc_torque_per_wheel(throttle, des_steering,torgue_regulated = -round(tmp_out * (float)THROTTLE_MAX) , torgue);
        }
        if(front_active)
            Send(&HoverSerial_front, torgue[0], torgue[1]);
        if(rear_active)
            Send(&HoverSerial_rear, torgue[2], torgue[3]);
        if (!((send_cnt++) % 7)) {
            if( last_time + 20000 < timeNow){
                lcd->set_power(0);
            }else{
                lcd->draw_screen(throttle, steering, des_steering, get_trailer_connected(), get_trailer(), torgue, torgue_regulated, speed, voltage, get_input_src());
            }
        }
    }
    digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
    vTaskDelay(1);
}
