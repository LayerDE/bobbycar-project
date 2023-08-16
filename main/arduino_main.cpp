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

#include <soc/rtc_wdt.h>

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

#include "pushed_follower.hpp"

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

static SerialFeedback SerialFeedback_front;
static SerialFeedback SerialFeedback_rear;

static SerialVariables SerialVar_front;
static SerialVariables SerialVar_rear;

volatile int speed_per_wheel[4];
volatile int64_t distance_per_wheel[4];
volatile uint16_t last_distance[4];
volatile int speed = 0;
volatile long last_time;
volatile int voltage =0;

bool front_active = false, rear_active = false;

static bool board_connected(SerialVariables *vars, unsigned long time){
    return (time - vars->lastUpdate) < 1000;
}

static bool board_connected_valid(SerialVariables *vars, unsigned long time){
    return (time - vars->lastValidPack) < 5000;
}

bool front_connected(unsigned long time){
    return board_connected(&SerialVar_front,time);
}

bool rear_connected(unsigned long time){
    return board_connected(&SerialVar_rear,time);
}

extern "C" bool front_connected_static(){
    return front_connected(millis());
}
extern "C" bool rear_connected_static(){
    return rear_connected(millis());
}

extern "C" bool front_connected_valid_static(){
    return board_connected_valid(&SerialVar_front, millis());
}
extern "C" bool rear_connected_valid_static(){
    return board_connected_valid(&SerialVar_rear, millis());
}

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
    if(!front_connected(last_time)){
        speed = calc_average((const int*)&(speed_per_wheel[2]),2);
        voltage = SerialFeedback_rear.batVoltage;
    }
    else if(!rear_connected(last_time)){
        speed = calc_average((const int*)&(speed_per_wheel[0]),2);
        voltage = SerialFeedback_front.batVoltage;
    }
    else{
        speed = calc_average((const int*)speed_per_wheel,4);
        voltage = SerialFeedback_front.batVoltage + SerialFeedback_rear.batVoltage / 2;
    }
}

pushed_follower *trailer;

// Arduino setup function. Runs in CPU 1
void setup() {
    rtc_wdt_protect_off();
    rtc_wdt_disable();
    const int task_count = INPUT_COUNT;
    printf("Hoverboard Serial v1.1\n");
    TaskHandle_t tasks[task_count] = {NULL, NULL, NULL, NULL};
    xTaskCreate(&init_gamepad, "init_gamepad", 2048 * 3, NULL, 5, &tasks[3]);
    xTaskCreate(&init_adc_task, "init_adc_task", 2048 * 2, NULL, 5, &tasks[0]);
    xTaskCreate(&usb_console_init, "usb_console_init", 2048 * 2, NULL, 5, &tasks[1]);
    xTaskCreate(&init_rc_in, "rc_in_init", 2048 * 2, NULL, 5, &tasks[2]);
    trailer = new pushed_follower(L_WHEELBASE, L_REAR_TO_HITCH, L_HITCH_TO_FOLLOWER_AXLE, deg2rad(27.5), 60, 50);
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
    set_input_src(DEFAULT_INPUT);
    set_mode(DEFAULT_MODE);
    xTaskCreate(&adc_task, "adc", 2048 * 2, NULL, 4, NULL);
    xTaskCreate(&tast_usb_console, "tast_usb_console", 2048 * 2, NULL, 2, NULL);
    // xTaskCreate(&tast_bt_console, "tast_bt_console", 2048 * 2, NULL, 2, NULL);
    xTaskCreate(&gamepad_task, "tast_bt_gamepad", 2048 * 2, NULL, 3, NULL);

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

void program_end(){
    delete lcd;
    delete trailer;
}

void loop() {
    if(is_dumping())
     return;
    unsigned long timeNow = millis();
    int torgue_regulated = 0;
    int throttle = get_throttle();
    float steering =  get_steering();
    float des_steering = get_des_steering();
    float trailer_angle = get_trailer();
    if(!(isNear(last_throttle,throttle,100)
            && isNear(last_steering,steering,deg2rad(0.5))
            && isNear(last_des_steering,des_steering, deg2rad(0.5)))){  // only for disableing the display
        last_time = timeNow;
        last_throttle = throttle;
        last_steering = steering;
        last_des_steering = des_steering;
    }    if(Receive(&HoverSerial_front, &SerialFeedback_front, &SerialVar_front, timeNow)){
        feedback_update();
        last_distance[0] = SerialFeedback_front.steps0;
        last_distance[1] = SerialFeedback_front.steps1;
    }
    front_active = front_connected(timeNow);
        if(Receive(&HoverSerial_rear, &SerialFeedback_rear, &SerialVar_rear, timeNow)){
        feedback_update();
        last_distance[2] = SerialFeedback_rear.steps0;
        last_distance[3] = SerialFeedback_rear.steps1;
    }
    rear_active = rear_connected(timeNow);
    if (iTimeSend <= timeNow){
        iTimeSend = timeNow + TIME_SEND;
        float tmpSteering = des_steering;
        double tmp_out = 0;
        if(get_trailer_control()){ // trailer beta control
            if(sign(throttle) == -1)
                //printf("%f\n",rad2deg(
                    tmpSteering = trailer->calc_alpha_linear(trailer_angle, trailer->calc_beta_const(des_steering));
                //));
        }
        if(get_trailer_connected()){ // trailer collision protection
            if(!trailer->protection(steering,trailer_angle,throttle)) // todo
                throttle = 0;
        }
        if (!get_steering_pid_active()){ // disable torgue vectoring at higher speeds for more savety
            tmpSteering = (throttle < 300) ? steering : ((throttle < 450) ? steering * (450.0 - throttle ) / 150.0: 0.0);
            calc_torque_per_wheel(throttle, tmpSteering, 0, torgue);
        }
        else{
            pid_update(tmpSteering);
            tmp_out = get_pid_steer();
            if(ABS(tmp_out) > 0.01)
                tmp_out += SIGN(tmp_out)* CLAMP((0.05-(double)speed/100.0),0,0.05);  
            calc_torque_per_wheel(throttle, des_steering,torgue_regulated = -round(tmp_out * (float)THROTTLE_MAX) , torgue);
        }
        add_log(
            tmp_out, // PID Out
            timeNow, // Time
            rad2deg(steering), // Real Steering
            rad2deg(tmpSteering), // Steeing to control
            get_trailer_connected() ? rad2deg(trailer_angle) : 0);// tailerangle
        if(front_active)
            Send(&HoverSerial_front, torgue[0], torgue[1]);
        if(rear_active)
            Send(&HoverSerial_rear, torgue[2], torgue[3]);
        if (!((send_cnt++) % 7)) {
            if( last_time + 20000 < timeNow){
                lcd->set_power(0);
            }else{
                lcd->draw_screen(throttle, steering, tmpSteering, get_trailer_connected(), get_trailer(), torgue, torgue_regulated,front_active,rear_active, speed, voltage, get_input_src());
            }
        }
    }
    digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);  //heartbeat on esp for quickcheck if the program is running
    vTaskDelay(1);
}
