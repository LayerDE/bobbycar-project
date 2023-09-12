/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2019 Ricardo Quesada

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

// Unijoysticle platform

#include "uni_platform_unijoysticle.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>

#include <argtable3/argtable3.h>
#include <driver/gpio.h>
#include <esp_chip_info.h>
#include <esp_console.h>
#include <esp_flash.h>
#include <esp_idf_version.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <hal/gpio_types.h>

#include "cmd_system.h"
#include "sdkconfig.h"
#include "uni_balance_board.h"
#include "uni_bt.h"
#include "uni_common.h"
#include "uni_config.h"
#include "uni_controller.h"
#include "uni_gamepad.h"
#include "uni_gpio.h"
#include "uni_hid_device.h"
#include "uni_hid_device_vendors.h"
#include "uni_joystick.h"
#include "uni_log.h"
#include "uni_mouse_quadrature.h"
#include "uni_platform.h"
#include "uni_platform_unijoysticle_2.h"
#include "uni_platform_unijoysticle_2plus.h"
#include "uni_platform_unijoysticle_800xl.h"
#include "uni_platform_unijoysticle_a500.h"
#include "uni_platform_unijoysticle_c64.h"
#include "uni_platform_unijoysticle_singleport.h"
#include "uni_property.h"
#include "uni_version.h"

#ifndef CONFIG_IDF_TARGET_ESP32
#error "This file can only be compiled for ESP32"
#endif  // CONFIG_IDF_TARGET_ESP32

// --- Defines / Enums

// To be used with Unijoysticle devices that only connect to one port.
// For exmaple, the Amiga device made by https://arananet.net/
// These devices have only one port, so they only cannot use JOYSTICK_PORT_A,
// and have 3 buttons mapped.
// Enabled if 1
#define PLAT_UNIJOYSTICLE_SINGLE_PORT 0

// In some board models not all GPIOs are set. Macro to simplify code for that.
#define SAFE_SET_BIT64(__value) (__value == -1) ? 0 : (1ULL << __value)

// 20 milliseconds ~= 1 frame in PAL
// 16.6 milliseconds ~= 1 frame in NTSC
// From: https://eab.abime.net/showthread.php?t=99970
//  Quickgun Turbo Pro:   7 cps
//  Zipstick:            13 cps
//  Quickshot 128F:      29 cps
//  Competition Pro:     62 cps
// One "Click-per-second" means that there is one "click" +  "release" in one second
// So the frequency must be doubled.
#define AUTOFIRE_CPS_QUICKGUN (7)          // ~71ms, ~4 frames
#define AUTOFIRE_CPS_ZIPSTICK (13)         // ~38ms, ~2 frames
#define AUTOFIRE_CPS_QUICKSHOT (29)        // ~17ms, ~1 frame
#define AUTOFIRE_CPS_COMPETITION_PRO (62)  // ~8ms, ~1/2 frame
#define AUTOFIRE_CPS_DEFAULT AUTOFIRE_CPS_QUICKGUN

// Balance Board defaults
#define BB_FIRE_MAX_FRAMES 25              // Max frames that fire can be kept pressed
#define BB_IDLE_THRESHOLD 1600             // Below this value, it is considered that noone is on top of the BB
#define BB_MOVE_THRESHOLD_DEFAULT 1500     // Diff in weight to consider a Movement
#define BB_FIRE_THRESHOLD_DEFAULT 1000000  // Max weight before staring the "de-accel" to trigger fire.

#define TASK_AUTOFIRE_PRIO (9)
#define TASK_PUSH_BUTTON_PRIO (8)
#define TASK_BLINK_LED_PRIO (7)

// Data coming from gamepad axis is different from mouse deltas.
// They need to be scaled down, otherwise the pointer moves too fast.
#define GAMEPAD_AXIS_TO_MOUSE_DELTA_RATIO (50)

enum {
    // Push buttons
    EVENT_BUTTON_0 = UNI_PLATFORM_UNIJOYSTICLE_PUSH_BUTTON_0,
    EVENT_BUTTON_1 = UNI_PLATFORM_UNIJOYSTICLE_PUSH_BUTTON_1,

    // Autofire group
    EVENT_AUTOFIRE_TRIGGER = 0,
    EVENT_AUTOFIRE_CONFIG = 1,
};

typedef enum {
    // Unknown model
    BOARD_MODEL_UNK,

    // Unijosyticle 2: Through hole version
    BOARD_MODEL_UNIJOYSTICLE2,

    // Unijoysticle 2 plus: SMT version
    BOARD_MODEL_UNIJOYSTICLE2_PLUS,

    // Unijoysticle 2 A500 version
    BOARD_MODEL_UNIJOYSTICLE2_A500,

    // Unijosyticle 2 C64 version
    BOARD_MODEL_UNIJOYSTICLE2_C64,

    // Unijosyticle 2 800XL version
    BOARD_MODEL_UNIJOYSTICLE2_800XL,

    // Unijosyticle Single port, like Arananet's Unijoy2Amiga
    BOARD_MODEL_UNIJOYSTICLE2_SINGLE_PORT,

    BOARD_MODEL_COUNT,
} board_model_t;

enum {
    BB_STATE_RESET,      // After fire
    BB_STATE_THRESHOLD,  // "fire threshold" detected
    BB_STATE_IN_AIR,     // in the air
    BB_STATE_FIRE,       // "fire pressed"
};

// CPU where the Quadrature task runs
#define QUADRATURE_MOUSE_TASK_CPU 1

// AtariST is not as fast as Amiga while processing quadrature events.
// Cap max events to the max that AtariST can process.
// This is true for the official AtariST mouse as well.
// This value was "calculated" using an AtariST 520.
// Might not be true for newer models, like the Falcon.
#define ATARIST_MOUSE_DELTA_MAX (28)

// --- Structs / Typedefs

// This is the "state" of the push button, and changes in runtime.
// This is the "mutable" part of the button that is stored in RAM.
// The "fixed" part is stored in ROM.
struct push_button_state {
    bool enabled;
    int64_t last_time_pressed_us;  // in microseconds
};

// --- Function declaration

static board_model_t get_uni_model_from_pins();

static void set_gamepad_seat(uni_hid_device_t* d, uni_gamepad_seat_t seat);
static void process_joystick(uni_hid_device_t* d, uni_gamepad_seat_t seat, const uni_joystick_t* joy);
static void process_mouse(uni_hid_device_t* d,
                          uni_gamepad_seat_t seat,
                          int32_t delta_x,
                          int32_t delta_y,
                          uint16_t buttons);
static void process_gamepad(uni_hid_device_t* d, uni_gamepad_t* gp);
static void process_balance_board(uni_hid_device_t* d, uni_balance_board_t* bb);
static void joy_update_port(const uni_joystick_t* joy, const gpio_num_t* gpios);
static void init_quadrature_mouse(void);
static int get_mouse_emulation_from_nvs(void);

// Interrupt handlers
static void handle_event_button(int button_idx);

// GPIO Interrupt handlers
static void gpio_isr_handler_button(void* arg);

static void pushbutton_event_task(void* arg);
static void auto_fire_task(void* arg);

static void maybe_enable_mouse_timers(void);

// Commands or Event related
static int cmd_swap_ports(int argc, char** argv);
static int cmd_set_gamepad_mode(int argc, char** argv);
static int cmd_get_gamepad_mode(int argc, char** argv);
static int cmd_set_autofire_cps(int argc, char** argv);
static int cmd_get_autofire_cps(int argc, char** argv);
static int cmd_set_bb_move_threshold(int argc, char** argv);
static int cmd_get_bb_move_threshold(int argc, char** argv);
static int cmd_set_bb_fire_threshold(int argc, char** argv);
static int cmd_get_bb_fire_threshold(int argc, char** argv);
static int cmd_set_mouse_emulation(int argc, char** argv);
static int cmd_get_mouse_emulation(int argc, char** argv);
static int cmd_version(int argc, char** argv);
static int get_bb_move_threshold_from_nvs(void);
static int get_bb_fire_threshold_from_nvs(void);
static void swap_ports(void);
static void try_swap_ports(uni_hid_device_t* d);
static void set_next_gamepad_mode(uni_hid_device_t* d);
static void set_gamepad_mode(uni_hid_device_t* d, uni_platform_unijoysticle_gamepad_mode_t mode);
static void get_gamepad_mode(uni_hid_device_t* d);
static void version(void);

static void blink_bt_led(int times);
static void maybe_enable_bluetooth(bool enabled);

// --- Consts (ROM)

static const bd_addr_t zero_addr = {0, 0, 0, 0, 0, 0};

// Keep them in the order of the defines
static const char* mouse_modes[] = {
    "unknown",  // UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_FROM_BOARD_MODEL
    "amiga",    // UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_AMIGA
    "atarist",  // UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_ATARIST
};

// --- Globals (RAM)
static const struct uni_platform_unijoysticle_variant* g_variant;
// Used as cache of g_variant->gpio_config
static const struct uni_platform_unijoysticle_gpio_config* g_gpio_config;

static EventGroupHandle_t g_pushbutton_group;
static EventGroupHandle_t g_autofire_group;

struct push_button_state g_push_buttons_state[UNI_PLATFORM_UNIJOYSTICLE_PUSH_BUTTON_MAX] = {0};

// Autofire
static bool g_autofire_a_enabled;
static bool g_autofire_b_enabled;

// Button "mode". Used in A500/C64/800XL
static int s_bluetooth_led_on;  // Used as a cache
static bool s_auto_enable_bluetooth = true;
// TODO: The Bluetooth Event should have an originator, instead of using this hack.
// When True, it means the "Unijosyticle" generated the Bluetooth-Enable event.
static bool s_skip_next_enable_bluetooth_event = false;

// Gets initialized at platform_init time.
static int balanceboard_move_threshold;
static int balanceboard_fire_threshold;

// Cache of mouse emulation type
static int mouse_emulation_cached;

// For the console
static struct {
    struct arg_str* value;
    struct arg_end* end;
} set_gamepad_mode_args;

static struct {
    struct arg_int* value;
    struct arg_end* end;
} set_autofire_cps_args;

static struct {
    struct arg_int* value;
    struct arg_end* end;
} set_bb_move_threshold_args;

static struct {
    struct arg_int* value;
    struct arg_end* end;
} set_bb_fire_threshold_args;

static struct {
    struct arg_str* value;
    struct arg_end* end;
} set_mouse_emulation_args;

static btstack_context_callback_registration_t cmd_callback_registration;

//
// Platform Overrides
//
static void unijoysticle_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    board_model_t model = get_uni_model_from_pins();

    switch (model) {
        case BOARD_MODEL_UNIJOYSTICLE2_PLUS:
            g_variant = uni_platform_unijoysticle_2plus_create_variant();
            break;
        case BOARD_MODEL_UNIJOYSTICLE2_A500:
            g_variant = uni_platform_unijoysticle_a500_create_variant();
            break;
        case BOARD_MODEL_UNIJOYSTICLE2_C64:
            g_variant = uni_platform_unijoysticle_c64_create_variant();
            break;
        case BOARD_MODEL_UNIJOYSTICLE2_800XL:
            g_variant = uni_platform_unijoysticle_800xl_create_variant();
            break;
        case BOARD_MODEL_UNIJOYSTICLE2_SINGLE_PORT:
            g_variant = uni_platform_unijoysticle_singleport_create_variant();
            break;
        case BOARD_MODEL_UNIJOYSTICLE2:
        default:
            g_variant = uni_platform_unijoysticle_2_create_variant();
            break;
    }
    // Cache gpio_config. One less reference
    g_gpio_config = g_variant->gpio_config;
    logi("Hardware detected: Unijoysticle%s\n", g_variant->name);

    gpio_config_t io_conf = {0};

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = 0;
    // Setup pins for Port A & B
    for (int i = 0; i < UNI_PLATFORM_UNIJOYSTICLE_JOY_MAX; i++) {
        io_conf.pin_bit_mask |= SAFE_SET_BIT64(g_gpio_config->port_a[i]);
        io_conf.pin_bit_mask |= SAFE_SET_BIT64(g_gpio_config->port_b[i]);
    }

    // Setup pins for LEDs
    for (int i = 0; i < UNI_PLATFORM_UNIJOYSTICLE_LED_MAX; i++)
        io_conf.pin_bit_mask |= SAFE_SET_BIT64(g_gpio_config->leds[i]);

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Set low all joystick GPIOs... just in case.
    for (int i = 0; i < UNI_PLATFORM_UNIJOYSTICLE_JOY_MAX; i++) {
        ESP_ERROR_CHECK(uni_gpio_set_level(g_gpio_config->port_a[i], 0));
        ESP_ERROR_CHECK(uni_gpio_set_level(g_gpio_config->port_b[i], 0));
    }

    // Turn On Player LEDs
    uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_J1], 1);
    uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_J2], 1);
    // Turn off Bluetooth LED
    uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_BT], 0);

    // Tasks should be created before the ISR, just in case an interrupt
    // gets called before the Task-that-handles-the-ISR gets triggered.

    // Split "events" from "auto_fire", since auto-fire is an on-going event.
    g_pushbutton_group = xEventGroupCreate();
    xTaskCreate(pushbutton_event_task, "bp.uni.button", 4096, NULL, TASK_PUSH_BUTTON_PRIO, NULL);

    g_autofire_group = xEventGroupCreate();
    xTaskCreate(auto_fire_task, "bp.uni.autofire", 4096, NULL, TASK_AUTOFIRE_PRIO, NULL);

    // Push Buttons
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (int i = 0; i < UNI_PLATFORM_UNIJOYSTICLE_PUSH_BUTTON_MAX; i++) {
        if (g_gpio_config->push_buttons[i].gpio == -1 || g_gpio_config->push_buttons[i].callback == NULL)
            continue;

        io_conf.intr_type = GPIO_INTR_ANYEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        // GPIOs 34~39 don't have internal Pull-up resistors.
        io_conf.pull_up_en =
            (g_gpio_config->push_buttons[i].gpio < GPIO_NUM_34) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        io_conf.pin_bit_mask = BIT64(g_gpio_config->push_buttons[i].gpio);
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        // "i" must match EVENT_BUTTON_0, value, etc.
        ESP_ERROR_CHECK(gpio_isr_handler_add(g_gpio_config->push_buttons[i].gpio, gpio_isr_handler_button, (void*)i));
    }
}

static void unijoysticle_on_init_complete(void) {
    // Turn off LEDs
    uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_J1], 0);
    uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_J2], 0);

    // Update Balance Board threshold
    balanceboard_move_threshold = get_bb_move_threshold_from_nvs();
    balanceboard_fire_threshold = get_bb_fire_threshold_from_nvs();

    if (g_variant->flags & UNI_PLATFORM_UNIJOYSTICLE_VARIANT_FLAG_QUADRATURE_MOUSE)
        init_quadrature_mouse();

    if (g_variant->on_init_complete)
        g_variant->on_init_complete();
}

static void unijoysticle_on_device_connected(uni_hid_device_t* d) {
    int connected = 0;
    if (d == NULL) {
        loge("ERROR: unijoysticle_on_device_connected: Invalid NULL device\n");
    }

    for (int i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; i++) {
        uni_hid_device_t* tmp_d = uni_hid_device_get_instance_for_idx(i);
        if (uni_bt_conn_is_connected(&tmp_d->conn))
            connected++;
    }

    if (connected == 2) {
        maybe_enable_bluetooth(false);
    }
}

static void unijoysticle_on_device_disconnected(uni_hid_device_t* d) {
    if (d == NULL) {
        loge("ERROR: unijoysticle_on_device_disconnected: Invalid NULL device\n");
        return;
    }
    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);

    if (ins->seat != GAMEPAD_SEAT_NONE) {
        // Turn off the LEDs
        if (ins->seat == GAMEPAD_SEAT_A || ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK)
            uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_J1], 0);
        if (ins->seat == GAMEPAD_SEAT_B || ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK)
            uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_J2], 0);

        ins->seat = GAMEPAD_SEAT_NONE;
        ins->gamepad_mode = UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL;
    }

    // Regarless of how many connections are active, enable Bluetooth connections.
    maybe_enable_bluetooth(true);

    maybe_enable_mouse_timers();
}

static int unijoysticle_on_device_ready(uni_hid_device_t* d) {
    int wanted_seat;

    if (d == NULL) {
        loge("ERROR: unijoysticle_on_device_ready: Invalid NULL device\n");
        return -1;
    }
    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);

    // Some safety checks. These conditions should not happen
    if ((ins->seat != GAMEPAD_SEAT_NONE) || (!uni_hid_device_has_controller_type(d))) {
        loge("ERROR: unijoysticle_on_device_ready: pre-condition not met\n");
        return -1;
    }

    uint32_t used_joystick_ports = 0;
    for (int i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; i++) {
        uni_hid_device_t* tmp_d = uni_hid_device_get_instance_for_idx(i);
        used_joystick_ports |= uni_platform_unijoysticle_get_instance(tmp_d)->seat;
    }

    // Either two gamepads are connected, or one is in Twin Stick mode.
    // Don't allow new connections.
    if (used_joystick_ports == (GAMEPAD_SEAT_A | GAMEPAD_SEAT_B))
        return -1;

    if (uni_hid_device_is_mouse(d))
        wanted_seat = g_variant->preferred_seat_for_mouse;
    else
        wanted_seat = g_variant->preferred_seat_for_joystick;

    ins->gamepad_mode = UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL;

    // If wanted port is already assigned, try with the next one.
    if (used_joystick_ports & wanted_seat) {
        logi("unijoysticle: Port %d already assigned, trying another one\n", wanted_seat);
        wanted_seat = (~wanted_seat) & GAMEPAD_SEAT_AB_MASK;
    }

    set_gamepad_seat(d, wanted_seat);

    maybe_enable_mouse_timers();

    return 0;
}

static void test_gamepad_select_button(uni_hid_device_t* d, uni_gamepad_t* gp) {
    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);
    bool already_pressed = ins->buttons_debouncer & MISC_BUTTON_BACK;

    if (gp->misc_buttons & MISC_BUTTON_BACK) {
        // 'Select' pressed
        if (already_pressed)
            return;
        ins->buttons_debouncer |= MISC_BUTTON_BACK;
        try_swap_ports(d);
    } else {
        ins->buttons_debouncer &= ~MISC_BUTTON_BACK;
    }
}

static void test_gamepad_start_button(uni_hid_device_t* d, uni_gamepad_t* gp) {
    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);
    bool already_pressed = ins->buttons_debouncer & MISC_BUTTON_HOME;

    if (gp->misc_buttons & MISC_BUTTON_HOME) {
        // 'Start' pressed
        if (already_pressed)
            return;
        ins->buttons_debouncer |= MISC_BUTTON_HOME;
        set_next_gamepad_mode(d);
    } else {
        ins->buttons_debouncer &= ~MISC_BUTTON_HOME;
    }
}

static void unijoysticle_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    if (d == NULL) {
        loge("ERROR: unijoysticle_on_device_gamepad_data: Invalid NULL device\n");
        return;
    }

    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);
    switch (ctl->klass) {
        case UNI_CONTROLLER_CLASS_GAMEPAD:
            process_gamepad(d, &ctl->gamepad);
            break;
        case UNI_CONTROLLER_CLASS_MOUSE:
            process_mouse(d, ins->seat, ctl->mouse.delta_x, ctl->mouse.delta_y, ctl->mouse.buttons);
            break;
        case UNI_CONTROLLER_CLASS_BALANCE_BOARD:
            process_balance_board(d, &ctl->balance_board);
        default:
            break;
    }
}

static int32_t unijoysticle_get_property(uni_platform_property_t key) {
    if (key != UNI_PLATFORM_PROPERTY_DELETE_STORED_KEYS)
        return -1;

    if (g_gpio_config->push_buttons[UNI_PLATFORM_UNIJOYSTICLE_PUSH_BUTTON_0].gpio == -1)
        return -1;

    // Hi-released, Low-pressed
    return !gpio_get_level(g_gpio_config->push_buttons[UNI_PLATFORM_UNIJOYSTICLE_PUSH_BUTTON_0].gpio);
}

static void unijoysticle_on_oob_event(uni_platform_oob_event_t event, void* data) {
    if (event == UNI_PLATFORM_OOB_BLUETOOTH_ENABLED) {
        // Turn on/off the BT led
        bool enabled = (bool)data;
        uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_BT], enabled);
        s_bluetooth_led_on = enabled;

        logi("unijoysticle: Bluetooth discovery mode is %s\n", enabled ? "enabled" : "disabled");

        if (!enabled && !s_skip_next_enable_bluetooth_event) {
            // Means that Bluetooth was disabled by user from the console.
            // If so, leave it disabled. The only way to enable it, is again from the console.
            s_auto_enable_bluetooth = false;
        }
        s_skip_next_enable_bluetooth_event = false;
        return;
    }

    if (event != UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON) {
        loge("ERROR: unijoysticle_on_device_oob_event: unsupported event: 0x%04x\n", event);
        return;
    }

    uni_hid_device_t* d = data;

    if (d == NULL) {
        loge("ERROR: unijoysticle_on_device_gamepad_event: Invalid NULL device\n");
        return;
    }

    try_swap_ports(d);
}

static void unijoysticle_device_dump(uni_hid_device_t* d) {
    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);

    logi("\tunijoysticle: ");
    if (uni_hid_device_is_mouse(d)) {
        logi("type=mouse, ");
    } else {
        logi("type=gamepad, mode=");
        if (ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK) {
            if (ins->swap_ports_in_twinstick)
                logi("twin stick swapped, ");
            else
                logi("twin stick, ");
        } else if (ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE) {
            logi("mouse, ");
        } else if (ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL) {
            logi("normal, ");
        } else {
            logi("unk, ");
        }
    }
    logi("seat=0x%02x\n", ins->seat);
}

static void init_quadrature_mouse(void) {
    // Values taken from:
    // * http://wiki.icomp.de/wiki/DE-9_Mouse
    // * https://www.waitingforfriday.com/?p=827#Commodore_Amiga
    // But they contradict on the Amiga pinout. Using "waitingforfriday" pinout.
    int x1, x2, y1, y2;
    mouse_emulation_cached = get_mouse_emulation_from_nvs();
    switch (mouse_emulation_cached) {
        case UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_AMIGA:
            x1 = 1;
            x2 = 3;
            y1 = 2;
            y2 = 0;
            logi("Unijoysticle: Using Amiga mouse emulation\n");
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_ATARIST:
            x1 = 1;
            x2 = 0;
            y1 = 2;
            y2 = 3;
            logi("Unijoysticle: Using AtariST mouse emulation\n");
            break;
        default:
            loge("Unijoysticle: Invalid mouse emulation mode\n");
            return;
    }

    // FIXME: These values are hardcoded for Amiga
    struct uni_mouse_quadrature_encoder_gpios port_a_x = {
        .a = g_gpio_config->port_a[x1],  // H-pulse (up)
        .b = g_gpio_config->port_a[x2],  // HQ-pulse (left)
    };

    struct uni_mouse_quadrature_encoder_gpios port_a_y = {
        .a = g_gpio_config->port_a[y1],  // V-pulse (down)
        .b = g_gpio_config->port_a[y2],  // VQ-pulse (right)
    };

    // Mouse AtariST is known to only work only one port A, but for the sake
    // of completness, both ports are configured on AtariST. Overkill ?
    struct uni_mouse_quadrature_encoder_gpios port_b_x = {
        .a = g_gpio_config->port_b[x1],  // H-pulse (up)
        .b = g_gpio_config->port_b[x2],  // HQ-pulse (left)
    };
    struct uni_mouse_quadrature_encoder_gpios port_b_y = {
        .a = g_gpio_config->port_b[y1],  // V-pulse (down)
        .b = g_gpio_config->port_b[y2],  // VQ-pulse (right)
    };

    uni_mouse_quadrature_init(QUADRATURE_MOUSE_TASK_CPU);
    uni_mouse_quadrature_setup_port(UNI_MOUSE_QUADRATURE_PORT_0, port_a_x, port_a_y);
    uni_mouse_quadrature_setup_port(UNI_MOUSE_QUADRATURE_PORT_1, port_b_x, port_b_y);
}

static void set_mouse_emulation_to_nvs(int mode) {
    uni_property_value_t value;
    value.u32 = mode;

    uni_property_set(UNI_PROPERTY_KEY_UNI_MOUSE_EMULATION, UNI_PROPERTY_TYPE_U32, value);
    logi("Done. Restart required. Type 'restart' + Enter\n");
}

static int get_mouse_emulation_from_nvs(void) {
    uni_property_value_t value;
    uni_property_value_t def;

    def.u32 = g_variant->default_mouse_emulation;

    value = uni_property_get(UNI_PROPERTY_KEY_UNI_MOUSE_EMULATION, UNI_PROPERTY_TYPE_U32, def);

    // Validate return value.
    if (value.u8 >= UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_COUNT ||
        value.u8 == UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_FROM_BOARD_MODEL)
        return UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_AMIGA;
    return value.u8;
}

static int cmd_set_mouse_emulation(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_mouse_emulation_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_mouse_emulation_args.end, argv[0]);
        return 1;
    }

    if (strcmp(set_mouse_emulation_args.value->sval[0], "amiga") == 0) {
        set_mouse_emulation_to_nvs(UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_AMIGA);
    } else if (strcmp(set_mouse_emulation_args.value->sval[0], "atarist") == 0) {
        set_mouse_emulation_to_nvs(UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_ATARIST);
    } else {
        loge("Invalid mouse emulation: %s\n", set_mouse_emulation_args.value->sval[0]);
        loge("Valid values: 'amiga' or 'atarist'\n");
        return 1;
    }

    return 0;
}

static int cmd_get_mouse_emulation(int argc, char** argv) {
    int mode = get_mouse_emulation_from_nvs();

    if (mode >= UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_COUNT) {
        logi("Invalid mouse emulation: %d\n", mode);
        return 1;
    }

    logi("%s\n", mouse_modes[mode]);
    return 0;
}

static void register_console_cmds_quadrature_mouse(void) {
    set_mouse_emulation_args.value = arg_str1(NULL, NULL, "<emulation>", "valid options: 'amiga' or 'atarist'");
    set_mouse_emulation_args.end = arg_end(2);

    const esp_console_cmd_t set_mouse_emulation = {
        .command = "set_mouse_emulation",
        .help =
            "Sets mouse emulation mode.\n"
            "  Default: amiga",
        .hint = NULL,
        .func = &cmd_set_mouse_emulation,
        .argtable = &set_mouse_emulation_args,
    };

    const esp_console_cmd_t get_mouse_emulation = {
        .command = "get_mouse_emulation",
        .help = "Returns mouse emulation mode",
        .hint = NULL,
        .func = &cmd_get_mouse_emulation,
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&set_mouse_emulation));
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_mouse_emulation));
}

static void unijoysticle_register_cmds(void) {
    set_gamepad_mode_args.value = arg_str1(NULL, NULL, "<mode>", "valid options: 'normal', 'twinstick' or 'mouse'");
    set_gamepad_mode_args.end = arg_end(2);

    set_autofire_cps_args.value = arg_int1(NULL, NULL, "<cps>", "clicks per second (cps)");
    set_autofire_cps_args.end = arg_end(2);

    set_bb_move_threshold_args.value = arg_int1(NULL, NULL, "<threshold>", "balance board 'move weight' threshold");
    set_bb_move_threshold_args.end = arg_end(2);

    set_bb_fire_threshold_args.value = arg_int1(NULL, NULL, "<threshold>", "balance board 'fire weight' threshold");
    set_bb_fire_threshold_args.end = arg_end(2);

    const esp_console_cmd_t swap_ports = {
        .command = "swap_ports",
        .help = "Swaps joystick ports",
        .hint = NULL,
        .func = &cmd_swap_ports,
    };

    const esp_console_cmd_t set_gamepad_mode = {
        .command = "set_gamepad_mode",
        .help =
            "Sets the gamepad mode.\n"
            "  At least one gamepad must be connected.\n"
            "  Default: normal",
        .hint = NULL,
        .func = &cmd_set_gamepad_mode,
        .argtable = &set_gamepad_mode_args,
    };

    const esp_console_cmd_t get_gamepad_mode = {
        .command = "get_gamepad_mode",
        .help =
            "Returns the gamepad mode.\n"
            "  At least one gamepad must be connected",
        .hint = NULL,
        .func = &cmd_get_gamepad_mode,
    };

    const esp_console_cmd_t set_autofire_cps = {
        .command = "set_autofire_cps",
        .help =
            "Sets the autofire 'clicks per second' (cps)\n"
            "Default: 7",
        .hint = NULL,
        .func = &cmd_set_autofire_cps,
        .argtable = &set_autofire_cps_args,
    };

    const esp_console_cmd_t get_autofire_cps = {
        .command = "get_autofire_cps",
        .help = "Returns the autofire 'clicks per second' (cps)",
        .hint = NULL,
        .func = &cmd_get_autofire_cps,
    };

    const esp_console_cmd_t version = {
        .command = "version",
        .help = "Gets the Unijoysticle version info",
        .hint = NULL,
        .func = &cmd_version,
    };

    const esp_console_cmd_t set_bb_move_threshold = {
        .command = "set_bb_move_threshold",
        .help =
            "Sets the Balance Board 'Move Weight' threshold\n"
            "Default: 5000",  // BB_MOVE_THRESHOLD_DEFAULT
        .hint = NULL,
        .func = &cmd_set_bb_move_threshold,
        .argtable = &set_bb_move_threshold_args,
    };

    const esp_console_cmd_t get_bb_move_threshold = {
        .command = "get_bb_move_threshold",
        .help = "Returns the Balance Board Move threshold (weight)",
        .hint = NULL,
        .func = &cmd_get_bb_move_threshold,
    };

    const esp_console_cmd_t set_bb_fire_threshold = {
        .command = "set_bb_fire_threshold",
        .help =
            "Sets the Balance Board 'Fire Weight' threshold\n"
            "Default: 50000",  // BB_FIRE_THRESHOLD_DEFAULT
        .hint = NULL,
        .func = &cmd_set_bb_fire_threshold,
        .argtable = &set_bb_fire_threshold_args,
    };

    const esp_console_cmd_t get_bb_fire_threshold = {
        .command = "get_bb_fire_threshold",
        .help = "Returns the Balance Board Fire threshold (weight)",
        .hint = NULL,
        .func = &cmd_get_bb_fire_threshold,
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&swap_ports));
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_gamepad_mode));
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_gamepad_mode));

    ESP_ERROR_CHECK(esp_console_cmd_register(&set_autofire_cps));
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_autofire_cps));

    if (g_variant->flags & UNI_PLATFORM_UNIJOYSTICLE_VARIANT_FLAG_QUADRATURE_MOUSE)
        register_console_cmds_quadrature_mouse();

    if (g_variant->register_console_cmds)
        g_variant->register_console_cmds();

    ESP_ERROR_CHECK(esp_console_cmd_register(&version));

    ESP_ERROR_CHECK(esp_console_cmd_register(&set_bb_move_threshold));
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_bb_move_threshold));
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_bb_fire_threshold));
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_bb_fire_threshold));
}

//
// Helpers
//

static const char* get_uni_model_from_nvs(void) {
    uni_property_value_t def;
    uni_property_value_t value;

    def.str = "Unknown";
    value = uni_property_get(UNI_PROPERTY_KEY_UNI_MODEL, UNI_PROPERTY_TYPE_STRING, def);
    return value.str;
}

static const char* get_uni_vendor_from_nvs(void) {
    uni_property_value_t def;
    uni_property_value_t value;

    def.str = "Unknown";
    value = uni_property_get(UNI_PROPERTY_KEY_UNI_VENDOR, UNI_PROPERTY_TYPE_STRING, def);
    return value.str;
}

static int get_uni_serial_number_from_nvs(void) {
    uni_property_value_t def;
    uni_property_value_t value;

    def.u32 = 0;
    value = uni_property_get(UNI_PROPERTY_KEY_UNI_SERIAL_NUMBER, UNI_PROPERTY_TYPE_U32, def);
    return value.u32;
}

static void set_autofire_cps_to_nvs(int cps) {
    uni_property_value_t value;
    value.u8 = cps;

    uni_property_set(UNI_PROPERTY_KEY_UNI_AUTOFIRE_CPS, UNI_PROPERTY_TYPE_U8, value);
    logi("Done\n");
}

static int get_autofire_cps_from_nvs(void) {
    uni_property_value_t value;
    uni_property_value_t def;

    def.u8 = AUTOFIRE_CPS_DEFAULT;

    value = uni_property_get(UNI_PROPERTY_KEY_UNI_AUTOFIRE_CPS, UNI_PROPERTY_TYPE_U8, def);
    return value.u8;
}

static void set_bb_move_threshold_to_nvs(int threshold) {
    uni_property_value_t value;
    value.u32 = threshold;

    uni_property_set(UNI_PROPERTY_KEY_UNI_BB_MOVE_THRESHOLD, UNI_PROPERTY_TYPE_U32, value);
    logi("Done\n");
}

static int get_bb_move_threshold_from_nvs(void) {
    uni_property_value_t value;
    uni_property_value_t def;

    def.u32 = BB_MOVE_THRESHOLD_DEFAULT;

    value = uni_property_get(UNI_PROPERTY_KEY_UNI_BB_MOVE_THRESHOLD, UNI_PROPERTY_TYPE_U32, def);
    return value.u32;
}

static void set_bb_fire_threshold_to_nvs(int threshold) {
    uni_property_value_t value;
    value.u32 = threshold;

    uni_property_set(UNI_PROPERTY_KEY_UNI_BB_FIRE_THRESHOLD, UNI_PROPERTY_TYPE_U32, value);
    logi("Done\n");
}

static int get_bb_fire_threshold_from_nvs(void) {
    uni_property_value_t value;
    uni_property_value_t def;

    def.u32 = BB_FIRE_THRESHOLD_DEFAULT;

    value = uni_property_get(UNI_PROPERTY_KEY_UNI_BB_FIRE_THRESHOLD, UNI_PROPERTY_TYPE_U32, def);
    return value.u32;
}

static board_model_t get_uni_model_from_pins(void) {
#if PLAT_UNIJOYSTICLE_SINGLE_PORT
    // Legacy: Only needed for Arananet's Unijoy2Amiga.
    // Single-port boards should ground GPIO 5. It will be detected in runtime.
    return BOARD_MODEL_UNIJOYSTICLE2_SINGLE_PORT;
#else
    // Cache value. Detection must only be done once.
    static board_model_t model = BOARD_MODEL_UNK;
    if (model != BOARD_MODEL_UNK)
        return model;

    // Detect hardware version based on GPIOs 4, 5, 15
    //              GPIO 4   GPIO 5    GPIO 15  GPIO 36   GPIO 39
    // Uni 2:       Hi       Hi        Hi
    // Uni 2+:      Low      Hi        Hi
    // Uni 2 A500:  Hi       Hi        Lo       Lo        Lo
    // Uni 2 800XL: Hi       Hi        Lo       Hi        Lo
    // Uni 2 C64:   Low      Hi        Lo
    // Single port: Hi       Low       Hi

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);

    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY);

    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);

    // GPIO 36-39 are input only and don't have internal Pull ups/downs.
    gpio_set_direction(GPIO_NUM_39, GPIO_MODE_INPUT);

    int gpio_4 = gpio_get_level(GPIO_NUM_4);
    int gpio_5 = gpio_get_level(GPIO_NUM_5);
    int gpio_15 = gpio_get_level(GPIO_NUM_15);
    int gpio_39 = gpio_get_level(GPIO_NUM_39);

    logi("Unijoysticle: Board ID values: %d,%d,%d\n", gpio_4, gpio_5, gpio_15);
    if (gpio_5 == 0)
        model = BOARD_MODEL_UNIJOYSTICLE2_SINGLE_PORT;
    else if (gpio_4 == 1 && gpio_15 == 1)
        model = BOARD_MODEL_UNIJOYSTICLE2;
    else if (gpio_4 == 0 && gpio_15 == 1)
        model = BOARD_MODEL_UNIJOYSTICLE2_PLUS;
    else if (gpio_4 == 1 && gpio_15 == 0 && gpio_39 == 0)
        model = BOARD_MODEL_UNIJOYSTICLE2_A500;
    else if (gpio_4 == 1 && gpio_15 == 0 && gpio_39 == 1)
        model = BOARD_MODEL_UNIJOYSTICLE2_800XL;
    else if (gpio_4 == 0 && gpio_15 == 0)
        model = BOARD_MODEL_UNIJOYSTICLE2_C64;
    else {
        logi("Unijoysticle: Invalid Board ID value: %d,%d,%d\n", gpio_4, gpio_5, gpio_15);
        model = BOARD_MODEL_UNIJOYSTICLE2;
    }

    // After detection, remove the pullup. The GPIOs might be used for something else after booting.
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_FLOATING);
    gpio_set_pull_mode(GPIO_NUM_5, GPIO_FLOATING);
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_FLOATING);
    gpio_set_pull_mode(GPIO_NUM_36, GPIO_FLOATING);
    return model;
#endif  // !PLAT_UNIJOYSTICLE_SINGLE_PORT
}

static void process_mouse(uni_hid_device_t* d,
                          uni_gamepad_seat_t seat,
                          int32_t delta_x,
                          int32_t delta_y,
                          uint16_t buttons) {
    ARG_UNUSED(d);

    if (!(g_variant->flags & UNI_PLATFORM_UNIJOYSTICLE_VARIANT_FLAG_QUADRATURE_MOUSE))
        return;

    static uint16_t prev_buttons = 0;

    if (mouse_emulation_cached == UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_ATARIST) {
        if (delta_x < -ATARIST_MOUSE_DELTA_MAX)
            delta_x = -ATARIST_MOUSE_DELTA_MAX;
        if (delta_x > ATARIST_MOUSE_DELTA_MAX)
            delta_x = ATARIST_MOUSE_DELTA_MAX;
        if (delta_y < -ATARIST_MOUSE_DELTA_MAX)
            delta_y = -ATARIST_MOUSE_DELTA_MAX;
        if (delta_y > ATARIST_MOUSE_DELTA_MAX)
            delta_y = ATARIST_MOUSE_DELTA_MAX;
    }

    logd("unijoysticle: seat: %d, mouse: x=%d, y=%d, buttons=0x%04x\n", seat, delta_x, delta_y, buttons);

    int port_idx = (seat == GAMEPAD_SEAT_A) ? UNI_MOUSE_QUADRATURE_PORT_0 : UNI_MOUSE_QUADRATURE_PORT_1;

    uni_mouse_quadrature_update(port_idx, delta_x, delta_y);

    if (buttons != prev_buttons) {
        prev_buttons = buttons;
        int fire, button2, button3;
        if (seat == GAMEPAD_SEAT_A) {
            fire = g_gpio_config->port_a[UNI_PLATFORM_UNIJOYSTICLE_JOY_FIRE];
            button2 = g_gpio_config->port_a[UNI_PLATFORM_UNIJOYSTICLE_JOY_BUTTON2];
            button3 = g_gpio_config->port_a[UNI_PLATFORM_UNIJOYSTICLE_JOY_BUTTON3];
        } else {
            fire = g_gpio_config->port_b[UNI_PLATFORM_UNIJOYSTICLE_JOY_FIRE];
            button2 = g_gpio_config->port_b[UNI_PLATFORM_UNIJOYSTICLE_JOY_BUTTON2];
            button3 = g_gpio_config->port_b[UNI_PLATFORM_UNIJOYSTICLE_JOY_BUTTON3];
        }
        if (fire != -1)
            gpio_set_level(fire, !!(buttons & BUTTON_A));
        if (button2 != -1)
            gpio_set_level(button2, !!(buttons & BUTTON_B));
        if (button3 != -1)
            gpio_set_level(button3, !!(buttons & BUTTON_X));
    }
}

static void process_joystick(uni_hid_device_t* d, uni_gamepad_seat_t seat, const uni_joystick_t* joy) {
    ARG_UNUSED(d);
    if (seat == GAMEPAD_SEAT_A) {
        joy_update_port(joy, g_gpio_config->port_a);
        g_autofire_a_enabled = joy->auto_fire;
    } else if (seat == GAMEPAD_SEAT_B) {
        joy_update_port(joy, g_gpio_config->port_b);
        g_autofire_b_enabled = joy->auto_fire;
    } else {
        loge("unijoysticle: process_joystick: invalid gamepad seat: %d\n", seat);
    }

    if (g_autofire_a_enabled || g_autofire_b_enabled) {
        xEventGroupSetBits(g_autofire_group, BIT(EVENT_AUTOFIRE_TRIGGER));
    }
}

static void process_gamepad(uni_hid_device_t* d, uni_gamepad_t* gp) {
    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);

    uni_joystick_t joy, joy_ext;
    memset(&joy, 0, sizeof(joy));
    memset(&joy_ext, 0, sizeof(joy_ext));

    switch (ins->gamepad_mode) {
        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL:
            // Special case when the accelerometer mode is enabled in Wii.
            // Use it as regular joystick
            if (d->controller_type == CONTROLLER_TYPE_WiiController &&
                d->controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_ACCEL)
                uni_joy_to_single_from_wii_accel(gp, &joy);
            else
                uni_joy_to_single_joy_from_gamepad(gp, &joy);
            process_joystick(d, ins->seat, &joy);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK:
            uni_joy_to_twinstick_from_gamepad(gp, &joy, &joy_ext);
            if (ins->swap_ports_in_twinstick) {
                process_joystick(d, GAMEPAD_SEAT_B, &joy);
                process_joystick(d, GAMEPAD_SEAT_A, &joy_ext);
            } else {
                process_joystick(d, GAMEPAD_SEAT_A, &joy);
                process_joystick(d, GAMEPAD_SEAT_B, &joy_ext);
            }
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE:
            // Allow to control the mouse with both axis. Use case:
            // - Right axis: easier to control with the right thumb (for right handed people)
            // - Left axis: easier to drag a window (move + button pressed)
            // ... but only if the axis have the same sign
            // Do: (x/ratio) + (rx/ratio), instead of "(x+rx)/ratio". We want to lose precision.
            process_mouse(
                d, ins->seat,
                (gp->axis_x / GAMEPAD_AXIS_TO_MOUSE_DELTA_RATIO) + (gp->axis_rx / GAMEPAD_AXIS_TO_MOUSE_DELTA_RATIO),
                (gp->axis_y / GAMEPAD_AXIS_TO_MOUSE_DELTA_RATIO) + (gp->axis_ry / GAMEPAD_AXIS_TO_MOUSE_DELTA_RATIO),
                gp->buttons);
            break;
        default:
            loge("unijoysticle: Unsupported emulation mode: %d\n", ins->gamepad_mode);
            break;
    }

    // Must be done at the end of the function.
    // These functions can override already set values.
    bool event_processed = false;

    if (g_variant->process_gamepad_misc_buttons)
        event_processed = g_variant->process_gamepad_misc_buttons(d, ins->seat, gp->misc_buttons);

    if (!event_processed) {
        test_gamepad_select_button(d, gp);
        test_gamepad_start_button(d, gp);
    }
}

static void process_balance_board(uni_hid_device_t* d, uni_balance_board_t* bb) {
    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);
    uni_joystick_t joy;
    memset(&joy, 0, sizeof(joy));

    // Low pass filter, assuming values arrive at the same framespeed
    ins->bb_smooth_down = mult_frac((bb->bl + bb->br) - ins->bb_smooth_down, 6, 100);
    ins->bb_smooth_top = mult_frac((bb->tl + bb->tr) - ins->bb_smooth_top, 6, 100);
    ins->bb_smooth_left = mult_frac((bb->tl + bb->bl) - ins->bb_smooth_left, 6, 100);
    ins->bb_smooth_right = mult_frac((bb->tr + bb->br) - ins->bb_smooth_right, 6, 100);

    logd("l=%d, r=%d, t=%d, d=%d\n", ins->bb_smooth_left, ins->bb_smooth_right, ins->bb_smooth_top,
         ins->bb_smooth_down);

    if ((ins->bb_smooth_top - ins->bb_smooth_down) > balanceboard_move_threshold)
        joy.up = true;
    else if ((ins->bb_smooth_down - ins->bb_smooth_top) > balanceboard_move_threshold)
        joy.down = true;

    if ((ins->bb_smooth_right - ins->bb_smooth_left) > balanceboard_move_threshold)
        joy.right = true;
    else if ((ins->bb_smooth_left - ins->bb_smooth_right) > balanceboard_move_threshold)
        joy.left = true;

    // State machine to detect whether we can trigger fire
    int sum = bb->tl + bb->tr + bb->bl + bb->br;
    ins->bb_fire_counter++;
    logd("SUM=%d, counter=%d, state=%d (%d,%d,%d,%d)\n", sum, ins->bb_fire_counter, ins->bb_fire_state, bb->tl, bb->tr,
         bb->bl, bb->br);
    switch (ins->bb_fire_state) {
        case BB_STATE_RESET:
            if (sum >= balanceboard_fire_threshold) {
                ins->bb_fire_state = BB_STATE_THRESHOLD;
                ins->bb_fire_counter = 0;
            }
            break;
        case BB_STATE_THRESHOLD:
            if (bb->tl < BB_IDLE_THRESHOLD && bb->tr < BB_IDLE_THRESHOLD && bb->bl < BB_IDLE_THRESHOLD &&
                bb->br < BB_IDLE_THRESHOLD) {
                ins->bb_fire_state = BB_STATE_IN_AIR;
                ins->bb_fire_counter = 0;
                break;
            }
            // Since threshold was triggered, it has 10 frames to get to "in_air", otherwise we reset the state.
            if (ins->bb_fire_counter > 10) {
                ins->bb_fire_state = BB_STATE_RESET;
                ins->bb_fire_counter = 0;
                break;
            }
            // Reset counter in case we are still above threshold
            if (sum >= balanceboard_fire_threshold) {
                ins->bb_fire_counter = 0;
                break;
            }
            break;
        case BB_STATE_IN_AIR:
            // Once in Air, it must be at least 2 frames in the air
            if (ins->bb_fire_counter > 2) {
                joy.fire = true;
                ins->bb_fire_state = BB_STATE_FIRE;
                ins->bb_fire_counter = 0;
                break;
            }
            if (bb->tl >= BB_IDLE_THRESHOLD || bb->tr >= BB_IDLE_THRESHOLD || bb->bl > BB_IDLE_THRESHOLD ||
                bb->br >= BB_IDLE_THRESHOLD) {
                ins->bb_fire_state = BB_STATE_RESET;
                ins->bb_fire_counter = 0;
            }
            break;
        case BB_STATE_FIRE:
            joy.fire = true;
            // Maintain "fire" pressed for 10 frames
            if (ins->bb_fire_counter > 10) {
                ins->bb_fire_state = BB_STATE_RESET;
                ins->bb_fire_counter = 0;
            }
            break;
    }

    process_joystick(d, ins->seat, &joy);
}

static void set_gamepad_seat(uni_hid_device_t* d, uni_gamepad_seat_t seat) {
    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);
    ins->seat = seat;

    logi("unijoysticle: device %s has new gamepad seat: %d\n", bd_addr_to_str(d->conn.btaddr), seat);

    // Fetch all enabled ports
    uni_gamepad_seat_t all_seats = GAMEPAD_SEAT_NONE;
    for (int i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; i++) {
        uni_hid_device_t* tmp_d = uni_hid_device_get_instance_for_idx(i);
        if (tmp_d == NULL)
            continue;
        if (bd_addr_cmp(tmp_d->conn.btaddr, zero_addr) != 0) {
            all_seats |= uni_platform_unijoysticle_get_instance(tmp_d)->seat;
        }
    }

    bool status_a = ((all_seats & GAMEPAD_SEAT_A) != 0);
    bool status_b = ((all_seats & GAMEPAD_SEAT_B) != 0);
    uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_J1], status_a);
    uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_J2], status_b);

    bool lightbar_or_led_set = false;
    if (d->report_parser.set_lightbar_color != NULL) {
        uint8_t red = 0;
        uint8_t green = 0;
        if (seat & 0x01)
            green = 0xff;
        if (seat & 0x02)
            red = 0xff;
        d->report_parser.set_lightbar_color(d, red, green, 0x00 /* blue*/);
        lightbar_or_led_set = true;
    }
    if (d->report_parser.set_player_leds != NULL) {
        d->report_parser.set_player_leds(d, seat);
        lightbar_or_led_set = true;
    }

    if (!lightbar_or_led_set && d->report_parser.set_rumble != NULL) {
        d->report_parser.set_rumble(d, 0x80 /* value */, 0x04 /* duration */);
    }
}

static void joy_update_port(const uni_joystick_t* joy, const gpio_num_t* gpios) {
    logd("up=%d, down=%d, left=%d, right=%d, fire=%d, bt2=%d, bt3=%d\n", joy->up, joy->down, joy->left, joy->right,
         joy->fire, joy->button2, joy->button3);

    uni_gpio_set_level(gpios[UNI_PLATFORM_UNIJOYSTICLE_JOY_UP], joy->up);
    uni_gpio_set_level(gpios[UNI_PLATFORM_UNIJOYSTICLE_JOY_DOWN], joy->down);
    uni_gpio_set_level(gpios[UNI_PLATFORM_UNIJOYSTICLE_JOY_LEFT], joy->left);
    uni_gpio_set_level(gpios[UNI_PLATFORM_UNIJOYSTICLE_JOY_RIGHT], joy->right);

    // Only update fire if auto-fire is off. Otherwise it will conflict.
    if (!joy->auto_fire) {
        uni_gpio_set_level(gpios[UNI_PLATFORM_UNIJOYSTICLE_JOY_FIRE], joy->fire);
    }

    if (g_variant->set_gpio_level_for_pot) {
        g_variant->set_gpio_level_for_pot(gpios[UNI_PLATFORM_UNIJOYSTICLE_JOY_BUTTON2], joy->button2);
        g_variant->set_gpio_level_for_pot(gpios[UNI_PLATFORM_UNIJOYSTICLE_JOY_BUTTON3], joy->button3);
    } else {
        uni_gpio_set_level(gpios[UNI_PLATFORM_UNIJOYSTICLE_JOY_BUTTON2], joy->button2);
        uni_gpio_set_level(gpios[UNI_PLATFORM_UNIJOYSTICLE_JOY_BUTTON3], joy->button3);
    }
}

static void pushbutton_event_task(void* arg) {
    // timeout of 100s
    const TickType_t xTicksToWait = pdMS_TO_TICKS(100000);
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(g_pushbutton_group, BIT(EVENT_BUTTON_0) | BIT(EVENT_BUTTON_1), pdTRUE,
                                               pdFALSE, xTicksToWait);

        // timeout ?
        if (bits == 0)
            continue;

        if (bits & BIT(EVENT_BUTTON_0))
            handle_event_button(EVENT_BUTTON_0);

        if (bits & BIT(EVENT_BUTTON_1))
            handle_event_button(EVENT_BUTTON_1);
    }
}

static void auto_fire_task(void* arg) {
    // timeout of 100s
    const TickType_t timeout = pdMS_TO_TICKS(100000);
    int ms = 1000 / get_autofire_cps_from_nvs() / 2;
    TickType_t delay_ticks = pdMS_TO_TICKS(ms);
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            g_autofire_group, BIT(EVENT_AUTOFIRE_TRIGGER) | BIT(EVENT_AUTOFIRE_CONFIG), pdTRUE, pdFALSE, timeout);

        // timeout ?
        if (bits == 0)
            continue;

        if (bits & BIT(EVENT_AUTOFIRE_CONFIG)) {
            ms = 1000 / get_autofire_cps_from_nvs() / 2;
            delay_ticks = pdMS_TO_TICKS(ms);
        }

        if (bits & BIT(EVENT_AUTOFIRE_TRIGGER)) {
            while (g_autofire_a_enabled || g_autofire_b_enabled) {
                if (g_autofire_a_enabled)
                    uni_gpio_set_level(g_gpio_config->port_a[UNI_PLATFORM_UNIJOYSTICLE_JOY_FIRE], 1);
                if (g_autofire_b_enabled)
                    uni_gpio_set_level(g_gpio_config->port_b[UNI_PLATFORM_UNIJOYSTICLE_JOY_FIRE], 1);

                vTaskDelay(delay_ticks);

                if (g_autofire_a_enabled)
                    uni_gpio_set_level(g_gpio_config->port_a[UNI_PLATFORM_UNIJOYSTICLE_JOY_FIRE], 0);
                if (g_autofire_b_enabled)
                    uni_gpio_set_level(g_gpio_config->port_b[UNI_PLATFORM_UNIJOYSTICLE_JOY_FIRE], 0);

                vTaskDelay(delay_ticks);
            }
        }
    }
}

static void gpio_isr_handler_button(void* arg) {
    int button_idx = (int)arg;

    // Stored din ROM
    const struct uni_platform_unijoysticle_push_button* pb = &g_gpio_config->push_buttons[button_idx];
    // Stored in RAM
    struct push_button_state* st = &g_push_buttons_state[button_idx];

    // Button released ?
    if (gpio_get_level(pb->gpio)) {
        st->last_time_pressed_us = esp_timer_get_time();
        return;
    }

    // Button pressed
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(g_pushbutton_group, BIT(button_idx), &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

static void handle_event_button(int button_idx) {
    // Basic software debouncer. If it fails, it could be improved with:
    // https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/
    const int64_t button_threshold_time_us = 300 * 1000;  // 300ms

    // Stored in ROM
    const struct uni_platform_unijoysticle_push_button* pb = &g_gpio_config->push_buttons[button_idx];
    // Stored in RAM
    struct push_button_state* st = &g_push_buttons_state[button_idx];

    // Regardless of the state, ignore the event if not enough time passed.
    int64_t now = esp_timer_get_time();
    if ((now - st->last_time_pressed_us) < button_threshold_time_us)
        return;

    st->last_time_pressed_us = now;

    // "up" button is released. Ignore event.
    if (gpio_get_level(pb->gpio)) {
        return;
    }

    // "down", button pressed.
    logi("handle_event_button(%d): %d -> %d\n", button_idx, st->enabled, !st->enabled);

    st->enabled = !st->enabled;
    pb->callback(button_idx);
}

static void cmd_callback(void* context) {
    uni_platform_unijoysticle_cmd_t cmd = (uni_platform_unijoysticle_cmd_t)context;
    switch (cmd) {
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_SWAP_PORTS:
            swap_ports();
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_GAMEPAD_MODE_TWINSTICK:
            set_gamepad_mode(NULL, UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_GAMEPAD_MODE_MOUSE:
            set_gamepad_mode(NULL, UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_GAMEPAD_MODE_NORMAL:
            set_gamepad_mode(NULL, UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_GAMEPAD_MODE_NEXT:
            set_next_gamepad_mode(NULL);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_GET_GAMEPAD_MODE:
            get_gamepad_mode(NULL);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_C64_POT_MODE_3BUTTONS:
            uni_platform_unijoysticle_c64_set_pot_mode(UNI_PLATFORM_UNIJOYSTICLE_C64_POT_MODE_3BUTTONS);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_C64_POT_MODE_5BUTTONS:
            uni_platform_unijoysticle_c64_set_pot_mode(UNI_PLATFORM_UNIJOYSTICLE_C64_POT_MODE_5BUTTONS);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_C64_POT_MODE_RUMBLE:
            uni_platform_unijoysticle_c64_set_pot_mode(UNI_PLATFORM_UNIJOYSTICLE_C64_POT_MODE_RUMBLE);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_C64_POT_MODE_PADDLE:
            uni_platform_unijoysticle_c64_set_pot_mode(UNI_PLATFORM_UNIJOYSTICLE_C64_POT_MODE_PADDLE);
            break;
        default:
            loge("Unijoysticle: invalid command: %d\n", cmd);
            break;
    }
}

static void version(void) {
    esp_chip_info_t info;
    uint32_t flash_size;
    esp_chip_info(&info);

#if ESP_IDF_VERSION_MAJOR == 4
    const esp_app_desc_t* app_desc = esp_ota_get_app_description();
#else
    const esp_app_desc_t* app_desc = esp_app_get_description();
#endif

    logi("Unijoysticle info:\n");
    logi("\tModel: %s\n", get_uni_model_from_nvs());
    logi("\tVendor: %s\n", get_uni_vendor_from_nvs());
    logi("\tSerial Number: %04d\n", get_uni_serial_number_from_nvs());
    logi("\tDetected Model: Unijoysticle %s\n", g_variant->name);

    if (g_variant->flags & UNI_PLATFORM_UNIJOYSTICLE_VARIANT_FLAG_QUADRATURE_MOUSE)
        logi("\tMouse Emulation: %s\n", mouse_modes[get_mouse_emulation_from_nvs()]);

    if (g_variant->print_version)
        g_variant->print_version();

    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        loge("Flash size failed\n");
        flash_size = 0;
    }

    logi("\nFirmware info:\n");
    logi("\tBluepad32 Version: v%s (%s)\n", UNI_VERSION, app_desc->version);
    logi("\tCompile Time: %s %s\n", app_desc->date, app_desc->time);

    logi("\n");
    cmd_system_version();
}

static void get_gamepad_mode(uni_hid_device_t* d) {
    // Change emulation mode
    int num_devices = 0;

    if (d == NULL) {
        for (int j = 0; j < CONFIG_BLUEPAD32_MAX_DEVICES; j++) {
            uni_hid_device_t* tmp_d = uni_hid_device_get_instance_for_idx(j);
            if (uni_bt_conn_is_connected(&tmp_d->conn)) {
                num_devices++;
                if (!uni_hid_device_is_mouse(tmp_d)) {
                    d = tmp_d;
                    break;
                }
            }
        }
    }

    if (d == NULL) {
        loge("unijoysticle: Cannot find a connected gamepad\n");
        return;
    }

    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);

    switch (ins->gamepad_mode) {
        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK:
            logi("twinstick\n");
            break;

        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE:
            logi("mouse\n");
            break;

        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL:
            logi("normal\n");
            break;

        default:
            logi("unknown\n");
            break;
    }
}

static void set_next_gamepad_mode(uni_hid_device_t* d) {
    uni_platform_unijoysticle_instance_t* ins;

    if (d == NULL) {
        for (int j = 0; j < CONFIG_BLUEPAD32_MAX_DEVICES; j++) {
            uni_hid_device_t* tmp_d = uni_hid_device_get_instance_for_idx(j);
            if (uni_bt_conn_is_connected(&tmp_d->conn)) {
                if (uni_hid_device_is_gamepad(tmp_d)) {
                    // Get the first valid gamepad device
                    d = tmp_d;
                    break;
                }
            }
        }
    }

    if (d == NULL) {
        loge("unijoysticle: Cannot find a connected gamepad\n");
        return;
    }

    ins = uni_platform_unijoysticle_get_instance(d);

    // Order is:
    // Normal -> Mouse -> Twin Stick -> Normal...
    switch (ins->gamepad_mode) {
        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL:
            if (g_variant->supported_modes & UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE)
                set_gamepad_mode(d, UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE);
            else if (g_variant->supported_modes & UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK)
                set_gamepad_mode(d, UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK);
            // else: Nothing
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE:
            if (g_variant->supported_modes & UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK)
                set_gamepad_mode(d, UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK);
            else
                set_gamepad_mode(d, UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_GAMEPAD_MODE_NORMAL);
            break;
        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK:
            set_gamepad_mode(d, UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL);
            break;
        default:
            loge("Unexpected value: %d", ins->gamepad_mode);
    }
}

static void set_gamepad_mode(uni_hid_device_t* d, uni_platform_unijoysticle_gamepad_mode_t mode) {
    int num_devices = 0;
    for (int j = 0; j < CONFIG_BLUEPAD32_MAX_DEVICES; j++) {
        uni_hid_device_t* tmp_d = uni_hid_device_get_instance_for_idx(j);
        if (uni_bt_conn_is_connected(&tmp_d->conn)) {
            num_devices++;
            if (uni_hid_device_is_gamepad(tmp_d) && d == NULL) {
                // Get the first valid gamepad device
                d = tmp_d;
            }
        }
    }

    if (d == NULL) {
        loge("unijoysticle: Cannot find a connected gamepad\n");
        return;
    }

    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);

    if (ins->gamepad_mode == mode) {
        logi("unijoysticle: Already in gamepad mode %d\n", mode);
        return;
    }

    if (ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NONE) {
        logi("unijoysticle: Unexpected gamepad mode: %d\n", ins->gamepad_mode);
        return;
    }

    switch (mode) {
        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK:
            if (num_devices != 1) {
                logi("unijoysticle: cannot change to Twin stick mode. Expected num_devices=1, actual=%d\n",
                     num_devices);

                // Reset to "normal" mode
                ins->gamepad_mode = UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL;
                blink_bt_led(1);
                logi("unijoysticle: Gamepad mode = normal\n");
                break;
            }

            ins->gamepad_mode = UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK;
            blink_bt_led(3);

            ins->prev_seat = ins->seat;
            set_gamepad_seat(d, GAMEPAD_SEAT_A | GAMEPAD_SEAT_B);
            logi("unijoysticle: Gamepad mode = twinstick\n");

            maybe_enable_bluetooth(false);
            break;

        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE:
            if (ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK) {
                set_gamepad_seat(d, ins->prev_seat);
                maybe_enable_bluetooth(num_devices < 2);
            }
            ins->gamepad_mode = UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE;
            blink_bt_led(2);
            logi("unijoysticle: Gamepad mode = mouse\n");
            break;

        case UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL:
            if (ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK) {
                set_gamepad_seat(d, ins->prev_seat);
                maybe_enable_bluetooth(num_devices < 2);
            }
            ins->gamepad_mode = UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL;
            blink_bt_led(1);
            logi("unijoysticle: Gamepad mode = normal\n");
            break;

        default:
            loge("unijoysticle: unsupported gamepad mode: %d\n", mode);
            break;
    }
    maybe_enable_mouse_timers();
}

static int cmd_set_gamepad_mode(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_gamepad_mode_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_gamepad_mode_args.end, argv[0]);
        return 1;
    }

    uni_platform_unijoysticle_cmd_t mode;

    if (strcmp(set_gamepad_mode_args.value->sval[0], "normal") == 0) {
        mode = UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_GAMEPAD_MODE_NORMAL;
    } else if (strcmp(set_gamepad_mode_args.value->sval[0], "twinstick") == 0) {
        mode = UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_GAMEPAD_MODE_TWINSTICK;
    } else if (strcmp(set_gamepad_mode_args.value->sval[0], "mouse") == 0) {
        mode = UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_GAMEPAD_MODE_MOUSE;
    } else {
        loge("Invalid mouse emulation: %s\n", set_gamepad_mode_args.value->sval[0]);
        loge("Valid values: 'normal', 'twinstick', or 'mouse'\n");
        return 1;
    }

    uni_platform_unijoysticle_run_cmd(mode);
    return 0;
}

static int cmd_get_gamepad_mode(int argc, char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uni_platform_unijoysticle_run_cmd(UNI_PLATFORM_UNIJOYSTICLE_CMD_GET_GAMEPAD_MODE);
    return 0;
}

static int cmd_version(int argc, char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    version();
    return 0;
}

static void swap_ports(void) {
    uni_hid_device_t* d;
    uni_platform_unijoysticle_instance_t* ins;
    uni_gamepad_seat_t prev_seat, new_seat;

    for (int i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; i++) {
        d = uni_hid_device_get_instance_for_idx(i);
        if (uni_bt_conn_is_connected(&d->conn)) {
            ins = uni_platform_unijoysticle_get_instance(d);

            if (ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK) {
                // Swap is done in the "Twin Stick" mode driver.
                ins->swap_ports_in_twinstick = !ins->swap_ports_in_twinstick;
                blink_bt_led(1);
                return;
            }

            prev_seat = ins->seat;
            new_seat = ~prev_seat & GAMEPAD_SEAT_AB_MASK;
            logi("unijoysticle: swap seat for %s: %d -> %d\n", bd_addr_to_str(d->conn.btaddr), prev_seat, new_seat);
            set_gamepad_seat(d, new_seat);
        }
    }

    // Clear joystick after switch to avoid having a line "On".
    uni_joystick_t joy;
    memset(&joy, 0, sizeof(joy));
    process_joystick(d, GAMEPAD_SEAT_A, &joy);
    process_joystick(d, GAMEPAD_SEAT_B, &joy);

    maybe_enable_mouse_timers();

    blink_bt_led(1);
}

// Call this function, instead of "swap_ports" when the user requested
// a swap port from a button press.
static void try_swap_ports(uni_hid_device_t* d) {
    if (get_uni_model_from_pins() == BOARD_MODEL_UNIJOYSTICLE2_SINGLE_PORT) {
        logi("INFO: unijoysticle_on_device_oob_event: No effect in single port boards\n");
        return;
    }

    uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);

    if (ins->seat == GAMEPAD_SEAT_NONE) {
        logi("unijoysticle: cannot swap port since device has joystick_port = GAMEPAD_SEAT_NONE\n");
        return;
    }

    if (ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK) {
        goto ok;
    }

    // Swap joystick ports except if there is a connected gamepad that doesn't have the "System" or "Select" button
    // pressed. Basically allow:
    //  - swapping mouse+gamepad
    //  - two gamepads while both are pressing the "system" or "select" button at the same time.
    for (int j = 0; j < CONFIG_BLUEPAD32_MAX_DEVICES; j++) {
        uni_hid_device_t* tmp_d = uni_hid_device_get_instance_for_idx(j);
        uni_platform_unijoysticle_instance_t* tmp_ins = uni_platform_unijoysticle_get_instance(tmp_d);
        if (uni_bt_conn_is_connected(&tmp_d->conn) &&                                  // Is it connected ?
            tmp_ins->seat != GAMEPAD_SEAT_NONE &&                                      // Does it have a seat ?
            tmp_ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL &&  // Is it in "Normal" mode ?
            ((tmp_d->controller.gamepad.misc_buttons & (MISC_BUTTON_SYSTEM | MISC_BUTTON_BACK)) == 0)  // misc pressed?
        ) {
            // If so, don't allow swap.
            logi("unijoysticle: to swap ports press 'system' button on both gamepads at the same time\n");
            uni_hid_device_dump_all();
            return;
        }
    }

ok:
    swap_ports();
}

static int cmd_swap_ports(int argc, char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uni_platform_unijoysticle_run_cmd(UNI_PLATFORM_UNIJOYSTICLE_CMD_SWAP_PORTS);
    return 0;
}

static int cmd_set_autofire_cps(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_autofire_cps_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_autofire_cps_args.end, argv[0]);
        return 1;
    }
    int cps = set_autofire_cps_args.value->ival[0];
    set_autofire_cps_to_nvs(cps);

    logi("New autofire cps: %d\n", cps);

    xEventGroupSetBits(g_autofire_group, BIT(EVENT_AUTOFIRE_CONFIG));
    return 0;
}

static int cmd_get_autofire_cps(int argc, char** argv) {
    int cps = get_autofire_cps_from_nvs();

    logi("%d\n", cps);
    return 0;
}

static int cmd_set_bb_move_threshold(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_bb_move_threshold_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_bb_move_threshold_args.end, argv[0]);
        return 1;
    }
    int threshold = set_bb_move_threshold_args.value->ival[0];
    set_bb_move_threshold_to_nvs(threshold);

    // Update static value
    balanceboard_move_threshold = threshold;

    logi("New Balance Board Move threshold: %d\n", threshold);
    return 0;
}

static int cmd_get_bb_move_threshold(int argc, char** argv) {
    int threshold = get_bb_move_threshold_from_nvs();

    logi("%d\n", threshold);
    return 0;
}

static int cmd_set_bb_fire_threshold(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_bb_fire_threshold_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_bb_fire_threshold_args.end, argv[0]);
        return 1;
    }
    int threshold = set_bb_fire_threshold_args.value->ival[0];
    set_bb_fire_threshold_to_nvs(threshold);

    // Update static value
    balanceboard_fire_threshold = threshold;

    logi("New Balance Board Fire threshold: %d\n", threshold);
    return 0;
}

static int cmd_get_bb_fire_threshold(int argc, char** argv) {
    int threshold = get_bb_fire_threshold_from_nvs();

    logi("%d\n", threshold);
    return 0;
}

static void maybe_enable_mouse_timers(void) {
    if (!(g_variant->flags & UNI_PLATFORM_UNIJOYSTICLE_VARIANT_FLAG_QUADRATURE_MOUSE))
        return;

    // Mouse support requires that the mouse timers are enabled.
    // Only enable them when needed
    bool enable_timer_0 = false;
    bool enable_timer_1 = false;

    for (int i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; i++) {
        uni_hid_device_t* d = uni_hid_device_get_instance_for_idx(i);
        if (uni_bt_conn_is_connected(&d->conn)) {
            uni_platform_unijoysticle_instance_t* ins = uni_platform_unijoysticle_get_instance(d);

            // Gamepad behaves like a mouse or is it a mouse?
            if ((uni_hid_device_is_gamepad(d) && ins->gamepad_mode == UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE) ||
                uni_hid_device_is_mouse(d)) {
                if (ins->seat == GAMEPAD_SEAT_A)
                    enable_timer_0 = true;
                else if (ins->seat == GAMEPAD_SEAT_B)
                    enable_timer_1 = true;
            }
        }
    }

    logi("Unijoysticle: mice timers enabled/disabled: port A=%d, port B=%d\n", enable_timer_0, enable_timer_1);

    if (enable_timer_0)
        uni_mouse_quadrature_start(UNI_MOUSE_QUADRATURE_PORT_0);
    else
        uni_mouse_quadrature_pause(UNI_MOUSE_QUADRATURE_PORT_0);

    if (enable_timer_1)
        uni_mouse_quadrature_start(UNI_MOUSE_QUADRATURE_PORT_1);
    else
        uni_mouse_quadrature_pause(UNI_MOUSE_QUADRATURE_PORT_1);
}

static void task_blink_bt_led(void* arg) {
    int times = (int)arg;

    while (times--) {
        uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_BT], 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_BT], 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    uni_gpio_set_level(g_gpio_config->leds[UNI_PLATFORM_UNIJOYSTICLE_LED_BT], s_bluetooth_led_on);

    // Kill itself
    vTaskDelete(NULL);
}

static void blink_bt_led(int times) {
    xTaskCreate(task_blink_bt_led, "bp.uni.blink", 2048, (void*)times, TASK_BLINK_LED_PRIO, NULL);
}

static void maybe_enable_bluetooth(bool enabled) {
    // Only enable/disable Bluetooth if automatic is on
    if (!s_auto_enable_bluetooth)
        return;
    // We are going to receive a "enable BT" event generated by us. Skip it.
    s_skip_next_enable_bluetooth_event = true;
    uni_bt_enable_new_connections_safe(enabled);
}

//
// "Protected": Called from variants
//
void uni_platform_unijoysticle_run_cmd(uni_platform_unijoysticle_cmd_t cmd) {
    cmd_callback_registration.callback = &cmd_callback;
    cmd_callback_registration.context = (void*)cmd;
    btstack_run_loop_execute_on_main_thread(&cmd_callback_registration);
}

void uni_platform_unijoysticle_on_push_button_mode_pressed(int button_idx) {
    ARG_UNUSED(button_idx);
    uni_platform_unijoysticle_run_cmd(UNI_PLATFORM_UNIJOYSTICLE_CMD_SET_GAMEPAD_MODE_NEXT);
}

void uni_platform_unijoysticle_on_push_button_swap_pressed(int button_idx) {
    ARG_UNUSED(button_idx);
    uni_platform_unijoysticle_run_cmd(UNI_PLATFORM_UNIJOYSTICLE_CMD_SWAP_PORTS);
}

//
// Public
//

struct uni_platform* uni_platform_unijoysticle_create(void) {
    static struct uni_platform plat = {
        .name = "unijoysticle2",
        .init = unijoysticle_init,
        .on_init_complete = unijoysticle_on_init_complete,
        .on_device_connected = unijoysticle_on_device_connected,
        .on_device_disconnected = unijoysticle_on_device_disconnected,
        .on_device_ready = unijoysticle_on_device_ready,
        .on_oob_event = unijoysticle_on_oob_event,
        .on_controller_data = unijoysticle_on_controller_data,
        .get_property = unijoysticle_get_property,
        .device_dump = unijoysticle_device_dump,
        .register_console_cmds = unijoysticle_register_cmds,
    };

    return &plat;
}

uni_platform_unijoysticle_instance_t* uni_platform_unijoysticle_get_instance(const uni_hid_device_t* d) {
    return (uni_platform_unijoysticle_instance_t*)&d->platform_data[0];
}
