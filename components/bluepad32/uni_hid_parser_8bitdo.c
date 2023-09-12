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

#include "uni_hid_parser_8bitdo.h"

#include "hid_usage.h"
#include "uni_controller.h"
#include "uni_hid_device.h"
#include "uni_hid_parser.h"
#include "uni_log.h"

// 8BitDo controllers support different "modes":
// - Nintendo Switch: it impersonates the Nintendo Switch controller
// - Windows: it impersonates the Xbox One S controller
// - macOS: it impersonates the PS4 DualShock 4 controller
// - Android: mappings are almost identical to Android controllers.
//
// This file handles the "Android" mode. The mappings with Android is almost
// identical to Android except that the layout of the A,B,X,Y buttons are
// swapped and other minor changes.
//
// The other modes are handled in the Switch, Xbox One S and PS4 files.

void uni_hid_parser_8bitdo_init_report(uni_hid_device_t* d) {
    uni_controller_t* ctl = &d->controller;
    memset(ctl, 0, sizeof(*ctl));

    ctl->klass = UNI_CONTROLLER_CLASS_GAMEPAD;
}

void uni_hid_parser_8bitdo_parse_usage(uni_hid_device_t* d,
                                       hid_globals_t* globals,
                                       uint16_t usage_page,
                                       uint16_t usage,
                                       int32_t value) {
    uint8_t hat;
    uni_controller_t* ctl = &d->controller;
    switch (usage_page) {
        case HID_USAGE_PAGE_GENERIC_DESKTOP:
            switch (usage) {
                case HID_USAGE_AXIS_X:
                    ctl->gamepad.axis_x = uni_hid_parser_process_axis(globals, value);
                    break;
                case HID_USAGE_AXIS_Y:
                    ctl->gamepad.axis_y = uni_hid_parser_process_axis(globals, value);
                    break;
                case HID_USAGE_AXIS_Z:
                    ctl->gamepad.axis_rx = uni_hid_parser_process_axis(globals, value);
                    break;
                case HID_USAGE_AXIS_RZ:
                    ctl->gamepad.axis_ry = uni_hid_parser_process_axis(globals, value);
                    break;
                case HID_USAGE_HAT:
                    hat = uni_hid_parser_process_hat(globals, value);
                    ctl->gamepad.dpad = uni_hid_parser_hat_to_dpad(hat);
                    break;
                case HID_USAGE_DPAD_UP:
                case HID_USAGE_DPAD_DOWN:
                case HID_USAGE_DPAD_RIGHT:
                case HID_USAGE_DPAD_LEFT:
                    uni_hid_parser_process_dpad(usage, value, &ctl->gamepad.dpad);
                    break;
                default:
                    logi("8BitDo: Unsupported page: 0x%04x, usage: 0x%04x, value=0x%x\n", usage_page, usage, value);
                    break;
            }
            break;
        case HID_USAGE_PAGE_SIMULATION_CONTROLS:
            switch (usage) {
                case HID_USAGE_ACCELERATOR:
                    ctl->gamepad.throttle = uni_hid_parser_process_pedal(globals, value);
                    break;
                case HID_USAGE_BRAKE:
                    ctl->gamepad.brake = uni_hid_parser_process_pedal(globals, value);
                    break;
                default:
                    logi("8BitDo: Unsupported page: 0x%04x, usage: 0x%04x, value=0x%x\n", usage_page, usage, value);
                    break;
            };
            break;
        case HID_USAGE_PAGE_BUTTON: {
            switch (usage) {
                case 0x01:  // Button A
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_B;
                    break;
                case 0x02:  // Button B
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_A;
                    break;
                case 0x03:
                    // Home Button for:
                    // M30
                    // SN30 Pro FW pre v2 (?)
                    if (value)
                        ctl->gamepad.misc_buttons |= MISC_BUTTON_SYSTEM;
                    break;
                case 0x04:  // Button X
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_Y;
                    break;
                case 0x05:  // Button Y
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_X;
                    break;
                case 0x06:  // No used
                    if (value)
                        logi("8BitDo: Unexpected PAGE_BUTTON usage=0x06\n");
                    break;
                case 0x07:
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_SHOULDER_L;
                    break;
                case 0x08:
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_SHOULDER_R;
                    break;
                case 0x09:
                    // SN30 Pro and gamepads with "trigger" buttons.
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_TRIGGER_L;
                    break;
                case 0x0a:
                    // SN30 Pro and gamepads with "trigger" buttons.
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_TRIGGER_R;
                    break;
                case 0x0b:  // "Select" button
                    if (value)
                        ctl->gamepad.misc_buttons |= MISC_BUTTON_BACK;
                    break;
                case 0x0c:  // "Start" button
                    if (value)
                        ctl->gamepad.misc_buttons |= MISC_BUTTON_HOME;
                    break;
                case 0x0d:
                    // Home Button for SN30 Pro FW v2+
                    if (value)
                        ctl->gamepad.misc_buttons |= MISC_BUTTON_SYSTEM;
                    break;
                case 0x0e:
                    // SN30 Pro and gamepads with "thumb" buttons.
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_THUMB_L;
                    break;
                case 0x0f:
                    // SN30 Pro and gamepads with "thumb" buttons.
                    if (value)
                        ctl->gamepad.buttons |= BUTTON_THUMB_R;
                    break;
                case 0x10:  // Not mapped
                    if (value)
                        logi("8BitDo: Unexpected PAGE_BUTTON usage=0x10\n");
                    break;
                default:
                    logi("8BitDo: Unsupported page: 0x%04x, usage: 0x%04x, value=0x%x\n", usage_page, usage, value);
                    break;
            }
            break;
        }
        case HID_USAGE_PAGE_GENERIC_DEVICE_CONTROLS:
            switch (usage) {
                case HID_USAGE_BATTERY_STRENGTH:
                    ctl->battery = value;
                    break;
                default:
                    if (value)
                        logi("8BitDo: Unsupported page: 0x%04x, usage: 0x%04x, value=0x%x\n", usage_page, usage, value);
                    break;
            }
            break;

        case HID_USAGE_PAGE_KEYBOARD_KEYPAD:
            // Used by 8BitDo Zero 2 in Keyboard mode
            switch (usage) {
                case 0x00:
                    // Unknown, but it is reported in each report
                    break;
                case 0x06:
                    ctl->gamepad.dpad |= value ? DPAD_UP : 0;
                    break;
                case 0x07:
                    ctl->gamepad.dpad |= value ? DPAD_DOWN : 0;
                    break;
                case 0x08:
                    ctl->gamepad.dpad |= value ? DPAD_LEFT : 0;
                    break;
                case 0x09:
                    ctl->gamepad.dpad |= value ? DPAD_RIGHT : 0;
                    break;
                case 0x0a:
                    ctl->gamepad.buttons |= value ? BUTTON_B : 0;
                    break;
                case 0x0b:
                    ctl->gamepad.buttons |= value ? BUTTON_Y : 0;
                    break;
                case 0x0c:
                    ctl->gamepad.buttons |= value ? BUTTON_X : 0;
                    break;
                case 0x0d:
                    ctl->gamepad.buttons |= value ? BUTTON_A : 0;
                    break;
                case 0x0e:
                    ctl->gamepad.buttons |= value ? BUTTON_SHOULDER_L : 0;
                    break;
                case 0x10:
                    ctl->gamepad.buttons |= value ? BUTTON_SHOULDER_R : 0;
                    break;
                case 0x11:
                    ctl->gamepad.misc_buttons |= value ? MISC_BUTTON_BACK : 0;
                    break;
                case 0x12:
                    ctl->gamepad.misc_buttons |= value ? MISC_BUTTON_HOME : 0;
                    break;
                case 0xe0:
                case 0xe1:
                case 0xe2:
                case 0xe3:
                case 0xe4:
                case 0xe5:
                case 0xe6:
                case 0xe7:
                    // Shift / Control / Alt keys. Ignore
                    break;
                default:
                    if (value)
                        logi("8BitDo: Unsupported page: 0x%04x, usage: 0x%04x, value=0x%x\n", usage_page, usage, value);
            }
            break;
        // unknown usage page
        default:
            logi("8BitDo: Unsupported page: 0x%04x, usage: 0x%04x, value=0x%x\n", usage_page, usage, value);
            break;
    }
}
