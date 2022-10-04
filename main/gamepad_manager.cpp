#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Bluepad32.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            GamepadProperties properties = gp->getProperties();
            Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
                           properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        printf("CALLBACK: Gamepad connected, but could not found empty slot\n");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        printf("CALLBACK: Gamepad disconnected, but not found in myGamepads\n");
    }
}

extern "C" void init_gpm(){
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    // BP32.forgetBluetoothKeys();
    BP32.enableNewBluetoothConnections(true);
}

extern "C" void gpm_read(int *throttle,int *steering, int *active){
    BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];
        if (myGamepad && myGamepad->isConnected()) {
            *steering = myGamepad->axisX();
            *throttle = myGamepad->throttle()-myGamepad->brake();  // (0 - 1023): throttle (AKA gas) button
            if (myGamepad->x()){
                if(*active != 2)
                    *active = 2;
                else
                    *active = 0;
            }
            // There are different ways to query whether a button is pressed.
            // By query each button individually:
            //  a(), b(), x(), y(), l1(), etc...
            if (myGamepad->a()) {
                static int colorIdx = 0;
                // Some gamepads like DS4 and DualSense support changing the color LED.
                // It is possible to change it by calling:
                switch (colorIdx % 3) {
                    case 0:
                        // Red
                        myGamepad->setColorLED(255, 0, 0);
                        break;
                    case 1:
                        // Green
                        myGamepad->setColorLED(0, 255, 0);
                        break;
                    case 2:
                        // Blue
                        myGamepad->setColorLED(0, 0, 255);
                        break;
                }
                colorIdx++;
            }

            if (myGamepad->b()) {
                // Turn on the 4 LED. Each bit represents one LED.
                static int led = 0;
                led++;
                // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
                // support changing the "Player LEDs": those 4 LEDs that usually indicate
                // the "gamepad seat".
                // It is possible to change them by calling:
                myGamepad->setPlayerLEDs(led & 0x0f);
            }

            if (myGamepad->x()) {
                // Duration: 255 is ~2 seconds
                // force: intensity
                // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
                // rumble.
                // It is possible to set it by calling:
                myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
            }

            // Another way to query the buttons, is by calling buttons(), or
            // miscButtons() which return a bitmask.
            // Some gamepads also have DPAD, axis and more.
            printf(
                "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, "
                "%4d, brake: %4d, throttle: %4d, misc: 0x%02x\n",
                i,                        // Gamepad Index
                myGamepad->dpad(),        // DPAD
                myGamepad->buttons(),     // bitmask of pressed buttons
                myGamepad->axisX(),       // (-511 - 512) left X Axis
                myGamepad->axisY(),       // (-511 - 512) left Y axis
                myGamepad->axisRX(),      // (-511 - 512) right X axis
                myGamepad->axisRY(),      // (-511 - 512) right Y axis
                myGamepad->brake(),       // (0 - 1023): brake button
                myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
                myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
            );

            // You can query the axis and other properties as well. See Gamepad.h
            // For all the available functions.
        }
    }
}