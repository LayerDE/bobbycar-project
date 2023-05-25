#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>

#include <crc32.h>
#include "config.h"
#include "bobbycar.hpp"


void Send(EspSoftwareSerial::UART* board, int16_t speed0, int16_t speed1) {
    SerialCommand Command;
    // Create command
    Command.start = (uint16_t)START_FRAME;
    Command.speed0 = (int16_t)speed0;
    Command.speed1 = (int16_t)speed1;
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

bool Receive(EspSoftwareSerial::UART* board, SerialFeedback* out, SerialVariables *vars,SerialFeedback *NewFeedback, unsigned long time) {
    uint16_t bufStartFrame;  // Buffer Start Frame
    // byte buffer[sizeof(SerialFeedback)];
    //  Check for new data availability in the Serial buffer
    bool data_complete = false;
    while(board->available()){
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
}