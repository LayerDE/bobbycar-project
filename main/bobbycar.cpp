#include <crc32.h>
#include "config.h"
#include "bobbycar.hpp"

#include "timecritical_function.h"

//#include <stdio.h>

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
    timecritical(board->write((uint8_t*)&Command, sizeof(SerialCommand)););
}

// ########################## RECEIVE ##########################
static inline uint8_t new_idx(uint8_t old_idx, uint8_t summand){
    return (old_idx + summand) % sizeof(SerialFeedback);
}

bool Receive(EspSoftwareSerial::UART* board, SerialFeedback* out, SerialVariables *vars, unsigned long time) {
    uint16_t bufStartFrame; // Buffer Start Frame
    // Check for new data availability in the Serial buffer
    bool data_complete = false;
    while(board->available()){
        vars->lastUpdate = time;
        vars->buffer[vars->idx = new_idx(vars->idx,1)] = board->read(); // Read the incoming byte
        bufStartFrame = ((uint16_t)(vars->buffer[new_idx(vars->idx, 2)]) << 8)
            | (uint16_t)vars->buffer[new_idx(vars->idx, 1)];  // Construct the start frame

            // Copy received data
        if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
        //printf("start frame...");
            SerialFeedback NewFeedback;
            int pack_start = new_idx(vars->idx, 1);

            memcpy(&NewFeedback, &(vars->buffer[pack_start]),sizeof(SerialFeedback)-pack_start);
            memcpy(&(((uint8_t*)&NewFeedback)[sizeof(SerialFeedback)-pack_start]), vars->buffer,pack_start);
            //for(int x = 0; x < sizeof(SerialFeedback); x++)
            //    ((uint8_t*)&NewFeedback)[x] = vars->buffer[new_idx(pack_start,x)];
            uint32_t checksum = calc_crc32((uint8_t*)&NewFeedback, sizeof(SerialFeedback) - sizeof(uint16_t)*2);
            uint32_t checksum_package = (uint32_t)NewFeedback.checksumL | ((uint32_t)NewFeedback.checksumH << 16);
            // printf("%x == %x\n",(unsigned int)checksum_package,(unsigned int)checksum);
            // Check validity of the new data
            if (checksum == checksum_package) {
                // printf("crc correct\n");
                // Copy the new data
                memcpy(out, &NewFeedback, sizeof(SerialFeedback));
                // Print data to built-in Serial
                vars->lastValidPack = time;
                data_complete = true;
            }/*else if(NewFeedback.start == START_FRAME){
                printf("crc incorrect:");
                for(int i = 0; i < sizeof(SerialFeedback); i++)
                    printf("%02X ", ((uint8_t*)(&NewFeedback))[i]);
                printf("\n");
            }
            else{printf("copy error");}*/
        }
    }
    return data_complete;
}

