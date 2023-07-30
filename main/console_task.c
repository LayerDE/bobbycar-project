#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "c_data.h"
#include "string_tools.h"
#include "command_interpreter.h"

void bt_console_init(void *ignore){
    vTaskDelete(NULL);
}
void usb_console_init(void *ignore){
    printf("console inited\n");
    vTaskDelete(NULL);
}

void tast_bt_console(void *ignore){
    vTaskDelete(NULL);
}

void tast_usb_console(void *ignore){
    char buffer[128];
    c_data string_buffer;
    c_data buffer2;
    c_data_spawn_ptr(&buffer2);
    c_data_spawn_ptr(&string_buffer);
    while(true){
        fgets(buffer, 128, stdin);
        if(strlen(buffer)>0){
            c_data_extend_raw(&string_buffer,buffer,strlen(buffer));
            printf("%s",buffer);
            if(strchr(buffer, '\n') != NULL){
                c_data_extend_raw(&string_buffer, &endl4ptr, sizeof(endl4ptr));
                exec(string_buffer.content,&buffer2);
                if(buffer2.size != 0){
                    c_data_extend_raw(&buffer2, &endl4ptr, sizeof(endl4ptr));
                    printf("%s",(char*)buffer2.content);
                }
                c_data_set_size(&string_buffer, 0);
                c_data_set_size(&buffer2, 0);
            }
            memset(buffer,'\0',128);
        }
        vTaskDelay(1);
    }
    c_data_delete_ptr(&buffer2);
    vTaskDelete(NULL);
}