#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const int bufferLength = 256;

FILE *read;
FILE *write;


int main(void)
{
    char buffer[bufferLength]; /* not ISO 90 compatible */

    read = fopen("in.txt", "r");
    write = fopen("out.txt", "w");
    int error = 0;
    int lines = 0;
    while(fgets(buffer, bufferLength, read)) {
        lines++;
        int count = 0;
        int i = 0;
        while(buffer[i] != ':' && buffer[i] != '\n' && buffer[i]);
        if(buffer[i] == ':')
            buffer[i++] = 0;
        else
            continue;
        if(strlen(buffer)==atoi(&buffer[i])){
            fprintf(write, "%s\n", buffer);
        }else{
            error++;
        }
        printf("Error rate: %i/%i = %f\n", error,lines,(float)error/(float)lines);
        memset(buffer,'\0',bufferLength);
    }

    fclose(read);
    fclose(write);
    exit(EXIT_SUCCESS);
}