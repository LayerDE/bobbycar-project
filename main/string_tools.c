#include <string.h>
#include <stdint.h>

uint8_t HexToBin(char in) {
    switch (in) {
    case '0':
        return 0;
    case '1':
        return 1;
    case '2':
        return 2;
    case '3':
        return 3;
    case '4':
        return 4;
    case '5':
        return 5;
    case '6':
        return 6;
    case '7':
        return 7;
    case '8':
        return 8;
    case '9':
        return 9;
    case 'a':
        return 10;
    case 'A':
        return 10;
    case 'b':
        return 11;
    case 'B':
        return 11;
    case 'c':
        return 12;
    case 'C':
        return 12;
    case 'd':
        return 13;
    case 'D':
        return 13;
    case 'e':
        return 14;
    case 'E':
        return 14;
    case 'f':
        return 15;
    case 'F':
        return 15;
    default:
        return -1;
    }
}

char BinToHex(uint8_t in) {
    switch (in & 0xf) {
    case 0:
        return '0';
    case 1:
        return '1';
    case 2:
        return '2';
    case 3:
        return '3';
    case 4:
        return '4';
    case 5:
        return '5';
    case 6:
        return '6';
    case 7:
        return '7';
    case 8:
        return '8';
    case 9:
        return '9';
    case 10:
        return 'A';
    case 11:
        return 'B';
    case 12:
        return 'C';
    case 13:
        return 'D';
    case 14:
        return 'E';
    case 15:
        return 'F';
    default:
        return '?';
    }
}

void replaceChar(char* str, char find, char replace){
    char *current_pos = strchr(str,find);
    while (current_pos) {
        *current_pos = replace;
        current_pos = strchr(current_pos,find);
    }
}

void HexStringToBin(const char* in, uint8_t* out) {
    const char* tmp = in;
    for (int i = 0; i < strlen(in) / 2; i++) {
        out[i] = HexToBin(in[2 * i]) << 4;
        out[i] += HexToBin(in[2 * i + 1]);
    }
    if (strlen(in) % 2)
        out[strlen(in) / 2 + 1] = tmp[strlen(in) - 1] << 4;
}

void ArrayToHex(const uint8_t* in, int len, char* out) {
    int y = 0;
    for (int x = 0; x < len; x++) {
        out[y++] = BinToHex(in[x] >> 4);
        out[y++] = BinToHex(in[x]);
    }
    out[y] = '\0';
}