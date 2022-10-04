#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
char HexToBin(char in);
char BinToHex(int in);
void HexStringToBin(const char* in, uint8_t* out);
void ArrayToHex(const uint8_t* in, int len, char* out);
void replaceChar(char* str, char find, char replace);
#ifdef __cplusplus
}
#endif