#include "Utils.h"

void printHexBuf(uint8_t *buf, int len) {
  for (int i = 0; i < len; i++)
    printf("0x%02X%c", buf[i], i < len - 1 ? ' ' : '\n');
}