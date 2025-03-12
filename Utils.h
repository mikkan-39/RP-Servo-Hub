#ifndef _UTILS_H
#define _UTILS_H

#include "pico/stdlib.h"
#include <algorithm>
#include <cstdint>
#include <stdio.h>

void printHexBuf(uint8_t *buf, int len);
uint16_t convertFloatToUShort(float value);

#endif