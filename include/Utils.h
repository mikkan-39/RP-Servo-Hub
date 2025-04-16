#ifndef _UTILS_H
#define _UTILS_H

#include "Config.h"
#include "Protocol.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <cstdint>
#include <stdio.h>

void printHexBuf(uint8_t *buf, int len);
uint16_t convertFloatToUShort(float value);
int pingServos(ServoData *servoData);
void setupServos(DesiredSettings *desiredSettings, ServoLimits *servoLimits);
bool readServos(ServoData *servoData);
void pidTick(ServoData *servoData, DesiredSettings *desiredSettings,
             ControllerState *controllerStates, float *controlOutputArray);
void writeServos(float *controlOutputArray);
void setServoEnableSafe(bool enabled, DesiredSettings *desiredSettings);

#endif