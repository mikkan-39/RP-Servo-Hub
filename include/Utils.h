#ifndef _UTILS_H
#define _UTILS_H

#include "Config.h"
#include "Protocol.h"
#include "hardware/pwm.h"
#include "pico-ssd1306/shapeRenderer/ShapeRenderer.h"
#include "pico-ssd1306/ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <cstdint>
#include <stdio.h>

using namespace pico_ssd1306;

void printHexBuf(uint8_t *buf, int len);
void sendRawBuf(uint8_t *buf, int len);
uint16_t convertFloatToUShort(float value);
int pingServos(ServoData *servoData);
void setupServos(DesiredSettings *desiredSettings, ServoLimits *servoLimits);
bool readServos(ServoData *servoData);
void pidTick(ServoData *servoData, DesiredSettings *desiredSettings,
             ControllerState *controllerStates, float *controlOutputArray);
void writeServos(float *controlOutputArray);
void setServoEnableSafe(bool enabled, DesiredSettings *desiredSettings);
int selfDiagnostics(SSD1306 *display, ServoData *servoData);

#endif