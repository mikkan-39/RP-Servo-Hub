#ifndef _COMMAND_PARSERS_H
#define _COMMAND_PARSERS_H

#include "Config.h"
#include "Protocol.h"
#include "UartUtils.h"
#include "Utils.h"
#include <vector>

void handlePing(ServoData *servoData);
void handleQueryServoPositions(ServoData *servoData);
void handleQueryServoSpeed(ServoData *servoData);

void handleSetServoPositions(const uint8_t *payload, uint8_t length,
                             DesiredSettings *desiredSettings, ServoData *servoData,
                             ControllerState *controllerStates);

void handleSetServoSpeeds(const uint8_t *payload, uint8_t length, DesiredSettings *desiredSettings);
void handleSetServoEnabled(const uint8_t *payload, uint8_t length, DesiredSettings *desiredSettings,
                           ServoData *servoData, ControllerState *controllerStates);
void handleSetServoAccelerations(const uint8_t *payload, uint8_t length,
                                 DesiredSettings *desiredSettings);
void handleSetServoForwardAccelerations(const uint8_t *payload, uint8_t length,
                                        DesiredSettings *desiredSettings);
void handleSetServoBackwardAccelerations(const uint8_t *payload, uint8_t length,
                                         DesiredSettings *desiredSettings);

#endif
