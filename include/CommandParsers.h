#ifndef _COMMAND_PARSERS_H
#define _COMMAND_PARSERS_H

#include "Config.h"
#include "Protocol.h"
#include "Utils.h"
#include <iomanip> // For formatting table output
#include <iostream>
#include <sstream> // For std::ostringstream]
#include <vector>

const std::string red = "\033[31m";
const std::string yellow = "\033[33m";
const std::string green = "\033[32m";
const std::string blue = "\033[34m";
const std::string reset = "\033[0m";

void handlePing(ServoData *servoData);
void handleQueryServoPositions(ServoData *servoData);
void handleQueryServoMoving(ServoData *servoData);
void handleQueryServoSpeed(ServoData *servoData);

void handleSetServoPositions(const std::string &command, DesiredSettings *desiredSettings);
void handleSetServoSpeeds(const std::string &command, DesiredSettings *desiredSettings);
void handleSetServoTorque(const std::string &command, DesiredSettings *desiredSettings);
void handleSetServoEnabled(const std::string &command, DesiredSettings *desiredSettings);

#endif
