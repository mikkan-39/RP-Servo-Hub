#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <algorithm>
#include <cstdint>
#include <stdio.h>

struct ServoData {
  bool pinged = false;
  unsigned char statusCode = 255; // 255 for unknown
  int16_t position = 0;
  int16_t previousPosition = 0;
  int16_t velocity = 0;
  int16_t acceleration = 0;
  int16_t motorCurrent = 0;
  bool wrapped = false;
};

struct DesiredSettings {
  bool enabled = false;
  int16_t targetPosition = 2048;
  int16_t targetSpeed = 1000; // 0 means no speed limit
  int16_t maxLoad = 0;        // 0 means no load limit, max 500
  int16_t minPosition = 0;
  int16_t maxPosition = 4095;
};

struct ControllerState {
  // Position control PID gains
  // TODO TEST THIS
  float kp = 5.0f;
  float ki = 0.25f;
  float kd = 4.0f;
  // Integral gain for current limiting
  float kiCurrent = 0.1f;

  // Controller state
  float integral = 0.0f;
  float integralCurrent = 0.0f;
  float previousError = 0.0f;

  int16_t positionSetpoint = 2048;
};

int updateServoControl(int servoID, int frequency, ServoData *servoData,
                       DesiredSettings *desiredSettings, ControllerState *controllerStates);

#endif