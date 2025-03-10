#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <algorithm>
#include <cstdint>

struct ServoData {
  bool pinged = false;
  unsigned char statusCode = 255; // 255 for unknown
  int16_t position = 0;
  int16_t velocity = 0;
  int16_t acceleration = 0;
  int16_t motorCurrent = 0;
};

struct DesiredSettings {
  int16_t targetPosition = 0;
  int16_t targetSpeed = 0; // 0 means no speed limit
  int16_t maxLoad = 0;     // 0 means no load limit
  int16_t minPosition = 0;
  int16_t maxPosition = 0;
};

struct ControllerState {
  // Position control PID gains
  float kp = 1.0f;
  float ki = 1.0f;
  float kd = 1.0f;
  // Integral gain for current limiting
  float kiCurrent = 0.5f;

  // Controller state
  float integral = 0.0f;
  float integralCurrent = 0.0f;
  float previousError = 0.0f;

  int16_t positionSetpoint = 0;
  // Sensor Wrapping
  int16_t positionOffset = 0;
};

void updateServoControl(int servoID, int frequency);

extern ServoData servoData[];
extern DesiredSettings desiredSettings[];
extern ControllerState controllerStates[];
extern float controlOutputArray[];

#endif