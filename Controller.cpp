#include "Controller.h"

float computePID(int servoID, int16_t setpoint, int16_t actualPosition, int16_t maxLoad,
                 int16_t current, ControllerState &controller) {
  float error = setpoint - actualPosition;

  // For 360-control
  // // Ensure that error is within the range -2048 to +2047
  // if (error > 2047) {
  //   error -= 4096;  // Wrap around in the negative direction
  // } else if (error < -2048) {
  //   error += 4096;  // Wrap around in the positive direction
  // }

  controller.integral += error;
  float derivative = error - controller.previousError;
  controller.previousError = error;

  // Current limiting inside PID control
  if (maxLoad > 0) {
    float currentError = float(maxLoad) - float(current);
    controller.integralCurrent += currentError;
    float currentLimitFactor =
        std::max(0.1f, 1.0f - controller.kiCurrent * controller.integralCurrent);
    return (controller.kp * error + controller.ki * controller.integral +
            controller.kd * derivative) *
           currentLimitFactor;
  }

  return controller.kp * error + controller.ki * controller.integral + controller.kd * derivative;
}

void updateServoControl(int servoID, int frequency) {
  // Retrieve sensor data
  int16_t position = servoData[servoID].position;
  int16_t velocity = servoData[servoID].velocity;
  int16_t acceleration = servoData[servoID].acceleration;
  int16_t current = servoData[servoID].motorCurrent;

  // Retrieve desired settings
  int16_t targetPosition = desiredSettings[servoID].targetPosition;
  int16_t targetSpeed = desiredSettings[servoID].targetSpeed;
  int16_t maxLoad = desiredSettings[servoID].maxLoad;
  int16_t minPosition = desiredSettings[servoID].minPosition;
  int16_t maxPosition = desiredSettings[servoID].maxPosition;

  // Retrieve PID state
  ControllerState &controller = controllerStates[servoID];

  // Estimate previous position
  int16_t previousPosition = position - (velocity / frequency);

  // Detect and handle position overflow/underflow
  if ((previousPosition > maxPosition && position < minPosition) ||
      (previousPosition < minPosition && position > maxPosition)) {
    // If we crossed a limit due to wrapping, set positionSetpoint to the nearest limit
    controlOutputArray[servoID] = 0;
    return;
  }

  // Determine position setpoint accounting for speed
  if (targetSpeed > 0) {
    int16_t delta = targetSpeed / frequency;
    if (std::abs(targetPosition - controller.positionSetpoint) < delta) {
      controller.positionSetpoint = targetPosition;
    } else {
      controller.positionSetpoint +=
          (targetPosition > controller.positionSetpoint) ? delta : -delta;
    }
  } else {
    controller.positionSetpoint = targetPosition;
  }

  // Apply hard positional limits
  if (controller.positionSetpoint < minPosition)
    controller.positionSetpoint = minPosition;
  if (controller.positionSetpoint > maxPosition)
    controller.positionSetpoint = maxPosition;

  // Compute PID control with integrated current limiting
  float controlOutput =
      computePID(servoID, controller.positionSetpoint, position, maxLoad, current, controller);

  // Store the output
  controlOutputArray[servoID] = controlOutput;
}
