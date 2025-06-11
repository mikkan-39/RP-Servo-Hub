#include "Controller.h"

constexpr float integralClamp = 1024.0f;

float computePID(int16_t actualPosition, int16_t maxLoad, int16_t current,
                 ControllerState &controller, bool wrapped) {
  float error = controller.positionSetpoint - actualPosition;

  // printf("Setpoint %d\n", setpoint);
  // printf("Position %d\n", actualPosition);

  if (wrapped) {
    if (error > 2047) {
      error -= 4096; // Wrap around in the negative direction
    } else if (error < -2048) {
      error += 4096; // Wrap around in the positive direction
    }
  }

  controller.integral += error;
  controller.integral = std::clamp(controller.integral, -integralClamp, integralClamp);
  float derivative = error - controller.previousError;
  controller.previousError = error;

  // printf("P %f\n", error * controller.kp);
  // printf("I %f\n", controller.integral * controller.ki);
  // printf("D %f\n", derivative * controller.kd);

  // Current limiting inside PID control
  if (maxLoad > 0) {
    float currentError = float(current) - float(maxLoad);
    controller.integralCurrent += currentError;
    controller.integralCurrent = std::clamp(controller.integralCurrent, 0.0f, integralClamp);
    float currentLimitFactor =
        std::clamp(1.0f - controller.kiCurrent * controller.integralCurrent, 0.1f, 1.0f);
    // printf("PresentCurrent %d\n", current);
    // printf("CurrentError %f\n", currentError);
    // printf("CurrentLimit %f\n", currentLimitFactor);
    return (controller.kp * error + controller.ki * controller.integral +
            controller.kd * derivative) *
           currentLimitFactor * -1.0f;
  }

  return (controller.kp * error + controller.ki * controller.integral +
          controller.kd * derivative) *
         -1.0f;
}

int updateServoControl(int servoID, int frequency, ServoData *servoData,
                       DesiredSettings *desiredSettings, ControllerState *controllerStates) {
  // Retrieve sensor data
  int16_t position = servoData[servoID].position;
  int16_t previousPosition = servoData[servoID].previousPosition;
  // int16_t velocity = servoData[servoID].velocity;
  // int16_t acceleration = servoData[servoID].acceleration;
  int16_t current = servoData[servoID].motorCurrent;

  // Retrieve desired settings
  int16_t targetSpeed = desiredSettings[servoID].targetSpeed;
  int16_t maxLoad = desiredSettings[servoID].maxLoad;
  int16_t minPosition = desiredSettings[servoID].minPosition;
  int16_t maxPosition = desiredSettings[servoID].maxPosition;

  // Apply hard positional limits
  if (desiredSettings[servoID].targetPosition < minPosition) {
    desiredSettings[servoID].targetPosition = minPosition;
  }
  if (desiredSettings[servoID].targetPosition > maxPosition) {
    desiredSettings[servoID].targetPosition = maxPosition;
  }
  int16_t targetPosition = desiredSettings[servoID].targetPosition;

  // printf("\n\nTarget position %d\n", targetPosition);
  // Retrieve PID state
  ControllerState &controller = controllerStates[servoID];

  // Estimate previous position
  // int16_t previousPosition = position - (velocity / frequency);

  // Detect and handle position overflow/underflow
  // if ((previousPosition > maxPosition && position < minPosition) ||
  //     (previousPosition < minPosition && position > maxPosition)) {
  //   // If we crossed a limit due to wrapping, set positionSetpoint to the nearest limit
  //   return 0; // HACK
  // }

  if (abs(previousPosition - position) > 2048) {
    servoData[servoID].wrapped = !servoData[servoID].wrapped;
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

  // Compute PID control with integrated current limiting
  float controlOutput =
      computePID(position, maxLoad, current, controller, servoData[servoID].wrapped);

  // Store the output
  return controlOutput;
}
