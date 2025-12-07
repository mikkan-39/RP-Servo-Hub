#include "Controller.h"

constexpr float integralClamp = 1024.0f;
constexpr float maxOutput = 4000.0f;
constexpr float derivativeFilterAlpha = 0.3f;  // 0.3 = 30% new, 70% old

float computePID(int16_t actualPosition, int16_t maxLoad, int16_t current,
                 ControllerState &controller, bool wrapped) {
  float position = static_cast<float>(actualPosition);
  float error = controller.positionSetpoint - position;

  if (wrapped) {
    if (error > 2047) {
      error -= 4096;
    } else if (error < -2048) {
      error += 4096;
    }
  }

  controller.integral += error;
  controller.integral = std::clamp(controller.integral, -integralClamp, integralClamp);

  // Derivative on measurement (not error) to avoid setpoint-change spikes
  float rawDerivative = -(position - controller.previousPosition);
  
  // Handle wrap-around in derivative calculation
  if (rawDerivative > 2047) {
    rawDerivative -= 4096;
  } else if (rawDerivative < -2048) {
    rawDerivative += 4096;
  }
  
  // Low-pass filter on derivative to reduce noise amplification
  controller.filteredDerivative = derivativeFilterAlpha * rawDerivative +
                                  (1.0f - derivativeFilterAlpha) * controller.filteredDerivative;
  controller.previousPosition = position;

  float output = controller.kp * error + 
                 controller.ki * controller.integral +
                 controller.kd * controller.filteredDerivative;

  // Current limiting inside PID control
  float currentLimitFactor = 1.0f;
  if (maxLoad > 0) {
    float currentError = float(current) - float(maxLoad);
    controller.integralCurrent += currentError;
    controller.integralCurrent = std::clamp(controller.integralCurrent, 0.0f, integralClamp);
    currentLimitFactor = std::clamp(1.0f - controller.kiCurrent * controller.integralCurrent, 0.1f, 1.0f);
  }

  output *= currentLimitFactor;
  output = std::clamp(output, -maxOutput, maxOutput);
  
  return output * -1.0f;
}

int updateServoControl(int servoID, int frequency, ServoData *servoData,
                       DesiredSettings *desiredSettings, ControllerState *controllerStates) {

  // Retrieve PID state
  ControllerState &controller = controllerStates[servoID];
  int16_t targetPosition = desiredSettings[servoID].targetPosition;

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
  int16_t accel = desiredSettings[servoID].acceleration;

  // Apply hard positional limits
  if (desiredSettings[servoID].targetPosition < minPosition) {
    desiredSettings[servoID].targetPosition = minPosition;
  }
  if (desiredSettings[servoID].targetPosition > maxPosition) {
    desiredSettings[servoID].targetPosition = maxPosition;
  }

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
    int16_t delta = std::max(1, targetSpeed / frequency);
    // if (servoID == 13) {
    //   printf("targetspeed, frequency and delta for servo 13: %d %d %d\n", targetSpeed, frequency,
    //          delta);
    // }
    if (std::abs(targetPosition - controller.positionSetpoint) < delta) {
      controller.positionSetpoint = targetPosition;
    } else {
      controller.positionSetpoint +=
          (targetPosition > controller.positionSetpoint) ? delta : -delta;
    }
  } else {
    controller.positionSetpoint = targetPosition;
  }

  // if (servoID == 13) {
  //   printf("setpoint for servo 13: %d \n", controller.positionSetpoint);
  // }

  // Compute PID control with integrated current limiting
  float controlOutput =
      computePID(position, maxLoad, current, controller, servoData[servoID].wrapped);

  // TODO sloppy, implement proper accel-decel
  float clamp = 1.f;
  if (accel > 0) {
    clamp = std::min(float(accel) / 1000, 1.f);
  }

  // Store the output
  return controlOutput * clamp;
}
