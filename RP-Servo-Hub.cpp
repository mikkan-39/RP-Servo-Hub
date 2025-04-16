#include <stdio.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <iostream>

#include "CommandParsers.h"
#include "Config.h"
#include "Controller.h"
#include "Protocol.h"
#include "Utils.h"

// +1 since servos are 1-indexed
ServoData servoData[ServoNum + 1];
DesiredSettings desiredSettings[ServoNum + 1] = {};
ControllerState controllerStates[ServoNum + 1] = {};
float controlOutputArray[ServoNum + 1] = {};

ServoLimits servoLimits[ServoNum + 1] = {};

void calculateServoLimits() {
  for (int i = 0; i <= ServoNum; ++i) {
    servoLimits[i].lowerAngle = 2048 + deviations[i][0];
    servoLimits[i].upperAngle = 2048 + deviations[i][1];
  }
}

bool tickCallback(repeating_timer_t *rt) {
  bool readSuccess = readServos(servoData);
  if (!readSuccess) {
    printf("WARNING: No servos found!\n");
    return true;
  }
  pidTick(servoData, desiredSettings, controllerStates, controlOutputArray);
  writeServos(controlOutputArray);
  return true;
}

bool testCallback(repeating_timer_t *rt) {
  if (desiredSettings[1].targetPosition == 3072) {
    desiredSettings[1].targetPosition = 1024;
    desiredSettings[1].targetSpeed = 2048;
  } else {
    desiredSettings[1].targetPosition = 3072;
    desiredSettings[1].targetSpeed = 0;
  }
  return true;
}

void cleanupAndExit() {
  for (u8 id = 1; id <= ServoNum; id++) {
    desiredSettings[id].enabled = false;
    controlOutputArray[id] = 0;
  }

  writeServos(controlOutputArray);
}

int main() {
  stdio_init_all();
  UartInit();
  calculateServoLimits();

  int foundServos = pingServos(servoData);

  setupServos(desiredSettings, servoLimits);

  struct repeating_timer PIDtimer;

  struct repeating_timer testTimer;

  add_repeating_timer_us(1000000 / tickRate, tickCallback, NULL, &PIDtimer);

  // add_repeating_timer_ms(3000, testCallback, NULL, &testTimer);

  std::cout << "INIT: Backbone subsystem started." << std::endl;

  std::string command;

  while (true) {
    std::getline(std::cin, command);

    if (command.empty()) {
      continue;
    }

    if (command == "PING") {
      pingServos(servoData);
      setupServos(desiredSettings, servoLimits);
      handlePing(servoData);
    } else if (command == "SERVOS_QUERY_POSITIONS") {
      handleQueryServoPositions(servoData);
    } else if (command == "SERVOS_QUERY_MOVING") {
      handleQueryServoMoving(servoData);
    } else if (command == "SERVOS_QUERY_SPEED") {
      handleQueryServoSpeed(servoData);
    } else if (command.rfind("SET_SERVO_POS", 0) == 0) {
      handleSetServoPositions(command, desiredSettings);
    } else if (command.rfind("SET_SERVO_SPEED", 0) == 0) {
      handleSetServoSpeeds(command, desiredSettings);
    } else if (command.rfind("SET_SERVO_TORQUE", 0) == 0) {
      handleSetServoTorque(command, desiredSettings);
    } else if (command.rfind("SET_SERVO_ENABLED", 0) == 0) {
      handleSetServoEnabled(command, desiredSettings);
    } else if (command == "EXIT") {
      cleanupAndExit();
    } else {
      std::cout << "ERROR: Unknown command to Backbone: " << command << std::endl;
      pingServos(servoData);
      setupServos(desiredSettings, servoLimits);
      handlePing(servoData);
    }
  }

  return 0;
}
