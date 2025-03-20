#include <stdio.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "Config.h"
#include "Controller.h"
#include "Protocol.h"
#include "Utils.h"

ServoData servoData[ServoNum];
DesiredSettings desiredSettings[ServoNum] = {};
ControllerState controllerStates[ServoNum] = {};
float controlOutputArray[ServoNum] = {};

bool tickCallback(repeating_timer_t *rt) {
  readServos(servoData);
  pidTick(servoData, desiredSettings, controllerStates, controlOutputArray);
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

int main() {
  stdio_init_all();
  UartInit();

  int foundServos = pingServos(servoData);

  if (!foundServos) {
    printf("No servos found. Exiting.\n");
    return 0;
  }

  printf("%d Servos pinged.\n", foundServos);

  setupServos();

  struct repeating_timer PIDtimer;

  struct repeating_timer testTimer;

  add_repeating_timer_us(1000000 / tickRate, tickCallback, NULL, &PIDtimer);

  add_repeating_timer_ms(3000, testCallback, NULL, &testTimer);

  while (true) {
  }
}
