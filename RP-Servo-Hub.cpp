#include <stdio.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "CommandParsers.h"
#include "Config.h"
#include "Controller.h"
#include "Core1.h"
#include "Protocol.h"
#include "UartUtils.h"
#include "Utils.h"
#include "hardware/spi.h"
#include "pico-ssd1306/ssd1306.h"

mutex_t ServoDataMutex;
mutex_t DesiredSettingsMutex;
mutex_t setupMutex;

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
  (void)rt;
  if (!mutex_try_enter(&setupMutex, 0)) {
    // Other core doing more important stuff, i.e.
    // pinging, setting up or shutting down servos
    return true;
  }
  bool readSuccess = readServos(servoData);
  if (!readSuccess) {
    mutex_exit(&setupMutex);
    return true;
  }
  pidTick(servoData, desiredSettings, controllerStates, controlOutputArray);
  writeServos(controlOutputArray);
  mutex_exit(&setupMutex);
  return true;
}

int main() {
  stdio_init_all();
  mutex_init(&ServoDataMutex);
  mutex_init(&DesiredSettingsMutex);
  mutex_init(&setupMutex);

  gpio_init(INDICTATOR_PIN);
  gpio_set_function(INDICTATOR_PIN, GPIO_FUNC_PWM);

  gpio_init(2);
  gpio_init(3);
  gpio_init(4);
  gpio_init(5);
  gpio_init(6);
  gpio_set_function(2, GPIO_FUNC_SPI);
  gpio_set_function(3, GPIO_FUNC_SPI);
  gpio_set_dir(4, GPIO_OUT);
  gpio_set_dir(5, GPIO_OUT);
  gpio_set_dir(6, GPIO_OUT);

  spi_init(spi0, (int)2e6);

  gpio_put(6, false);
  gpio_put(4, true);

  sleep_ms(250);

  SSD1306 display = SSD1306(Size::W128xH64);
  display.setOrientation(1);

  UartInit();
  calculateServoLimits();

  // // For setting up new servos
  // // Unlock
  // uint8_t addr[10] = {0};
  // ServoWriteBuf(1, ServoInstruction::WRITE, ServoAddr::LOCK, addr, 1);
  // ServoReceive(addr, 10);
  // // Write new ID
  // addr[0] = 13;
  // ServoWriteBuf(1, ServoInstruction::WRITE, ServoAddr::ID, addr, 1);
  // ServoReceive(addr, 10);

  pingServos(servoData);

  int diagnosticsCode = selfDiagnostics(&display, servoData);

  if (diagnosticsCode != 0) {
    return 0;
  }

  setupServos(desiredSettings, servoLimits);

  struct repeating_timer PIDtimer;

  add_repeating_timer_us(-1000000 / tickRate, tickCallback, NULL, &PIDtimer);

  multicore_launch_core1(core1_thread);

  while (true) {
    tight_loop_contents();
  };

  return 0;
}
