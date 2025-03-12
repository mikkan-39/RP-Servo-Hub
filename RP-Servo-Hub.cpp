#include <stdio.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "Controller.h"
#include "Protocol.h"
#include "Utils.h"

constexpr int ServoNum = 10;
constexpr int tickRate = 100;

int ConnectedServos = 0;
u8 ConnectedIDs[ServoNum];
ServoData servoData[ServoNum] = {};
DesiredSettings desiredSettings[ServoNum] = {};
ControllerState controllerStates[ServoNum] = {};
float controlOutputArray[ServoNum] = {};

void pingServos() {
  for (int id = 1; id < ServoNum; id++) {
    ServoWriteBuf(id, PING, 0, 0, 0);
    int responselen = 6;
    u8 response[responselen];

    int receivedBytes = ServoReceive(response, responselen);

    if (receivedBytes == 0)
      continue;

    ServoResponse result = ServoDecode(response, responselen);
    if (result.valid) {
      ConnectedIDs[ConnectedServos] = id;
      ConnectedServos++;
      servoData[id].pinged = true;
      servoData[id].statusCode = result.status;
    }
  }
}

void setupServos() {
  u8 modes[ConnectedServos];
  std::fill(modes, modes + ConnectedServos, 0x02);
  SyncWriteU8(ConnectedIDs, ConnectedServos, modes, MODE);
}

void readServos() {
  int dataLen = 4;
  int rxPacketLen = dataLen + 6;
  SyncRead(ConnectedIDs, ConnectedServos, PRESENT_POSITION_L, dataLen);

  for (int i = 0; i < ConnectedServos; i++) {
    u8 packet[rxPacketLen];
    int receivedBytes = ServoReceive(packet, rxPacketLen);

    if (receivedBytes != rxPacketLen)
      continue;

    ServoResponse result = ServoDecode(packet, rxPacketLen);
    if (!result.valid)
      continue;

    servoData[result.id].statusCode = result.status;
    servoData[result.id].position = SCS2Host(result.parameters[0], result.parameters[1]);
    servoData[result.id].velocity = SCS2Host(result.parameters[2], result.parameters[3]);
    servoData[result.id].motorCurrent = SCS2Host(result.parameters[4], result.parameters[5]);
  }

  dataLen = 2;
  rxPacketLen = dataLen + 6;
  SyncRead(ConnectedIDs, ConnectedServos, PRESENT_CURRENT_L, dataLen);

  for (int i = 0; i < ConnectedServos; i++) {
    u8 packet[rxPacketLen];
    int receivedBytes = ServoReceive(packet, rxPacketLen);

    if (receivedBytes != rxPacketLen)
      continue;

    ServoResponse result = ServoDecode(packet, rxPacketLen);
    if (!result.valid)
      continue;

    servoData[result.id].statusCode = result.status;
    servoData[result.id].motorCurrent = SCS2Host(result.parameters[0], result.parameters[1]);
  }
}

void pidTick() {
  u16 pwmValues[ConnectedServos];

  for (int i = 0; i < ConnectedServos; i++) {
    int id = ConnectedIDs[i];
    if (servoData[id].statusCode == 0)
      updateServoControl(id, tickRate);
    u16 pwm = convertFloatToUShort(controlOutputArray[id]);
    pwmValues[i] = pwm;
  }

  // SyncWriteU16(ConnectedIDs, ConnectedServos, pwmValues, GOAL_TIME_L);
  u8 buff[2];
  Host2SCS(buff + 0, buff + 1, pwmValues[0]);
  ServoWriteBuf(1, WRITE, GOAL_TIME_L, buff, 2);
}

bool tickCallback(repeating_timer_t *rt) {
  readServos();
  pidTick();
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

  pingServos();

  if (!ConnectedServos) {
    printf("No servos found. Exiting.\n");
    return 0;
  }

  printf("%d Servos pinged.\n", ConnectedServos);

  setupServos();

  struct repeating_timer PIDtimer;

  struct repeating_timer testTimer;

  add_repeating_timer_ms(10, tickCallback, NULL, &PIDtimer);

  add_repeating_timer_ms(3000, testCallback, NULL, &testTimer);

  while (true) {
  }
}
