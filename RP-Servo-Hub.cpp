#include <stdio.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "Controller.h"
#include "Protocol.h"
#include "Utils.h"

constexpr int ServoNum = 10;

int ConnectedServos = 0;
u8 ConnectedIDs[ServoNum];
ServoData servoData[ServoNum] = {};
DesiredSettings desiredSettings[ServoNum] = {};
ControllerState controllerStates[ServoNum] = {};
float controlOutputArray[ServoNum] = {};

int main() {
  stdio_init_all();
  UartInit();

  int ConnectedServos = pingServos();

  if (!ConnectedServos) {
    printf("No servos found. Exiting.\n");
    return;
  }

  printf("%d Servos pinged.", ConnectedServos);

  readServos();
}

int pingServos() {
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

void readServos() {
  int dataLen = 6;
  int rxPacketLen = dataLen + 5;
  SyncRead(ConnectedIDs, ConnectedServos, PRESENT_POSITION_L, dataLen);

  for (int i = 0; i < ConnectedServos; i++) {
    u8 packet[rxPacketLen];
    int receivedBytes = ServoReceive(packet, rxPacketLen);

    if (receivedBytes != rxPacketLen)
      continue;

    ServoResponse result = ServoDecode(packet, rxPacketLen);
    if (!result.valid)
      continue;

    servoData[result.id].position = SCS2Host(result.parameters[0], result.parameters[1]);
    servoData[result.id].velocity = SCS2Host(result.parameters[2], result.parameters[3]);
    servoData[result.id].motorCurrent = SCS2Host(result.parameters[4], result.parameters[5]);
  }
}