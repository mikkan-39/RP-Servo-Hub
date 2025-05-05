#include "Utils.h"

void printHexBuf(uint8_t *buf, int len) {
  for (int i = 0; i < len; i++)
    printf("0x%02X%c", buf[i], i < len - 1 ? ' ' : '\n');
}

uint16_t convertFloatToUShort(float value) {
  // Cap the value within the range of -4096 to 4096
  value = std::clamp(value, -4096.0f, 4096.0f);

  // Determine the direction bit (bit 10)
  uint16_t directionBit = (value < 0) ? 0x0400 : 0x0000;

  // Convert value to range 0-1000
  uint16_t magnitude = static_cast<uint16_t>((std::abs(value) / 4096.0f) * 1000.0f);

  if (magnitude < 50 && magnitude > 10)
    magnitude = 50;

  // Ensure magnitude is within bounds
  magnitude = std::min(magnitude, static_cast<uint16_t>(1000));

  // Combine magnitude with direction bit
  return magnitude | directionBit;
}

// Amount of servos, which responded to ping correctly
int ConnectedServos = 0;
u8 ConnectedIDs[ServoNum];

int pingServos(ServoData *servoData) {
  ConnectedServos = 0;

  for (int id = 1; id <= ServoNum; id++) {
    ServoWriteBuf(id, ServoInstruction::PING, 0, 0, 0);
    int responselen = 6;
    u8 response[responselen];

    int receivedBytes = ServoReceive(response, responselen);

    if (receivedBytes == 0)
      continue;

    ServoResponse result = ServoDecode(response, responselen);
    if (result.valid) {
      mutex_enter_blocking(&ServoDataMutex);
      ConnectedIDs[ConnectedServos] = id;
      ConnectedServos++;
      servoData[id].pinged = true;
      servoData[id].statusCode = result.status;
      mutex_exit(&ServoDataMutex);
    }
  }

  return ConnectedServos;
}

void setupServos(DesiredSettings *desiredSettings, ServoLimits *servoLimits) {
  mutex_enter_blocking(&DesiredSettingsMutex);
  for (int i = 0; i <= ServoNum; i++) {
    desiredSettings[i].minPosition = servoLimits[i].lowerAngle;
    desiredSettings[i].maxPosition = servoLimits[i].upperAngle;
  }
  mutex_exit(&DesiredSettingsMutex);
  u8 modes[ConnectedServos];
  std::fill(modes, modes + ConnectedServos, 0x02);
  SyncWriteU8(ConnectedIDs, ConnectedServos, modes, ServoAddr::MODE);
}

bool readServos(ServoData *servoData) {
  if (ConnectedServos == 0 && pingServos(servoData) == 0)
    return false;

  int dataLen = 4;
  int rxPacketLen = dataLen + 6;
  SyncRead(ConnectedIDs, ConnectedServos, ServoAddr::PRESENT_POSITION_L, dataLen);

  for (int i = 0; i < ConnectedServos; i++) {
    u8 packet[rxPacketLen];
    int receivedBytes = ServoReceive(packet, rxPacketLen);

    if (receivedBytes != rxPacketLen)
      continue;

    ServoResponse result = ServoDecode(packet, rxPacketLen);
    if (!result.valid)
      continue;

    mutex_enter_blocking(&ServoDataMutex);
    servoData[result.id].statusCode = result.status;
    servoData[result.id].position = SCS2Host(result.parameters[0], result.parameters[1]);
    servoData[result.id].velocity = SCS2Host(result.parameters[2], result.parameters[3]);
    servoData[result.id].motorCurrent =
        SCS2Host(result.parameters[4], result.parameters[5]); // TODO What is this?
    mutex_exit(&ServoDataMutex);
  }

  dataLen = 2;
  rxPacketLen = dataLen + 6;
  SyncRead(ConnectedIDs, ConnectedServos, ServoAddr::PRESENT_CURRENT_L, dataLen);

  for (int i = 0; i < ConnectedServos; i++) {
    u8 packet[rxPacketLen];
    int receivedBytes = ServoReceive(packet, rxPacketLen);

    if (receivedBytes != rxPacketLen)
      continue;

    ServoResponse result = ServoDecode(packet, rxPacketLen);
    if (!result.valid)
      continue;

    mutex_enter_blocking(&ServoDataMutex);
    servoData[result.id].statusCode = result.status;
    servoData[result.id].motorCurrent = SCS2Host(result.parameters[0], result.parameters[1]);
    mutex_exit(&ServoDataMutex);
  }

  return true;
}

void pidTick(ServoData *servoData, DesiredSettings *desiredSettings,
             ControllerState *controllerStates, float *controlOutputArray) {
  mutex_enter_blocking(&DesiredSettingsMutex);
  for (int i = 0; i < ConnectedServos; i++) {
    int id = ConnectedIDs[i];
    if (servoData[id].statusCode == 0 && desiredSettings[id].enabled) {
      controlOutputArray[id] =
          updateServoControl(id, tickRate, servoData, desiredSettings, controllerStates);
    } else {
      controlOutputArray[id] = 0;
    }
  }
  mutex_exit(&DesiredSettingsMutex);
}

void writeServos(float *controlOutputArray) {
  u16 pwmValues[ConnectedServos];
  for (int i = 0; i < ConnectedServos; i++) {
    int id = ConnectedIDs[i];
    u16 pwm = convertFloatToUShort(controlOutputArray[id]);
    pwmValues[i] = pwm;
  }
  SyncWriteU16(ConnectedIDs, ConnectedServos, pwmValues, ServoAddr::GOAL_TIME_L); // TODO
  // u8 buff[2];
  // Host2SCS(buff + 0, buff + 1, pwmValues[0]);
  // ServoWriteBuf(1, ServoInstruction::WRITE, ServoAddr::GOAL_TIME_L, buff, 2);
}

void setServoEnableSafe(int id, bool enabled, DesiredSettings *desiredSettings,
                        ControllerState *controllerStates) {
  mutex_enter_blocking(&DesiredSettingsMutex);
  if (enabled) {
    desiredSettings[id].enabled = true;
    ControllerState newState;
    controllerStates[id] = newState;
  } else {
    desiredSettings[id].enabled = false;
  }
  mutex_exit(&DesiredSettingsMutex);
}