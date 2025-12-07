#include "CommandParsers.h"

const char *ServoNamesById[] = {
    "NULL",            // ID 0
    "SHOULDER_MAIN_R", // ID 1
    "SHOULDER_MAIN_L", // ID 2
    "SHOULDER_TILT_R", // ID 3
    "SHOULDER_TILT_L", // ID 4
    "ELBOW_ROTATE_R",  // ID 5
    "ELBOW_ROTATE_L",  // ID 6
    "ELBOW_MAIN_R",    // ID 7
    "ELBOW_MAIN_L",    // ID 8
    "HIP_ROTATE_R",    // ID 9
    "HIP_ROTATE_L",    // ID 10
    "HIP_TILT_R",      // ID 11
    "HIP_TILT_L",      // ID 12
    "HIP_MAIN_R",      // ID 13
    "HIP_MAIN_L",      // ID 14
    "KNEE_R",          // ID 15
    "KNEE_L",          // ID 16
    "FOOT_MAIN_R",     // ID 17
    "FOOT_MAIN_L",     // ID 18
    "FOOT_TILT_R",     // ID 19
    "FOOT_TILT_L",     // ID 20
    "HEAD_HORIZONTAL", // ID 21
    "HEAD_VERTICAL",   // ID 22
};

void handlePing(ServoData *servoData) {
  int bytesPerServo = 2;
  const uint8_t payload_length = ServoNum * bytesPerServo;
  uint8_t packet[3 + payload_length + 1]; // header +
                                          // payload +
                                          // checksum
  packet[0] = START_BYTE;
  packet[1] = StatusCodes::NO_SERVOS;
  packet[2] = payload_length;

  uint8_t *payload_ptr = &packet[3];

  mutex_enter_blocking(&ServoDataMutex);
  for (uint8_t i = 0; i < ServoNum; ++i) {
    payload_ptr[i * bytesPerServo] = i + 1;
    payload_ptr[i * bytesPerServo + 1] = servoData[i + 1].statusCode;
    if (servoData[i + 1].statusCode == 0) {
      packet[1] = StatusCodes::OK;
    }
  }
  mutex_exit(&ServoDataMutex);

  packet[3 + payload_length] = calculate_checksum(packet, 3 + payload_length);

  sendResponse(packet, 3 + payload_length + 1);
}

void handleQueryServoPositions(ServoData *servoData) {
  int bytesPerServo = 3;
  const uint8_t payload_length = ServoNum * bytesPerServo;
  uint8_t packet[3 + payload_length + 1]; // header +
                                          // payload +
                                          // checksum
  packet[0] = START_BYTE;
  packet[1] = StatusCodes::OK;
  packet[2] = payload_length;

  uint8_t *payload_ptr = &packet[3];

  mutex_enter_blocking(&ServoDataMutex);
  for (uint8_t i = 0; i < ServoNum; ++i) {
    payload_ptr[i * bytesPerServo] = i + 1;
    uint8_t *fbytes = (uint8_t *)&servoData[i + 1].position;
    payload_ptr[i * bytesPerServo + 1] = fbytes[0];
    payload_ptr[i * bytesPerServo + 2] = fbytes[1];
  }
  mutex_exit(&ServoDataMutex);

  packet[3 + payload_length] = calculate_checksum(packet, 3 + payload_length);

  sendResponse(packet, 3 + payload_length + 1);
}

void handleQueryServoSpeed(ServoData *servoData) {
  int bytesPerServo = 3;
  const uint8_t payload_length = ServoNum * bytesPerServo;
  uint8_t packet[3 + payload_length + 1]; // header +
                                          // payload +
                                          // checksum
  packet[0] = START_BYTE;
  packet[1] = StatusCodes::OK;
  packet[2] = payload_length;

  uint8_t *payload_ptr = &packet[3];

  mutex_enter_blocking(&ServoDataMutex);
  for (uint8_t i = 0; i < ServoNum; ++i) {
    payload_ptr[i * bytesPerServo] = i + 1;
    uint8_t *fbytes = (uint8_t *)&servoData[i + 1].velocity;
    payload_ptr[i * bytesPerServo + 1] = fbytes[0];
    payload_ptr[i * bytesPerServo + 2] = fbytes[1];
  }
  mutex_exit(&ServoDataMutex);

  packet[3 + payload_length] = calculate_checksum(packet, 3 + payload_length);

  sendResponse(packet, 3 + payload_length + 1);
}

void handleSetServoPositions(const uint8_t *payload, uint8_t length,
                             DesiredSettings *desiredSettings, ServoData *servoData,
                             ControllerState *controllerStates) {
  size_t i = 0;
  mutex_enter_blocking(&DesiredSettingsMutex);
  while (i + 2 <= length) {
    uint8_t id = payload[i++];
    desiredSettings[id].enabled = true;
    controllerStates[id].positionSetpoint = servoData[id].position;
    int16_t value = (int16_t)(payload[i] | (payload[i + 1] << 8));
    i += 2;
    if (id > ServoNum)
      continue;
    desiredSettings[id].targetPosition = value;
  }
  mutex_exit(&DesiredSettingsMutex);
}

void handleSetServoSpeeds(const uint8_t *payload, uint8_t length,
                          DesiredSettings *desiredSettings) {
  size_t i = 0;
  mutex_enter_blocking(&DesiredSettingsMutex);
  while (i + 2 <= length) {
    uint8_t id = payload[i++];
    int16_t value = (int16_t)(payload[i] | (payload[i + 1] << 8));
    i += 2;
    if (id > ServoNum)
      continue;
    desiredSettings[id].targetSpeed = value;
  }
  mutex_exit(&DesiredSettingsMutex);
}

void handleSetServoAccelerations(const uint8_t *payload, uint8_t length,
                                 DesiredSettings *desiredSettings) {
  size_t i = 0;
  mutex_enter_blocking(&DesiredSettingsMutex);
  while (i + 2 <= length) {
    uint8_t id = payload[i++];
    int16_t value = (int16_t)(payload[i] | (payload[i + 1] << 8));
    i += 2;
    if (id > ServoNum)
      continue;
    desiredSettings[id].acceleration = value;
    desiredSettings[id].deceleration = value;
  }
  mutex_exit(&DesiredSettingsMutex);
}

void handleSetServoForwardAccelerations(const uint8_t *payload, uint8_t length,
                                        DesiredSettings *desiredSettings) {
  size_t i = 0;
  mutex_enter_blocking(&DesiredSettingsMutex);
  while (i + 2 <= length) {
    uint8_t id = payload[i++];
    int16_t value = (int16_t)(payload[i] | (payload[i + 1] << 8));
    i += 2;
    if (id > ServoNum)
      continue;
    desiredSettings[id].acceleration = value;
  }
  mutex_exit(&DesiredSettingsMutex);
}

void handleSetServoBackwardAccelerations(const uint8_t *payload, uint8_t length,
                                         DesiredSettings *desiredSettings) {
  size_t i = 0;
  mutex_enter_blocking(&DesiredSettingsMutex);
  while (i + 2 <= length) {
    uint8_t id = payload[i++];
    int16_t value = (int16_t)(payload[i] | (payload[i + 1] << 8));
    i += 2;
    if (id > ServoNum)
      continue;
    desiredSettings[id].deceleration = value;
  }
  mutex_exit(&DesiredSettingsMutex);
}

void handleSetServoEnabled(const uint8_t *payload, uint8_t length, DesiredSettings *desiredSettings,
                           ServoData *servoData, ControllerState *controllerStates) {
  size_t i = 0;
  mutex_enter_blocking(&DesiredSettingsMutex);
  while (i + 2 <= length) {
    uint8_t id = payload[i++];
    bool value = *(uint8_t *)(payload + i) ? true : false;
    i += 1;
    if (id > ServoNum)
      continue;
    desiredSettings[id].enabled = value;
    desiredSettings[id].targetPosition = servoData[id].position;
    controllerStates[id].positionSetpoint = servoData[id].position;
  }
  mutex_exit(&DesiredSettingsMutex);
}