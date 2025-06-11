#include "Utils.h"

void printHexBuf(uint8_t *buf, int len) {
  for (int i = 0; i < len; i++)
    printf("0x%02X%c", buf[i], i < len - 1 ? ' ' : '\n');
}

void sendRawBuf(uint8_t *buf, int len) {
  for (int i = 0; i < len; i++)
    stdio_putchar_raw(buf[i]);
}

uint16_t convertFloatToUShort(float value) {
  // Cap the value within the range of -4096 to 4096
  value = std::clamp(value, -4096.0f, 4096.0f);

  // Determine the direction bit (bit 10)
  uint16_t directionBit = (value < 0) ? 0x0400 : 0x0000;

  // Convert value to range 0-1000
  uint16_t magnitude = static_cast<uint16_t>((std::abs(value) / 4096.0f) * 1000.0f);

  // if (magnitude < 50 && magnitude > 10)
  //   magnitude = 50;

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

  int dataLen = 2;
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
    servoData[result.id].previousPosition = servoData[result.id].position;
    servoData[result.id].position = SCS2Host(result.parameters[0], result.parameters[1]);
    servoData[result.id].velocity =
        servoData[result.id].position - servoData[result.id].previousPosition;
    // servoData[result.id].velocity =
    // SCS2Host(result.parameters[2], result.parameters[3]);
    // servoData[result.id].motorCurrent =
    //     SCS2Host(result.parameters[4],
    //     result.parameters[5]);
    //     // TODO What is this?
    mutex_exit(&ServoDataMutex);
  }

  // dataLen = 2;
  // rxPacketLen = dataLen + 6;
  // SyncRead(ConnectedIDs, ConnectedServos, ServoAddr::PRESENT_CURRENT_L, dataLen);

  // for (int i = 0; i < ConnectedServos; i++) {
  //   u8 packet[rxPacketLen];
  //   int receivedBytes = ServoReceive(packet, rxPacketLen);

  //   if (receivedBytes != rxPacketLen)
  //     continue;

  //   ServoResponse result = ServoDecode(packet, rxPacketLen);
  //   if (!result.valid)
  //     continue;

  //   mutex_enter_blocking(&ServoDataMutex);
  //   servoData[result.id].statusCode = result.status;
  //   servoData[result.id].motorCurrent = SCS2Host(result.parameters[0], result.parameters[1]);
  //   mutex_exit(&ServoDataMutex);
  // }

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

unsigned char logoBitmap[] = {
    // 64x64px
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00,
    0x00, 0x00, 0xdf, 0xff, 0xff, 0xf8, 0x3c, 0x00, 0x00, 0x00, 0xdf, 0xff, 0xff, 0xff, 0xe7, 0x00,
    0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x3b, 0x80, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x0d, 0xc0,
    0x00, 0x01, 0xa0, 0x00, 0x40, 0x00, 0x02, 0xe0, 0x00, 0x01, 0xe0, 0x00, 0x40, 0x00, 0x01, 0x60,
    0x00, 0x03, 0x40, 0x00, 0x40, 0x00, 0x00, 0xb0, 0x00, 0x06, 0xc0, 0x00, 0x40, 0x00, 0x00, 0xb0,
    0x00, 0x0e, 0x80, 0x07, 0xf8, 0x00, 0x00, 0x58, 0x00, 0x1d, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x58,
    0x00, 0x3a, 0x00, 0x01, 0x80, 0x00, 0x00, 0x78, 0x00, 0x76, 0x00, 0x03, 0x80, 0x00, 0x00, 0x28,
    0x01, 0xc8, 0x00, 0x07, 0x00, 0x00, 0x00, 0x28, 0x0f, 0x30, 0x00, 0x06, 0x00, 0x00, 0x00, 0x2c,
    0x1c, 0xe0, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x2c, 0x1b, 0x80, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x2c,
    0x1e, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x2c, 0x18, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x28,
    0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x03, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xfe, 0x00,
    0x0a, 0x00, 0x1f, 0xfe, 0xd8, 0x00, 0x00, 0x28, 0x0f, 0x00, 0x03, 0xc0, 0xc0, 0x00, 0x00, 0x78,
    0x0d, 0x00, 0x07, 0x80, 0xe0, 0x00, 0x00, 0x58, 0x0d, 0x00, 0x07, 0x01, 0xe0, 0x00, 0x00, 0x58,
    0x05, 0x80, 0x06, 0x01, 0xe0, 0x00, 0x00, 0x50, 0x06, 0x80, 0x00, 0x01, 0xe0, 0x00, 0x00, 0xb0,
    0x06, 0x80, 0x00, 0x01, 0x60, 0x00, 0x00, 0xb0, 0x03, 0xc0, 0x00, 0x03, 0x70, 0x00, 0x01, 0xa0,
    0x03, 0x40, 0xf1, 0xe2, 0x71, 0xf7, 0x81, 0x60, 0x01, 0x40, 0x71, 0xc2, 0x70, 0xe3, 0x01, 0x60,
    0x01, 0xa0, 0x73, 0x82, 0x78, 0x63, 0x02, 0xc0, 0x01, 0xa0, 0x7b, 0x86, 0x38, 0x67, 0x02, 0xc0,
    0x00, 0xd0, 0x5f, 0x87, 0xf8, 0x6b, 0x05, 0x80, 0x00, 0xd0, 0x5d, 0x84, 0x38, 0x7b, 0x05, 0x80,
    0x00, 0x68, 0x4d, 0x84, 0x38, 0x73, 0x0b, 0x00, 0x00, 0x2c, 0x4d, 0x8c, 0x3c, 0x63, 0x1e, 0x00,
    0x00, 0x34, 0x49, 0xcc, 0x3c, 0xe3, 0x16, 0x00, 0x00, 0x1a, 0xe3, 0xfe, 0x7f, 0xf7, 0xac, 0x00,
    0x00, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x58, 0x00, 0x00, 0x0d, 0x80, 0x00, 0x00, 0x00, 0xd8, 0x00,
    0x00, 0x06, 0xc0, 0x00, 0x00, 0x01, 0xb0, 0x00, 0x00, 0x03, 0x60, 0x00, 0x00, 0x03, 0x60, 0x00,
    0x00, 0x01, 0xb0, 0x00, 0x00, 0x06, 0xc0, 0x00, 0x00, 0x00, 0xd8, 0x00, 0x00, 0x0d, 0x80, 0x00,
    0x00, 0x00, 0x6c, 0x00, 0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x66, 0x00, 0x00,
    0x00, 0x00, 0x19, 0x00, 0x00, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x0e, 0xc0, 0x01, 0xb8, 0x00, 0x00,
    0x00, 0x00, 0x07, 0x70, 0x06, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xd8, 0x19, 0xc0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xe6, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0xde, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1e, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Pwm has to be initted prior to calling this
void tone(uint gpio, uint frequency) {
  gpio_set_function(gpio, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(gpio);

  float clkdiv = 1.0f;
  uint32_t sys_clk = 125000000; // Default system clock

  // Calculate wrap value
  uint32_t wrap = (sys_clk / frequency) - 1;
  if (wrap > 0xFFFF) {
    // Wrap too big for 16-bit PWM counter
    wrap = 0xFFFF;
    clkdiv = (float)sys_clk / ((wrap + 1) * frequency);
    if (clkdiv > 256.0f)
      clkdiv = 256.0f; // max divider
  }

  pwm_set_clkdiv(slice, clkdiv);
  pwm_set_wrap(slice, wrap);
  pwm_set_chan_level(slice, pwm_gpio_to_channel(gpio), wrap / 2);
  pwm_set_enabled(slice, true);
}

void noTone(uint gpio) {
  uint slice = pwm_gpio_to_slice_num(gpio);
  pwm_set_chan_level(slice, pwm_gpio_to_channel(gpio), 0);
}

int selfDiagnostics(SSD1306 *display, ServoData *servoData) {
  uart_tx_wait_blocking(SERVO_UART_ID);

  display->clear();
  display->addBitmapImage(0, 0, 64, 64, logoBitmap);
  drawText(display, font_12x16, "SELF", 72, 16);
  drawText(display, font_12x16, "TEST", 72, 40);
  display->sendBuffer();
  sleep_ms(1000);

  pwm_config cfg = pwm_get_default_config();
  uint slice_num = pwm_gpio_to_slice_num(INDICTATOR_PIN);
  pwm_init(slice_num, &cfg, true);
  uint16_t wrap = 30000;
  pwm_set_wrap(slice_num, wrap);

  // SERVO BUS TEST
  uint8_t testData[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  uart_write_blocking(SERVO_UART_ID, testData, 4);
  uart_tx_wait_blocking(SERVO_UART_ID);
  uint8_t receivedData[4];
  int receivedLen = ServoReceive(receivedData, 4);
  if (receivedLen != 4 || memcmp(testData, receivedData, 4)) {
    display->clear();
    drawText(display, font_12x16, "BUS ERR", 22, 24);
    display->sendBuffer();
    for (int i = 0; i < 5; i++) {
      pwm_set_gpio_level(INDICTATOR_PIN, wrap / 2);
      sleep_ms(100);
      pwm_set_gpio_level(INDICTATOR_PIN, 0);
      sleep_ms(10);
    }
    return 1;
  }

  while (uart_is_readable_within_us(SERVO_UART_ID, 1000)) {
  }

  pingServos(servoData);

  display->clear();
  // SERVOS TEST
  for (int id = 1; id <= ServoNum; id++) {
    int x = (((id - 1) / 6) % 4) * 32;
    int y = ((id - 1) % 6) * 11;

    char str[6]; // 2 digits + '\0'
    snprintf(str, sizeof(str), "ID:%02d", id);

    drawText(display, font_5x8, str, x, y);
    if (servoData[id].pinged) {
      fillRect(display, x, y, x + 25, y + 8, WriteMode::INVERT);
    }
  }
  display->sendBuffer();

  if (ConnectedServos == 0) {
    pwm_set_clkdiv(slice_num, 1);
    for (int i = 0; i < 3; i++) {
      pwm_set_gpio_level(INDICTATOR_PIN, wrap / 2);
      sleep_ms(100);
      pwm_set_gpio_level(INDICTATOR_PIN, 0);
      sleep_ms(10);
    }

    display->clear();
    drawText(display, font_12x16, "NO SERVOS", 10, 24);
    display->sendBuffer();

    return 0;
  }

  // ALL OK
  pwm_set_clkdiv(slice_num, 1);
  pwm_set_gpio_level(INDICTATOR_PIN, wrap / 2);
  sleep_ms(500);
  pwm_set_gpio_level(INDICTATOR_PIN, 0);
  pwm_set_clkdiv(slice_num, 1.25);
  pwm_set_gpio_level(INDICTATOR_PIN, wrap / 2);
  sleep_ms(500);
  pwm_set_gpio_level(INDICTATOR_PIN, 0);

  return 0;
}