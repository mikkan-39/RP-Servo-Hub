#include "Core1.h"

void cleanupAndExit() {
  if (multicore_lockout_start_timeout_us(1000)) {
    sleep_ms(1);
    mutex_enter_blocking(&DesiredSettingsMutex);
    for (u8 id = 1; id <= ServoNum; id++) {
      desiredSettings[id].enabled = false;
      controlOutputArray[id] = 0;
    }
    mutex_exit(&DesiredSettingsMutex);
    writeServos(controlOutputArray);
    multicore_lockout_end_blocking();
    send_status(StatusCodes::OK);
  } else {
    multicore_lockout_end_blocking();
    send_status(StatusCodes::LOCKOUT_FAIL);
  }
}

void parse_command(uint8_t command, uint8_t length, const uint8_t *payload) {
  switch (command) {
  case MainCommands::PING:
    if (multicore_lockout_start_timeout_us(1000)) {
      if (!pingServos(servoData)) {
        send_status(StatusCodes::NO_SERVOS);
        break;
      }
      setupServos(desiredSettings, servoLimits);
      handlePing(servoData);
      multicore_lockout_end_blocking();
    } else {
      multicore_lockout_end_blocking();
      send_status(StatusCodes::LOCKOUT_FAIL);
    }
    break;
  case MainCommands::EXIT:
    cleanupAndExit();
    send_status(StatusCodes::OK);
    break;
  case MainCommands::QUERY_POSITIONS:
    handleQueryServoPositions(servoData);
    break;
  case MainCommands::QUERY_SPEED:
    handleQueryServoSpeed(servoData);
    break;
  case MainCommands::SET_POS:
    handleSetServoPositions(payload, length, desiredSettings);
    send_status(StatusCodes::OK);
    break;
  case MainCommands::SET_SPEED:
    handleSetServoSpeeds(payload, length, desiredSettings);
    send_status(StatusCodes::OK);
    break;
  case MainCommands::SET_ENABLED:
    handleSetServoEnabled(payload, length, desiredSettings);
    send_status(StatusCodes::OK);
    break;
  default:
    send_status(StatusCodes::INVALID_COMMAND);
    break;
  }
}

static bool syncing = true;
void on_serial_rx(uint8_t byte) {
  static uint8_t buffer[256];
  static uint16_t pos = 0;
  static uint8_t expected_len = 0;

  if (syncing) {
    if (byte == START_BYTE) {
      pos = 0;
      buffer[pos++] = byte;
      syncing = false;
    }
    return;
  }

  buffer[pos++] = byte;

  if (pos == 2) {
    // Received PACKET_TYPE
  } else if (pos == 3) {
    // Received PACKET_LENGTH
    expected_len = buffer[2];
  } else if (pos == 3 + expected_len + 1) {
    // Full packet + checksum received
    if (validate_checksum(buffer, pos)) {
      parse_command(buffer[1], buffer[2], buffer + 3);
    } else {
      send_status(StatusCodes::WRONG_CHECKSUM);
    }
    syncing = true;
  }

  if (pos >= sizeof(buffer)) {
    send_status(StatusCodes::BUFFER_OVERFLOW);
    syncing = true;
  }
}

void core1_thread() {

#ifdef STDIO_COMMS
  while (true) {
    char uart_data = stdio_getchar_timeout_us(1000);
    if (uart_data != PICO_ERROR_TIMEOUT) {
      on_serial_rx(uart_data);
    } else {
      if (!syncing) {
        syncing = true;
        send_status(StatusCodes::INCOMPLETE_REQUEST);
      }
    }
  }
#else
  while (true) {
    if (uart_is_readable_within_us(UART_ID, 1000)) {
      char uart_data = uart_getc(UART_ID);
      on_serial_rx(uart_data);
    } else {
      if (!syncing) {
        syncing = true;
        send_status(StatusCodes::INCOMPLETE_REQUEST);
      }
    }
  }
#endif
}
