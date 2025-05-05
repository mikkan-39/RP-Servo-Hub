#ifndef _UARTUTILS_H
#define _UARTUTILS_H

#include "Config.h"
#include "Utils.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define START_BYTE 0xAA
enum MainCommands : uint8_t {
  PING = 0x01,
  QUERY_POSITIONS = 0x02,
  QUERY_SPEED = 0x03,
  SET_POS = 0x04,
  SET_SPEED = 0x05,
  SET_ENABLED = 0x06,
  EXIT = 0x07,
};

enum StatusCodes : uint8_t {
  OK = 0x00,
  INVALID_COMMAND = 0x01,
  INVALID_ARG = 0x02,
  WRONG_CHECKSUM = 0x03,
  INCOMPLETE_REQUEST = 0x04,
  BUFFER_OVERFLOW = 0x05,
  NO_SERVOS = 0x06,
  LOCKOUT_FAIL = 0x07,
};

void sendResponse(uint8_t *src, size_t len);
uint8_t calculate_checksum(uint8_t *data, int length);
bool validate_checksum(uint8_t *data, int length);

void send_status(uint8_t status);

#endif