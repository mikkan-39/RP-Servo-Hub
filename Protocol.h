#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>

typedef unsigned char u8;
typedef unsigned short u16;

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 1000000

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for
// information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define BROADCAST_ID 0xFE;

enum ServoInstructions {
  PING = 0x01,       // Query the working status
  READ = 0x02,       // Query data in the Control Table
  WRITE = 0x03,      // Write data into the Control Table
  REG_WRITE = 0x04,  // Similar to WRITE DATA, the servo does not act until the ACTION instruction
  REG_ACTION = 0x05, // Action that applies REG WRITE writes
  SYNC_READ = 0x82,  // Servos respond one by one
  SYNC_WRITE = 0x83, // For simultaneous control of multiple servos
  RESET = 0x06       // Reset control table to factory value
};

enum ServoAddr {
  //-------EEPROM(read only)--------
  MODEL_L = 3,
  MODEL_H = 4,
  //-------EEPROM(read & write)-----
  ID = 5,
  BAUDRATE = 6,
  MIN_ANGLE_LIMIT_L = 9,
  MIN_ANGLE_LIMIT_H = 10,
  MAX_ANGLE_LIMIT_L = 11,
  MAX_ANGLE_LIMIT_H = 12,
  CW_DEAD = 26,
  CCW_DEAD = 27,
  OFS_L = 31,
  OFS_H = 32,
  MODE = 33,
  //-------SRAM(read & write)-------
  TORQUE_ENABLE = 40,
  ACC = 41,
  GOAL_POSITION_L = 42,
  GOAL_POSITION_H = 43,
  GOAL_TIME_L = 44,
  GOAL_TIME_H = 45,
  GOAL_SPEED_L = 46,
  GOAL_SPEED_H = 47,
  TORQUE_LIMIT_L = 48,
  TORQUE_LIMIT_H = 49,
  LOCK = 55,
  //-------SRAM(read only)----------
  PRESENT_POSITION_L = 56,
  PRESENT_POSITION_H = 57,
  PRESENT_SPEED_L = 58,
  PRESENT_SPEED_H = 59,
  PRESENT_LOAD_L = 60,
  PRESENT_LOAD_H = 61,
  PRESENT_VOLTAGE = 62,
  PRESENT_TEMPERATURE = 63,
  MOVING = 66,
  PRESENT_CURRENT_L = 69,
  PRESENT_CURRENT_H = 70,
};

struct ServoResponse {
  uint8_t id;
  uint8_t length;
  uint8_t status;
  uint8_t parameters[16];
  uint8_t checksum;
  bool valid;
};

void UartInit();
void HalfDuplexWrite(u8 *data, int length);
void HalfDuplexWrite(u8 data);
void Host2SCS(u8 *DataL, u8 *DataH, u16 Data);
u16 SCS2Host(u8 DataL, u8 DataH);
int ServoReceive(u8 *data, int len);
void ServoWriteBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
void SyncRead(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen);
ServoResponse ServoDecode(uint8_t *data, uint8_t dataLength);

#endif
