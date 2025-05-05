#include "UartUtils.h"

void sendResponse(uint8_t *src, size_t len) {
#ifdef STDIO_COMMS
  sendRawBuf(src, len);
#else
  uart_write_blocking(UART_ID, src, len);
#endif
}

uint8_t calculate_checksum(uint8_t *data, int length) {
  char crc = 0x00;
  char extract;
  char sum;
  for (int i = 0; i < length; i++) {
    extract = *data;
    for (char tempI = 8; tempI; tempI--) {
      sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum)
        crc ^= 0x8C;
      extract >>= 1;
    }
    data++;
  }
  return crc;
}

bool validate_checksum(uint8_t *data, int length) {
  uint8_t crc = calculate_checksum(data, length - 1);
  return crc == data[length - 1];
}

void send_status(uint8_t status) {
  uint8_t packet[] = {START_BYTE, status, 0x00, 0x00};
  packet[3] = calculate_checksum(packet, 3);
  sendResponse(packet, 4);
}
