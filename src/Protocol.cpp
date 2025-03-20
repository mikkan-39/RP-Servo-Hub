#include "Protocol.h"

void UartInit() {
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void HalfDuplexWrite(u8 *data, int length) {
  hw_clear_bits(&uart_get_hw(UART_ID)->cr, 0x00000200);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_NULL);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

  uart_write_blocking(UART_ID, data, length);
  uart_tx_wait_blocking(UART_ID);

  gpio_set_function(UART_TX_PIN, GPIO_FUNC_NULL);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  hw_set_bits(&uart_get_hw(UART_ID)->cr, 0x00000200);

  // Dirty hack because RX disabling does not work
  while (uart_is_readable(UART_ID)) {
    uart_getc(UART_ID);
  }
}

void HalfDuplexWrite(u8 data) {
  u8 buf[1] = {data};
  HalfDuplexWrite(buf, 1);
}

void Host2SCS(u8 *DataL, u8 *DataH, u16 Data) {
  *DataH = (Data >> 8);
  *DataL = (Data & 0xff);
}

u16 SCS2Host(u8 DataL, u8 DataH) {
  u16 Data;
  Data = DataH;
  Data <<= 8;
  Data |= DataL;
  return Data;
}

// Returns the number of received bytes
int ServoReceive(u8 *data, int len) {
  int i = 0;
  int ComData;
  while (i < len) {
    if (uart_is_readable_within_us(UART_ID, 100)) {
      ComData = uart_getc(UART_ID);
      if (data) {
        data[i] = ComData;
      }
      i++;
    } else {
      break;
    }
  }

  return i;
}

void ServoWriteBuf(u8 ID, u8 Function, u8 MemAddr, u8 *nDat, u8 nLen) {
  u8 msgLen = nDat ? (nLen + 3) : 2;
  u8 CheckSum = ID + msgLen + Function + (nDat ? MemAddr : 0);

  u8 buff[6 + nLen];
  buff[0] = 0xff;
  buff[1] = 0xff;
  buff[2] = ID;
  buff[3] = msgLen;
  buff[4] = Function;

  u8 index = 5;
  if (nDat) {
    buff[index++] = MemAddr;
    for (u8 i = 0; i < nLen; i++) {
      buff[index++] = nDat[i];
      CheckSum += nDat[i];
    }
  }

  buff[index] = ~CheckSum;

  HalfDuplexWrite(buff, index + 1);
}

void SyncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen) {
  u8 mesLen = ((nLen + 1) * IDN + 4);
  u8 checkSum = BROADCAST_ID + mesLen + ServoInstruction::SYNC_WRITE + MemAddr + nLen;
  u8 buff[7 + (IDN * (nLen + 1)) + 1];

  buff[0] = 0xFF;
  buff[1] = 0xFF;
  buff[2] = BROADCAST_ID;
  buff[3] = mesLen;
  buff[4] = ServoInstruction::SYNC_WRITE;
  buff[5] = MemAddr;
  buff[6] = nLen;

  u8 index = 7;
  for (u8 i = 0; i < IDN; i++) {
    buff[index++] = ID[i];
    checkSum += ID[i];
    for (u8 j = 0; j < nLen; j++) {
      buff[index++] = nDat[i * nLen + j];
      checkSum += nDat[i * nLen + j];
    }
  }

  buff[index] = ~checkSum;
  HalfDuplexWrite(buff, index + 1);
}

void SyncWriteU16(u8 ID[], u8 IDN, u16 Values[], u8 MemAddr) {
  u8 offbuf[2 * IDN];
  for (u8 i = 0; i < IDN; i++) {
    Host2SCS(offbuf + i * 2, offbuf + i * 2 + 1, Values[i]);
  }
  SyncWrite(ID, IDN, MemAddr, offbuf, 2);
}

void SyncWriteU8(u8 ID[], u8 IDN, u8 Values[], u8 MemAddr) {
  SyncWrite(ID, IDN, MemAddr, Values, 1);
}

void SyncRead(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen) {
  u8 checkSum = (4 + BROADCAST_ID) + IDN + MemAddr + nLen + ServoInstruction::SYNC_READ;
  u8 i;
  u8 buff[7 + IDN];

  buff[0] = 0xFF;
  buff[1] = 0xFF;
  buff[2] = BROADCAST_ID;
  buff[3] = IDN + 4;
  buff[4] = ServoInstruction::SYNC_READ;
  buff[5] = MemAddr;
  buff[6] = nLen;

  u8 index = 7;
  for (i = 0; i < IDN; i++) {
    buff[index++] = ID[i];
    checkSum += ID[i];
  }
  checkSum = ~checkSum;
  buff[index] = checkSum;
  HalfDuplexWrite(buff, index + 1);
}

ServoResponse ServoDecode(uint8_t *data, uint8_t dataLength) {
  ServoResponse response = {0, 0, 0, {0}, 0, false};

  // Ensure minimum length (header + ID + length + status + checksum)
  if (dataLength < 6 || data[0] != 0xFF || data[1] != 0xFF) {
    return response;
  }

  response.id = data[2];
  response.length = data[3];
  response.status = data[4];

  // Ensure reported length matches actual received length
  if (dataLength != response.length + 4) {
    return response;
  }

  // Extract parameters
  uint8_t checksum = response.id + response.length + response.status;
  for (uint8_t i = 0; i < response.length - 2; i++) {
    response.parameters[i] = data[5 + i];
    checksum += response.parameters[i];
  }

  // Extract received checksum
  response.checksum = data[5 + response.length - 2];

  // Validate checksum
  if ((~checksum & 0xFF) == response.checksum) {
    response.valid = true;
  }

  return response;
}
