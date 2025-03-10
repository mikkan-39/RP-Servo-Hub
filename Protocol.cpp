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

void ServoWriteBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Function) {
  u8 msgLen = nDat ? (nLen + 3) : 2;
  u8 CheckSum = ID + msgLen + Function + (nDat ? MemAddr : 0);

  u8 bBuf[6 + nLen];
  bBuf[0] = 0xff;
  bBuf[1] = 0xff;
  bBuf[2] = ID;
  bBuf[3] = msgLen;
  bBuf[4] = Function;

  u8 index = 5;
  if (nDat) {
    bBuf[index++] = MemAddr;
    for (u8 i = 0; i < nLen; i++) {
      bBuf[index++] = nDat[i];
      CheckSum += nDat[i];
    }
  }

  bBuf[index] = ~CheckSum;

  HalfDuplexWrite(bBuf, index + 1);
}

void SyncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen) {
  u8 mesLen = ((nLen + 1) * IDN + 4);
  u8 Sum = 0xfe + mesLen + SYNC_WRITE + MemAddr + nLen;

  u8 bBuf[7 + (IDN * (nLen + 1)) + 1];

  bBuf[0] = 0xff;
  bBuf[1] = 0xff;
  bBuf[2] = 0xfe; // Broadcast ID
  bBuf[3] = mesLen;
  bBuf[4] = SYNC_WRITE;
  bBuf[5] = MemAddr;
  bBuf[6] = nLen;

  u8 index = 7;
  for (u8 i = 0; i < IDN; i++) {
    bBuf[index++] = ID[i];
    Sum += ID[i];
    for (u8 j = 0; j < nLen; j++) {
      bBuf[index++] = nDat[i * nLen + j];
      Sum += nDat[i * nLen + j];
    }
  }

  bBuf[index] = ~Sum;

  HalfDuplexWrite(bBuf, index + 1);
}

inline void SyncWriteU16(u8 ID[], u8 IDN, u16 Values[], u8 MemAddr) {
  u8 offbuf[2 * IDN];
  for (u8 i = 0; i < IDN; i++) {
    Host2SCS(offbuf + i * 2, offbuf + i * 2 + 1, Values[i]);
  }
  SyncWrite(ID, IDN, MemAddr, offbuf, 2);
}

inline void SyncWriteU8(u8 ID[], u8 IDN, u8 Values[], u8 MemAddr) {
  SyncWrite(ID, IDN, MemAddr, Values, 1);
}