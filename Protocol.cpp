#include "Protocol.h"

void UartInit() {
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void HalfDuplexWrite(u8 *data, int length) {
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  hw_clear_bits(&uart1_hw->cr, 0x00000200);
  uart_write_blocking(uart1, data, length);
  uart_tx_wait_blocking(UART_ID);
  hw_set_bits(&uart1_hw->cr, 0x00000200);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_NULL);
}

void HalfDuplexWrite(u8 data) {
  u8 buf[1] = {data};
  HalfDuplexWrite(buf, 1);
}

int ServoReceive(u8 *data, int len) {
  int i = 0;
  int ComData;
  while (1) {
    if (uart_is_readable_within_us(UART_ID, 100)) {
      ComData = uart_getc(UART_ID);
      if (data) {
        data[i] = ComData;
      }
      i++;
    } else {
      break;
    }

    if (i >= len) {
      break;
    }
  }
  return i;
}

void ServoWriteBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Function) {
  u8 msgLen = 2;
  u8 bBuf[6];
  u8 CheckSum = 0;
  bBuf[0] = 0xff;
  bBuf[1] = 0xff;
  bBuf[2] = ID;
  bBuf[4] = Function;
  if (nDat) {
    msgLen += nLen + 1;
    bBuf[3] = msgLen;
    bBuf[5] = MemAddr;
    HalfDuplexWrite(bBuf, 6);

  } else {
    bBuf[3] = msgLen;
    HalfDuplexWrite(bBuf, 5);
  }
  CheckSum = ID + msgLen + Function + MemAddr;
  u8 i = 0;
  if (nDat) {
    for (i = 0; i < nLen; i++) {
      CheckSum += nDat[i];
    }
    HalfDuplexWrite(nDat, nLen);
  }
  HalfDuplexWrite(~CheckSum);
}
