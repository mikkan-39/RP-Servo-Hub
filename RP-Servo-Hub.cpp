#include <stdio.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "Protocol.h"
#include "Utils.h"

int main() {
  stdio_init_all();

  UartInit();

  ServoWriteBuf(1, 0, 0, 0, PING);

  int responselen = 6;
  u8 response[responselen];

  if (ServoReceive(response, responselen) != responselen) {
    printf("Receive length mismatch.\n");
  };

  printf("Response: ");
  printHexBuf(response, responselen);
}
