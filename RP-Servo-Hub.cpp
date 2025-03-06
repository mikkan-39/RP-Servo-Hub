#include <stdio.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "Protocol.h"

int main() {
  stdio_init_all();

  UartInit();

  uint8_t data[6] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};

  hw_clear_bits(&uart1_hw->cr, 0x00000200);
  uart_write_blocking(UART_ID, data, 6);
  uart_tx_wait_blocking(UART_ID);
  hw_set_bits(&uart1_hw->cr, 0x00000200);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_NULL);

  printf("\nRequest:  ", data);
  for (int i = 0; i < 6; i++)
    printf("0x%02X%c", data[i], i < 5 ? ' ' : '\n');

  //   data[0] = 0xBE;
  //   data[1] = 0xEF;
  //   uart_write_blocking(UART_ID, data, 2);
  //   uart_tx_wait_blocking(UART_ID);

  printf("Response: ");
  while (uart_is_readable_within_us(UART_ID, 1000)) {
    printf("0x%02X ", uart_getc(UART_ID));
  }
  printf("\n");
}
