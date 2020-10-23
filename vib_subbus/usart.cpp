#include <Arduino.h>
#include <string.h>
#include <assert.h>
#include "usart.h"

/*! Buffer for the receive ringbuffer */
static uint8_t UART_rx_buffer[UART_RX_BUFFER_SIZE];

/*! Buffer to accumulate output before sending */
static uint8_t UART_tx_buffer[UART_TX_BUFFER_SIZE+1];
 /*! The number of characters in the tx buffer */
static int nc_tx, cp_tx;

volatile int UART_tx_busy = 0;

void uart_init(void) {
  Serial.setTimeout(0);
  UART_tx_busy = 0;
  nc_tx = cp_tx = 0;
}

int uart_recv(uint8_t *buf, int nbytes) {
  return Serial.readBytes(buf, nbytes);
}

void uart_flush_input(void) {
  char lbuf[80];
  while (Serial.readBytes(lbuf,80) > 0);
}

void uart_send_char(int8_t c) {
  if (nc_tx >= UART_TX_BUFFER_SIZE) {
    uart_flush_output();
    if (nc_tx >= UART_TX_BUFFER_SIZE && cp_tx > 0 && cp_tx < nc_tx) {
      memmove(&UART_tx_buffer[0], &UART_tx_buffer[cp_tx], nc_tx-cp_tx);
      nc_tx -= cp_tx;
      cp_tx = 0;
    }
  }
  // This could happen. It shouldn't if our responses stay
  // under the TX bfr size and we do not entertain requests
  // until the buffer is flushed.
  assert(nc_tx < UART_TX_BUFFER_SIZE);
  UART_tx_buffer[nc_tx++] = (uint8_t)c;
}

void uart_flush_output(void) {
  int nc = nc_tx - cp_tx;
  if (nc) {
    UART_tx_busy = 1;
    int av = Serial.availableForWrite();
    if (av > 0) {
      if (nc > av) nc = av;
      int nc1 = Serial.write(&UART_tx_buffer[cp_tx], nc);
      cp_tx += nc1;
      if (cp_tx >= nc_tx) {
        cp_tx = nc_tx = 0;
        UART_tx_busy = 0;
      }
    }
  } else {
    nc_tx = cp_tx = 0;
    UART_tx_busy = 0;
  }
}
