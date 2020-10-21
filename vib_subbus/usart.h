#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#include <stdint.h>

/*! The rx buffer size for USART */
#define UART_RX_BUFFER_SIZE 64
#define UART_TX_BUFFER_SIZE 512

extern volatile int UART_tx_busy;

void uart_init(void);
int uart_recv(uint8_t *buf, int nbytes);
void uart_send_char(int8_t c);
void uart_flush_input(void);
void uart_flush_output(void);

#endif
