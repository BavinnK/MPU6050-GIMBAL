#ifndef MY_USART_H
#define MY_USART_H

#include <avr/io.h>

// Config macros
#define CLK_SPEED 16000000
#define BAUD 9600
#define MY_UBRR (CLK_SPEED/16/BAUD - 1)

void USART_init(unsigned int ubrr);
void USART_TX(unsigned char data);
unsigned char USART_RX(void);
void USART_strTransmit(char *str);

#endif
