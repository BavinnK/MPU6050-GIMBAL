#include "MyUSART.h"
#define clk_speed 16000000
#define baud 9600
#define my_ubrr (clk_speed/16/baud-1)
void USART_init(unsigned int ubrr){
  UBRR0H=(unsigned char)(ubrr>>8);
  UBRR0L=(unsigned char)ubrr;
  UCSR0B=(1<<RXEN0)|(1<<TXEN0);
  UCSR0C=(1<<UCSZ00)|(1<<UCSZ01);
}
void USART_TX(unsigned char data){
  while(!(UCSR0A&(1<<UDRE0)));
  UDR0=data;
}
unsigned char USART_RX(void){
while(!(UCSR0A&(1<<RXC0)));
  return UDR0;
}
void USART_strTransmit(char*str){
  while(*str){
    USART_TX(*str++);
  }
}

