/* 
 *	Basis
 *	2009 Benjamin Reh und Joachim Schleicher
 */
#ifndef UART_H
#define UART_H

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/iom2560.h>
#include <stdlib.h>

#define UART_NUM 2

//USART
#define BAUD 115200UL          // Baudrate


#ifndef F_CPU
#warning "F_CPU war noch nicht definiert, wird nun nachgeholt mit 16000000"
#define F_CPU 16000000UL
#endif


#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD)

#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
  //#error "UARTConfig - Error greater than 1% "
#endif


void uartInit(unsigned long int uart_id, unsigned long int ubrr);

int uart_putc(unsigned int uart_id, unsigned char c);

void uart_puts(unsigned int uart_id, char *s);

void uart_puts_pgm (unsigned int uart_id, const char* PROGMEM  str);

void uart_puti (unsigned int uart_id, int16_t i);

unsigned char uart_getc(unsigned int uart_id);

#define uart_data_waiting() (UCSR0A & (1<<RXC0))



#endif
