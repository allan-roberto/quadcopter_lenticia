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



/******                Serial com speed    *********************************/
/* This is the speed of the serial interfaces */
#define SERIAL0_COM_SPEED 115200
#define SERIAL1_COM_SPEED 115200
#define SERIAL2_COM_SPEED 115200
#define SERIAL3_COM_SPEED 115200

#define UART_NUM 0

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





#define UART_NUMBER 4

#define RX_BUFFER_SIZE 256 // 256 RX buffer is needed for GPS communication (64 or 128 was too short)

#define TX_BUFFER_SIZE 128

//void    SerialOpen(uint8_t port, uint32_t baud);
uint8_t SerialRead(uint8_t port);
void    SerialWrite(uint8_t port,uint8_t c);
uint8_t SerialAvailable(uint8_t port);
void    SerialEnd(uint8_t port);
uint8_t SerialPeek(uint8_t port);
uint8_t    SerialTXfree(uint8_t port);
uint8_t SerialUsedTXBuff(uint8_t port);
void    SerialSerialize(uint8_t port,uint8_t a);
void    UartSendData(uint8_t port);
static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];

#endif
