/* 
 *	Basis
 *	2009 Benjamin Reh und Joachim Schleicher
 */


#include <uart.h>

char input_uart = 0;
unsigned int low;
unsigned int high;

// USART-Init
void uartInit(unsigned long int uart_id, unsigned long int ubrr)
{
	  uint8_t h = ((F_CPU  / 4 / ubrr -1) / 2) >> 8;
	  uint8_t l = ((F_CPU  / 4 / ubrr -1) / 2);
	  switch (uart_id) {
	      case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
	      case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
	      case 2: UCSR2A  = (1<<U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2); break;
	      case 3: UCSR3A  = (1<<U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3); break;
	  }

}
void uartEnd(uint8_t uart_id) {
  switch (uart_id) {
      case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
      case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1)); break;
      case 2: UCSR2B &= ~((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)|(1<<UDRIE2)); break;
      case 3: UCSR3B &= ~((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)|(1<<UDRIE3)); break;
  }
}

int uart_putc(unsigned int uart_id, unsigned char c)
{
	switch(uart_id){
	case 0:
		while(!(UCSR0A & (1 << UDRE0)));
		UDR0 = c;
			break;
	case 1:
		while(!(UCSR1A & (1 << UDRE1)));
		UDR1 = c;
			break;
	case 2:
		while(!(UCSR2A & (1 << UDRE2)));
		UDR2 = c;
			break;
	case 3:
		while(!(UCSR3A & (1 << UDRE3)));
		UDR3 = c;
			break;
	}


	return 0;
}

void uart_puts (unsigned int uart_id, char *s)
{
	while (*s)
	{
		uart_putc(uart_id, *s);
		//SerialWrite(uart_id, *s);
		s++;
	}
}

void uart_puts_pgm (unsigned int uart_id, const char* PROGMEM  str)
{

	while (1)
	{
		char c = (char) pgm_read_byte (str);
		uart_putc(uart_id, c);
		if ('\0' == c)
			return;
		str++;
	}
}

void uart_puti (unsigned int uart_id, int16_t i)
{

	char buffer[10];
	itoa(i, buffer, 10);
	uart_puts(uart_id, buffer);
}


unsigned char uart_getc(unsigned int uart_id)
{
	switch(uart_id){
	case 0:
		while (!(UCSR0A & (1<<RXC0)));
			return UDR0;
			break;
	case 1:
		while (!(UCSR1A & (1<<RXC1)));
			return UDR1;
			break;
	case 2:
		while (!(UCSR2A & (1<<RXC2)));
			return UDR2;
			break;
	case 3:
		while (!(UCSR3A & (1<<RXC3)));
			return UDR3;
			break;
	}
return 0;
}

ISR (USART0_RX_vect)
{
	char ReceivedByte ;
	ReceivedByte = UDR0 ; // Fetch the received byte value into the variable "ByteReceived"
	//UDR0 = ReceivedByte ; // Echo back the received byte back to the computer
	input_uart = ReceivedByte;

}
ISR (USART1_RX_vect)
{
	char ReceivedByte ;
	ReceivedByte = UDR1 ; // Fetch the received byte value into the variable "ByteReceived"
	//UDR0 = ReceivedByte ; // Echo back the received byte back to the computer
	input_uart = ReceivedByte;

}
ISR (USART2_RX_vect)
{
	char ReceivedByte ;
	ReceivedByte = UDR2 ; // Fetch the received byte value into the variable "ByteReceived"
	//UDR0 = ReceivedByte ; // Echo back the received byte back to the computer
	input_uart = ReceivedByte;

}
ISR (USART3_RX_vect)
{
	char ReceivedByte ;
	ReceivedByte = UDR3 ; // Fetch the received byte value into the variable "ByteReceived"
	//UDR0 = ReceivedByte ; // Echo back the received byte back to the computer
	input_uart = ReceivedByte;

}


/*ISR(USART0_TX_vect)
{
	uart_puts(0, "Tx Interrupt\n\r");
}

ISR(USART1_TX_vect)
{
	uart_puts(1, "Tx Interrupt\n\r");
}
ISR(USART2_TX_vect)
{
	uart_puts(2, "Tx Interrupt\n\r");
}
ISR(USART3_TX_vect)
{
	uart_puts(3, "Tx Interrupt\n\r");
}*/


  //ISR(USART0_RX_vect)  { store_uart_in_buf(UDR0, 0); }
  //ISR(USART1_RX_vect)  { store_uart_in_buf(UDR1, 1); }
  //ISR(USART2_RX_vect)  { store_uart_in_buf(UDR2, 2); }
  //ISR(USART3_RX_vect)  { store_uart_in_buf(UDR3, 3); }



uint8_t SerialTXfree(uint8_t port) {
		return (serialHeadTX[port] == serialTailTX[port]);
	}


void SerialWrite(uint8_t port,uint8_t c){
	SerialSerialize(port,c);UartSendData(port);
}
void SerialSerialize(uint8_t port,uint8_t a) {
  uint8_t t = serialHeadTX[port];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][port] = a;
  serialHeadTX[port] = t;
}

void UartSendData(uint8_t port) {
    switch (port) {
      case 0: UCSR0B |= (1<<UDRIE0); break;
      case 1: UCSR1B |= (1<<UDRIE1); break;
      case 2: UCSR2B |= (1<<UDRIE2); break;
      case 3: UCSR3B |= (1<<UDRIE3); break;
    }
}
uint8_t SerialRead(uint8_t port) {
  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

uint8_t SerialAvailable(uint8_t port) {
  return ((uint8_t)(serialHeadRX[port] - serialTailRX[port]))%RX_BUFFER_SIZE;
}

ISR(USART0_UDRE_vect) { // Serial 0 on a MEGA
	uint8_t t = serialTailTX[0];
	if (serialHeadTX[0] != t) {
		if (++t >= TX_BUFFER_SIZE) t = 0;
		while(!(UCSR0A & (1 << UDRE0)));
		UDR0 = serialBufferTX[t][0];  // Transmit next byte in the ring
		serialTailTX[0] = t;
	}
	if (t == serialHeadTX[0]) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
}


ISR(USART1_UDRE_vect) { // Serial 1 on a MEGA or on a PROMICRO
	uint8_t t = serialTailTX[1];
	if (serialHeadTX[1] != t) {
		if (++t >= TX_BUFFER_SIZE) t = 0;
		while(!(UCSR1A & (1 << UDRE1)));
		UDR1 = serialBufferTX[t][1];  // Transmit next byte in the ring
		serialTailTX[1] = t;
	}
	if (t == serialHeadTX[1]) UCSR1B &= ~(1<<UDRIE1);
}

ISR(USART2_UDRE_vect) { // Serial 2 on a MEGA
	uint8_t t = serialTailTX[2];
	if (serialHeadTX[2] != t) {
		if (++t >= TX_BUFFER_SIZE) t = 0;
		while(!(UCSR2A & (1 << UDRE2)));
		UDR2 = serialBufferTX[t][2];
		serialTailTX[2] = t;
	}
	if (t == serialHeadTX[2]) UCSR2B &= ~(1<<UDRIE2);
}
ISR(USART3_UDRE_vect) { // Serial 3 on a MEGA
	uint8_t t = serialTailTX[3];
	if (serialHeadTX[3] != t) {
		if (++t >= TX_BUFFER_SIZE) t = 0;
		while(!(UCSR3A & (1 << UDRE3)));
		UDR3 = serialBufferTX[t][3];
		serialTailTX[3] = t;
	}
	if (t == serialHeadTX[3]) UCSR3B &= ~(1<<UDRIE3);
}
void store_uart_in_buf(uint8_t data, uint8_t portnum) {
  #if defined(SPEKTRUM)
    if (portnum == SPEK_SERIAL_PORT) {
  #endif
  #if defined(SBUS)
    if (portnum == SBUS_SERIAL_PORT) {
  #endif
  #if defined(SPEKTRUM) || defined(SBUS)
      if (!spekFrameFlags) {
        sei();
        uint32_t spekTimeNow = (timer0_overflow_count << 8) * (64 / clockCyclesPerMicrosecond()); //Move timer0_overflow_count into registers so we don't touch a volatile twice
        uint32_t spekInterval = spekTimeNow - spekTimeLast;                                       //timer0_overflow_count will be slightly off because of the way the Arduino core timer interrupt handler works; that is acceptable for this use. Using the core variable avoids an expensive call to millis() or micros()
        spekTimeLast = spekTimeNow;
        if (spekInterval > 5000) {  //Potential start of a Spektrum frame, they arrive every 11 or every 22 ms. Mark it, and clear the buffer.
          serialTailRX[portnum] = 0;
          serialHeadRX[portnum] = 0;
          spekFrameFlags = 0x01;
        }
        cli();
      }
    }
  #endif

  uint8_t h = serialHeadRX[portnum];
  serialBufferRX[h++][portnum] = data;
  if (h >= RX_BUFFER_SIZE) h = 0;
  serialHeadRX[portnum] = h;
}
