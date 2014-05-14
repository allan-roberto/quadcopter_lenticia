/* 
 *	Basis
 *	2009 Benjamin Reh und Joachim Schleicher
 */


#include <uart.h>

char input_uart = 0;

// USART-Init
void uartInit(unsigned int uart_id, unsigned int ubrr)
{
	switch(uart_id){
	case 0:
		/* Enable receiver and transmitter */
		UCSR0B = ((1 << TXEN0 ) | (1 << RXEN0 ));

		/* Set frame format: 8N1*/
		UCSR0C = 0x06;

		UBRR0H = (unsigned char)(ubrr>>8);
		UBRR0L = (unsigned char)ubrr;


		break;
	case 1:
		/* Enable receiver and transmitter */
		UCSR1B = ((1 << TXEN1 ) | (1 << RXEN1 ));

		/* Set frame format: 8N1*/
		UCSR1C = 0x06;

		UBRR1H = (unsigned char)(ubrr>>8);
		UBRR1L = (unsigned char)ubrr;

			break;
	case 2:
		/* Enable receiver and transmitter */
		UCSR2B = ((1 << TXEN2 ) | (1 << RXEN2 ));

		/* Set frame format: 8N1*/
		UCSR2C = 0x06;

		UBRR2H = (unsigned char)(ubrr>>8);
		UBRR2L = (unsigned char)ubrr;

			break;
	case 3:
		/* Enable receiver and transmitter */
		UCSR3B = ((1 << TXEN3 ) | (1 << RXEN3 ));

		/* Set frame format: 8N1*/
		UCSR3C = 0x06;

		UBRR3H = (unsigned char)(ubrr>>8);
		UBRR3L = (unsigned char)ubrr;

			break;
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


ISR(USART0_TX_vect)
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
}
