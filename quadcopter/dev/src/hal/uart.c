/* 
 *	Basis
 *	2009 Benjamin Reh und Joachim Schleicher
 */


#include <uart.h>

char input_uart = 0;

//Schnittstelle initialisieren:
// USART-Init
void uartInit(unsigned int ubrr)
{
	/* Enable receiver and transmitter */
	UCSR0B = ((1 << TXEN0 ) | (1 << RXEN0 ));

	/* Set frame format: 8N1*/
	UCSR0C = 0x06;

	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;

}

// Ein Zeichen senden
int uart_putc(unsigned char c)
{
	while(!(UCSR0A & (1 << UDRE0))); 	// warte, bis UDR bereit ist, d.h. letztes Zeichen den Sendepuffer verlassen hat
	UDR0 = c;				//Zeichen senden
	return 0;
}

//ganze Zeichenkette senden
void uart_puts (char *s)
{
	while (*s)
	{   				// so lange *s != '\0' also ungleich dem "String-Endezeichen" 
		uart_putc(*s);
		s++;
	}
}

//ganze Zeichenkette aus program space senden
void uart_puts_pgm (const char* PROGMEM  str)
{
	while (1)
	{   				// so lange c != '\0' also ungleich dem "String-Endezeichen" 
		char c = (char) pgm_read_byte (str);
		uart_putc(c);
		if ('\0' == c)
			return;
		str++;
	}
}

//Zahl ausgeben
void uart_puti (int16_t i) 
{
	char buffer[10];
	itoa(i, buffer, 10);
	uart_puts(buffer);
}


//Ein Zeichen empfangen
unsigned char uart_getc(void)
{
	while (!(UCSR0A & (1<<RXC0)));	// warten bis Zeichen verfuegbar
		return UDR0;		// Zeichen aus UDR an Aufrufer zurueckgeben
}

ISR (USART0_RX_vect)
{
	char ReceivedByte ;
	ReceivedByte = UDR0 ; // Fetch the received byte value into the variable "ByteReceived"
	//UDR0 = ReceivedByte ; // Echo back the received byte back to the computer
	input_uart = ReceivedByte;

}


ISR(USART0_TX_vect)
{
	uart_puts ("Tx Interrupt\n\r");
}
