/*
 * main.c
 *
 *  Created on: 07/02/2014
 *      Author: aras
 */

#include <avr/interrupt.h>
#include <lib_xcopter.h>
#include <stdint.h>
#include <uart.h>
#include <util/delay.h>


char buffer[40] =  "some characters";
int16_t data[3];
uint16_t value = 1000;
int main(void)
{
	uartInit(UBRR_VAL);
	init_accelerometer();
	sei();

	while(1)
	{
		sprintf(buffer, "Accelerometer");
		uart_puts (buffer);
		uart_puts ("\n\r");

		sprintf(buffer, "data[0]: %d",data[0]);
		uart_puts (buffer);
		uart_puts ("\n\r");

		sprintf(buffer, "data[0]: %d",data[1]);
		uart_puts (buffer);
		uart_puts ("\n\r");

		sprintf(buffer, "data[0]: %d",data[2]);
		uart_puts (buffer);
		uart_puts ("\n\r");

		_delay_ms(2);
		uart_putc (12);
		accelerometer_get_data(&data[0],&data[1],&data[2]);
	}
	return 0;
}
