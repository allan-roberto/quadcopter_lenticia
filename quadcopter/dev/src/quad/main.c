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
int16_t giro_data[3];
int main(void)
{
	uartInit(UBRR_VAL);
	init_gyro();
	sei();

	while(1)
	{
		sprintf(buffer, "Gyroscope");
		uart_puts (buffer);
		uart_puts ("\n\r");

		sprintf(buffer, "data[0]: %d",giro_data[0]);
		uart_puts (buffer);
		uart_puts ("\n\r");

		sprintf(buffer, "data[1]: %d",giro_data[1]);
		uart_puts (buffer);
		uart_puts ("\n\r");

		sprintf(buffer, "data[2]: %d",giro_data[2]);
		uart_puts (buffer);
		uart_puts ("\n\r");

		gyro_get_data(&giro_data[0],&giro_data[1],&giro_data[2]);
		_delay_ms(50);
		uart_putc(12);

	}
	return 0;
}
