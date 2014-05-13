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
double data[3];
uint16_t value = 1000;
#define UART_NUM 2
int main(void)
{
	uartInit(UART_NUM, UBRR_VAL);
	init_accelerometer();
	sei();
	while(1)
	{

		sprintf(buffer, "[0]: %f \n\r",data[0]);
		uart_puts (UART_NUM, buffer);

		sprintf(buffer, "[1]: %f \n\r",data[1]);
		uart_puts (UART_NUM, buffer);

		sprintf(buffer, "[2]: %f \n\r",data[2]);
		uart_puts (UART_NUM, buffer);


		_delay_ms(10);
		//uart_putc (UART_NUM, 12);
		accelerometer_get_data(&data[0],&data[1],&data[2]);
	}
	while(1)
	{

	}
	return 0;
}
