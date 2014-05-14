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
uint16_t i = 0;
#define UART_NUM 2
int main(void)
{
	uartInit(UART_NUM, UBRR_VAL);
	init_accelerometer();
	sei();
	sprintf(buffer, "Accelerometer \n\r");
	uart_puts (UART_NUM, buffer);
	for(i=0;i<1000;i++){

		sprintf(buffer, "%3.3f %d\r\n",data[2],i);
		uart_puts (UART_NUM, buffer);
		_delay_ms(53);
		_delay_us(270);
		accelerometer_get_data(&data[0],&data[1],&data[2]);
	}

	return 0;
}
