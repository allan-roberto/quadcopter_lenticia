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
uint16_t i = 0;
#define UART 2
int main(void)
{
	i2c_init();
	uartInit(UART,UBRR_VAL);
	uart_puts (UART, buffer);
	uart_puts (UART, "\r\n");
	sei();
	init_gyro();

	for(i=0;i<10000;i++){

		gyro_get_data(&giro_data[0],&giro_data[1],&giro_data[2]);
		sprintf(buffer, " %d %d %d\r\n",giro_data[0],giro_data[1],giro_data[2]);
		uart_puts (UART, buffer);
		_delay_ms(55);
		_delay_us(290);
	}
	return 0;
}
