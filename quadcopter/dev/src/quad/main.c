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
int main(void)
{
	uartInit(UBRR_VAL);
	init_gyro();
	sei();
	sprintf(buffer, "Gyroscope");
	uart_puts (buffer);
	uart_puts ("\n\r");
	for(i=0;i<1000;i++){

		gyro_get_data(&giro_data[0],&giro_data[1],&giro_data[2]);
		sprintf(buffer, " %d %d\r\n",giro_data[0],i);
		uart_puts (buffer);
		_delay_ms(55);
		_delay_us(290);
	}
	return 0;
}
