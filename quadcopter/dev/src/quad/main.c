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
uint16_t value = 1000;
int main(void)
{
	uartInit(UBRR_VAL);
	init_motor(MOTOR_D5);
	init_motor(MOTOR_D6);
	init_motor(MOTOR_D7);
	init_motor(MOTOR_D8);

	sei();

	while(1)
	{
		set_throtle(MOTOR_D5,)

	}
	return 0;
}
