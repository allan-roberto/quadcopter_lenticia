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
	uartInit(0,UBRR_VAL);
	init_pwm();
	_delay_ms(2000);
	init_motor(MOTOR_D8);

	set_throtle(MOTOR_D8,0);
	_delay_ms(2000);
	set_throtle(MOTOR_D8,100);
	_delay_ms(2000);
	set_throtle(MOTOR_D8,200);
	_delay_ms(2000);
	set_throtle(MOTOR_D8,400);
	_delay_ms(2000);
	set_throtle(MOTOR_D8,600);
	_delay_ms(2000);
	set_throtle(MOTOR_D8,800);
	_delay_ms(2000);

	disable_motor(MOTOR_D8);
	sei();

	while(1)
	{

	}
	return 0;
}
