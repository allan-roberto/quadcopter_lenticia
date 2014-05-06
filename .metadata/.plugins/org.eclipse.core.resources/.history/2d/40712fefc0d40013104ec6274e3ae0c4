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
uint16_t value = 0;
int main(void)
{
	uartInit(UBRR_VAL);
	init_pwm();
	sei();

	while(1)
	{
		if(value < 17700){

			set_pwm(MOTOR_D5,value);
			set_pwm(MOTOR_D5,value);
			set_pwm(MOTOR_D5,value);
			set_pwm(MOTOR_D5,value);

			value++;
			_delay_ms(1);
		}
		else{
			value = 0;
		}

	}
	return 0;
}
