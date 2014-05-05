/*
 * motor.c
 *
 *  Created on: 04/05/2014
 *      Author: allan_amorim
 */
#include <motor.h>
#include <pwm.h>
#include <util/delay.h>
#include <types.h>

bool init_motor(uint8_t index){

	if((index < MOTOR_D8) || (index > MOTOR_D5)){
		return 1;
	}
	set_pwm(index,MOTOR_MAX_VALUE);
	_delay_ms(2000);
	set_pwm(index,MOTOR_MIN_VALUE);
	_delay_ms(2000);

	return 0;
}

void set_throtle(uint8_t value){

}
