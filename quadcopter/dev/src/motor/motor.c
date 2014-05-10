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
#include <uart.h>

bool init_motor(uint8_t index){


	if((index < MOTOR_D8) || (index > MOTOR_D5)){
		sprintf("")
		return 1;
	}
	set_pwm(index,MOTOR_MAX_VALUE);
	_delay_ms(7000); //wait 7 seconds until ESC get max value stored.
	set_pwm(index,MOTOR_MIN_VALUE);
	_delay_ms(4000); //wait 4 seconds until ESC get min value stored.
	return 0;
}

bool set_throtle(uint8_t motor_id,uint16_t value){

	bool ret_err = 0;
	if((motor_id < MOTOR_D8) || (motor_id > MOTOR_D5)){
		ret_err = 1;
		return ret_err;
	}

	value += 1000;

	if((value < 0) || (value > 1000)){
		ret_err = 1;
		return ret_err;
	}
	else{
		value = value << 3;
	}
	set_pwm(motor_id,value);
	return ret_err;
}

bool disable_motor(uint8_t motor_id){

	bool ret_err = 0;
	if((motor_id < MOTOR_D8) || (motor_id > MOTOR_D5)){
		ret_err = 1;
		return ret_err;
	}
	set_pwm(motor_id,MOTOR_DISABLED);
	return ret_err;
}
