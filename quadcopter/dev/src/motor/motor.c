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

bool set_throtle(uint8_t motor_id,uint16_t value){

	bool ret_err = 0;
	if((motor_id < MOTOR_D8) || (motor_id > D46)){
		ret_err = 1;
		return ret_err;
	}
	if((value < 1000) || (value > 2000)){
		ret_err = 1;
		return ret_err;
	}
	set_pwm(motor_id,(value << 3));
	return ret_err;
}

bool disable_motor(uint8_t motor_id){

	bool ret_err = 0;
	if((motor_id < MOTOR_D8) || (motor_id > D46)){
		ret_err = 1;
		return ret_err;
	}
	set_pwm(motor_id,MOTOR_DISABLED);
	return ret_err;
}
