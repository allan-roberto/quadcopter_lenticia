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

bool init_servo(uint8_t index){

	if((index < MOTOR_D8) || (index > MOTOR_D5)){
		return 1;
	}
	//set_pwm(index,MOTOR_MAX_VALUE);
	//_delay_ms(2000);
	//set_pwm(index,MOTOR_MIN_VALUE);
	//_delay_ms(2000);

	return 0;
}

bool set_servo_pos(uint8_t servo_id, uint8_t pos){

	bool ret_err = 0;
		if((servo_id == SERVO_D10) || (servo_id == SERVO_D9)){
			//this is due a low resolution timer
			if((servo_id < 30) || (servo_id > 60)){
				ret_err = 1;
				return ret_err;
			}

		}else {
				ret_err = 1;
				return ret_err;
		}
		set_pwm(servo_id,pos);
		return ret_err;

}
