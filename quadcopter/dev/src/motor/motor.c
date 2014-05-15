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

#define UART 2
extern char buffer[40];
bool init_motor(uint8_t index){


	if((index < MOTOR_D8) || (index > MOTOR_D5)){
		return 1;
	}
    sprintf(buffer, "init_motor MOTOR_MAX_VALUE");
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
    set_throtle(index,MOTOR_MAX_VALUE);

	_delay_ms(4000); //wait 7 seconds until ESC get max value stored.


    sprintf(buffer, "init_motor MOTOR_MIN_VALUE");
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
    set_throtle(index,MOTOR_MIN_VALUE);

	_delay_ms(4000); //wait 4 seconds until ESC get min value stored.
	return 0;
}
/*
 * set_throttle
 * - Defines how much throttle to a specific motor
 * motor_id
 * - Valid IDs  MOTOR_D5, MOTOR_D6, MOTOR_D7, MOTOR_D8
 * value
 * - throttle to deliver: from 0 to 1000
 */

bool set_throttle(uint8_t motor_id, uint16_t value){

	bool ret_err = 0;
	if((motor_id < MOTOR_D8) || (motor_id > MOTOR_D5)){
		ret_err = 1;
	    sprintf(buffer, "Error set_throtle, motor: %d",motor_id);
	    uart_puts (UART,buffer);
	    uart_puts (UART,"\n\r");
		return ret_err;
	}

	value += 1000;

	if(((value < 1000) || (value > 2000)) && (value != 0)){

		ret_err = 1;
	    sprintf(buffer, "Error set_throtle value: %d",value);
	    uart_puts (UART,buffer);
	    uart_puts (UART,"\n\r");
		return ret_err;
	}
	else{
		value = value << 3;
	}
#if 1
    sprintf(buffer, "set_throtle value: %d,  motor: %d",value,motor_id);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
#endif
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
