/* 
 *	Basis
 *	2009 Benjamin Reh
 */

#ifndef PWM_H
#define PWM_H
#include <stdint.h>            // has to be added to use uint8_t
#include <types.h>


#define MOTOR_DISABLED 0
#define MOTOR_ENABLED  1
/*
#define D10 0
#define D9 	1
#define D8 	2
#define D7 	3
#define D6 	4
#define D5 	5
#define D3 	6
#define D2 	7
#define D44 8
#define D45 9
#define D46 10
*/

enum e_pwm_pin{
	SERVO_D10 = 0,
	SERVO_D9,
	MOTOR_D8,
	MOTOR_D7,
	MOTOR_D6,
	MOTOR_D5,
	D3,
	D2,
	D44,
	D45,
	D46
};

void init_pwm(void);

/* void set_pwm(uint8_t output,uint8_t value)
 * from SERVO_D10 to SERVO_D9
 * 		value max: 2000us
 * 		value min: 1000us
 * from MOTOR_D8 to D46
 * 		value max: 2000us
 * 		value min: 1000us
 * output: D10,D9,MOTOR_D8,MOTOR_D7,	MOTOR_D6,MOTOR_D5,D3,D2,D44,D45,D46
 */
bool set_pwm(uint8_t output,uint16_t period);

#endif
