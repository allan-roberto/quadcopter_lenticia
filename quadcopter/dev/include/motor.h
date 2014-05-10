/*
 * motor.h
 *
 *  Created on: 04/05/2014
 *      Author: allan_amorim
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include <types.h>

#define MOTOR_MAX_VALUE 1800
#define MOTOR_MIN_VALUE 1000

bool init_motor(uint8_t index);
bool set_throtle(uint8_t motor_id,uint16_t value);


#endif /* MOTOR_H_ */
