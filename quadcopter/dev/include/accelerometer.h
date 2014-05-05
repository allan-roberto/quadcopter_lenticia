/*
 * accelerometer.h
 *
 *  Created on: Feb 11, 2014
 *      Author: root
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdlib.h> // enthaelt itoa, ltoa
#include "types.h"




#define BMA180_ADDRESS 0x40
#define ACC_1G 255
#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}

void init_accelerometer(void);
void accelerometer_get_data(int16_t *X, int16_t *Y, int16_t *Z );
void ACC_Common();

#endif /* ACCELEROMETER_H_ */
