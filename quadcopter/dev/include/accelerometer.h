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
<<<<<<< HEAD
void accelerometer_get_angle(float *X, float *Y, float *Z );
=======
>>>>>>> 6ebd0f1d41274e793ae18541b3851831f5ebf74b
void accelerometer_get_data(float *X, float *Y, float *Z );
void ACC_Common();

#endif /* ACCELEROMETER_H_ */
