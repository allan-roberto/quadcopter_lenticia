/*
 * magnetometer.h
 *
 *  Created on: May 26, 2014
 *      Author: tpv
 */

#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdlib.h>
#include <types.h>
#include <def.h>

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  Y; imu.magADC[PITCH]  = X; imu.magADC[YAW]  = -Z;}

#define HMC5883


void init_magnetometer();
uint8_t Mag_getADC();
void getADC();
void Device_Mag_getADC();
void magnetometer_get_raw_data(int16_t *X, int16_t *Y, int16_t *Z );
uint8_t magnetometer_get_data(void) ;

#endif /* MAGNETOMETER_H_ */
