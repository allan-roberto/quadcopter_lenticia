/*
 * imu.h
 *
 *  Created on: May 26, 2014
 *      Author: tpv
 */

#ifndef IMU_H_
#define IMU_H_

#include <accelerometer.h>
#include <gyro.h>
#include <magnetometer.h>
#include <fix16.h>

void imu_get_angles(fix16_t angles[]);
void imu_get_rates(fix16_t rate[]);
void init_imu(void);

#endif /* IMU_H_ */
