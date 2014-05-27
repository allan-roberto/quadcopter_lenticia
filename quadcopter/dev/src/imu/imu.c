/*
 * imu.c
 *
 *  Created on: May 26, 2014
 *      Author: tpv
 */

#include <imu/imu.h>

extern imu_t imu;

void imu_get_angles(float angles[]){


	magnetometer_get_data();
	accelerometer_get_data();

	angles[0] 	= atan2f((float)imu.accADC[ROLL] ,(float)imu.accADC[YAW]);
	angles[1] 	= atan2f((float)imu.accADC[PITCH],(float)imu.accADC[YAW]);
	angles[2]	= atan2f((float)imu.magADC[PITCH] ,(float)imu.magADC[ROLL]);
	angles[0] = angles[0] * 57,32;
	angles[1] = angles[1] * 57,32;
	angles[2] = angles[2] * 57,32;


}
void imu_get_rates(float rate[]){
	gyro_get_data();
	rate[0] = (float)imu.gyroADC[ROLL];
	rate[1] = (float)imu.gyroADC[PITCH];
	rate[2] = (float)imu.gyroADC[YAW];
}
void init_imu(void){

	init_magnetometer();
	init_gyro();
	init_accelerometer();

}
