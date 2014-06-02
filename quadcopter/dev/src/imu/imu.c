/*
 * imu.c
 *
 *  Created on: May 26, 2014
 *      Author: tpv
 */

#include <imu/imu.h>
#include <fix16.h>

extern imu_t imu;

void imu_get_angles(fix16_t angles[]){


	magnetometer_get_data();
	accelerometer_get_data();

	angles[0] 	= fix16_atan2((fix16_t)imu.accADC[ROLL] ,(fix16_t)imu.accADC[YAW]);
	angles[1] 	= fix16_atan2((fix16_t)imu.accADC[PITCH],(fix16_t)imu.accADC[YAW]);
	angles[2]	= fix16_atan2((fix16_t)imu.magADC[PITCH] ,(fix16_t)imu.magADC[ROLL]);

	angles[0] = fix16_rad_to_deg(angles[0]);
	angles[1] = fix16_rad_to_deg(angles[1]);
	angles[2] = fix16_rad_to_deg(angles[2]);



}
void imu_get_rates(fix16_t rate[]){
	gyro_get_data();
	rate[0] = fix16_from_int(imu.gyroADC[ROLL]);
	rate[1] = fix16_from_int(imu.gyroADC[PITCH]);
	rate[2] = fix16_from_int(imu.gyroADC[YAW]);
}
void init_imu(void){

	init_magnetometer();
	init_gyro();
	init_accelerometer();

}
