/*
 * main.c
 *
 *  Created on: 07/02/2014
 *      Author: aras
 */

#include <avr/interrupt.h>
#include <lib_xcopter.h>
#include <stdint.h>
#include <util/delay.h>
#include <kalman.h>
#include <imu/imu.h>


char buffer[200] =  "some characters";

uint16_t i = 0;
//extern gyroZero[3];
#define ANGLE 1


double rate[3] 	= {0.0, 0.0, 0.0};
double angle[3] 	= {0.0, 0.0, 0.0};

imu_t imu;
global_conf_t global_conf;
conf_t conf;
uint8_t rawADC[6];
uint32_t currentTime;


int main(void)
{
	uartInit(UART_NUM,230400);
	i2c_init();
	init_imu();
	init_kalman_timer();

	sei();
	while(1){

	}

	return 0;
}

SIGNAL (TIMER0_COMPA_vect)
{
	//static long int control = 0;
	//if(++control < 10 ) return;
	//control = 0;

#if 1
	imu_get_rates(rate);
	imu_get_angles(angle);

#endif

#if 0
		sprintf(buffer, " %d %d %d %d %d %d \r\n",	accel_data[0], giro_data[0],
													accel_data[1], giro_data[1],
													accel_data[2], giro_data[02]);
		uart_puts(UART_NUM,buffer);
#endif

#if 0
		magnetometer_get_raw_data(&mag_data[0],&mag_data[1],&mag_data[2]);
		sprintf(buffer, " %2.3f %2.3f %2.3f \r\n",	(float)mag_data[0]* 0.00256,
													(float)mag_data[1]* 0.00256,
													(float)mag_data[2]* 0.00256);
		uart_puts(UART_NUM,buffer);
#endif

#if 0
		Mag_getADC();
		sprintf(buffer, " %2.3f %2.3f %2.3f \r\n",	(float)imu.magADC[ROLL] 	* 0.00256,
													(float)imu.magADC[PITCH] 	* 0.00256,
													(float)imu.magADC[YAW] 		* 0.00256);
		uart_puts(UART_NUM,buffer);
#endif

#if 0
		/*
		 * angle[row],
		 * angle[pitch],
		 * angle[yaw],
		 * gyro_raw[row],
		 * gyro_raw[pitch],
		 * gyro_raw[yaw]
		 */
		sprintf(buffer, " %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f \r\n",	angle[ROLL],
													angle[PITCH],
													angle[YAW],
													rate[ROLL],
													rate[PITCH],
													rate[YAW]);
		uart_puts(UART_NUM,buffer);
#endif

#if 0
		sprintf(buffer, " %d \r\n",	accel_data[0]);
		uart_puts(UART_NUM,buffer);
#endif

#if 0
		sprintf(buffer, " %d\r\n", giro_data[0]);
		uart_puts(UART_NUM,buffer);
#endif


		stateUpdate(rate);
		kalmanUpdate(angle);


}
