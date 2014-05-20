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


char buffer[100] =  "some characters";

uint16_t i = 0;
//extern gyroZero[3];
#define ANGLE 1


uint16_t giro_data[3] 	= {0, 0, 0};
uint16_t accel_data[3] 	= {0, 0, 0};
float angle_[3] 	= {0.0, 0.0, 0.0};


int main(void)
{
	uartInit(UART_NUM,230400);
	i2c_init();
	init_gyro();
	init_accelerometer();
	init_kalman_timer();
	sei();
	while(1){

	}

	return 0;
}

SIGNAL (TIMER0_COMPA_vect)
{

#if 1
		gyro_get_raw_data(&giro_data[0],&giro_data[1],&giro_data[2]);
		accelerometer_get_angles(&angle_[ROLL], &angle_[PITCH], &angle_[YAW]);
#endif

#if 0
		sprintf(buffer, " %d %d %d %d %d %d \r\n",	accel_data[0], giro_data[0],
													accel_data[1], giro_data[1],
													accel_data[2], giro_data[02]);
		uart_puts(UART_NUM,buffer);
#endif

#if 1
		/*
		 * angle[row],
		 * angle[pitch],
		 * angle[yaw],
		 * gyro_raw[row],
		 * gyro_raw[pitch],
		 * gyro_raw[yaw]
		 */
		sprintf(buffer, " %3.2f %3.2f %3.2f %d %d %d \r\n",	angle_[0],
													angle_[1],
													angle_[2],
													giro_data [0],
													giro_data [1],
													giro_data [2]);
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


		//stateUpdate(giro_data[1]);

		//kalmanUpdate(angle_);


}
