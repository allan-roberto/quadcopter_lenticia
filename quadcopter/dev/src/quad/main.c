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


char buffer[40] =  "some characters";

uint16_t i = 0;
//extern gyroZero[3];
#define ANGLE 1


uint16_t giro_data[3] 	= {0, 0, 0};
uint16_t accel_data[3] 	= {0, 0, 0};


int main(void)
{
	uartInit(UART_NUM,UBRR_VAL);
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

		gyro_get_raw_data(&giro_data[0],&giro_data[1],&giro_data[2]);
		accelerometer_get_raw_data(&accel_data[0],&accel_data[1],&accel_data[2]);

#if 0
		sprintf(buffer, " %d %d %d %d %d %d \r\n",	accel_data[0], giro_data[0],
													accel_data[1], giro_data[1],
													accel_data[2], giro_data[02]);
#endif

#if 1
		sprintf(buffer, " %d %d %d %d %d %d \r\n",	accel_data[0],
													accel_data[1],
													accel_data[2],
													giro_data [0],
													giro_data [1],
													giro_data [02]);
#endif

#if 0
		sprintf(buffer, " %d\r\n", giro_data[0]);
#endif
		uart_puts(UART_NUM,buffer);


}
