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

fix16_t rate[3] 	= {0.0, 0.0, 0.0};
fix16_t angle[3] 	= {0.0, 0.0, 0.0};

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
	init_kalman_matrices();
	init_kalman_timer();

#if 0
	x_updated[0] = fix16_from_dbl(500);
	x_updated[1] = fix16_from_dbl(-377);
	x_updated[2] = fix16_from_dbl(200);
	x_updated[3] = fix16_from_dbl(4440);
	x_updated[4] = fix16_from_dbl(0);
	x_updated[5] = fix16_from_dbl(0.01);

	A[0][0] = fix16_from_dbl(1);
	A[0][1] = fix16_from_dbl(-0.01);
	A[1][1] = fix16_from_dbl(1);
	A[2][2] = fix16_from_dbl(1);
	A[2][3] = fix16_from_dbl(-0.01);
	A[3][3] = fix16_from_dbl(1);
	A[4][4] = fix16_from_dbl(1);
	A[4][5] = fix16_from_dbl(-0.01);
	A[5][5] = fix16_from_dbl(1);

	mult_matrix((fix16_t *)tmp1,(fix16_t *)A,(fix16_t *)x_updated,6,6,6,1,1);
#endif



	sei();
	while(1){

	}

	return 0;
}

SIGNAL (TIMER0_COMPA_vect)
{

	imu_get_rates(rate);
	imu_get_angles(angle);
	stateUpdate(rate);
	kalmanUpdate(angle);
#ifdef DEBUG_KALMAN_MATRICES_STATE
	print_matrices();
#endif



}
