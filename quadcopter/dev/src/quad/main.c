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


char buffer[256] =  "some characters";
uint8_t kalman_enabled = 0;
uint8_t gps_enabled = 0;
uint16_t i = 0;
#define ANGLE 1

fix16_t rate[3] 	= {0.0, 0.0, 0.0};
fix16_t angle[3] 	= {0.0, 0.0, 0.0};

imu_t imu;
global_conf_t global_conf;
conf_t conf;
uint8_t rawADC[6];
uint32_t currentTime;
flags_struct_t f;

extern int16_t  GPS_angle[2];
extern int32_t  GPS_coord[2];
extern int32_t  GPS_home[2];
extern int32_t  GPS_hold[2];
extern uint8_t  GPS_numSat;

extern uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
extern uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
extern uint8_t  GPS_update;                              // a binary toogle to distinct a GPS position update
extern uint16_t GPS_ground_course;                       //                   - unit: degree*10
extern uint8_t  GPS_Present;                             // Checksum from Gps serial
extern uint8_t  GPS_Enable;


int main(void)
{
	uartInit(0,SERIAL0_COM_SPEED);
	i2c_init();
	init_imu();
	init_kalman_matrices();
	init_kalman_timer();
	CONFIG_LED_MODE;

	sei();

	GPS_SerialInit();

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




	while(1){


		if(gps_enabled){

			GPS_NewData();

			sprintf(buffer, "GPS_coord[LAT]:%d \n\r",GPS_coord[LAT]);
			uart_puts (UART_NUM,buffer);

			sprintf(buffer, "GPS_coord[LON]:%d \n\r",GPS_coord[LON]);
			uart_puts (UART_NUM,buffer);

			sprintf(buffer, "GPS_altitude: %d \n\r",GPS_altitude);
			uart_puts (UART_NUM,buffer);

			sprintf(buffer, "GPS_speed: %d \n\r",GPS_speed);
			uart_puts (UART_NUM,buffer);

			sprintf(buffer, "GPS_ground_course: %d \n\r",GPS_ground_course);
			uart_puts (UART_NUM,buffer);

			sprintf(buffer, "GPS_numSat: %d \n\r",GPS_numSat);
			uart_puts (UART_NUM,buffer);

			sprintf(buffer, "GPS_Present: %d \n\r",GPS_Present);
			uart_puts (UART_NUM,buffer);


			gps_enabled = 0;
		}
		if(kalman_enabled){

			imu_get_rates(rate);
			imu_get_angles(angle);
			stateUpdate(rate);
			kalmanUpdate(angle);
			#ifdef DEBUG_KALMAN_MATRICES_STATE
				print_matrices();
			#endif
			kalman_enabled = 0;

		}

	}

	return 0;
}

SIGNAL (TIMER0_COMPA_vect)
{
	//kalman_enabled = 1;
	i++;
	if(i>100){
		gps_enabled = 1;
		i = 0;
	}

}
