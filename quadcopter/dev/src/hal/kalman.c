/*
 * kalman.c
 *
 *  Created on: Feb 28, 2014
 *      Author: root
 */


/***************************************************************************/
/* kalman.c                                                                */
/* 1-D Kalman filter Algorithm, using an inclinometer and gyro             */
/* Author: Rich Chi Ooi                                                    */
/* Version: 1.0                                                            */
/* Date:30.05.2003                                                         */
/* Adapted from Trammel Hudson(hudson@rotomotion.com)                      */
/* -------------------------------                                         */
/* Compilation  procedure:                                                 */
/*       Linux                                                             */
/*      gcc68 -c  XXXXXX.c (to create object file)                         */
/*      gcc68 -o  XXXXXX.hex XXXXXX.o ppwa.o                               */
/*Upload data :								   */
/*	ul filename.txt				      		           */
/***************************************************************************/
/* In this version:                                                        */
/***************************************************************************/
/* This is a free software; you can redistribute it and/or modify          */
/* it under the terms of the GNU General Public License as published       */
/* by the Free Software Foundation; either version 2 of the License,       */
/* or (at your option) any later version.                                  */
/*                                                                         */
/* this code is distributed in the hope that it will be useful,            */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of          */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           */
/* GNU General Public License for more details.                            */
/*                                                                         */
/* You should have received a copy of the GNU General Public License       */
/* along with Autopilot; if not, write to the Free Software                */
/* Foundation, Inc., 59 Temple Place, Suite 330, Boston,                   */
/* MA  02111-1307  USA                                                     */
/***************************************************************************/

#include <math.h>
#include <kalman.h>
#include <uart.h>
extern char buffer[40];

/*
 * The state is updated with gyro rate measurement every 20ms
 * change this value if you update at a different rate.
 */

static const float dt = 0.02;

/*
 * The covariance matrix.This is updated at every time step to
 * determine how well the sensors are tracking the actual state.
 */
static float P[2][2] = {{ 1, 0 },
	                { 0, 1 }};

/*
 * Our two states, the angle and the gyro bias.As a byproduct of computing
 * the angle, we also have an unbiased angular rate available.These are
 * read-only to the user of the module.
 */
float angle;
float q_bias;
float rate;


/*
 * The R represents the measurement covariance noise.R=E[vvT]
 * In this case,it is a 1x1 matrix that says that we expect
 * 0.1 rad jitter from the inclinometer.
 * for a 1x1 matrix in this case v = 0.1
 */
static const float R_angle = 0.001 ;


/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the inclinometer
 * relative to the gyros.
 */
static const float Q_angle = 0.001;
static const float Q_gyro  = 0.0015;


/*
 * state_update is called every dt with a biased gyro measurement
 * by the user of the module.  It updates the current angle and
 * rate estimate.
 *
 * The pitch gyro measurement should be scaled into real units, but
 * does not need any bias removal.  The filter will track the bias.
 *
 *	  A = [ 0 -1 ]
 *	      [ 0  0 ]
 */
void stateUpdate(const float q_m){

	float q;
	float Pdot[4];

	/* Unbias our gyro */
	q = q_m - q_bias;

	/*
	 * Compute the derivative of the covariance matrix
	 * (equation 22-1)
	 *	Pdot = A*P + P*A' + Q
	 *
	 */
	Pdot[0] = Q_angle - P[0][1] - P[1][0];	/* 0,0 */
	Pdot[1] = - P[1][1];		        /* 0,1 */
	Pdot[2] = - P[1][1];	         	/* 1,0 */
	Pdot[3] = Q_gyro;			/* 1,1 */

	/* Store our unbias gyro estimate */
	rate = q;

	/*
	 * Update our angle estimate
	 * angle += angle_dot * dt
	 *       += (gyro - gyro_bias) * dt
	 *       += q * dt
	 */
	angle += q * dt;

	/* Update the covariance matrix */
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
}


/*
 * kalman_update is called by a user of the module when a new
 * inclinoometer measurement is available.
 *
 * This does not need to be called every time step, but can be if
 * the accelerometer data are available at the same rate as the
 * rate gyro measurement.
 *
 *         H  = [ 1 0 ]
 *
 * because the angle measurement directly corresponds to the angle
 * estimate and the angle measurement has no relation to the gyro
 * bias.
 */
void kalmanUpdate(const float incAngle)
{
	/* Compute our measured angle and the error in our estimate */
	float angle_m = incAngle;
	float angle_err = angle_m - angle;

	/*
	 * h_0 shows how the state measurement directly relates to
	 * the state estimate.
 	 *
	 *      H = [h_0 h_1]
	 *
	 * The h_1 shows that the state measurement does not relate
	 * to the gyro bias estimate.  We don't actually use this, so
	 * we comment it out.
	 */
	float h_0 = 1;
	/* const float		h_1 = 0; */

	/*
	 * Precompute PH' as the term is used twice
	 * Note that H[0,1] = h_1 is zero, so that term is not not computed
	 */
	const float PHt_0 = h_0*P[0][0]; /* + h_1*P[0][1] = 0*/
	const float PHt_1 = h_0*P[1][0]; /* + h_1*P[1][1] = 0*/

	/*
	 * Compute the error estimate:
	 * (equation 21-1)
	 *
	 *	E = H P H' + R
	 */
	float E = R_angle +(h_0 * PHt_0);

	/*
	 * Compute the Kalman filter gains:
	 * (equation 21-2)
	 *
	 *	K = P H' inv(E)
       	 */
	float K_0 = PHt_0 / E;
	float K_1 = PHt_1 / E;

	/*
	 * Update covariance matrix:
	 * (equation 21-3)
	 *
	 *	P = P - K H P
         * Let
	 *      Y = H P
	 */
	float Y_0 = PHt_0;  /*h_0 * P[0][0]*/
	float Y_1 = h_0 * P[0][1];

	P[0][0] -= K_0 * Y_0;
	P[0][1] -= K_0 * Y_1;
	P[1][0] -= K_1 * Y_0;
	P[1][1] -= K_1 * Y_1;

	/*
	 * Update our state estimate:
	 *
	 *	Xnew = X + K * error
	 *
	 * err is a measurement of the difference in the measured state
	 * and the estimate state.  In our case, it is just the difference
	 * between the inclinometer measured angle and the estimated angle.
	 */
	angle	+= K_0 * angle_err;
	q_bias	+= K_1 * angle_err;

	sprintf(buffer, "angle: %d, q_bias: %d \r\n",angle, q_bias);
	uart_puts(UART_NUM,buffer);
}

void init_kalman(void){

#if 1 //16,372ms
	TCNT0  = 0x00;
	TCCR0A = 1 << WGM01;	// CTC Mode
	TCCR0B = (1<<CS02) | (1<<CS00);
	OCR0A = 255;
	TIMSK0  |= 1<<OCIE0A;
	sei();				// enable interrupts
#endif

}

#if 1
SIGNAL (TIMER0_COMPA_vect)
{

	uart_putc (UART, 10);

}
#else
SIGNAL (TIMER2_COMPA_vect)
{

	uart_putc (UART, 10);
}
#endif
