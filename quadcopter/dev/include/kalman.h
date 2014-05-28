/*
 * kalman.h
 *
 *  Created on: 16/05/2014
 *      Author: allan_amorim
 */

#ifndef KALMAN_H_
#define KALMAN_H_


void stateUpdate(double rate[]);
void kalmanUpdate(double angle[]);
void print_matrix(double * A, int m, int n, char *string);

#endif /* KALMAN_H_ */
