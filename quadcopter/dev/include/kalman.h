/*
 * kalman.h
 *
 *  Created on: 16/05/2014
 *      Author: allan_amorim
 */

#ifndef KALMAN_H_
#define KALMAN_H_

void kalmanUpdate(const float incAngle);
void stateUpdate(const float q_m);

#endif /* KALMAN_H_ */
