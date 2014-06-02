/*
 * kalman.h
 *
 *  Created on: 16/05/2014
 *      Author: allan_amorim
 */

#ifndef KALMAN_H_
#define KALMAN_H_
#include <stdio.h>
#include <fix16.h>

void stateUpdate(fix16_t rate[]);
void kalmanUpdate(fix16_t angle[]);
inline void print_matrix(fix16_t *A, int m, int n, char *string, int print);
inline void mult_matrix(fix16_t *prod, fix16_t *MAT1, fix16_t *MAT2,int mat1_linha,int mat1_coluna,int mat2_linha, int mat2_coluna,int print_enable);
inline void init_matrix(fix16_t *A, int m, int n);
inline void sum_matrix(fix16_t *sum, fix16_t *MAT1, fix16_t *MAT2,int mat_linha,int mat_coluna,int print_enable);
inline void subtract_matrix(fix16_t *sub, fix16_t *MAT1, fix16_t *MAT2,int mat_linha,int mat_coluna,int print_enable);
void init_kalman_matrices(void);
inline void print_matrices(void);

fix16_t A[6][6];
fix16_t AT[6][6];
fix16_t B[6][3];
fix16_t H[3][6];
fix16_t HT[6][3];
fix16_t Q[6][6];
fix16_t R[3][3];
fix16_t kalman_gain[6][3];
fix16_t p_predicted[6][6];
fix16_t p_updated[6][6];
fix16_t x_predicted[6];
fix16_t x_updated[6];
fix16_t I[6][6];
fix16_t u[3]; //Atualizar essa vari'avel com o gyro
fix16_t z[3]; //Atualizar essa vari'avel com o accel

//#define DEBUG_KALMAN_MATRICES_STATE
#define PRINT_ENABLE 0
//#define DEBUG_KALMAN_INPUTS
#define DEBUG_KALMAN_OUPUT
//#define DEBUG_KALMAN_EQUATIONS
//#define DEBUG_KALMAN_MULT_MATRIX

#endif /* KALMAN_H_ */
