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
void print_matrix(double *A, int m, int n, char *string, int print);
void mult_matrix(double *prod, double *MAT1, double *MAT2,int mat1_linha,int mat1_coluna,int mat2_linha, int mat2_coluna,int print_enable);
void init_matrix(double *A, int m, int n);
void sum_matrix(double *sum, double *MAT1, double *MAT2,int mat_linha,int mat_coluna,int print_enable);
void subtract_matrix(double *sub, double *MAT1, double *MAT2,int mat_linha,int mat_coluna,int print_enable);
#endif /* KALMAN_H_ */
