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
static inline void print_matrix(fix16_t *A, int m, int n, char *string, int print);
static inline void mult_matrix(fix16_t *prod, fix16_t *MAT1, fix16_t *MAT2,int mat1_linha,int mat1_coluna,int mat2_linha, int mat2_coluna,int print_enable);
static inline void init_matrix(fix16_t *A, int m, int n);
static inline void sum_matrix(fix16_t *sum, fix16_t *MAT1, fix16_t *MAT2,int mat_linha,int mat_coluna,int print_enable);
static inline void subtract_matrix(fix16_t *sub, fix16_t *MAT1, fix16_t *MAT2,int mat_linha,int mat_coluna,int print_enable);
#endif /* KALMAN_H_ */
