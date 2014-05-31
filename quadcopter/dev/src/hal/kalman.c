/*
 * kalman.c
 *
 *  Created on: Feb 28, 2014
 *      Author: root
 */

#include <math.h>
#include <kalman.h>
#include <uart.h>
#include <stdio.h>
#include <fix16.h>



#define TETA 		0
#define TETA_BIAS 	1
#define GAMA 		2
#define GAMA_BIAS 	3
#define PHI 		4
#define PHI_BIAS 	5

#define OMEGA_TETA	0
#define OMEGA_GAMA	1
#define OMEGA_PHI	2

#define PRINT_ENABLE 1
#define PRINT_DISABLE 0

extern char buffer[200];

#define dt 0.01

fix16_t p_predicted[6][6];
fix16_t p_updated[6][6];
fix16_t I[6][6];
fix16_t x_predicted[6] = { 0, 0, 0, 0, 0, 0 };
fix16_t x_updated[6] = { 0, 0, 0, 0, 0, 0 };

fix16_t R[3][3];
fix16_t Q[6][6];
fix16_t A[6][6];
fix16_t AT[6][6];
fix16_t B[6][3];
fix16_t kalman_gain[6][3];
fix16_t H[3][6];
fix16_t HT[6][3];

void stateUpdate(fix16_t rate[]){
	//uint8_t lin = 0, col = 0, k = 0;

	fix16_t u[3] = {1,1,1}; //Atualizar essa vari'avel com o gyro
	fix16_t tmp[6][6];
	init_matrix((fix16_t *)tmp,6,6);
	fix16_t tmp1[6][1];
	init_matrix((fix16_t *)tmp1,6,1);
	fix16_t tmp2[6][1];
	init_matrix((fix16_t *)tmp2,6,1);
	u[0] = rate[0];
	u[1] = rate[1];
	u[2] = rate[2];


	/*
	 * %Time Update (“Predict”)
	 * % Project the state ahead
	 * x_predicted(:,i)  = A * x_updated(:,i-1) +  B * u(:,i-1);   #u(i) == giro_rw_data(i)
	 */

	mult_matrix((fix16_t *)tmp1,(fix16_t *)A,(fix16_t *)x_updated,6,6,6,1,PRINT_DISABLE);
	mult_matrix((fix16_t *)tmp2,(fix16_t *)B,(fix16_t *)u,6,3,3,1,PRINT_DISABLE);
	sum_matrix((fix16_t *)x_predicted,(fix16_t *)tmp1,(fix16_t *)tmp2,6,1,PRINT_DISABLE);

	/*
	 *  % Project the error covariance ahead
	 *  p_predicted  = A * p_updated * A' + Q;
	 *
	*/

	mult_matrix((fix16_t *)tmp,(fix16_t *)A,(fix16_t *)p_updated,6,6,6,6,PRINT_DISABLE);
	mult_matrix((fix16_t *)p_predicted,(fix16_t *)tmp,(fix16_t *)AT,6,6,6,6,PRINT_DISABLE);
	mult_matrix((fix16_t *)p_predicted,(fix16_t *)tmp,(fix16_t *)Q,6,6,6,6,PRINT_DISABLE);



}


/*
 * kalman_update is called by a user of the module when a new
 * inclinometer measurement is available.
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
void kalmanUpdate(fix16_t angle[])
{
	/**********************************************************************/
	/*
	 *   %Measurement Update (“Correct”
	 *   %Compute the Kalman gain
	 *   kalman_gain  = p_predicted*H' * inv(H*p_predicted*H' + R);
	 */

	fix16_t z[3] = {1,1,1}; //Atualizar essa vari'avel com o accel
	z[0] = angle[0];
	z[1] = angle[1];
	z[2] = angle[2];

	fix16_t tmp[3][6];
	fix16_t tmp2[3][3];
	fix16_t tmp3[6][3];
	fix16_t tmp4[3];
	fix16_t tmp5[6];
	fix16_t tmp6[6][6];

	init_matrix((fix16_t *)tmp,3,6);
	init_matrix((fix16_t *)tmp2,3,3);
	init_matrix((fix16_t *)tmp3,6,3);
	init_matrix((fix16_t *)tmp4,3,1);
	init_matrix((fix16_t *)tmp5,6,1);
	init_matrix((fix16_t *)tmp6,6,6);

	mult_matrix((fix16_t *)tmp,(fix16_t *)H,(fix16_t *)p_predicted,3,6,6,6,PRINT_DISABLE);

	mult_matrix((fix16_t *)tmp2,(fix16_t *)tmp,(fix16_t *)HT,3,6,6,3,PRINT_DISABLE);

	sum_matrix((fix16_t *)tmp2,(fix16_t *)tmp2,(fix16_t *)R,3,3,PRINT_DISABLE);
	//return;

	//inv(H*p_predicted*H' + R)
	tmp2[0][0] = 1/(tmp2[0][0]);
	tmp2[1][1] = 1/(tmp2[1][1]);
	tmp2[2][2] = 1/(tmp2[2][2]);


	mult_matrix((fix16_t *)tmp3,(fix16_t *)p_predicted,(fix16_t *)HT,6,6,6,3,PRINT_DISABLE);

	mult_matrix((fix16_t *)kalman_gain,(fix16_t *)tmp3,(fix16_t *)tmp2,6,3,3,3,PRINT_DISABLE);

	/**********************************************************************/
	/*
	 *   %Update estimate with measurement zk
	 *   x_updated(:,i) = x_predicted(:,i) + kalman_gain*(z(:,i) - H * x_predicted(:,i));
	 */

	mult_matrix((fix16_t *)tmp4,(fix16_t *)H,(fix16_t *)x_predicted,3,6,6,1,PRINT_DISABLE);

	subtract_matrix(tmp4,z,tmp4,3,1,PRINT_DISABLE);

	mult_matrix((fix16_t *)tmp5,(fix16_t *)kalman_gain,(fix16_t *)tmp4,6,3,3,1,PRINT_DISABLE);

	sum_matrix((fix16_t *)x_updated,(fix16_t *)x_predicted,(fix16_t *)tmp5,6,1,PRINT_DISABLE);

	/**********************************************************************/
	/*
	 *   %Update the error covariance
	 *   p_updated  = (eye(6,6) - kalman_gain * H) * p_predicted;
	 */

	mult_matrix((fix16_t *)tmp6,(fix16_t *)kalman_gain,(fix16_t *)H,6,3,3,6,PRINT_DISABLE);

	subtract_matrix((fix16_t *)tmp6,(fix16_t *)I,(fix16_t *)tmp6,6,6,PRINT_DISABLE);

	mult_matrix((fix16_t *)p_updated,(fix16_t *)tmp6,(fix16_t *)p_predicted,6,6,6,6,PRINT_DISABLE);

	sprintf(buffer, " %2.2f %2.2f %2.2f \r\n",fix16_to_dbl(x_updated[0]), fix16_to_dbl(x_updated[2]),fix16_to_dbl(x_updated[4]));
	uart_puts(UART_NUM,buffer);
}

static inline void  print_matrix(fix16_t *A, int m, int n, char *string, int print){
    // A = input matrix (m x n)
	if(!print) return;
    int i,j;
    uart_puts(UART_NUM,string);
    sprintf(buffer, "\r\n"); uart_puts(UART_NUM,buffer);
    for (i=0; i<m; i++){
        for (j=0;j<n;j++){
        		sprintf(buffer,"%3.2f ",(double)A[(i*n)+j]); uart_puts(UART_NUM,buffer);
        		sprintf(buffer, "\t"); uart_puts(UART_NUM,buffer);
          }
        sprintf(buffer, "\r\n"); uart_puts(UART_NUM,buffer);
    }
}

static inline void  mult_matrix(fix16_t *prod, fix16_t *MAT1, fix16_t *MAT2,int mat1_linha,int mat1_coluna,int mat2_linha, int mat2_coluna,int print_enable){
	fix16_t lin,col,k,tmp_prod;
	print_matrix(MAT1,mat1_linha,mat1_coluna,"MAT1",print_enable);
	print_matrix(MAT2,mat2_linha,mat2_coluna,"MAT2",print_enable);
	for(lin = 0 ; lin < mat1_linha ; lin++){
		for(col = 0 ; col < mat2_coluna ; col++){
			for(k=0;k<mat1_coluna;k++){

				if( MAT1[(lin*mat1_coluna) + k] && MAT2[(col*mat2_linha) + k]){
					tmp_prod = fix16_mul( MAT1[(lin*mat1_coluna) + k],MAT2[(col*mat2_linha) + k]);
					prod[(lin*mat1_coluna) + col] = fix16_add(prod[(lin*mat1_coluna) + col],tmp_prod);
				}
			}

		}
	}
	print_matrix(prod,mat1_linha,mat2_coluna,"prod",print_enable);
}
static inline void  sum_matrix(fix16_t *sum, fix16_t *MAT1, fix16_t *MAT2,int mat_linha,int mat_coluna,int print_enable){
	int lin,col;
	print_matrix(MAT1,mat_linha,mat_coluna,"MAT1",print_enable);
	print_matrix(MAT2,mat_linha,mat_coluna,"MAT2",print_enable);
	for(lin = 0 ; lin < mat_linha ; lin++){
		for(col = 0 ; col < mat_coluna ; col++){
				sum[(lin*mat_coluna) + col] = fix16_add(MAT1[(lin*mat_coluna) + col], MAT2[(lin*mat_coluna) + col]);

		}
	}
	print_matrix(sum,mat_linha,mat_coluna,"sum",print_enable);
}

static inline void subtract_matrix(fix16_t *sub, fix16_t *MAT1, fix16_t *MAT2,int mat_linha,int mat_coluna,int print_enable){
	int lin,col;
	print_matrix(MAT1,mat_linha,mat_coluna,"MAT1",print_enable);
	print_matrix(MAT2,mat_linha,mat_coluna,"MAT2",print_enable);
	for(lin = 0 ; lin < mat_linha ; lin++){
		for(col = 0 ; col < mat_coluna ; col++){

			sub[(lin*mat_coluna) + col] = fix16_ssub(MAT1[(lin*mat_coluna) + col], MAT2[(lin*mat_coluna) + col]);
		}
	}
	print_matrix(sub,mat_linha,mat_coluna,"sub",print_enable);
}
static inline void init_matrix(fix16_t *A, int m, int n){

    int i,j;
    for (i=0; i<m; i++){
        for (j=0;j<n;j++){
        		A[(i*n)+j] = 0;
          }
    }
}
void init_kalman_matrices(void){

	/*fix16_t p_predicted[6][6] ={{ 10.0,      0.0,    0.0,    0.0,    0.0,    0.0, },
								{  0.0, 	 10.0,   0.0,    0.0,    0.0,    0.0, },
								{  0.0, 	 0.0, 	 10.0,   0.0,    0.0,    0.0, },
								{  0.0, 	 0.0,    0.0,    10.0,   0.0,    0.0, },
								{  0.0, 	 0.0,    0.0,    0.0,    10.0,   0.0, },
								{  0.0, 	 0.0,    0.0,    0.0,    0.0,    10.0,}};*/

	p_predicted[0][0] = fix16_from_int(100);
	p_predicted[1][1] = fix16_from_int(100);
	p_predicted[2][2] = fix16_from_int(100);
	p_predicted[3][3] = fix16_from_int(100);
	p_predicted[4][4] = fix16_from_int(100);
	p_predicted[5][5] = fix16_from_int(100);


	/*fix16_t p_updated[6][6]   = {	{ 10.0, 0, 0, 0, 0, 0 },
		                				{ 0, 10.0, 0, 0, 0, 0 },
		                				{ 0, 0, 10.0, 0, 0, 0 },
		                				{ 0, 0, 0, 10.0, 0, 0 },
		                				{ 0, 0, 0, 0, 10.0, 0 },
		                				{ 0, 0, 0, 0, 0, 10.0 }};*/

	p_updated[0][0] = fix16_from_int(100);
	p_updated[1][1] = fix16_from_int(100);
	p_updated[2][2] = fix16_from_int(100);
	p_updated[3][3] = fix16_from_int(100);
	p_updated[4][4] = fix16_from_int(100);
	p_updated[5][5] = fix16_from_int(100);

	/*fix16_t I[6][6] = {{ 1, 0, 0, 0, 0, 0 },
		                	{ 0, 1, 0, 0, 0, 0 },
		                	{ 0, 0, 1, 0, 0, 0 },
		                	{ 0, 0, 0, 1, 0, 0 },
		                	{ 0, 0, 0, 0, 1, 0 },
		                	{ 0, 0, 0, 0, 0, 1 }};*/

	I[0][0] = fix16_from_int(1);
	I[1][1] = fix16_from_int(1);
	I[2][2] = fix16_from_int(1);
	I[3][3] = fix16_from_int(1);
	I[4][4] = fix16_from_int(1);
	I[5][5] = fix16_from_int(1);


	/*
	 * Our two states(x_predicted), the angle and the gyro bias.As a byproduct of computing
	 * the angle, we also have an unbiased angular rate available.These are
	 * read-only to the user of the module.
	 */
	//fix16_t x_predicted[6] = { 0, 0, 0, 0, 0, 0 };
	//fix16_t x_updated[6] = { 0, 0, 0, 0, 0, 0 };

	//double p_predicted[6][6];
	//double p_updated[6][6];
	/*
	 * The R represents the measurement covariance noise.R=E[vvT]

	fix16_t R[3][3] = {{ 1, 0, 0},
		                	{ 0, 1, 0},
		                	{ 0, 0, 1}};*/


	R[0][0] = fix16_from_int(1);
	R[1][1] = fix16_from_int(1);
	R[2][2] = fix16_from_int(1);
	/*
	 * Q is a 2x2 matrix that represents the process covariance noise.
	 * In this case, it indicates how much we trust the inclinometer
	 * relative to the gyros.

	fix16_t Q[6][6] ={		{0.1, 0,  	 0,    	 0,   	 0,   	 0},
			 	 	 	 	{0,    	 0.1, 0,      0,      0,      0},
			 	 	 	 	{0,    	 0,  	 0.5758, 0,      0,      0},
			 	 	 	 	{0,    	 0,    	 0,      0.1926, 0,      0},
			 	 	 	 	{0,    	 0,    	 0,    	 0,      0.2057, 0},
			 	 	 	 	{0,      0,    	 0,   	 0,      0,      0.6161}};*/

	Q[0][0] = fix16_from_float(0.1);
	Q[1][1] = fix16_from_float(0.1);
	Q[2][2] = fix16_from_float(0.5758);
	Q[3][3] = fix16_from_float(0.1926);
	Q[4][4] = fix16_from_float(0.2057);
	Q[5][5] = fix16_from_float(0.6161);

	/*fix16_t A[6][6] = {	 {  1, 	 	-dt,      0,       0,     0,       0 },
	       	  	  	  	  	 {  0,       1,       0,       0,     0,       0 },
	       	  	  	  	  	 {  0,       0,       1, 	  -dt,    0,       0 },
	       	  	  	  	  	 {  0,       0,       0,       1,     0,       0 },
	       	  	  	  	  	 {  0,       0,       0,       0,     1, 	  -dt},
	       	  	  	  	  	 {  0,       0,       0,       0,     0,       1 }};*/
	A[0][0] = fix16_from_int(1);
	A[0][1] = fix16_from_float(-dt);
	A[1][1] = fix16_from_int(1);
	A[2][2] = fix16_from_int(1);
	A[2][3] = fix16_from_float(-dt);
	A[3][3] = fix16_from_int(1);
	A[4][4] = fix16_from_int(1);
	A[4][5] = fix16_from_float(-dt);
	A[5][5] = fix16_from_int(1);


	/*fix16_t AT[6][6] = 	 {{  1, 	  0,       0,       0,     0,       0 },
	       	  	  	  	  	  { -dt,      1,       0,       0,     0,       0 },
	       	  	  	  	  	  {  0,       0,       1, 	    0,     0,       0 },
	       	  	  	  	  	  {  0,       0,      -dt,      1,     0,       0 },
	       	  	  	  	  	  {  0,       0,       0,       0,     1, 	   	0 },
	       	  	  	  	  	  {  0,       0,       0,       0,    -dt,      1 }};*/

	AT[0][0] = fix16_from_int(1);
	AT[1][0] = fix16_from_int(-dt);
	AT[1][1] = fix16_from_int(1);
	AT[2][2] = fix16_from_int(1);
	AT[3][2] = fix16_from_int(1);
	AT[3][3] = fix16_from_int(-dt);
	AT[4][4] = fix16_from_int(1);
	AT[5][4] = fix16_from_int(-dt);
	AT[5][5] = fix16_from_int(1);



	/*fix16_t B[6][3]= 	   {{dt,  0,  0 },
	 	                	{ 0,  0,  0 },
	 	                	{ 0,  dt, 0 },
	 	                	{ 0,  0,  0 },
	 	                	{ 0,  0,  dt},
	 	                	{ 0,  0,  0 }};*/

	B[0][0] = fix16_from_float(dt);
	B[2][1] = fix16_from_float(dt);
	B[4][2] = fix16_from_float(dt);



	/*fix16_t H[3][6] =		{{1, 	 0,  	 0,    	 0,   	 0,   	 0},
			 	 	 	 	{0,    	 0, 	 1,      0,      0,      0},
			 	 	 	 	{0,    	 0,  	 0, 	 0,      1,      0}};*/

	H[0][0] = fix16_from_int(1);
	H[1][2] = fix16_from_int(1);
	H[2][4] = fix16_from_int(1);

	/*fix16_t HT[6][3] =	{{1, 	 0,  	 0},
			 	 	 	 	 {0,   	 0,   	 0},
			 	 	 	 	 {0, 	 1, 	 0},
			 	 	 	 	 {0,     0,      0},
			 	 	 	 	 {0, 	 0,  	 1},
	 	 	 	 	 	 	 {0,     0,      0}};*/

	HT[0][0] = fix16_from_int(1);
	HT[2][1] = fix16_from_int(1);;
	HT[4][2] = fix16_from_int(1);;

}
