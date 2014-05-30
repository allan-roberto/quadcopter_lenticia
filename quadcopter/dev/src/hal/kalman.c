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

/*
 * The state is updated with gyro rate measurement every 10ms
 * change this value if you update at a different rate.
 */
#define dt 0.01

/*
 * The covariance matrix.This is updated at every time step to
 * determine how well the sensors are tracking the actual state.
 */
double p_predicted[6][6] = {{ 100.0,  0.0,    0.0,    0.0,    0.0,    0.0, },
							{ 0.0, 	 100.0,   0.0,    0.0,    0.0,    0.0, },
							{ 0.0, 	 0.0, 	 100.0,   0.0,    0.0,    0.0, },
							{ 0.0, 	 0.0,    0.0,    100.0,   0.0,    0.0, },
							{ 0.0, 	 0.0,    0.0,    0.0,    100.0,   0.0, },
							{ 0.0, 	 0.0,    0.0,    0.0,    0.0,    100.0,}};

 double p_updated[6][6]   = {	{ 100.0, 0, 0, 0, 0, 0 },
	                				{ 0, 100.0, 0, 0, 0, 0 },
	                				{ 0, 0, 100.0, 0, 0, 0 },
	                				{ 0, 0, 0, 100.0, 0, 0 },
	                				{ 0, 0, 0, 0, 100.0, 0 },
	                				{ 0, 0, 0, 0, 0, 100.0 }};

 const double I[6][6] = {{ 1, 0, 0, 0, 0, 0 },
	                	{ 0, 1, 0, 0, 0, 0 },
	                	{ 0, 0, 1, 0, 0, 0 },
	                	{ 0, 0, 0, 1, 0, 0 },
	                	{ 0, 0, 0, 0, 1, 0 },
	                	{ 0, 0, 0, 0, 0, 1 }};
/*
 * Our two states(x_predicted), the angle and the gyro bias.As a byproduct of computing
 * the angle, we also have an unbiased angular rate available.These are
 * read-only to the user of the module.
 */
static double x_predicted[6] = { 0, 0, 0, 0, 0, 0 };
static double x_updated[6] = { 0, 0, 0, 0, 0, 0 };
//double p_predicted[6][6];
//double p_updated[6][6];
/*
 * The R represents the measurement covariance noise.R=E[vvT]
 */
 const double R[3][3] = {  { 1, 0, 0},
	               	   	   { 0, 1, 0},
	               	   	   { 0, 0, 1}};


/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the inclinometer
 * relative to the gyros.
 */
  double Q[6][6] ={		{0.0046, 0,  	 	0,    	 	0,   	 	0,   	 	0},
		 	 	 	 	{0,    	 	0.0038, 	0,      	0,      	0,      	0},
		 	 	 	 	{0,    	 	0,  	 	0.5758, 	0,      	0,      	0},
		 	 	 	 	{0,    	 	0,    	 	0,      	0.1926, 	0,      	0},
		 	 	 	 	{0,    	 	0,    	 	0,    	 	0,      	0.2057, 	0},
		 	 	 	 	{0,      	0,    	 	0,   	 	0,      	0,      	0.6161}};


/*
 * state_update is called every dt with a biased gyro measurement
 * by the user of the module.  It updates the current angle and
 * rate estimate.
 *
 * The pitch gyro measurement should be scaled into real units, but
 * does not need any bias removal.  The filter will track the bias.
 */

 double A[6][6] = {{  1, 	 	-dt,      0,       0,     0,       0 },
       	  	  	  	  	 {  0,       1,       0,       0,     0,       0 },
       	  	  	  	  	 {  0,       0,       1, 	  -dt,    0,       0 },
       	  	  	  	  	 {  0,       0,       0,       1,     0,       0 },
       	  	  	  	  	 {  0,       0,       0,       0,     1, 	  -dt},
       	  	  	  	  	 {  0,       0,       0,       0,     0,       1 }};

  double AT[6][6] = 	 {{  1, 	  0,       0,       0,     0,       0 },
       	  	  	  	  	  { -dt,      1,       0,       0,     0,       0 },
       	  	  	  	  	  {  0,       0,       1, 	    0,     0,       0 },
       	  	  	  	  	  {  0,       0,      -dt,      1,     0,       0 },
       	  	  	  	  	  {  0,       0,       0,       0,     1, 	   	0 },
       	  	  	  	  	  {  0,       0,       0,       0,    -dt,      1 }};

  double B[6][3]= {{dt,  0,  0 },
 	                	{ 0,  0,  0 },
 	                	{ 0,  dt, 0 },
 	                	{ 0,  0,  0 },
 	                	{ 0,  0,  dt},
 	                	{ 0,  0,  0 }};

  double kalman_gain[6][3]= 	{	{ 0,  0,  0 },
 	                				{ 0,  0,  0 },
 	                				{ 0,  0,  0 },
 	                				{ 0,  0,  0 },
 	                				{ 0,  0,  0 },
 	                				{ 0,  0,  0 }};

  double H[3][6] =		{{1, 	 0,  	 0,    	 0,   	 0,   	 0},
		 	 	 	 	{0,    	 0, 	 1,      0,      0,      0},
		 	 	 	 	{0,    	 0,  	 0, 	 0,      1,      0}};

  double HT[6][3] =		{{1, 	 0,  	 0},
		 	 	 	 	 {0,   	 0,   	 0},
		 	 	 	 	 {0, 	 1, 	 0},
		 	 	 	 	 {0,     0,      0},
		 	 	 	 	 {0, 	 0,  	 1},
 	 	 	 	 	 	 {0,     0,      0}};

void stateUpdate(double rate[]){

	double u[3] = {1,1,1}; //Atualizar essa vari'avel com o gyro
	double tmp[6][6];
	init_matrix((double*)tmp,6,6);
	double tmp2[6][1];
	init_matrix((double*)tmp2,6,1);
	u[0] = rate[0];
	u[1] = rate[1];
	u[2] = rate[2];
	//sprintf(buffer," %3.2f %3.2f %3.2f \r\n",u[0],u[2],u[3]); uart_puts(UART_NUM,buffer);

	/*
	 * %Time Update (“Predict”)
	 * % Project the state ahead
	 * x_predicted(:,i)  = A * x_updated(:,i-1) +  B * u(:,i-1);   #u(i) == giro_rw_data(i)
	 */
	//uart_puts(UART_NUM,"x_predicted(:,i)  = A * x_updated(:,i-1) +  B * u(:,i-1);");
	mult_matrix((double*)tmp2,(double*)B,(double*)u,6,3,3,1,PRINT_DISABLE);
	mult_matrix((double*)tmp,(double*)A,(double*)x_updated,6,6,6,1,PRINT_DISABLE);
	sum_matrix((double*)x_predicted,(double*)tmp2,(double*)tmp,6,1,PRINT_DISABLE);
	init_matrix((double*)tmp,6,6);
	init_matrix((double*)tmp2,6,1);

	/*
	 *  % Project the error covariance ahead
	 *  p_predicted  = A * p_updated * A' + Q;
	 */
	//uart_puts(UART_NUM,"p_predicted  = A * p_updated * A' + Q;");
	mult_matrix((double*)tmp,(double*)A,(double*)p_updated,6,6,6,6,PRINT_DISABLE);
	mult_matrix((double*)p_predicted,(double*)tmp,(double*)AT,6,6,6,6,PRINT_DISABLE);
	sum_matrix((double*)p_predicted,(double*)p_predicted,(double*)Q,6,6,PRINT_DISABLE);
	init_matrix((double*)tmp,6,6);

}


/*
 * kalman_update is called by a user of the module when a new
 * inclinometer measurement is available.
 */
void kalmanUpdate(double angle[])
{
	/**********************************************************************/
	/*
	 *   %Measurement Update (“Correct”
	 *   %Compute the Kalman gain
	 *   kalman_gain  = p_predicted*H' * inv(H*p_predicted*H' + R);
	 */

	double z[3] = {1,1,1}; //Atualizar essa vari'avel com o accel
	z[0] = angle[0];
	z[1] = angle[1];
	z[2] = angle[2];
	//sprintf(buffer," %3.2f %3.2f %3.2f \r\n",z[0],z[2],z[3]); uart_puts(UART_NUM,buffer);

	double tmp[3][6];
	double tmp2[3][3];
	double tmp3[6][3];
	double tmp4[3];
	double tmp5[6];
	double tmp6[6][6];

	init_matrix((double*)tmp,3,6);
	init_matrix((double*)tmp2,3,3);
	init_matrix((double*)tmp3,6,3);
	init_matrix((double*)tmp4,3,1);
	init_matrix((double*)tmp5,6,1);
	init_matrix((double*)tmp6,6,6);
	//uart_puts(UART_NUM,"kalman_gain  = p_predicted*H' * inv(H*p_predicted*H' + R);");
	mult_matrix((double*)tmp,(double*)H,(double*)p_predicted,3,6,6,6,PRINT_DISABLE);
	mult_matrix((double*)tmp2,(double*)tmp,(double*)HT,3,6,6,3,PRINT_DISABLE);
	sum_matrix((double*)tmp2,(double*)tmp2,(double*)R,3,3,PRINT_DISABLE);
	;
	//inv(H*p_predicted*H' + R)
	tmp2[0][0] = 1/(tmp2[0][0]);
	tmp2[1][1] = 1/(tmp2[1][1]);
	tmp2[2][2] = 1/(tmp2[2][2]);

	mult_matrix((double*)tmp3,(double*)p_predicted,(double*)HT,6,6,6,3,PRINT_DISABLE);
	mult_matrix((double*)kalman_gain,(double*)tmp3,(double*)tmp2,6,3,3,3,PRINT_DISABLE);
	init_matrix((double*)tmp3,6,3);

	/**********************************************************************/
	/*
	 *   %Update estimate with measurement zk
	 *   x_updated(:,i) = x_predicted(:,i) + kalman_gain*(z(:,i) - H * x_predicted(:,i));
	 */
	//uart_puts(UART_NUM,"x_updated(:,i) = x_predicted(:,i) + kalman_gain*(z(:,i) - H * x_predicted(:,i));");
	mult_matrix((double*)tmp4,(double*)H,(double*)x_predicted,3,6,6,1,PRINT_DISABLE);
	subtract_matrix((double*)tmp4,(double*)z,(double*)tmp4,3,1,PRINT_DISABLE);
	mult_matrix((double*)tmp5,(double*)kalman_gain,(double*)tmp4,6,3,3,1,PRINT_DISABLE);
	sum_matrix((double*)x_updated,(double*)x_predicted,(double*)tmp5,6,1,PRINT_DISABLE);
	init_matrix((double*)tmp4,3,1);
	init_matrix((double*)tmp5,6,1);


	/**********************************************************************/
	/*
	 *   %Update the error covariance
	 *   p_updated  = (eye(6,6) - kalman_gain * H) * p_predicted;
	 */
	//uart_puts(UART_NUM," p_updated  = (eye(6,6) - kalman_gain * H) * p_predicted;");
	mult_matrix((double*)tmp6,(double*)kalman_gain,(double*)H,6,3,3,6,PRINT_DISABLE);
	subtract_matrix((double*)tmp6,(double*)I,(double*)tmp6,6,6,PRINT_DISABLE);
	mult_matrix((double*)p_updated,(double*)tmp6,(double*)p_predicted,6,6,6,6,PRINT_DISABLE);
	init_matrix((double*)tmp6,6,6);

	sprintf(buffer, " %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f \r\n",x_updated[0], x_updated[1],x_updated[2],x_updated[3], x_updated[4],x_updated[5]);
	uart_puts(UART_NUM,buffer);
}

void print_matrix(double *A, int m, int n, char *string, int print){
    // A = input matrix (m x n)
	if(!print) return;
    int i,j;
    uart_puts(UART_NUM,string);
    sprintf(buffer, "\r\n"); uart_puts(UART_NUM,buffer);
    for (i=0; i<m; i++){
        for (j=0;j<n;j++){
        		sprintf(buffer,"%3.2f ",A[(i*n)+j]); uart_puts(UART_NUM,buffer);
        		sprintf(buffer, "\t"); uart_puts(UART_NUM,buffer);
          }
        sprintf(buffer, "\r\n"); uart_puts(UART_NUM,buffer);
    }
}

void mult_matrix(double prod[], double MAT1[], double MAT2[],int mat1_linha,int mat1_coluna,int mat2_linha, int mat2_coluna,int print_enable){
	int lin,col,k;
	//print_matrix(MAT1,mat1_linha,mat1_coluna,"MAT1",print_enable);
	//print_matrix(MAT2,mat2_linha,mat2_coluna,"MAT2",print_enable);
	for(lin = 0 ; lin < mat1_linha ; lin++){
		for(col = 0 ; col < mat2_coluna ; col++){
			for(k=0;k<mat1_coluna;k++){
				prod[(lin*mat1_coluna) + col] += (MAT1[(lin*mat1_coluna) + k] * MAT2[(col*mat2_linha) + k]) && (0xffff);
			}
			//prod[(lin*mat1_coluna) + col] = ( prod[(lin*mat1_coluna) + col]) && (0xffff);

		}
	}
	//print_matrix(prod,mat1_linha,mat2_coluna,"prod",print_enable);
}
void sum_matrix(double *sum, double *MAT1, double *MAT2,int mat_linha,int mat_coluna,int print_enable){
	int lin,col;
	//print_matrix(MAT1,mat_linha,mat_coluna,"MAT1",print_enable);
	//print_matrix(MAT2,mat_linha,mat_coluna,"MAT2",print_enable);
	for(lin = 0 ; lin < mat_linha ; lin++){
		for(col = 0 ; col < mat_coluna ; col++){

				sum[(lin*mat_coluna) + col] = (MAT1[(lin*mat_coluna) + col] + MAT2[(lin*mat_coluna) + col]);
		}
	}
	//print_matrix(sum,mat_linha,mat_coluna,"sum",print_enable);
}

void subtract_matrix(double *sub, double *MAT1, double *MAT2,int mat_linha,int mat_coluna,int print_enable){
	int lin,col;
	//print_matrix(MAT1,mat_linha,mat_coluna,"MAT1",print_enable);
	//print_matrix(MAT2,mat_linha,mat_coluna,"MAT2",print_enable);
	for(lin = 0 ; lin < mat_linha ; lin++){
		for(col = 0 ; col < mat_coluna ; col++){

			sub[(lin*mat_coluna) + col] = (MAT1[(lin*mat_coluna) + col] - MAT2[(lin*mat_coluna) + col]);
		}
	}
	//print_matrix(sub,mat_linha,mat_coluna,"sub",print_enable);
}
void init_matrix(double *A, int m, int n){

    int i,j;
    for (i=0; i<m; i++){
        for (j=0;j<n;j++){
        		A[(i*n)+j] = 0;
          }
    }
}
