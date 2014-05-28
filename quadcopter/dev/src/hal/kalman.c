/*
 * kalman.c
 *
 *  Created on: Feb 28, 2014
 *      Author: root
 */

#include <math.h>
#include <kalman.h>
#include <uart.h>



#define TETA 		0
#define TETA_BIAS 	1
#define GAMA 		2
#define GAMA_BIAS 	3
#define PHI 		4
#define PHI_BIAS 	5

#define OMEGA_TETA	0
#define OMEGA_GAMA	1
#define OMEGA_PHI	2



extern char buffer[200];

/*
 * The state is updated with gyro rate measurement every 10ms
 * change this value if you update at a different rate.
 */
#define dt 0.1

/*
 * The covariance matrix.This is updated at every time step to
 * determine how well the sensors are tracking the actual state.
 */
double p_predicted[6][6] = {{ 10.0,  0.0,    0.0,    0.0,    0.0,    0.0 },
							{ 0.0, 	 10.0,   0.0,    0.0,    0.0,    0.0 },
							{ 0.0, 	 0.0, 	 10.0,   0.0,    0.0,    0.0 },
							{ 0.0, 	 0.0,    0.0,    10.0,   0.0,    0.0 },
							{ 0.0, 	 0.0,    0.0,    0.0,    10.0,   0.0 },
							{ 0.0, 	 0.0,    0.0,    0.0,    0.0,    10.0}};

 double p_updated[6][6]   = {	{ 10.0, 0, 0, 0, 0, 0 },
	                				{ 0, 10.0, 0, 0, 0, 0 },
	                				{ 0, 0, 10.0, 0, 0, 0 },
	                				{ 0, 0, 0, 10.0, 0, 0 },
	                				{ 0, 0, 0, 0, 10.0, 0 },
	                				{ 0, 0, 0, 0, 0, 10.0 }};

 double I[6][6] = {{ 1, 0, 0, 0, 0, 0 },
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
double x_predicted[6];
double x_updated[6];
//double p_predicted[6][6];
//double p_updated[6][6];
/*
 * The R represents the measurement covariance noise.R=E[vvT]
 */
 double R[3][3] = {{ 1, 0, 0},
	                	{ 0, 1, 0},
	                	{ 0, 0, 1}};


/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the inclinometer
 * relative to the gyros.
 */
  double Q[6][6] ={{0.1, 0,  	 0,    	 0,   	 0,   	 0},
		 	 	 	 	{0,    	 0.1, 0,      0,      0,      0},
		 	 	 	 	{0,    	 0,  	 0.5758, 0,      0,      0},
		 	 	 	 	{0,    	 0,    	 0,      0.1926, 0,      0},
		 	 	 	 	{0,    	 0,    	 0,    	 0,      0.2057, 0},
		 	 	 	 	{0,      0,    	 0,   	 0,      0,      0.6161}};


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

  double AT[6][6] = {{  1, 	  0,       0,       0,     0,       0 },
       	  	  	  	  	  { -dt,      1,       0,       0,     0,       0 },
       	  	  	  	  	  {  0,       0,       1, 	    0,     0,       0 },
       	  	  	  	  	  {  0,       0,      -dt,      1,     0,       0 },
       	  	  	  	  	  {  0,       0,       0,       0,     1, 	   0 },
       	  	  	  	  	  {  0,       0,       0,       0,    -dt,      1 }};

  double B[6][3]= {{dt,  0,  0 },
 	                	{ 0,  0,  0 },
 	                	{ 0,  dt, 0 },
 	                	{ 0,  0,  0 },
 	                	{ 0,  0,  dt},
 	                	{ 0,  0,  0 }};

  double kalman_gain[6][3]= {	{ 0,  0,  0 },
 	                				{ 0,  0,  0 },
 	                				{ 0,  0,  0 },
 	                				{ 0,  0,  0 },
 	                				{ 0,  0,  0 },
 	                				{ 0,  0,  0 }};

  double H[3][6] ={{1, 	 0,  	 0,    	 0,   	 0,   	 0},
		 	 	 	 	{0,    	 0, 	 1,      0,      0,      0},
		 	 	 	 	{0,    	 0,  	 0, 	 0,      1,      0}};

  double HT[6][3] ={{1, 	 0,  	 0},
		 	 	 	 	 {0,   	 0,   	 0},
		 	 	 	 	 {0, 	 1, 	 0},
		 	 	 	 	 {0,     0,      0},
		 	 	 	 	 {0, 	 0,  	 1},
 	 	 	 	 	 	 {0,     0,      0}};

void stateUpdate(double rate[]){
	uint8_t lin = 0, col = 0, k = 0;

	double u[3] = {1,1,1}; //Atualizar essa vari'avel com o gyro
	double tmp[6][6];
	u[0] = rate[0];
	u[1] = rate[1];
	u[2] = rate[2];
	//sprintf(buffer, "%2.2f ",u[0]); uart_puts(UART_NUM,buffer);
	//sprintf(buffer, "%2.2f ",u[1]); uart_puts(UART_NUM,buffer);
	//sprintf(buffer, "%2.2f ",u[2]); uart_puts(UART_NUM,buffer);


	print_matrix(x_predicted,6,1,"x_predicted");
	print_matrix(p_predicted,6,6,"p_predicted");
	print_matrix(x_updated,6,1,"x_updated");
	print_matrix(Q,6,1,"Q");

	return;
	/*
	 * %Time Update (“Predict”)
	 * % Project the state ahead
	 * x_predicted(:,i)  = A * x_updated(:,i-1) +  B * u(:,i-1);   #u(i) == giro_rw_data(i)
	 */

	x_predicted[TETA] 		= A[0][0] * x_updated[TETA] 	+ A[0][1] * x_updated[TETA_BIAS] + B[0][0]*u[OMEGA_TETA];
	x_predicted[TETA_BIAS] 	= A[1][1] * x_updated[TETA_BIAS];
	x_predicted[GAMA] 		= A[2][2] * x_updated[GAMA] 	+ A[2][3] * x_updated[GAMA_BIAS] + B[2][1]*u[OMEGA_GAMA];
	x_predicted[GAMA_BIAS] 	= A[3][3] * x_updated[GAMA_BIAS];
	x_predicted[PHI] 		= A[4][4] * x_updated[PHI]  	+ A[4][5] * x_updated[PHI_BIAS]  + B[4][2]*u[OMEGA_PHI];
	x_predicted[PHI_BIAS] 	= A[5][5] * x_updated[PHI_BIAS];

	/*
	 *  % Project the error covariance ahead
	 *  p_predicted  = A * p_updated * A' + Q;
	 */
	//A*p_update
	for(lin = 0 ; lin < 6 ; lin++){
		for(col = 0 ; col < 6 ; col++){
			for(k=0;k<6;k++)
				tmp[lin][col] += A[lin][k] * p_updated[k][lin];
		}
	}

	//tmp[0][0] = p_updated[0][0] +  A[0][1] * p_updated[1][0];
	//tmp[0][0] = p_updated[0][0] +  A[0][1] * p_updated[1][0];

	//tmp[0][0] = p_updated[0][0] +  A[0][1] * p_updated[1][0];
	//tmp[0][0] = p_updated[0][0] +  A[0][1] * p_updated[1][0];

	//tmp[0][0] = p_updated[0][0] +  A[0][1] * p_updated[1][0];
	//tmp[0][0] = p_updated[0][0] +  A[0][1] * p_updated[1][0];

	//tmp[0][0] = p_updated[0][0] +  A[0][1] * p_updated[1][0];
	//tmp[0][0] = p_updated[0][0] +  A[0][1] * p_updated[1][0];

	//A*p_update*A'
	for(lin = 0 ; lin < 6 ; lin++){
		for(col = 0 ; col < 6 ; col++){
			for(k=0;k<6;k++)
				p_predicted[lin][col] += tmp[lin][k] * AT[k][lin];
		}
	}
	//p_predicted  = A * p_updated * A' + Q;

	for(lin = 0 ; lin < 6 ; lin++){
		for(col = 0 ; col < 6 ; col++){
			p_predicted[lin][col] = tmp[lin][col] + Q[lin][col];
		}
	}



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

	//sprintf(buffer, "%2.2f ",z[0]); uart_puts(UART_NUM,buffer);
	//sprintf(buffer, "%2.2f ",z[1]); uart_puts(UART_NUM,buffer);
	//sprintf(buffer, "%2.2f ",z[2]); uart_puts(UART_NUM,buffer);
	double tmp[3][6];
	double tmp2[3][3];
	double tmp3[6][3];
	double tmp4[3];
	double tmp5[6];
	double tmp6[6][6];
	uint8_t lin = 0,col = 0,k = 0;
	print_matrix(I,6,6,"I");
	print_matrix(H,3,6,"H");
	print_matrix(p_predicted,6,6,"p_predicted");
	return;

	//H*p_predicted
	for(lin = 0 ; lin < 3 ; lin++){
		for(col = 0 ; col < 6 ; col++){
			for(k=0;k<6;k++)
			tmp[lin][col] += H[lin][k] * p_predicted[k][col];
		}
	}

	//H*p_predicted*H'
	for(lin = 0 ; lin < 3 ; lin++){
		for(col = 0 ; col < 3 ; col++){
			for(k=0;k<6;k++)
			tmp2[lin][col] += tmp[lin][k] * HT[k][col];
		}
	}

	//(H*p_predicted*H' + R)
	for(lin = 0 ; lin < 3 ; lin++){
		for(col = 0 ; col < 3 ; col++){
			tmp2[lin][col] = tmp2[lin][col] + R[lin][col];
		}
	}
	//inv(H*p_predicted*H' + R)
	tmp2[0][0] = 1/(tmp2[0][0]);
	tmp2[1][1] = 1/(tmp2[1][1]);
	tmp2[2][2] = 1/(tmp2[2][2]);

	//p_predicted*H'
	for(lin = 0 ; lin < 6 ; lin++){
		for(col = 0 ; col < 3 ; col++){
			for(k=0;k<6;k++)
			tmp3[lin][col] += p_predicted[lin][k] * HT[k][col];
		}
	}
	//p_predicted*H' * inv(H*p_predicted*H' + R)
	for(lin = 0 ; lin < 6 ; lin++){
		for(col = 0 ; col < 3 ; col++){
			for(k=0;k<3;k++)
			kalman_gain[lin][col] += tmp3[lin][k] * tmp2[k][col];
		}
	}
	/**********************************************************************/
	/*
	 *   %Update estimate with measurement zk
	 *   x_updated(:,i) = x_predicted(:,i) + kalman_gain*(z(:,i) - H * x_predicted(:,i));
	 */
	//H*p_predicted
	for(lin = 0 ; lin < 3 ; lin++){
			for(k=0;k<6;k++)
			tmp4[lin] += H[lin][k] * x_predicted[k];
	}
	//z(:,i) - H * x_predicted
	for(lin = 0 ; lin < 3 ; lin++){
			tmp4[lin] = z[lin] - tmp4[lin];
	}
	//kalman_gain*(z(:,i) - H * x_predicted(:,i)

	for(lin = 0 ; lin < 6 ; lin++){
			for(k=0;k<3;k++)
			tmp5[lin] += kalman_gain[lin][k] * tmp4[k];
	}
	//x_updated(:,i) = x_predicted(:,i) + kalman_gain*(z(:,i) - H * x_predicted(:,i))
	for(lin = 0 ; lin < 6 ; lin++){
		x_updated[lin] = x_predicted[lin] + tmp5[lin];
	}

	/**********************************************************************/
	/*
	 *   %Update the error covariance
	 *   p_updated  = (eye(6,6) - kalman_gain * H) * p_predicted;
	 */
	//kalman_gain * H
	for(lin = 0 ; lin < 6 ; lin++){
		for(col = 0 ; col < 6 ; col++){
			for(k=0;k<3;k++)
			tmp6[lin][col] += kalman_gain[lin][k] * H[k][col];
		}
	}

	//(eye(6,6) - kalman_gain * H)
	for(lin = 0 ; lin < 6 ; lin++){
		for(col = 0 ; col < 6 ; col++){
			tmp6[lin][col] = I[lin][k] - tmp6[k][col];
		}
	}
	//p_updated  = (eye(6,6) - kalman_gain * H) * p_predicted
	for(lin = 0 ; lin < 6 ; lin++){
		for(col = 0 ; col < 6 ; col++){
			for(k=0;k<6;k++)
				p_updated[lin][col] += tmp6[lin][k] * p_predicted[k][col];
		}
	}

	//sprintf(buffer, " %2.2f %2.2f %2.2f \r\n",x_updated[0], x_updated[2],x_updated[4]);
	//uart_puts(UART_NUM,buffer);
}

void print_matrix(double* A, int m, int n, char *string){
    // A = input matrix (m x n)
    int i,j;
    uart_puts(UART_NUM,string);
    sprintf(buffer, "\r\n"); uart_puts(UART_NUM,buffer);
    for (i=0; i<m; i++){
        for (j=0;j<n;j++){
        	sprintf(buffer,"%3.2f ",A[i][j]); uart_puts(UART_NUM,buffer);
        	sprintf(buffer, "\t"); uart_puts(UART_NUM,buffer);
        }
        sprintf(buffer, "\r\n"); uart_puts(UART_NUM,buffer);
    }
}

