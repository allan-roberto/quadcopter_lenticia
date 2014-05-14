/*
 * main.c
 *
 *  Created on: 07/02/2014
 *      Author: aras
 */

#include <avr/interrupt.h>
#include <lib_xcopter.h>
#include <stdint.h>
#include <uart.h>
#include <util/delay.h>


char buffer[40] =  "some characters";
<<<<<<< HEAD
int16_t giro_data[3];
uint16_t i = 0;
int main(void)
{
	uartInit(UBRR_VAL);
	init_gyro();
	sei();
	sprintf(buffer, "Gyroscope");
	uart_puts (buffer);
	uart_puts ("\n\r");
	for(i=0;i<1000;i++){

		gyro_get_data(&giro_data[0],&giro_data[1],&giro_data[2]);
		sprintf(buffer, " %d %d\r\n",giro_data[0],i);
		uart_puts (buffer);
		_delay_ms(55);
		_delay_us(290);
=======
double data[3];
uint16_t value = 1000;
#define UART_NUM 2
int main(void)
{
	uartInit(UART_NUM, UBRR_VAL);
	init_accelerometer();
	sei();
	while(1)
	{

		sprintf(buffer, "[0]: %f \n\r",data[0]);
		uart_puts (UART_NUM, buffer);

		sprintf(buffer, "[1]: %f \n\r",data[1]);
		uart_puts (UART_NUM, buffer);

		sprintf(buffer, "[2]: %f \n\r",data[2]);
		uart_puts (UART_NUM, buffer);


		_delay_ms(10);
		//uart_putc (UART_NUM, 12);
		accelerometer_get_data(&data[0],&data[1],&data[2]);
	}
	while(1)
	{

>>>>>>> 519b813e85efe2fb9e21637c2ff3e762afd3d219
	}
	return 0;
}
