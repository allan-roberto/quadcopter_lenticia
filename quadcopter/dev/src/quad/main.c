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

#define UART 2

char buffer[40] =  "some characters";
uint16_t throttle_vector[10] = {50,100,150,200,250,300,400,200,100,0};
uint16_t value = 0;
int main(void)
{

	uartInit(UART,UBRR_VAL);
	init_pwm();

	sei();
	_delay_ms(2000);
	init_motor(MOTOR_D8);


	value = throttle_vector[0];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);

	value = throttle_vector[1];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);


	value = throttle_vector[2];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);


	value = throttle_vector[3];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);

	value = throttle_vector[4];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);

	value = throttle_vector[5];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);

	value = throttle_vector[6];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);

	value = throttle_vector[7];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);

	value = throttle_vector[8];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);

	value = throttle_vector[9];
	set_throtle(MOTOR_D8,value);
    sprintf(buffer, "set_throtle : %d",value);
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");

	_delay_ms(2000);
	disable_motor(MOTOR_D8);
    sprintf(buffer, "disable_motor");
    uart_puts (UART,buffer);
    uart_puts (UART,"\n\r");
	_delay_ms(2000);


	while(1)
	{
<<<<<<< HEAD
		set_throtle(MOTOR_D5,)
=======
>>>>>>> b78598cca9b122182c214888fe85c3d11b07ead4

	}
	return 0;
}
