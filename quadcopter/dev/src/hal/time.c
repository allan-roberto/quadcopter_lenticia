/*
 * time.c
 *
 *  Created on: 01/05/2014
 *      Author: root
 */

#include <time.h>
#include <avr/io.h>    // Needed to use interrupts

void init_timer(void){

	TCCR1A = 0;
	TCCR1B = 0x03;
}

uint16_t read_timer(void){
	uint16_t timer_value;
	timer_value = TCNT1;
	return timer_value;
}

