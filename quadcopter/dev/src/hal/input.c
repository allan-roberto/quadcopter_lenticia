/*
 * input.c
 * Controller is the board who is in charge of the main control of quadcopter
 * it has a group of input (PBK) pins capable of generating external interrupts, whose
 * are going to be used to receive rc receiver commands.
 *
 *
 *  Created on: Apr 30, 2014
 *      Author: root
 */

#include <input.h>
#include <time.h>

#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/io.h>


uint16_t delta = 0;

uint16_t start_pos[8] = {0,0,0,0,0,0,0,0};
uint16_t end_pos[8] = {0,0,0,0,0,0,0,0};
uint16_t positive[8] = {0,0,0,0,0,0,0,0};
//uint8_t input_pin_changed;
uint8_t pin_pos = 0;
uint8_t receiver_mask = 0;
uint8_t started = 0;
#define TIMER_MAX_COUNT 0xFFFF
ISR (PCINT2_vect)
{




	if(get_interrupt_pin_status()){

		start_pos[pin_pos] = read_timer();
		started = 1;
	}
	else {

		if(started)
		{
			end_pos[pin_pos] = read_timer();
			if(end_pos[pin_pos] >= start_pos[pin_pos]){
				delta = end_pos[pin_pos] - start_pos[pin_pos];
				positive[pin_pos] = delta;
			}
			else{
				delta = start_pos[pin_pos] - end_pos[pin_pos];
				delta = TIMER_MAX_COUNT - delta;
				positive[pin_pos] = delta;
			}
			started = 0;

			pin_pos++;
			if(pin_pos >= 6){
				pin_pos = 0;
			}
			set_monitored_pin(1<<pin_pos);
		}

	}

}

void init_input(void)
{
	// PB0,PB1,PB2,PB3,PB4,PB5,PB6,PB7, (PCINT16, PCINT17, PCINT18, PCINT19, PCINT20, PCINT21, PCINT22, PCINT23 pin) are now inputs
	DDRK = 0x00;// Clear all PK pins
    PORTK = 0xff; // turn On all  Pull-ups
    PCICR = 0x04;    // set Pin Change Interrupt Enable 2
    set_monitored_pin(0x01);

}

uint8_t get_interrupt_pin_status(void){

	static uint8_t last_input_state = 0;
	uint8_t current_input_state;
	uint8_t input_pin_changed;
	uint8_t input_pin_edge;
	current_input_state = (PINK & receiver_mask);
	input_pin_changed = (uint8_t)(last_input_state ^ current_input_state);
	input_pin_edge = (input_pin_changed & current_input_state);
	last_input_state = current_input_state;
	return input_pin_edge;

}
void set_monitored_pin(uint8_t pos){

	 receiver_mask = pos;
	 PCMSK2 = receiver_mask;
}

