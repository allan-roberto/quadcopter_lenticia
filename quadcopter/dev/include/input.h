/*
 * input.h
 *
 *  Created on: Apr 30, 2014
 *      Author: root
 */

#ifndef INPUT_H_
#define INPUT_H_
#include <stdint.h>            // has to be added to use uint8_t
#include <avr/interrupt.h>    // Needed to use interrupts
void init_input(void);
uint8_t get_interrupt_pin_status(void);
void set_monitored_pin(uint8_t pin);

#endif /* INPUT_H_ */
