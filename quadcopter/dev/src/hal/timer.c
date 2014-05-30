/* 
 *	Basis
 *	2009 Benjamin Reh
 */
#include <avr/interrupt.h>
#include <timer.h>

volatile uint32_t ms_timer;

void timerInit(void)
{
	TCNT2  = 0x00;
	TCCR2A = 1<<WGM21;
	TCCR2B = (1<<CS22) | (1<<CS20);
	OCR2A = OCR2A_VAL;
	TIMSK2  |= 1<<OCIE2A;
	sei();
	
}

SIGNAL (TIMER2_COMPA_vect)
{
	ms_timer++;
}

uint32_t getMsTimer() {
	uint32_t ret;
	cli(); // interrupts aus
	ret = ms_timer; // Wert aus volatile Variable kopieren
	sei(); // interrupts wieder an
	return ret;
}

void init_kalman_timer(void){

 //16,372ms -->255
	TCNT0  = 0x00;
	TCCR0A = 1 << WGM01;	// CTC Mode
	TCCR0B = (1<<CS02) | (1<<CS00);
	//OCR0A = 156; //10ms
	OCR0A = 80; //10ms
	TIMSK0  |= 1<<OCIE0A;

}




