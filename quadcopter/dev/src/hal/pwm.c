/* 
 *	Basis
 *	2009 Benjamin Reh
 */

#include <avr/io.h>
#include <pwm.h>

uint_fast16_t value;

void init_pwm(void)
{
	/*
	 * This mode and thi prescaler generate 122Hz aprox.
	 * D10 --> PB4 - OC2A (PWM Output A for Timer2)
	 * D9  --> PH6 - OC2B (PWM Output B for Timer2)
	 */
	DDRB =  0x10;
	DDRH =  0x40;
	TCCR2A = ((1<<WGM20) |(1<<COM2A1) | (1<<COM2B1)); // Updown counting
	TCCR2B = ((1<<CS22) | (1<<CS21));   //prescaler: fosc/256


	/*
	 *  THis value will  generate a 450Hz pwm frequency
	 * 	MOTOR_D8 --> PH5 - OC4C (PWM Output C for Timer4)
	 * 	MOTOR_D7 --> PH4 - OC4B (PWM Output B for Timer4)
	 * 	MOTOR_D6 --> PH3 - OC4A (PWM Output A for Timer4)
	 */
	DDRH = 0x38;
	TCCR4A = ((1<<WGM41) | (1<<COM4A1) | (1<<COM4B1) | (1<<COM4C1));
	TCCR4B = ((1<<WGM43) | (1<<CS40)); //No pre-scaler
	ICR4 = 17776;
	//OCR4A = 0xFFFF;
	//OCR4B = 0xFFFF;
	//OCR4C = 0xFFFF;

	/*
	 * THis value will  generate a 450Hz pwm frequency
	 * D3 --> PE5 - OC3C (PWM Output C for Timer3)
	 * D2 --> PE4 - OC3B (PWM Output B for Timer3)
	 * MOTOR_D5 --> PE3 - OC3A (PWM Output A for Timer3)
	 */
	DDRE |=  0x38;
	TCCR3A = ((1<<WGM31) | (1<<COM3A1) | (1<<COM3B1) | (1<<COM3C1));
	TCCR3B = ((1<<WGM33) | (1<<CS30)); //No pre-scaler
	ICR3 = 17777;
	//OCR3A = 0xFFFF;
	//OCR3B = 0xFFFF;
	//OCR3C = 0xFFFF;

	/*
	 * THis value will  generate a 450Hz pwm frequency
	 * D44 --> PL5 - OC5C (PWM Out/OCR5C = 0xFFFF;put C for Timer5)
	 * D45 --> PL4 - OC5B (PWM Output B for Timer5)
	 * D46 --> PL3 - OC5A (PWM Output A for Timer5)
	 */
	DDRL |=  0x38;
	TCCR5A = ((1<<WGM51) | (1<<COM5A1) | (1<<COM5B1) | (1<<COM5C1));
	TCCR5B = ((1<<WGM53) | (1<<CS50)); //No pre-scaler
	ICR5 = 17777; //
	//OCR5A = 0xFFFF;
	//OCR5B = 0xFFFF;
	//OCR5C = 0xFFFF;

}


inline bool set_pwm(uint8_t output,uint16_t period)
{

	//bool ret_err = 0;
	//if((output < SERVO_D10) || (output > D46)){
	//	ret_err = 1;
	//	return ret_err;
	//}
	/*
	 * period is only allowed to be zero or inside an specific range
	 */
/*	if(period != 0){

		if((output == SERVO_D10) || (output == SERVO_D9)){
			//this is due a low resolution timer
			if((period < 30) || (period > 60)){
				ret_err = 1;
				return ret_err;
			}

			value = period;

		}else { //Other outputs
			if((period < 1000) || (period > 2000)){
				ret_err = 1;
				return ret_err;
			}
			//value = (period << 3);
			value = period;
		}

	}else{

		value = period;
	}*/
	value = period;
	switch(output)
	{
	//D10 --> PB4 - OC2A (PWM Output A for Timer2)
	case SERVO_D10:
			OCR2A = value;
			break;
	//D9  --> PH6 - OC2B (PWM Output B for Timer2)
	case SERVO_D9:
			OCR2B = value;
			break;
	//MOTOR_D8 --> PH5 - OC4C (PWM Output C for Timer4)
	case MOTOR_D8:
			OCR4C = value;
			break;
	//MOTOR_D7 --> PH4 - OC4B (PWM Output B for Timer4)
	case MOTOR_D7:
			OCR4B = value;
			break;
	//MOTOR_D6 --> PH3 - OC4A (PWM Output A for Timer4)
	case MOTOR_D6:
			OCR4A = value;
			break;
	//D3 --> PE5 - OC3C (PWM Output C for Timer3)
	case D3:
			OCR3C = value;
			break;
	//D2 --> PE4 - OC3B (PWM Output B for Timer3)
	case D2:
			OCR3B = value;
			break;
	//MOTOR_D5 --> PE3 - OC3A (PWM Output A for Timer3)
	case MOTOR_D5:
			OCR3A = value;
			break;
	//D44 --> PL5 - OC5C (PWM Output C for Timer5)
	case D44:
			OCR5C = value;
			break;
	//D45 --> PL4 - OC5B (PWM Output B for Timer5)
	case D45:
			OCR5B = value;
			break;
	//D46 --> PL3 - OC5A (PWM Output A for Timer5)
	case D46:
			OCR5A = value;
			break;
	}

	return 0;
}

bool enable_pwm(uint8_t index, bool on_off){

	return set_pwm(index,on_off);
}