/*
 * CruiseControlV2.c
 *
 * Created: 11/17/2015 11:04:12 AM
 *  Author: Admin
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "CruiseControlV2.h"

#define SET_POINT		26


volatile uint16_t counter;

volatile uint8_t counter1s;


volatile uint16_t pwmDC = 8000;

volatile uint16_t pwmDC255 = 128;

volatile float p_term = 500.0000;
volatile float i_term = 50.00000;
volatile float d_term = 50.00000;

ISR (TIMER0_COMPA_vect){
	
	counter1s++;
	
	if (counter1s >= 100)
	{
		counter1s = 0;
		
		//pwmDC = pid(SET_POINT,counter);
		//pwmDC =  (pwmDC >> 6);
		
		/*
		if (counter < SET_POINT)
		{
			pwmDC += 5;
			if (pwmDC>=250)
			{
				pwmDC=255;
			}
			
		} else{
			
			if (counter > SET_POINT)
			{
				pwmDC -= 5;
				
				if (pwmDC<5)
				{
					pwmDC=5;
				}
			}
			
			
		}
		
		updatePWM();
		*/
		
		if (counter < (SET_POINT-1) )
		{
			pwmDC255 += 5;
			
			if (pwmDC255 >= 250)
			{
				pwmDC255 = 255;//upper limit
			}
			
		}
		
		if (counter > (SET_POINT+1))
		{
			pwmDC255 -= 5;
			
			if (pwmDC255<=10)
			{
				pwmDC255 =10;
			}
		}
		
		updatePWM255();
		
		
		counter =0;
	}
	
}


ISR (INT0_vect){
	counter++;
}


int main(void)
{
	
	/*
	DDRA &= ~(1<<DDA1);
	PORTA |= (1<<PORTA1); //PULL UP
	*/
	
	initPWM();
	initTimerRPM();
	initINT0();
	
	sei();
    while(1)
    {
        //TODO:: Please write your application code 
		/*
		if (!(PINA & (1<<PINA1)))
		{
			PORTB ^= (1<<PORTB0);
			PORTB ^= (1<<PORTB2);
		}*/
		
		
		
    }
}



void initINT0(void){
	
	/*DDRD &= ~(1<<PD0);
	PORTD |= (1<<PD0);
	*/
	//EICRA = (1<<ISC01)|(1<<ISC00); //falling edge
	
	
	
	//PORTE |= (1<<PE5);	
	//EICRB = (1<<ISC51);
	//EIMSK = (1<<INT5);
	
	MCUCR = (1<<ISC00)|(1<<ISC01);
	GIMSK = (1<<INT0);
	
	
}

void initTimerRPM(void){
	/*
	OCR3A = 15625;
	TIMSK3 = (1<<OCIE3A);
	TCCR3A = 0; //non inverting mode
	TCCR3B = (1<<WGM32)|(1<<CS32)|(1<<CS30);// prescaler 1024
	*/
	
	OCR0A= 78; //10ms
	
	TCCR0A = (1<<0); //CTC0

	TCCR0B = (1<<CS02)|(1<<CS00); // 1024
	
	TIMSK |= (1<<OCIE0A);
}


void initPWM(void){
	
	
	
	
	OCR1C = 255;
	OCR1D = 128;
	OCR1A = 128;
	
	DDRA = (1<<DDA1);
	DDRB = (1<<DDB5)|(1<<DDB0)|(1<<DDB2)|(1<<DDB1);
	PORTB |= (1<<PORTB0);
	TCCR1A = 0;
	TCCR1B = (1<<CS13)|(1<<CS12);//2048 prescaler
	TCCR1C = (1<<COM1D1)|(1<<PWM1D); //clear on compare
	TCCR1E |= (1<<OC1OE5); 
	
	

	
}

void updatePWM(void){
	
	
	OCR1D = (uint8_t)pwmDC;
	
}

void updatePWM255(void){
	
	OCR1D = (uint8_t)pwmDC255;
	
}


uint16_t pid(uint16_t setPoint, uint16_t measure){
	
	uint16_t error;
	static uint16_t integral;
	static uint16_t lastError;
	int16_t output;
	
	if (measure > setPoint)
	{
		measure = setPoint;
	}
	error = setPoint - measure;
	integral += error;
	
	if (integral > 250)
	{
		integral =0;
	}
	
	output = (int16_t) ( (float)error * p_term    + (float)(error-lastError)* d_term + (float)integral * i_term );
	
	lastError = error;
	
	if (output<0)
	{
		output=0;
	}
	
	return (uint16_t)output;
	
	
	
}


