/*
 * CruiseControl.c
 *
 * Created: 11/3/2015 11:02:58 PM
 *  Author: Admin
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "CruiseControl.h"

#define SET_POINT	26


#define F_CPU 16000000
#define BAUDRATEPC 9600                           // define baud
#define BAUDRATE ((F_CPU)/(BAUDRATEPC*16UL)-1)            // set baud rate value for UBRR

#define FREQ 16000; //1ms = 1Khz  freq for PWM

#define print(str) serialPCtxArray(str,sizeof(str)); serialPCtx('\n'); serialPCtx('\r')

volatile int32_t counter=0;
volatile char str2[10];

volatile char command;
volatile char flags;
volatile uint16_t pwmDC = 8000;

volatile float p_term = 200.0000;
volatile float i_term = 20.00000;
volatile float d_term = 20.00000;

volatile char itmp[10];


#define autoRPM				5
#define readRPM				4
#define changeP				3
#define decreaseDCflag		2
#define increaseDCflag		1
#define sendMeasureFlag		0



ISR(USART0_RX_vect){
	
	command = UDR0;
	switch (command)
	{
		case '0':
			print("CruiseControlv1");
		
		break;
		
		case '1': // send measures or not
			flags ^= (1<<sendMeasureFlag);
		break;
		
		case '2':
			initPWM('A');// change direction
			updatePWM();
		break;
		
		case '3':
			initPWM('B');
			updatePWM();
			break;
		case '4': //increase DC manually
			flags |= (1<<increaseDCflag);
			break;
		
		case '5':// decrease DC manually
			flags |= (1<<decreaseDCflag);
			break;
		case '6':
		    p_term = readFloatFromBytes();
			/*
			serialPCtxArray((char*)(&p_term),4);
			serialPCtx('\n');*/
			dtostrf(p_term, 3, 5, itmp); print(itmp);
			break;
		case '7':
			i_term = readFloatFromBytes();
			dtostrf(i_term,3,5,itmp); print(itmp);
			break;
		case '8':
			d_term = readFloatFromBytes();
			dtostrf(d_term,3,5,itmp); print(itmp);
			break;
		case '9': 
			print("p: "); dtostrf(p_term,3,5,itmp); print(itmp); 
			print("i: "); dtostrf(i_term,3,5,itmp); print(itmp);
			print("d: "); dtostrf(d_term,3,5,itmp); print(itmp);
			break;
		case 'A':
			flags ^= (1<<autoRPM);
			break;
			
			
			break;
		default:
		break;
		
		
		
	}
	
	
}

ISR(TIMER3_COMPA_vect){
	
	if ( flags & (1<<sendMeasureFlag))
	{
		itoa(counter,str2,10);
		print(str2);
	}
	if (flags & (1<<autoRPM)){
	pwmDC = pid(SET_POINT,counter);
	
	utoa(pwmDC,str2,10);
	print(str2);
	updatePWM();
	
	}
	
	
	
	counter=0;
	
	
	
	
	
	/*
	itoa(counter,str2,10);
	print(str2);
	counter=0;
	*/
	
}

ISR (INT5_vect){
	counter++;
	//serialPCtx('A');
}


int main(void)
{
	initADC();
	initSerialPC();
	initPWM('A');
	initTimerRPM();
	initINT0();
	
	sei();
	
	uint16_t result;
	char str[10];
	
    while(1)
    {
		
		
		
		
		switch (flags){
			
			case (1<<sendMeasureFlag):
				
				/*
				_delay_ms(100);
				result = counter;
				itoa(result,str,10);
				print(str);
				*/
				
			break;
			
			case (1<<increaseDCflag):
				pwmDC += 1000;
				updatePWM();
				flags &= ~(1<<increaseDCflag);
			break;
			
			case (1<<decreaseDCflag):
				pwmDC -= 1000;
				updatePWM();
				flags &= ~(1<<decreaseDCflag);
			break;
			
			
		
			
			
			default:
			break;
		}
		
		
    }
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



void initINT0(void){
	
	/*DDRD &= ~(1<<PD0);
	PORTD |= (1<<PD0);
	*/
	//EICRA = (1<<ISC01)|(1<<ISC00); //falling edge
	
	
	
	//PORTE |= (1<<PE5);	
	EICRB = (1<<ISC51);
	EIMSK = (1<<INT5);
	
	
}

void initTimerRPM(void){
	
	OCR3A = 15625;
	TIMSK3 = (1<<OCIE3A);
	TCCR3A = 0; //non inverting mode
	TCCR3B = (1<<WGM32)|(1<<CS32)|(1<<CS30);// prescaler 1024
	
	
}


void initPWM(char output){
	
	//mode 15, top ICR1H, output OC1A and OC1B
		
	ICR1 = FREQ;
	OCR1A = pwmDC; //50% pwm
	OCR1B = pwmDC;
	
	if (output == 'A')
	{
		TCCR1A = (1<<WGM11)|(1<<COM1A1); //non inverting mode
		TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS10); // no prescaler
		DDRB |= (1<<PB5)|(1<<PB6); //pwm output
		PORTB &= ~(1<<PB6); //keep the other output low
		
	}else{
		TCCR1A = (1<<WGM11)|(1<<COM1B1); //non inverting mode
		TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS10); // no prescaler
		DDRB |= (1<<PB5)|(1<<PB6); //pwm output
		PORTB &= ~(1<<PB5); // keep the other output low
		
	}
	
	
	
}

void updatePWM(void){
	
	if ( TCCR1A & (1<<COM1A1)){
		
		//we are using the OC1A output
		OCR1A = pwmDC;
		
	}else{
		//we are using the OC1B output
		OCR1B = pwmDC;
		
		
	}
	
	
}



uint16_t getADC(void){
	
	
	
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC));
	return ADC;
	
	
}

void initADC(void){
	
	ADMUX |= (1<<REFS0);  //internal 1.1vref
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //enable, clock/16
	//ADC0
	
}


void initSerialPC(void){
	//UBRR0 = 8;
	UBRR0 = BAUDRATE;
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);          // enable receiver ,transmitter and rx interrupt
	//UCSR0B= (1<<TXEN0)|(1<<RXEN0);

	UCSR0C= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
}

void serialPCtx(uint8_t byte){

	while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
	UDR0 = byte;


}

float readFloatFromBytes(void) {
	union u_tag {
		char b[4];
		float val;
	
	} u;
	
	
	u.b[0] = readChar();
	u.b[1] = readChar();
	u.b[2] = readChar();
	u.b[3] = readChar();
	
	//u.val=7.7;
	
	/*
	u.b[0]= 0x66;
	u.b[1]=0x66;
	u.b[2]=0xf6;
	u.b[3]=0x40;
	*/
	return u.val;
}







char readChar(){
	while((UCSR0A &(1<<RXC0)) == 0);
	return UDR0;
}


void serialPCtxArray(uint8_t* array,uint8_t tam){

	for (uint8_t i=0; i<tam;i++)
	{
		serialPCtx(array[i]);
	}


}


uint8_t average8(uint8_t value){

	static uint8_t average[8];
	uint16_t sum=0;
	for (uint8_t i=7; i>0;i--)
	{//shift measures
		average[i]=average[i-1];
		sum+=average[i-1];

	}
	average[0]=value;
	sum+=average[0];

	return (sum >> 3);//shifting by 3 means dividin by 8

}
uint8_t average4(uint8_t value){

	static uint8_t average[4];
	uint16_t sum=0;
	for (uint8_t i=3; i>0;i--)
	{
		average[i]=average[i-1];
		sum+=average[i-1];

	}
	average[0]=value;
	sum+=average[0];

	return (sum >> 2);

}
