#include "stm32f446xx.h"

void initADC1(void); 

int main(void){

	volatile int result = 0;  //Volatile sodass Variable in Speicher abgelegt wird und von Debugger gelesen werden kann 
	/*Init functions*/
	initADC1();
	
	/*main loop*/
	while(1){
		
		ADC1->CR2 |= 0x40000000;
		while(!(ADC1->SR & 2));
		result = ADC1->DR; 
		//result = getADC();
		//printf("%d", result); 
	}
}

void initADC1(void){

	/*Configure clock*/
	RCC->AHB1ENR |= 1;						//Enable GPIOA clock 
	GPIOA->MODER |= (3<<2); 			//Analog mode PA1
	
	/*Configure ADC1*/
	RCC->APB2ENR |= (1<<8);		//Enable ADC1 clock 
	ADC1->SQR3 = 1;								//Conversion sequence starts at channel 1
	ADC1->SQR1 = 0; 							//Conversion length 1
	ADC1->CR2 = 0;								//clear
	ADC1->CR2 |= (1<<0); 					//Enable ADC1
	
}