#include "stm32f446xx.h"
#include "string.h"
#include "stdio.h"

#define LENGTH 20

void initTIM2(void); 
void TIM2_IRQHandler(void);
void initADC1(void); 

volatile uint8_t adc1[LENGTH]; 
volatile int16_t adcValue1; 
int i1, n; 

int main(void)
{

	/*Init functions*/
	initADC1();
	initTIM2(); 
	
	/*main loop*/
	while(1)
	{	
		
			ADC1->CR2 |= (1<<30);		//Starts the adc conversion 
			while(!(ADC1->SR & 2));
			adc1[i1++]= (ADC1->DR >> 8);			//The highest 4 bits of the 12 bit adc value are saved in the array
			adc1[i1++]=  ADC1->DR;
			adcValue1 = (adc1[n] | (adc1[n+1]));
			n++; 
			if((n == (LENGTH-1)) || (i1 == (LENGTH-1)))
			{
				n = 0; 
				i1 = 0; 
				memset(adc1, 0, sizeof(adc1));
			}
		
	}
}

void TIM2_IRQHandler(void){

	TIM2->SR = 0; 													//Clear interrupt flag
	GPIOA->ODR ^= (1<<0);										//toggle PA0 
}


void initTIM2(void){

	/*Disable interrupt*/
	__disable_irq(); 						
	
	/*Init GPIOA Port*/
	RCC->AHB1ENR |= 1; 					
	GPIOA->MODER &= ~(3<<0);		//Clear Pin PA0 
	GPIOA->MODER |= (1<<0);			//Set Pin PA0 as an output 

	/*Init TIM2*/
	RCC->APB1ENR |= 1; 
	TIM2->PSC = 16 - 1; 	//16 MHz divided by 16 => 1Mhz
	TIM2->ARR = 100 - 1; 	//Counter 100 => 1Mhz/100 = 10 kHz
	TIM2->CR1 = 1; 

	/*Setup interrupt*/
	TIM2->DIER |= 1; 
	NVIC_EnableIRQ(TIM2_IRQn);
	
	/*Enable interrupt*/
	__enable_irq();

}

void initADC1(void){

	/*Configure clock*/
	RCC->AHB1ENR |= 1;						//Enable GPIOA clock 
	GPIOA->MODER |= (3<<2); 			//Analog mode PA1
	
	/*Configure ADC1*/
	RCC->APB2ENR |= (1<<8);				//Enable ADC1 clock 
	ADC1->SQR3 = 1;								//Conversion sequence starts at channel 1
	ADC1->SQR1 = 0; 						  //Conversion length 1
	ADC1->SMPR2 |= (111 < 3); 		//480 cycles sampling time, Reference Manual 13.13.5, Berechnung 2. Tag 
	ADC1->CR2 = 0;								//clear
	ADC1->CR2 = (1<<0); //|| (1<<11); 					//Enable ADC1 and align the data to the left side for an easier signed value calculation
	
}