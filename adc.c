#include "stm32f446xx.h"

#define LENGTH 20

void initTIM2(void); 
void TIM2_IRQHandler(void);
void initADC1(void); 
void initUSART2(void); 
void sendUSART2(int8_t number);

volatile uint16_t result = 0; 
volatile int adcFlag = 0; 
volatile uint16_t adc[LENGTH]; 
volatile int i, n;

int main(void){

	/*Init functions*/
	initADC1();
	initTIM2(); 
	initUSART2(); 
	
	/*main loop*/
	while(1)
	{	
			//if(adcFlag == 1)
			//{
					//adc[i++]= ADC1->DR;
					//ADC1->CR2 |= (1<<30);				//Start conversion ADC
					//while(!(ADC1->SR & 2));
				//	adcFlag = 0; 
			//}	
			if(i == LENGTH)
			{
				for(n = 0; n < LENGTH; n++)
					sendUSART2(adc[n]);
				i = 0;
			}
	}
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

void TIM2_IRQHandler(void){

	TIM2->SR = 0; 					//Clear interrupt flag
	GPIOA->ODR ^= (1<<0);			//toggle PA0
  //adcFlag = 1; 	//enable ADC measurement
	if(i < LENGTH)
		adc[i++]= ADC1->DR;
	ADC1->CR2 |= (1<<30);
	
	
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
	ADC1->CR2 = (1<<0); 					//Enable ADC1
	
}

void initUSART2(void){
 
	/*Configure Clock*/
	RCC->AHB1ENR |= 1; 
	RCC->APB1ENR |= (1 << 17);
	
	/*Configure PA2 for USART2_TX */
	GPIOA->AFR[0] |= (7 << 8);
	GPIOA->AFR[0] |= (7 << 12);
	GPIOA->MODER |= (2 << 4);
	GPIOA->MODER |= (2 << 6);
	
	/*Configure UART*/
	USART2->BRR |= (104 << 4) | (3 << 0);   //Mantissa and Fraction
	USART2->CR1 = 0x00; 									 	//Reset all
	USART2->CR1 &= ~(1<<12); 								//Word length 
	USART2->CR1 |= (1<<3);									//Enable TE
	USART2->CR2 = 0x00;											//1 Stop Bit
	USART2->CR3 = 0x00;											//No flow control 
	USART2->CR1 |= (1<<13);									//Enable USART2
	
}

void sendUSART2(int8_t number){

	volatile char cache = 0;
	cache = (char)number;									//Typumwandlung 
	while(!(USART2->SR & (1<<7)));
	USART2->DR |= cache; 
	
}