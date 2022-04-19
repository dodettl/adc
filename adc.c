#include "stm32f446xx.h"

void initTIM2(void); 
void TIM2_IRQHandler(void);
void initADC1(void); 

int main(void){

	/*Init functions*/
	initADC1();
	initTIM2(); 
	
	/*main loop*/
	while(1);
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
	TIM2->ARR = 100 - 1; 	//Counter 50 => 1Mhz/100 = 10 kHz
	TIM2->CR1 = 1; 
	//TIM2->CCR2

	/*Stup interrupt*/
	TIM2->DIER |= 1; 
	NVIC_EnableIRQ(TIM2_IRQn);
	
	/*Enable interrupt*/
	__enable_irq();

}

void TIM2_IRQHandler(void){

	static volatile uint8_t counter; 
	volatile uint16_t result = 0; 
	
	if((counter == 1) || (counter ==3))
	{
		ADC1->CR2 |= 0x40000000;
		while(!(ADC1->SR & 2));
		result = ADC1->DR;
	}
	if(counter == 2)
	{
		GPIOA->ODR ^= (1<<0);			//toggle green led 
	}
	if(counter/4)								//Divison benötigt man mehr Takte, ist aufwendiger für die ALU 
	{
		counter = 0; 
	}
	counter++;
	
	TIM2->SR = 0; 					//Clear interrupt flag
}

void initADC1(void){

	/*Configure clock*/
	RCC->AHB1ENR |= 1;						//Enable GPIOA clock 
	GPIOA->MODER |= (3<<2); 			//Analog mode PA1
	
	/*Configure ADC1*/
	RCC->APB2ENR |= (1<<8);				//Enable ADC1 clock 
	ADC1->SQR3 = 1;								//Conversion sequence starts at channel 1
	ADC1->SQR1 = 0; 							//Conversion length 1
	ADC1->CR2 = 0;								//clear
	ADC1->CR2 = (1<<0) | (1<<28) | (11<<24); 					//Enable ADC1, enable trigger rising edge, enable timer2 cc2 
	
}