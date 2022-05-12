#include "stm32f446xx.h"
#include "string.h"
#include "stdio.h" 
#include "Peripherie.h"

/*Defines for the digital low pass filter*/
#define k 4

void initTIM2(void); 
void TIM2_IRQHandler(void);
void initADC1(void); 
void initUSART2(void); 
void sendUSART2(int8_t number);
int16_t digitalFilter(int16_t inputValue); 

volatile int16_t adcValue0, transmitValue; 

//  curTicks = msTicks;
//  while ((msTicks - curTicks) < dlyTicks);
//}

int main(void)
{

	/*Init functions*/
  //SysTick_Config(SystemCoreClock /1000);         /* SysTick 1 msec irq       */
  initCAN2();                                    /* initialize CAN interface */
	
	/*main loop*/
	while(1)
	{	
		  /*Apply a digital low pass filter to the measured ADC value*/
			transmitValue = digitalFilter(adcValue0); 
			/*High Byte*/
			sendUSART2((transmitValue>>8));
			/*Low Byte*/
			sendUSART2(transmitValue);
	}
}



	if(adcValue1 == 0)
	{
		adcValue1 = ADC1->DR;
	}
	else
	{
		adcValue0 = adcValue1 - ADC1->DR;
		adcValue1 = 0; 
	}
	ADC1->CR2 |= (1<<30);								//Starts the adc conversion 
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

int16_t digitalFilter(int16_t inputValue){
	static int32_t _2nw; 
	int32_t _2ny; 
	
	_2ny = inputValue + _2nw;
	_2nw = inputValue + _2ny - (_2ny>>k);
	
	return (_2ny>>(k+1)); 
}