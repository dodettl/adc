#include "stm32f446xx.h"
#include "string.h"
#include "stdio.h"

#define LENGTH  20
#define LENGTH2 40


void initTIM2(void); 
void TIM2_IRQHandler(void);
void initADC1(void); 
void initUSART2(void); 
void sendUSART2(int8_t number);

volatile int16_t adcValue1, adcValue2, adcValue3; 
volatile uint16_t adc1[LENGTH]; 
volatile uint16_t adc2[LENGTH];
volatile uint8_t sendValue; 
/* Array for the difference between value adc1 and adc2 */
volatile int8_t adc3[LENGTH2]; 
/*Increment variables*/
volatile int i1, i2, i3, n;

int main(void)
{

	/*Init functions*/
	initADC1();
	initTIM2(); 
	initUSART2(); 
	
	/*main loop*/
	while(1)
	{	
			if((i1 == LENGTH) && (i2 == LENGTH))
			{
				for(n = 0; n < (LENGTH - 1); n++)
				{

					adcValue1 = adc1[n]; 
					adcValue2 = adc2[n];
					adcValue3 = adcValue2 - adcValue1; 
					adc3[i3] = (adcValue3 >> 8); 
					adc3[i3+1] = adcValue3;
					sendValue = (adc2[n] >> 8);
					sendUSART2(sendValue);
					sendValue = adc2[n];
					sendUSART2(sendValue);
					sendValue = (adc1[n] >> 8);
					sendUSART2(sendValue);
					sendValue = adc1[n];
					sendUSART2(sendValue);
					sendUSART2(adc3[i3]);
					sendUSART2(adc3[++i3]);
					/* In order to skip a byte an extra increment is made at the end of the for loop */
					i3++;
				}
				i1 = 0;
				i2 = 0;
				i3 = 0;				
				memset(adc1, 0, sizeof(adc1));
				memset(adc2, 0, sizeof(adc2)); 
				memset(adc3, 0, sizeof(adc3));
			}
	}
}

void TIM2_IRQHandler(void){

	TIM2->SR = 0; 													//Clear interrupt flag
	GPIOA->ODR ^= (1<<0);										//toggle PA0
	if((i1 < LENGTH) || (i2 < LENGTH))
	{
		if(GPIOA->ODR && (0x01))   					 //Rising edge 
		{
			/* Low values */
			adc1[i1++]= ADC1->DR;
		}
		else
		{
			/* High values */
			adc2[i2++] = ADC1->DR; 
		}
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