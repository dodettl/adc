/**
  ******************************************************************************
  * @file           : main.c
	* @author					: Dettling, Dominik 
  * @brief          : In this file all peripherie functions for the LVDT 
	*										are called
  ******************************************************************************
  */
/* Includess -------------------------------------------------------------------*/
#include "stm32f446xx.h"
#include "string.h"
#include "stdio.h" 
#include "Peripherie.h"

/* Global Variables ------------------------------------------------------------*/
/**
  * @brief  global variables for Delay function
  */
volatile uint32_t msTicks;      /* counts 1ms timeTicks     */
/**
  * @brief  global variables to sent the measured value
  */
volatile int16_t transmitValue; /*Variable to transmit the measured value*/

/* Function Prototypes ----------------------------------------------------------*/
/**
  * @brief  The function gets the current tick of the system 
	*					and increments the counter variable
	*					Necessary for the Delay function
  * @param  None
  * @retval None
  */
void SysTick_Handler(void);
/**
  * @brief  The function stays in a while loop for the 
	*					determine wait time in ms
  * @param  dlyTicks variable for delay time in ms
  * @retval None
  */
void Delay(uint32_t dlyTicks);


int main(void)
{

	/*Init functions*/
	initADC1();
	initUSART2();
	initTIM2();
  initCAN2();                                  
	SysTick_Config(SystemCoreClock /1000);         /* SysTick 1 msec irq       */
	
	/*main loop*/
	while(1)
	{	
				
				CAN_TxRdy = 0;				
				/* save the value in the CAN array */
				*((uint16_t *)CAN_TxMsg.data) = transmitValue;  //Pointer typecast: 16 Bit value is saved directly in the first two positions of the 8 Bit array																												
				/* transmit message */
				wrMsgCAN2(&CAN_TxMsg); 
		
				/*To save the data or display it on Pc
				* set the ENABLE_DEBUGGING_INTERFACE = 1
				* to transmit it via UART*/
				if(ENABLE_DEBUGGING_INTERFACE == 1)
				{
						/*High Byte*/
						sendUSART2((transmitValue>>8));
						/*Low Byte*/
						sendUSART2(transmitValue);
				}
				/*Delay to change the interval in which 
				* the data via CAN and UART is sent */
				Delay(100); 
	}
}

/* In the ISR the output pin is toggled
*  each time and the ADC value is read out.
*  Every second time the difference between the first and 
*  the second ADC value is calculated, filtered with the 
*	 IIR filter and added to the variable sum. 
*	 If the counter reaches 10, all 10 values 
*	 in sum are divided by 10 and hand over to 
*	 the global variable transmitValue. 
*/
void TIM2_IRQHandler(void){

	static uint16_t adcValue1; /*Saves the first ADC value*/
	static int32_t sum; 			/*Saves the sum of 10 subtracted ADC values*/
	static uint8_t counter; 	/*Is increment by each call, saves the value of the current amount of ADC values*/
	
	TIM2->SR = 0; 													//Clear interrupt flag
	GPIOA->ODR ^= (1<<0);										//Toggle Output pin ==> PWM signal
	
	/*If the output pin is high 
	* the first ADC value is saved
	* If the output pin is low the else body 
	* is executed
	*/
	if(GPIOA->ODR & (1<<0))
	{
		adcValue1 = ADC1->DR;									//Reading out the first ADC value 
	}
	else
	{
		sum += digitalIIRFilter((adcValue1 - ADC1->DR));		//The digital filter is used to filter the final ADC value 
																												//(The difference between the first ADC value and the second one
		counter++; 																//Is increment by each value added to the sum 
		if(counter == 10)
			{
				transmitValue = sum/10; 							//The sum uped values are divided by 10 to get the value which is sent via CAN and UART
				sum = 0; 															//The sum is set to 0 
				counter = 0; 													//The counter is set to 0
			}
	}
	ADC1->CR2 |= (1<<30);								//Starts the adc conversion 

}


/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

