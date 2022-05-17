#include "stm32f446xx.h"
#include "string.h"
#include "stdio.h" 
#include "Peripherie.h"


volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
volatile int16_t transmitValue; 

void Delay(uint32_t dlyTicks);
void SysTick_Handler(void);

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
				
				if(ENABLE_DEBUGGING_INTERFACE == 1)
				{
						/*High Byte*/
						sendUSART2((transmitValue>>8));
						/*Low Byte*/
						sendUSART2(transmitValue);
				}
				Delay(100); 
	}
}


void TIM2_IRQHandler(void){

	static uint16_t adcValue1; 
	static int32_t sum; 
	static uint8_t counter; 
	
	TIM2->SR = 0; 													//Clear interrupt flag
	GPIOA->ODR ^= (1<<0);	
	
	if(GPIOA->ODR & (1<<0))
	{
		adcValue1 = ADC1->DR;
	}
	else
	{
		sum += digitalIIRFilter((adcValue1 - ADC1->DR));
		counter++; 
			if(counter == 10)
			{
				transmitValue = sum/10; 
				sum = 0; 
				counter = 0; 
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

