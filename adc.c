#include "stm32f446xx.h"
#include "string.h"
#include "stdio.h" 
#include "Peripherie.h"


volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
volatile int16_t adcValue0, filteredIIRValue, transmitValue; 

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
		
			filteredIIRValue = digitalIIRFilter(adcValue0);
			transmitValue = digitalFIRFilter(filteredIIRValue);
			if(transmitValue != 0)
			{
					CAN_TxRdy[1] = 0;				
					/*High Byte*/
					CAN_TxMsg[1].data[0] = (transmitValue>>8);
					/*Low Byte*/
					CAN_TxMsg[1].data[1] = transmitValue; 
					/* transmit message */
					wrMsgCAN2(&CAN_TxMsg[1]);               
					
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
}


void TIM2_IRQHandler(void){

	static uint16_t adcValue1; 
	
	TIM2->SR = 0; 													//Clear interrupt flag
	GPIOA->ODR ^= (1<<0);	
	
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

