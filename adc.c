#include "stm32f446xx.h"
#include "string.h"
#include "stdio.h" 
#include "Peripherie.h"

unsigned int val_Tx = 0;          						    /* Globals used for display */

volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
//void SysTick_Handler(void) {
//  msTicks++;                        /* increment counter necessary in Delay() */
//}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
//void Delay (uint32_t dlyTicks) {
//  uint32_t curTicks;

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
			/* tx msg on CAN Ctrl #2    */
		  if(CAN_TxRdy[1]) 
			{
				CAN_TxRdy[1] = 0;				
				/* data[0] = ADC value */
				CAN_TxMsg[1].data[0] = 16;
				CAN_TxMsg[1].data[1] = 128; //val_Tx++;              
				//for (int i = 1; i < 8; i++) CAN_TxMsg[1].data[i] = 0x77;
				/* transmit message */
				wrMsgCAN2(&CAN_TxMsg[1]);               
				//if(val_Tx == 15) val_Tx = 0; 
			}
			/* delay for 100ms          */		
			//Delay (100);               
	}
}


