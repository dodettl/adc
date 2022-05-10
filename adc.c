#include "stm32f446xx.h"
#include "string.h"
#include "stdio.h" 
#include "Peripherie.h"

int i; 

unsigned int val_Tx = 0;          						    /* Globals used for display */

volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
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

void can_Init(void) {

  CAN_setup();                                  /* setup CAN Controller #2  */

  CAN_start();                                  /* start CAN Controller #2  */

  CAN_waitReady();                              /* wait til tx mbx is empty */
}


int main(void)
{

	/*Init functions*/
	
  SysTick_Config(SystemCoreClock /1000);         /* SysTick 1 msec irq       */
  can_Init();                                    /* initialize CAN interface */

  CAN_TxMsg[1].id = 33;                           /* initialize msg to send   */
  for (i = 0; i < 8; i++) CAN_TxMsg[0].data[i] = 0;
  CAN_TxMsg[1].len = 1;
  CAN_TxMsg[1].format = STANDARD_FORMAT;
	CAN_TxMsg[1].type = DATA_FRAME;
	
	/*main loop*/
	while(1)
	{	
		  if (CAN_TxRdy[1]) {                           /* tx msg on CAN Ctrl #2    */
      CAN_TxRdy[1] = 0;
			
      CAN_TxMsg[1].data[0] = val_Tx++;              /* data[0] = ADC value      */
			for (i = 1; i < 8; i++) CAN_TxMsg[1].data[i] = 0x77;
      CAN_wrMsg (2, &CAN_TxMsg[1]);               /* transmit message         */
				
			if(val_Tx == 15) val_Tx = 0; 
    }
			if (CAN2->TSR & CAN_TSR_RQCP0) {          /* request completed mbx 0        */
		CAN2->TSR |= CAN_TSR_RQCP0;             /* reset request complete mbx 0   */
	  CAN_TxRdy[1] = 1; 
  }

    Delay (100);                                  /* delay for 100ms          */		
	}
}



