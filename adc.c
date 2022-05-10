#include "stm32f446xx.h"
#include "string.h"
#include "stdio.h" 
#include "Peripherie.h"

int i; 

unsigned int val_Tx = 0, val_Rx = 0;              /* Globals used for display */

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

#ifndef __NO_SYSTICK
  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
#else
  for (curTicks = 0; curTicks < (dlyTicks * 0x1000); curTicks++) __NOP();
#endif
}

void can_Init(void) {

  CAN_setup (1);                                  /* setup CAN Controller #1  */
#ifndef __TEST
  CAN_setup (2);                                  /* setup CAN Controller #2  */
#endif
  CAN_wrFilter (1, 33, STANDARD_FORMAT);          /* Enable reception of msgs */

#ifdef __TEST
  CAN_testmode(1, CAN_BTR_SILM | CAN_BTR_LBKM);   // Loopback, Silent Mode (self-test)
#endif

  CAN_start (1);                                  /* start CAN Controller #1  */
#ifndef __TEST
  CAN_start (2);                                  /* start CAN Controller #2  */
#endif

  CAN_waitReady (1);                              /* wait til tx mbx is empty */
#ifndef __TEST
  CAN_waitReady (2);                              /* wait til tx mbx is empty */
#endif
}


int main(void)
{

	/*Init functions*/
	#ifndef __NO_SYSTICK
  SysTick_Config(SystemCoreClock /1000);         /* SysTick 1 msec irq       */
	#endif
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

      CAN_TxMsg[1].data[0] = val_Tx;              /* data[0] = ADC value      */
			for (i = 1; i < 8; i++) CAN_TxMsg[1].data[i] = 0x77;
      CAN_wrMsg (2, &CAN_TxMsg[1]);               /* transmit message         */
    }	
		Delay (10);                                   /* delay for 10ms           */

    if (CAN_RxRdy[0]) {                           /* rx msg on CAN Ctrl #1    */
      CAN_RxRdy[0] = 0;

      val_Rx = CAN_RxMsg[0].data[0];
    }
		
    Delay (500);                                  /* delay for 500ms          */

		
	}
}



