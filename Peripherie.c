#include <stm32f4xx.h>
#include "Peripherie.h"

/* CAN identifier type */
#define CAN_ID_STD            ((uint32_t)0x00000000)  /* Standard Id          */
#define CAN_ID_EXT            ((uint32_t)0x00000004)  /* Extended Id          */

/* CAN remote transmission request */
#define CAN_RTR_DATA          ((uint32_t)0x00000000)  /* Data frame           */
#define CAN_RTR_REMOTE        ((uint32_t)0x00000002)  /* Remote frame         */

CAN_msg       CAN_TxMsg[2];                      /* CAN message for sending */
CAN_msg       CAN_RxMsg[2];                      /* CAN message for receiving */                                

uint32_t      CAN_TxRdy[2] = {0,0};              /* CAN HW ready to transmit a message */
uint32_t      CAN_RxRdy[2] = {0,0};              /* CAN HW received a message */

static uint32_t CAN_filterIdx[2] = {0,0};        /* static variable for the filter index */


/*----------------------------------------------------------------------------
  setup CAN interface
 *----------------------------------------------------------------------------*/
void CAN_setup (void)  {
  CAN_TypeDef *pCAN = CAN2;
  uint32_t brp;

	/* Enable clock for CAN2 and GPIOB */
	RCC->APB1ENR   |= (1 << 25) | (1 << 26);
	RCC->AHB1ENR   |= (1 <<  1);
	/* CAN2, we use PB5, PB6 */
	GPIOB->MODER   &= ~(( 3 << ( 5*2)) | ( 3 << ( 6*2)));
	GPIOB->MODER   |=  (( 2 << ( 5*2)) | ( 2 << ( 6*2)));
	GPIOB->OTYPER  &= ~(( 1 <<   5   ) | ( 1 <<   6   ));
	GPIOB->OSPEEDR &= ~(( 3 << ( 5*2)) | ( 3 << ( 6*2)));
	GPIOB->PUPDR   &= ~(( 3 << ( 5*2)) | ( 3 << ( 6*2)));
	GPIOB->AFR[0]  &= ~((15 << ( 5*4)) | (15 << ( 6*4)));
	GPIOB->AFR[0]  |=  (( 9 << ( 5*4)) | ( 9 << ( 6*4)));

	NVIC_EnableIRQ   (CAN2_TX_IRQn);         /* Enable CAN2 interrupts */
	NVIC_EnableIRQ   (CAN2_RX0_IRQn);


  pCAN->MCR = (CAN_MCR_INRQ   |           /* initialisation request           */
               CAN_MCR_NART    );         /* no automatic retransmission      */
                                          /* only FIFO 0, tx mailbox 0 used!  */
  while (!(pCAN->MSR & CAN_MCR_INRQ));

  pCAN->IER = (CAN_IER_FMPIE0 |           /* enable FIFO 0 msg pending IRQ    */
               CAN_IER_TMEIE    );        /* enable Transmit mbx empty IRQ    */

  /* Note: this calculations fit for CAN (APB1) clock = 42MHz */
  brp  = (16000000 / 7) / 500000;         /* baudrate is set to 500k bit/s   42000000  */
                                                                          
  /* set BTR register so that sample point is at about 71% bit time from bit start */
  /* TSEG1 = 5, TSEG2 = 2, SJW = 3 => 1 CAN bit = 7 TQ, sample at 75%      */
  pCAN->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((        0x0F) << 16) | (          0x3FF));
  pCAN->BTR |=  ((((3-1) & 0x03) << 24) | (((2-1) & 0x07) << 20) | (((5-1) & 0x0F) << 16) | ((brp-1) & 0x3FF));
}


/*----------------------------------------------------------------------------
  leave initialisation mode
 *----------------------------------------------------------------------------*/
void CAN_start (void)  {
  CAN_TypeDef *pCAN = CAN2;

  pCAN->MCR &= ~CAN_MCR_INRQ;             /* normal operating mode, reset INRQ*/
#ifndef __TEST
  while (pCAN->MSR & CAN_MCR_INRQ);
#endif
}

/*----------------------------------------------------------------------------
  check if transmit mailbox is empty
 *----------------------------------------------------------------------------*/
void CAN_waitReady (void)  {
  CAN_TypeDef *pCAN = CAN2;

  while ((pCAN->TSR & CAN_TSR_TME0) == 0);  /* Transmit mailbox 0 is empty    */
  CAN_TxRdy[2-1] = 1;
}

/*----------------------------------------------------------------------------
  wite a message to CAN peripheral and transmit it
 *----------------------------------------------------------------------------*/
void CAN_wrMsg (uint32_t ctrl, CAN_msg *msg)  {
  CAN_TypeDef *pCAN = (ctrl == 1) ? CAN1 : CAN2;

  pCAN->sTxMailBox[0].TIR  = (uint32_t)0; /* reset TXRQ bit */
                                          /* Setup identifier information */
  if (msg->format == STANDARD_FORMAT) {   /*    Standard ID                   */
    pCAN->sTxMailBox[0].TIR |= (uint32_t)(msg->id << 21) | CAN_ID_STD;
  } else {                                /* Extended ID                      */
    pCAN->sTxMailBox[0].TIR |= (uint32_t)(msg->id <<  3) | CAN_ID_EXT;
  }

                                          /* Setup type information           */
  if (msg->type == DATA_FRAME)  {         /* DATA FRAME                       */
    pCAN->sTxMailBox[0].TIR |= CAN_RTR_DATA;
  } else {                                /* REMOTE FRAME                     */
    pCAN->sTxMailBox[0].TIR |= CAN_RTR_REMOTE;
  }
                                          /* Setup data bytes                 */
  pCAN->sTxMailBox[0].TDLR = (((uint32_t)msg->data[3] << 24) | 
                              ((uint32_t)msg->data[2] << 16) |
                              ((uint32_t)msg->data[1] <<  8) | 
                              ((uint32_t)msg->data[0])        );
  pCAN->sTxMailBox[0].TDHR = (((uint32_t)msg->data[7] << 24) | 
                              ((uint32_t)msg->data[6] << 16) |
                              ((uint32_t)msg->data[5] <<  8) |
                              ((uint32_t)msg->data[4])        );
                                          /* Setup length                     */
  pCAN->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
  pCAN->sTxMailBox[0].TDTR |=  (msg->len & CAN_TDT0R_DLC);

  //pCAN->IER |= CAN_IER_TMEIE;                 /* enable  TME interrupt        */
  pCAN->sTxMailBox[0].TIR |=  CAN_TI0R_TXRQ;  /* transmit message             */

	
}

/*----------------------------------------------------------------------------
  CAN transmit interrupt handler
 *----------------------------------------------------------------------------*/
void CAN2_TX_IRQHandler (void) {

  if (CAN2->TSR & CAN_TSR_RQCP0) {          /* request completed mbx 0        */
    CAN2->TSR |= CAN_TSR_RQCP0;             /* reset request complete mbx 0   */
    CAN2->IER &= ~CAN_IER_TMEIE;            /* disable  TME interrupt         */
	  CAN_TxRdy[1] = 1; 
  }
}
