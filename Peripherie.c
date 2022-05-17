#include <stm32f4xx.h>
#include "Peripherie.h"

/* CAN identifier type */
#define CAN_ID_STD            ((uint32_t)0x00000000)  // Standard Id

/* CAN remote transmission request */
#define CAN_RTR_DATA          ((uint32_t)0x00000000)  //Data frame 

/* CAN Tx variables */
CAN_msg       CAN_TxMsg;                   //CAN message for sending                              
uint32_t      CAN_TxRdy = 0;               //CAN tx mailbox ready to transmit a message

/*----------------------------------------------------------------------------
  CAN Peripherie functions
 *----------------------------------------------------------------------------*/
/**
  * @brief  The function calls all necessary functions for the CAN init 
	*					in the proper order
  * @param  None
  * @retval None
  */
void initCAN2(void) 
{
	/* setup CAN Controller #2  */
  setupCAN2();                                  
  /* start CAN Controller #2  */
  startCAN2();                                  
  /* wait untill tx mbx is empty */
  waitReadyCAN2();  
	/* init Tx message CAN2 */
	initMsgCAN2();
}
/**
  * @brief  The function sets all necessary Bits for the Pin setup using CAN2
  * @param  None
  * @retval None
  */
void setupCAN2(void)  
{
  CAN_TypeDef *pCAN = CAN2;
  uint32_t brp;

	/* Enable clock for CAN2 and GPIOB */
	RCC->APB1ENR   |= (1 << 26) | (1 << 25);		//Clock enable für CAN Peripherie 
	RCC->AHB1ENR   |= (1 <<  1);
	/* CAN2, use PB5, PB6 */
	GPIOB->MODER   &= ~(( 3 << ( 5*2)) | ( 3 << ( 6*2)));
	GPIOB->MODER   |=  (( 2 << ( 5*2)) | ( 2 << ( 6*2)));
	GPIOB->OTYPER  &= ~(( 1 <<   5   ) | ( 1 <<   6   ));
	GPIOB->OSPEEDR &= ~(( 3 << ( 5*2)) | ( 3 << ( 6*2)));
	GPIOB->PUPDR   &= ~(( 3 << ( 5*2)) | ( 3 << ( 6*2)));
	GPIOB->AFR[0]  &= ~((15 << ( 5*4)) | (15 << ( 6*4)));
	GPIOB->AFR[0]  |=  (( 9 << ( 5*4)) | ( 9 << ( 6*4)));

	NVIC_EnableIRQ(CAN2_TX_IRQn);         /* Enable CAN2 interrupts */

  pCAN->MCR = (CAN_MCR_INRQ   |           /* initialisation request           */
               CAN_MCR_NART    );         /* no automatic retransmission      */
                                          /* only FIFO 0, tx mailbox 0 used!  */
  while (!(pCAN->MSR & CAN_MCR_INRQ));

//TODO: Schaune ob FMPIE0 notwendig ist oder nicht !!!!!!
  pCAN->IER = (CAN_IER_TMEIE);          /* enable FIFO 0 msg pending IRQ    */
																				/* enable Transmit mbx empty IRQ   CA */

  /* Note: this calculations fit for CAN (APB1) clock = 16MHz */
  brp  = (16000000 / 7) / 500000;         /* baudrate is set to 500k bit/s   16000000  */
                                                                          
  /* set BTR register so that sample point is at about 75% bit time from bit start */
  /* TSEG1 = 5, TSEG2 = 2, SJW = 3 => 1 CAN bit = 7 TQ, sample at 75%      */
  pCAN->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((        0x0F) << 16) | (          0x3FF));
  pCAN->BTR |=  ((((3-1) & 0x03) << 24) | (((2-1) & 0x07) << 20) | (((5-1) & 0x0F) << 16) | ((brp-1) & 0x3FF));
}
/**
  * @brief  The function starts the CAN2 peripherie
  * @param  None
  * @retval None
  */
void startCAN2(void)  
{
  CAN_TypeDef *pCAN = CAN2;

  pCAN->MCR &= ~CAN_MCR_INRQ;             /* normal operating mode, reset INRQ*/
#ifndef __TEST
  while (pCAN->MSR & CAN_MCR_INRQ);
#endif
}

/**
  * @brief  The function waits until the CAN tx mailbox is empty and 
	*					new data for transmission can be stored there
  * @param  None
  * @retval None
  */
void waitReadyCAN2(void)  
{
  CAN_TypeDef *pCAN = CAN2;

  while ((pCAN->TSR & CAN_TSR_TME0) == 0);  /* Transmit mailbox 0 is empty    */
  CAN_TxRdy = 1;
}
/**
  * @brief  The function set the parameter for the CAN data frame
  * @param  None
  * @retval None
  */
void initMsgCAN2(void)
{
	/* initialize msg to send   */
  CAN_TxMsg.id = 33;                           
  CAN_TxMsg.len = 2;
  CAN_TxMsg.format = STANDARD_FORMAT;
	CAN_TxMsg.type = DATA_FRAME;
}
/**
  * @brief  The function writes the CAN message in the register of the 
	*					tx mailbox. The finish transmit interrupt is enabled 
	*					and the CAN message is transmitted. 
  * @param  msg pointer to the CAN data array 
  * @retval None
  */
void wrMsgCAN2(CAN_msg *msg)
{
  CAN_TypeDef *pCAN = CAN2; 
	
  pCAN->sTxMailBox[0].TIR  = (uint32_t)0; /* reset TXRQ bit */
                                          
	/* Setup identifier information */
  pCAN->sTxMailBox[0].TIR |= (uint32_t)(msg->id << 21) | CAN_ID_STD;

  /* Setup type information           */
  pCAN->sTxMailBox[0].TIR |= CAN_RTR_DATA;

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

  pCAN->IER |= CAN_IER_TMEIE;                 /* enable  TME interrupt        */
  pCAN->sTxMailBox[0].TIR |=  CAN_TI0R_TXRQ;  /* transmit message             */
}
/**
  * @brief  Interrupt which is called when the CAN message is transmitted. 
	*					The purpose of the interrupt is to set the CAN tx ready flag. 
  * @param  None 
  * @retval None
  */
void CAN2_TX_IRQHandler (void) 
{
  if (CAN2->TSR & CAN_TSR_RQCP0)           /* request completed mbx 0        */
  {
		CAN2->TSR |= CAN_TSR_RQCP0;             /* reset request complete mbx 0   */
    CAN2->IER &= ~CAN_IER_TMEIE;            /* disable  TME interrupt         */
	  CAN_TxRdy = 1; 
  }
}
/*----------------------------------------------------------------------------
  UART Peripherie functions
 *----------------------------------------------------------------------------*/
/**
  * @brief  The function init the USART2 peripherie.  
  * @param  None 
  * @retval None
  */
void initUSART2(void){
 
	/*Configure Clock*/
	RCC->AHB1ENR |= 1; 											//Enable Clock for GPIOA 
	RCC->APB1ENR |= (1 << 17);							//Enable Clock for UART 
	
	/*Configure PA2 and PA3 for USART2_TX */
	GPIOA->AFR[0] |= (7 << 8);							//Enable alternate function UART for PA2
	GPIOA->AFR[0] |= (7 << 12);							//Enable alternate function UART for PA3
	GPIOA->MODER |= (2 << 4);								//Enable alternate function mode for PA2
	GPIOA->MODER |= (2 << 6);								//Enable alternate function mode for PA3
	
	/*Configure UART*/
	USART2->BRR |= (104 << 4) | (3 << 0);   //Mantissa and Fraction
	USART2->CR1 = 0x00; 									 	//Reset all
	USART2->CR1 &= ~(1<<12); 								//Word length 
	USART2->CR1 |= (1<<3);									//Enable TE
	USART2->CR2 = 0x00;											//1 Stop Bit
	USART2->CR3 = 0x00;											//No flow control 
	USART2->CR1 |= (1<<13);									//Enable USART2
	
}

/**
  * @brief  The function sends the UART message.   
  * @param  int8_t variable handing over the message 
  * @retval None
  */
void sendUSART2(int8_t number){

	volatile char cache = 0;
	cache = (char)number;									//Typumwandlung 
	while(!(USART2->SR & (1<<7)));
	USART2->DR |= cache; 
	
}
/*----------------------------------------------------------------------------
  Digital filter functions
 *----------------------------------------------------------------------------*/
/**
  * @brief  The function uses an IIR filter to implement a low pass filter 
  * @param  int16_t variable hands over the analog value to the IIR filter 
  * @retval int16_t variable returns the filtered value
  */
int16_t digitalIIRFilter(int16_t inputValue)
{
	static int32_t _2nw; 
	int32_t _2ny; 
	
	_2ny = inputValue + _2nw;
	_2nw = inputValue + _2ny - (_2ny>>k);
	
	return (_2ny>>(k+1)); 
}

/*----------------------------------------------------------------------------
  TIM Peripherie functions
 *----------------------------------------------------------------------------*/
/**
  * @brief  The function inits the TIM2 peripherie and enabling the TIM2 interrupt
  * @param  None
  * @retval None
  */
void initTIM2(void){

	/*Disable interrupt*/
	__disable_irq(); 						
	
	/*Init GPIOA Port*/
	RCC->AHB1ENR |= 1; 					//Enable Clock for GPIOA 		
	GPIOA->MODER &= ~(3<<0);		//Clear Pin PA0 
	GPIOA->MODER |= (1<<0);			//Set Pin PA0 as an output 

	/*Init TIM2*/
	RCC->APB1ENR |= 1; 					//Enable Clock for TIM2
	TIM2->PSC = 16 - 1; 				//16 MHz divided by 16 => 1Mhz
	TIM2->ARR = 100 - 1; 				//Counter 100 => 1Mhz/100 = 10 kHz
	TIM2->CR1 = 1; 							//Enable counter
	
	/*Setup interrupt*/
	TIM2->DIER |= 1; 						//Enable Update Interrupt 
	NVIC_EnableIRQ(TIM2_IRQn);
	
	/*Enable interrupt*/
	__enable_irq();

}
/*----------------------------------------------------------------------------
  ADC Peripherie functions
 *----------------------------------------------------------------------------*/
/**
  * @brief  The function inits the ADC1 peripherie 
  * @param  None
  * @retval None
  */
void initADC1(void){

	/*Configure clock*/
	RCC->AHB1ENR |= 1;						//Enable GPIOA clock 
	RCC->APB2ENR |= (1<<8);				//Enable ADC1 clock 
	
	/*Configure ADC1*/
	GPIOA->MODER |= (3<<2); 			//Enable alternate function mode for PA1
	ADC1->SQR3 = 1;								//Conversion sequence starts at channel 1
	ADC1->SQR1 = 0; 						  //Conversion length 1
	ADC1->SMPR2 |= (111 < 3); 		//480 cycles sampling time, Reference Manual 13.13.5
	ADC1->CR2 = 0;								//clear
	ADC1->CR2 = (1<<0); 					//Enable ADC1 and align the data to the left side for an easier signed value calculation
	
}