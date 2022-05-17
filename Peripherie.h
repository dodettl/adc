/**
  ******************************************************************************
  * @file           : peripherie.h
	* @author					: Dettling, Dominik 
  * @brief          : Header for peripherie.c file.
  *                   Header file of the used peripherals for the lvdt sensor
  ******************************************************************************
  */
#ifndef __PERIPHERIE_H
#define __PERIPHERIE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Defines -------------------------------------------------------------------*/

/**
  * @brief  defines for CAN
  */
#define STANDARD_FORMAT  0
#define DATA_FRAME       0

/**
  * @brief  defines for digital IIR filter
  */
#define k 4

/**
  * @brief  define to enable USART2 in main.c
  */
#define ENABLE_DEBUGGING_INTERFACE 0

/* Typedefs -----------------------------------------------------------------*/
/**
  * @brief  init structure CAN message
  */
typedef struct  {
  unsigned int   id;                    /* 29 bit identifier */
  unsigned char  data[8];               /* Data field */
  unsigned char  len;                   /* Length of data field in bytes */
  unsigned char  format;                /* 0 - STANDARD, 1- EXTENDED IDENTIFIER */
  unsigned char  type;                  /* 0 - DATA FRAME, 1 - REMOTE FRAME */
} CAN_msg;

/* Gobal Variables ----------------------------------------------------------*/
/**
  * @brief  global variables for CAN 
  */
extern CAN_msg CAN_TxMsg;      			/* CAN messge for sending */                              
extern unsigned int CAN_TxRdy;      /* CAN HW ready to transmit a message */

/* Function prototypes -------------------------------------------------------*/

/*----------------------------------------------------------------------------
  CAN Peripherie function prototypes
 *----------------------------------------------------------------------------*/
/**
  * @brief  function prototypes for CAN2
  */
/**
  * @brief  The function calls all necessary functions for the CAN init 
	*					in the proper order
  * @param  None
  * @retval None
  */
void initCAN2(void); 
/**
  * @brief  The function sets all necessary Bits for the Pin setup using CAN2
  * @param  None
  * @retval None
  */
void setupCAN2(void);
/**
  * @brief  The function starts the CAN2 peripherie
  * @param  None
  * @retval None
  */
void startCAN2(void);
/**
  * @brief  The function waits until the CAN tx mailbox is empty and 
	*					new data for transmission can be stored there
  * @param  None
  * @retval None
  */
void waitReadyCAN2(void);
/**
  * @brief  The function set the parameter for the CAN data frame
  * @param  None
  * @retval None
  */
void initMsgCAN2(void); 
/**
  * @brief  The function writes the CAN message in the register of the 
	*					tx mailbox. The finish transmit interrupt is enabled 
	*					and the CAN message is transmitted. 
  * @param  msg pointer to the CAN data array 
  * @retval None
  */
void wrMsgCAN2(CAN_msg *msg);
/*----------------------------------------------------------------------------
  ADC Peripherie function prototypes
 *----------------------------------------------------------------------------*/
 /**
  * @brief  The function inits the ADC1 peripherie 
  * @param  None
  * @retval None
  */
void initADC1(void);
/*----------------------------------------------------------------------------
  UART Peripherie function prototypes
 *----------------------------------------------------------------------------*/
/**
  * @brief  The function init the USART2 peripherie.  
  * @param  None 
  * @retval None
  */
void initUSART2(void); 
/**
  * @brief  The function sends the UART message.   
  * @param  int8_t variable handing over the message 
  * @retval None
  */
void sendUSART2(int8_t number);
/*----------------------------------------------------------------------------
  Digital filter function prototypes
 *----------------------------------------------------------------------------*/
/**
  * @brief  The function uses an IIR filter to implement a low pass filter 
  * @param  int16_t variable hands over the analog value to the IIR filter 
  * @retval int16_t variable returns the filtered value
  */
int16_t digitalIIRFilter(int16_t inputValue); 
/*----------------------------------------------------------------------------
  TIM peripherie function prototypes
 *----------------------------------------------------------------------------*/
/**
  * @brief  The function inits the TIM2 peripherie and enabling the TIM2 interrupt
  * @param  None
  * @retval None
  */
void initTIM2(void); 
void TIM2_IRQHandler(void);


void Delay(uint32_t dlyTicks);
void SysTick_Handler(void);


#endif