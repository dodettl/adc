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
extern CAN_msg CAN_TxMsg[2];      			/* CAN messge for sending */                              
extern unsigned int CAN_TxRdy[2];      /* CAN HW ready to transmit a message */

/* Function prototypes -------------------------------------------------------*/
/**
  * @brief  function prototypes for CAN2
  */
void initCAN2(void); 
void setupCAN2(void);
void startCAN2(void);
void waitReadyCAN2(void);
void initMsgCAN2(void); 
void wrMsgCAN2(CAN_msg *msg);

void Delay(uint32_t dlyTicks);
void SysTick_Handler(void);

void initTIM2(void); 
void TIM2_IRQHandler(void);
void initADC1(void); 
void initUSART2(void); 
void sendUSART2(int8_t number);
int16_t digitalIIRFilter(int16_t inputValue); 
int16_t digitalFIRFilter(int16_t inputValue); 


#endif