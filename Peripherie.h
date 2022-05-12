#ifndef __PERIPHERIE_H
#define __PERIPHERIE_H

#include "stdint.h"

#define STANDARD_FORMAT  0

#define DATA_FRAME       0


typedef struct  {
  unsigned int   id;                    /* 29 bit identifier */
  unsigned char  data[8];               /* Data field */
  unsigned char  len;                   /* Length of data field in bytes */
  unsigned char  format;                /* 0 - STANDARD, 1- EXTENDED IDENTIFIER */
  unsigned char  type;                  /* 0 - DATA FRAME, 1 - REMOTE FRAME */
} CAN_msg;

/* Functions defined in module CAN.c */
void initCAN2(void); 
void setupCAN2(void);
void startCAN2(void);
void waitReadyCAN2(void);
void initMsgCAN2(void); 
void wrMsgCAN2(CAN_msg *msg);

extern CAN_msg       CAN_TxMsg[2];      /* CAN messge for sending */
extern CAN_msg       CAN_RxMsg[2];      /* CAN message for receiving */                                
extern unsigned int  CAN_TxRdy[2];      /* CAN HW ready to transmit a message */
extern unsigned int  CAN_RxRdy[2];      /* CAN HW received a message */

#endif