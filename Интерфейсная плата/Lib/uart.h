/**
  ******************************************************************************
  * @file           : uart.h
  * @brief          : Header for uart.c file.
  *                   This file contains the UART defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 FABLIGHT Electronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *******************************************************************************
  */
#ifndef UART_H_
#define UART_H_
  
/********************************************************************************/
#include <stdint.h>
    
/********************************************************************************/
#define BAUDRATE_115200     0x1A1
#define BAUDRATE_921600     0x34
#define BAUDRATE_1M         48
#define DEV_ADDR 0x02

/********************************************************************************/

typedef struct
{
    uint8_t REG_ADDR;       /*!< 7 bit - R/W  0-6 bit register addr*/
    uint8_t L_LEN;          /*!< Low part of message length in bait */
    uint8_t H_LEN;          /*!< Hi  part of message length in bait*/
} Message_Head_Typedef;

typedef struct
{
    uint8_t DATA_0;         
    uint8_t DATA_1;
    uint8_t DATA_2;
} Message_Data_Typedef;

typedef struct
{
    uint8_t                 ADDR;
    Message_Head_Typedef    HEAD;
    Message_Data_Typedef    DATA;
} Message_Typedef;

/********************************************************************************/
void MX_UART_Init(void);
void Start_DMA_receive_header(void);
void End_UART_Tx(void);
void TransmitUART(void);
void End_Receive_Head (void);
void End_Receive_Data (void);
void Check_Uart_Receive(void);
void ReadRegister(void);
void WriteRegister(void);



#endif /* UART_H_ */

/************************ (C) COPYRIGHT FABLIGHT Electronics *****END OF FILE****/
