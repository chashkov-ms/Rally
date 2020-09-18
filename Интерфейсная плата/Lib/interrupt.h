/**
  ******************************************************************************
  * @file           : Interrupt.h
  * @brief          : Header for Interrupt.c file.
  *                   This file contains the Interrupt defines of the application.
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
#ifndef INTERRUPT_H_
#define INTERRUPT_H_

/********************************************************************************/
#include "stm32f030x6.h"

/********************************************************************************/
typedef struct
{
    __IO uint32_t* Port_IDR;
    uint8_t Pin;
} Port_Check_Typedef;

/********************************************************************************/
void MX_Interrupt_Init(void);
void DMA1_Channel2_3_IRQHandler(void);
void USART1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void EXTI4_15_IRQHandler(void);
void Check (void);

#endif /* INTERRUPT_H_ */

/************************ (C) COPYRIGHT FABLIGHT Electronics *****END OF FILE****/
