/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef MAIN_H_
#define MAIN_H_
 
/********************************************************************************/
#include "Common_types.h"

/********************************************************************************/
int SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA1_Init(void);
void MX_DMA2_Init(void);
void MX_DMA3_Init(void);
void MX_TIM_17_Init(void);
void Set_Bright(LED_Typedef, int LED);

#endif /* MAIN_H_ */

/************************ (C) COPYRIGHT FABLIGHT Electronics *****END OF FILE****/
