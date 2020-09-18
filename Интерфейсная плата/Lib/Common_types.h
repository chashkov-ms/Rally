/**
  ******************************************************************************
  * @file           : common_types.h
  * @brief          : Header for common types.
  *                   This file contains the common types defines of the application.
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
#ifndef COMMON_DEF_H_
#define COMMON_DEF_H_

/********************************************************************************/
#include "stm32f030x6.h"
#include <stdint.h>

/********************************************************************************/
/** @addtogroup Peripheral_declaration
  * @{
  */  

#define TIM6_BASE             (APBPERIPH_BASE + 0x00001000)
#define TIM6               ((TIM_Basic_TypeDef *) TIM6_BASE)

/********************************************************************************/
typedef struct
{
    uint8_t LEDx_R;     /*!< LEDx R control status bait,    Address offset: 0x00 */
    uint8_t LEDx_G;     /*!< LEDx G control status bait,    Address offset: 0x01 */
    uint8_t LEDx_B;     /*!< LEDx B control status bait,    Address offset: 0x02*/
} LED_Typedef;

typedef struct
{
    uint8_t     BTN;    /*!< Button control status bait,    Address offset: 0x00 */
    LED_Typedef LED1;   /*!< LED1 control status bait,    Address offset: 0x01 */
    LED_Typedef LED2;   /*!< LED2 control status bait,    Address offset: 0x02 */
    LED_Typedef LED3;   /*!< LED3 control status bait,    Address offset: 0x03 */
    LED_Typedef LED4;   /*!< LED4 control status bait,    Address offset: 0x04 */
    uint8_t     UART_rec_f;
} Control_Typedef;

typedef struct
{
  __IO uint32_t CR1;          /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;         /*!< TIM slave Mode Control register,     Address offset: 0x08 */
  __IO uint32_t DIER;         /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;           /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;          /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;        /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;        /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;         /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;          /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;          /*!< TIM prescaler register,              Address offset: 0x28 */
  __IO uint32_t ARR;          /*!< TIM auto-reload register,            Address offset: 0x2C */
} TIM_Basic_TypeDef;

#endif /* COMMON_DEF_H_ */

/************************ (C) COPYRIGHT FABLIGHT Electronics *****END OF FILE****/
