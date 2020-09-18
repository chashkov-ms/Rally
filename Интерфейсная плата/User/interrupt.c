/*
 * Interrupt.c
 *
 *  Created on: 25.12 2019
 *      Author: Chashkov
 */
/********************************************************************************/
 #include "interrupt.h"
 #include "stm32f030x6.h"
 #include "Common_types.h"
 #include "uart.h"
 
 
/********************************************************************************/
extern Control_Typedef CONTROL_System;
     
/********************************************************************************/
 /**
  * @brief Interrupt Configuration
  * @retval None
  */
void MX_Interrupt_Init(void){
    
    __enable_irq ();
    
    NVIC_SetPriority (TIM14_IRQn, 1);           //Set MiddlePriority
    NVIC_EnableIRQ(TIM14_IRQn);       

    NVIC_SetPriority (DMA1_Channel2_3_IRQn, 1); //Set MiddlePriority
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);       
       
    NVIC_EnableIRQ(USART1_IRQn);                //High Priority
    
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG->EXTICR[0U] = SYSCFG_EXTICR1_EXTI3_PB;
    EXTI->RTSR  |= EXTI_RTSR_TR3;
    EXTI->FTSR  |= EXTI_FTSR_TR3;
    EXTI->IMR   |= EXTI_IMR_MR3;
    NVIC_SetPriority (EXTI2_3_IRQn, 2);         //Set LowPriority
    NVIC_EnableIRQ(EXTI2_3_IRQn);

    SYSCFG->EXTICR[1U] = SYSCFG_EXTICR1_EXTI0_PB;
    SYSCFG->EXTICR[3U] = SYSCFG_EXTICR1_EXTI3_PA;
    EXTI->RTSR  |= EXTI_RTSR_TR4 | EXTI_RTSR_TR15;  //Rise Front Enable  
    EXTI->FTSR  |= EXTI_FTSR_TR4 | EXTI_FTSR_TR15;  //Fall Front Enable
    EXTI->IMR   |= EXTI_IMR_MR4 | EXTI_IMR_MR15;
    NVIC_SetPriority (EXTI4_15_IRQn, 2);            //Set LowPriority
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    
}

/********************************************************************************/
/**
  * @brief DMA1_Channel2_3_IRQHandler
  * @retval None
  */
void DMA1_Channel2_3_IRQHandler(void){
    
    if ((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF2;
        End_UART_Tx();
    }
    if ((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF3;
        if (CONTROL_System.UART_rec_f == 0)
        {    
            End_Receive_Head();
            return;
        }
        if (CONTROL_System.UART_rec_f == 1) 
            End_Receive_Data ();
    }               
}

/********************************************************************************/ 
/**
  * @brief USART1_IRQHandler
  * @retval None
  */
void USART1_IRQHandler(void){
    
    int adress = USART1->RDR;
    
    if ((USART1->ISR & USART_ISR_RTOF) == USART_ISR_RTOF)
    {
        USART1->ICR |= USART_ICR_FECF | USART_ICR_RTOCF;
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;      // Disable DMA channel
        USART1->CR3 &= ~USART_CR3_DMAR;          // Disable DMA for Receiver
        USART1->CR1 |= USART_CR1_RXNEIE;        // Enable Interrupt RxD
        CONTROL_System.UART_rec_f = 0;          // Receive End
        return;
    }
    if (adress == DEV_ADDR)
        Start_DMA_receive_header();
    else
        USART1->RQR |= USART_RQR_MMRQ;
}

/********************************************************************************/
/**
  * @brief EXTI2_3_IRQHandler
  * Button 2
  * @retval None
  */
void EXTI2_3_IRQHandler(void){
    
    EXTI->PR |= EXTI_PR_PR3;                // Clear InterruptFlag
    Check();
}   

/********************************************************************************/
/**
  * @brief EXTI4_15_IRQHandler
  * Button 1 and Button 3
  * @retval None
  */
void EXTI4_15_IRQHandler(void){
    
    if (((EXTI->PR >> 4)&0x1) == 1)         
    {
        EXTI->PR |= EXTI_PR_PR4;                            //Clear InterruptFlag       
        Check();
    }
    if (((EXTI->PR >> 15)&0x1) == 1)
    {
        EXTI->PR |= EXTI_PR_PR15;        //Clear InterruptFlag
        Check();
    }
}

/********************************************************************************/
/**
  * @brief Check
  * Check button
  * @retval None
  */
void Check (void){
    
    Port_Check_Typedef Buttons [3] = {{&(GPIOB->IDR), 4}, {&(GPIOB->IDR), 3}, {&(GPIOA->IDR), 15}};
    uint8_t new_status = 0;
    int i = 0;
    
    for (i=0; i < 3; i++)
    {
        if ((( *Buttons[i].Port_IDR >> Buttons[i].Pin)&0x1) == 1)
            new_status |= 0 << i;
        else
            new_status |= 1 << i;
    }
    if (( new_status ^ CONTROL_System.BTN) & 0x7)
    {
        CONTROL_System.BTN = new_status;
        GPIOB->BSRR |= GPIO_BSRR_BR_5;          // Down UI_INT 
    }
}

 /************************ (C) COPYRIGHT FABLIGHT Electronics *****END OF FILE****/
