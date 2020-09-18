/*
 * main.c
 *
 *  Created on: 20.12 2019
 *      Author: Chashkov
 */
/********************************************************************************/
#include "main.h"
#include "uart.h"
#include "interrupt.h"

/********************************************************************************/
Control_Typedef CONTROL_System = {0};  /* State LED and Button structure */       
uint16_t PWM_Data[256] = {0};

/********************************************************************************/
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(){
    
    if (SystemClock_Config() == 1)
        while(1);
    
    MX_GPIO_Init();
    MX_UART_Init();
    MX_DMA1_Init();
    MX_DMA2_Init();
    MX_DMA3_Init();
    
    Set_Bright(CONTROL_System.LED1,1);
    Set_Bright(CONTROL_System.LED2,2);
    Set_Bright(CONTROL_System.LED3,3);
    Set_Bright(CONTROL_System.LED4,4);
    
    MX_TIM_17_Init();
    MX_Interrupt_Init();
    
    
	while (1)
    {
        if (CONTROL_System.UART_rec_f == 2)
        {
            CONTROL_System.UART_rec_f = 0;
            Check_Uart_Receive();
        }
    }
	return 0;
}

/********************************************************************************/
/**
  * @brief System Clock Configuration
  *
  * Configure System Clock to 48 MHz
  *
  * @retval 0 - sucsess
            1 - PLL error
  */
int SystemClock_Config(void){
    
    int StartUpCounter = 0;
    FLASH->ACR |= FLASH_ACR_LATENCY;
 /*---Configure PLL---*/
    RCC->CFGR |= RCC_CFGR_PLLMUL12 | RCC_CFGR_PLLSRC_HSI_DIV2 ; // PLL from HSI/2 and MULT=12
    RCC->CR |= (1<<RCC_CR_PLLON_Pos); //Start PLL
            
    //Wait sucsess start or time-out end
    for(StartUpCounter=0; ; StartUpCounter++)
    {
        //Check sucsessful start and break from cycle
        if(RCC->CR & RCC_CR_PLLRDY)
            break;    
        //If didn't start PLL then
        //Off PLL and retuurn error
        if(StartUpCounter > 0x1000)
        {
            RCC->CR &= ~(1<<RCC_CR_PLLON_Pos); //STOP PLL
            return 1;
        }
    }
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_SW_PLL; //SYSCLOK from PLL
    
    //Wait until PLL will use
    while((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
    
    return 0;
}

/********************************************************************************/
/**
  * @brief Port Configuration
  * @retval None
  */
void MX_GPIO_Init(void){

/* ---Configure Port A---*/
/*  A0-A11   as HI speed pull-up output
    A13-A14  as SWDIO and SWCLCK (default)   
    A15      as Low speed floating input (default)*/
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |=   GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0
                    | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0
                    | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0
                    | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0;
    GPIOA->OSPEEDR |=  GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR1 | GPIO_OSPEEDR_OSPEEDR2
                     | GPIO_OSPEEDR_OSPEEDR3 | GPIO_OSPEEDR_OSPEEDR4 | GPIO_OSPEEDR_OSPEEDR5
                     | GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7 | GPIO_OSPEEDR_OSPEEDR8
                     | GPIO_OSPEEDR_OSPEEDR9 | GPIO_OSPEEDR_OSPEEDR10 | GPIO_OSPEEDR_OSPEEDR11;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR2_0
                  | GPIO_PUPDR_PUPDR3_0 | GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0
                  | GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR8_0
                  | GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0;
    GPIOA->BSRR |=  GPIO_BSRR_BR_0 | GPIO_BSRR_BR_1 | GPIO_BSRR_BR_2
                  | GPIO_BSRR_BR_3 | GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5
                  | GPIO_BSRR_BR_6 | GPIO_BSRR_BR_7 | GPIO_BSRR_BR_8
                  | GPIO_BSRR_BR_9 | GPIO_BSRR_BR_10 | GPIO_BSRR_BR_11;          //B5<-1 
    
/* ---Configure Port B---*/
/*  B3,B4   as Low speed floating input (default)
    B5      as Hi speed pull-up output
    B6, B7  as UART1 */
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER |= GPIO_MODER_MODER5_0;
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR5;
    GPIOB->BSRR |= GPIO_BSRR_BS_5;          //B5<-1 
    
    GPIOB->MODER |= GPIO_MODER_MODER6;      //B6 configure as ANALOG because use another devices
                                            //It's configure in interrupt from any BUTTON
    GPIOB->MODER |= GPIO_MODER_MODER7_1;
}

/********************************************************************************/
/**
  * @brief DMA1 Configuration
  *
  * Chanel          1
  * Memory          PWM_Data
  * Memory          PortA->ODR
  * Direction       Memory->Memory
  * PeriferalSize   16 bit
  * MemorySize      16 bit
  * Mode            Cicular
  * Priority        Medium
  * Size            1 transactions
  * CircleSise      256

  * @retval None
  */
void MX_DMA1_Init(void){
    
    RCC->AHBENR |= RCC_AHBENR_DMAEN;
    
    DMA1_Channel1->CPAR = (uint32_t)&GPIOA->ODR;
    DMA1_Channel1->CMAR = (uint32_t)&PWM_Data;
    DMA1_Channel1->CCR |= DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC |
                          DMA_CCR_DIR;
    DMA1_Channel1->CNDTR = 256;
    DMA1_Channel1->CCR |= DMA_CCR_EN;       // Enable DMA channel    
}

/********************************************************************************/
/**
  * @brief DMA2 Configuration
  *
  * Chanel          2
  * Periferal       USART_Tx
  * Memory          UART_Tx_Messag
  * Direction       Memory->Perifelal
  * PeriferalSize   8 bit
  * MemorySize      8 bit
  * Mode            Normal
  * Priority        Medium
  * Size            7 transactions

  * @retval None
  */
void MX_DMA2_Init(void){
    
    extern Message_Typedef UART_Tx_Message;
    
    RCC->AHBENR |= RCC_AHBENR_DMAEN;
    
    DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR);
    DMA1_Channel2->CMAR = (uint32_t)&UART_Tx_Message;
    DMA1_Channel2->CCR |= DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
    DMA1_Channel2->CNDTR = 7;    
}

/********************************************************************************/
/**
  * @brief DMA3 Configuration
  *
  * Chanel          2
  * Periferal       USART_Rx
  * Memory          UART_Rx_Messag
  * Direction       Perifelal->Mempry
  * PeriferalSize   8 bit
  * MemorySize      8 bit
  * Mode            Normal
  * Priority        Medium
  * Size            3 transaction

  * @retval None
  */
void MX_DMA3_Init(void){
    
    extern Message_Typedef UART_Rx_Message;
    
    DMA1_Channel3->CPAR = (uint32_t)&(USART1->RDR);
    DMA1_Channel3->CMAR = (uint32_t)&UART_Rx_Message;
    DMA1_Channel3->CCR |= DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_TCIE;
    DMA1_Channel3->CNDTR = 3;    
}

/********************************************************************************/
/**
  * @brief Timer6 Configuration
  * F_t6 = 50 kHz
  * @retval None
  */
void MX_TIM_17_Init(void){
    
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    
    TIM17->DIER |= TIM_DIER_UDE;

    TIM17->PSC = 479;
    TIM17->ARR = 0x1;
    TIM17->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
}

/********************************************************************************/
/**
  * @brief Timer6 Configuration
  * @param bright 
  * @param LED Number of set LED (1...4)
  * @retval None
  */
void Set_Bright(LED_Typedef bright, int LED){
    
    uint16_t* Data = PWM_Data;
    uint16_t word = 0;
    uint16_t mask = ~(0x7 << ((LED-1)*3));
    uint16_t save_word = 0;
   
    for (int cnt=0; cnt < 255; cnt++)
    {
        word = 0;
        save_word = *Data & mask;
        if (cnt < bright.LEDx_R)
            word = 1;
        if (cnt < bright.LEDx_G)
            word |= 1<<1;
        if (cnt < bright.LEDx_B)
            word |= 1<<2;
        
        *Data = save_word | (word << ((LED-1)*3));
        Data++;
    }
}

/************************ (C) COPYRIGHT FABLIGHT Electronics *****END OF FILE****/
