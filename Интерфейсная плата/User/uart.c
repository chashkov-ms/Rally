/*
 * uart.c
 *
 *  Created on: 24.12 2019
 *      Author: Chashkov
 */
 /********************************************************************************/
#include "uart.h" 
#include "stm32f030x6.h"
#include "Common_types.h"
#include "main.h"

/********************************************************************************/
Message_Typedef UART_Rx_Message;
Message_Typedef UART_Tx_Message = {DEV_ADDR,0,0};
/********************************************************************************/
/**
  * @brief UART Configuration
  *
  * BaudRate    921600
  * WordLength  8 bit
  * StopBit     1 bit length
  * Parity      NO
  * Tx          OFF
  * Rx          ON  
  *
  * @retval None
  */
void MX_UART_Init(void){
    
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   // ON Timing USART
    
    USART1->BRR = BAUDRATE_115200;
    USART1->CR1 &= ~(USART_CR1_M            // WordLength 8 bit
                    | USART_CR1_PCE);       // Disable Parity    
    USART1->CR2 &= ~USART_CR2_STOP;         // 1 bit Stop
    USART1->CR1 |=  USART_CR1_RE            // ON Receive
                    | USART_CR1_RXNEIE      // Enable Interrupt RxD       
                    | USART_CR1_MME;        // Enable MuteMode
    USART1->RTOR = 24;
    USART1->CR2 |= USART_CR2_RTOEN;

    USART1->CR1 |= USART_CR1_UE;            // Enable USART
}

/**
  * @brief UART start receive header
  *
  * @retval None
  */
void Start_DMA_receive_header(void){
    
    USART1->CR1 &= ~USART_CR1_RXNEIE;       // Disable Interrupt RxD
    USART1->CR3 |= USART_CR3_DMAR;          // Enable DMA for Receiver
    USART1->RTOR = 24;
    USART1->ICR |= USART_ICR_FECF | USART_ICR_RTOCF;
    USART1->CR2 |= USART_CR2_RTOEN;
    USART1->CR1 |= USART_CR1_RTOIE;         // Receive Timeout Enable
    
    
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;      // Disable DMA channel
    DMA1->IFCR |= DMA_IFCR_CTCIF3;          // Clear flag DMA receive
    
    DMA1_Channel3->CMAR = (uint32_t)&UART_Rx_Message.HEAD;
    DMA1_Channel3->CNDTR = 3;
    DMA1_Channel3->CCR |= DMA_CCR_EN;       // Enable DMA channel
}

/**
  * @brief UART End Tx session
  *
  * @retval None
  */
void End_UART_Tx(void){
    
    USART1->CR1 &= ~USART_CR1_TXEIE;        // Disable Interrupt TxD
    while ((USART1->ISR & USART_ISR_TC) != USART_ISR_TC);   // Wait last bait
    
    GPIOB->MODER |= GPIO_MODER_MODER6;      // PB6 = ANALOG 
    
    USART1->CR1 &= ~USART_CR1_TE;           // OFF Transmitter
    USART1->CR3 &= ~USART_CR3_DMAT;         // Disable DMA for Transmitter
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;      // Disable DMA channel
}

/**
  * @brief Configure UART DMA Tx
  * @param start_byte_addr  start address of reading or writing bytes
  * @param nubbyte          number of reading or writing bytes
  * @retval None
  */
void TransmitUART(void){
    
    extern Control_Typedef CONTROL_System;
    int data_num_byte = 0;
    int length = 0;
    uint8_t start_byte_addr = UART_Rx_Message.HEAD.REG_ADDR & 0x7F;
    uint8_t* ptr_data;
    uint8_t* tx_buf =   &UART_Tx_Message.DATA.DATA_0;
    
    switch(start_byte_addr)
    {
        case 0:
            ptr_data = &CONTROL_System.BTN;
            break;
        case 1:
            ptr_data = &CONTROL_System.LED1.LEDx_R;
            break;
        case 2:
            ptr_data = &CONTROL_System.LED2.LEDx_R;
            break;
        case 3:
            ptr_data = &CONTROL_System.LED3.LEDx_R;
            break;
        case 4:
            ptr_data = &CONTROL_System.LED4.LEDx_R;
            break;
        default:
            ptr_data = &CONTROL_System.BTN;
            break;
    }

    UART_Tx_Message.HEAD.REG_ADDR = UART_Rx_Message.HEAD.REG_ADDR;
    
    length = (int)UART_Rx_Message.HEAD.H_LEN << 8;
    length |= UART_Rx_Message.HEAD.L_LEN & 0xFF;
    while (data_num_byte < length)
    {
        if ((data_num_byte >= 1) && (start_byte_addr == 0))
            break;
        if (data_num_byte >= 3)
            break;
        if (start_byte_addr >= 5)
            break;
        *tx_buf = *ptr_data;
        tx_buf++;
        ptr_data++;
        data_num_byte++;
    }
    UART_Tx_Message.HEAD.L_LEN = data_num_byte & 0xFF;
    UART_Tx_Message.HEAD.H_LEN = (data_num_byte>>8) & 0xFF;

    GPIOB->MODER &= ~GPIO_MODER_MODER6;          // clear PB6 MODER
    GPIOB->MODER |= GPIO_MODER_MODER6_1;        // PB6 = UART_TX 
    GPIOB->MODER |= GPIO_MODER_MODER6_1;        // PB6 = UART_TX 
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR6;

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;      // Disable DMA channel
    DMA1->IFCR |= DMA_IFCR_CTCIF3;          // Clear flag DMA trans
    DMA1_Channel2->CMAR = (uint32_t)&UART_Tx_Message;
    if ((UART_Rx_Message.HEAD.REG_ADDR & 0x80) == 0x80)     //  Check Read or Write
        DMA1_Channel2->CNDTR = data_num_byte+4;
    else
        DMA1_Channel2->CNDTR = 4;
    
    USART1->CR1 |= USART_CR1_TE;            // ON Transmitter
    USART1->CR3 |= USART_CR3_DMAT;          // Enable DMA for Transmitter
    DMA1_Channel2->CCR |= DMA_CCR_EN;       // Enable DMA channel
    
    USART1->CR1 |= USART_CR1_TXEIE;         // Enable Interrupt TxD    
}

/**
  * @brief UART End receive head
  *
  * @retval None
  */

void End_Receive_Head (void){
    
    extern Control_Typedef CONTROL_System;
    uint16_t length;
    
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;      // Disable DMA channel
    DMA1_Channel3->CMAR = (uint32_t)&UART_Rx_Message.DATA;
    
    length = UART_Rx_Message.HEAD.H_LEN<<8 | UART_Rx_Message.HEAD.L_LEN;
    if ((UART_Rx_Message.HEAD.REG_ADDR & 0x80) == 0x80)     //  Check Read or Write
    {
        USART1->CR3 &= ~USART_CR3_DMAR;          // Disable DMA for Receiver
        USART1->CR1 |= USART_CR1_RXNEIE;        // Enable Interrupt RxD
        if (length == 0)
        {
            TransmitUART();    
            CONTROL_System.UART_rec_f = 0;          // Receive End
            return;
        }
        else
        {
            CONTROL_System.UART_rec_f = 2;          // Receive End
            return;
        }
    }
    else
    {
        switch (length)
        {
            case 0:
                USART1->CR3 &= ~USART_CR3_DMAR;          // Disable DMA for Receiver
                USART1->CR1 |= USART_CR1_RXNEIE;        // Enable Interrupt RxD
                TransmitUART();    
                CONTROL_System.UART_rec_f = 0;          // Receive End
                return;
            case 1:
                CONTROL_System.UART_rec_f = 1;          // Receive Data
                DMA1_Channel3->CNDTR = 1;
                break;
            case 2:
                CONTROL_System.UART_rec_f = 1;          // Receive Data
                DMA1_Channel3->CNDTR = 2;
                break;
            case 3:
                CONTROL_System.UART_rec_f = 1;          // Receive Data
                DMA1_Channel3->CNDTR = 3;
                break;
            default:
                CONTROL_System.UART_rec_f = 1;          // Receive Data
                DMA1_Channel3->CNDTR = 3;
                break;
        }
    }
    DMA1_Channel3->CCR |= DMA_CCR_EN;       // Enable DMA channel
}

/**
  * @brief UART End receive data
  *
  * @retval None
  */

void End_Receive_Data (void){
    extern Control_Typedef CONTROL_System;
        
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;      // Disable DMA channel
    
    USART1->CR3 &= ~USART_CR3_DMAR;         // Disable DMA for Receiver

    USART1->CR1 &= ~USART_CR1_RTOIE;        // Disable Receiver Timeout interrupt
    USART1->CR2 &= ~USART_CR2_RTOEN;
    USART1->CR1 |= USART_CR1_RXNEIE;        // Enable Interrupt RxD
    CONTROL_System.UART_rec_f = 2;          // Receive Data
}

/**
  * @brief Processing UART Receive Data
  *
  * @retval None
  */
void Check_Uart_Receive(void){
    
    extern Control_Typedef CONTROL_System;
    
    if ((UART_Rx_Message.HEAD.REG_ADDR & 0x80) == 0x80)     //  Check Read or Write
        ReadRegister();
    else
        WriteRegister();
}

/**
  * @brief Read Device from Device Registers
  *
  * @retval None
  */
void ReadRegister(void){

    extern Control_Typedef CONTROL_System;
    uint16_t address = UART_Rx_Message.HEAD.REG_ADDR & 0x7F;
        
    switch(address)
    {
        case 0:
            GPIOB->BSRR |= GPIO_BSRR_BS_5;          // Up UI_INT 
            TransmitUART();
            break;
        case 1:           
            UART_Tx_Message.DATA.DATA_0 = CONTROL_System.LED1.LEDx_R;
            UART_Tx_Message.DATA.DATA_1 = CONTROL_System.LED1.LEDx_G;
            UART_Tx_Message.DATA.DATA_2 = CONTROL_System.LED1.LEDx_B;
            TransmitUART();
            break;
        case 2:
            UART_Tx_Message.DATA.DATA_0= CONTROL_System.LED2.LEDx_R;
            UART_Tx_Message.DATA.DATA_1= CONTROL_System.LED2.LEDx_G;
            UART_Tx_Message.DATA.DATA_2= CONTROL_System.LED2.LEDx_B;
            TransmitUART();
            break;
        case 3:
            UART_Tx_Message.DATA.DATA_0= CONTROL_System.LED3.LEDx_R;
            UART_Tx_Message.DATA.DATA_1= CONTROL_System.LED3.LEDx_G;
            UART_Tx_Message.DATA.DATA_2= CONTROL_System.LED3.LEDx_B;
            TransmitUART();
            break;
        case 4:
            UART_Tx_Message.DATA.DATA_0= CONTROL_System.LED4.LEDx_R;
            UART_Tx_Message.DATA.DATA_1= CONTROL_System.LED4.LEDx_G;
            UART_Tx_Message.DATA.DATA_2= CONTROL_System.LED4.LEDx_B;
            TransmitUART();
            break;
        default:
            TransmitUART();
            break;
    }    
}

/**
  * @brief Write Data to Device Registers
  *
  * @retval None
  */
void WriteRegister(void){
    
    extern Control_Typedef CONTROL_System;

    switch(UART_Rx_Message.HEAD.REG_ADDR & 0x7F)
    {
        case 0:
            CONTROL_System.BTN = UART_Rx_Message.DATA.DATA_0;
            TransmitUART();
            break;
        case 1:
            CONTROL_System.LED1.LEDx_R = UART_Rx_Message.DATA.DATA_0;
            CONTROL_System.LED1.LEDx_G = UART_Rx_Message.DATA.DATA_1;
            CONTROL_System.LED1.LEDx_B = UART_Rx_Message.DATA.DATA_2;
            Set_Bright(CONTROL_System.LED1,1);
            TransmitUART();        
            break;
        case 2:
            CONTROL_System.LED2.LEDx_R = UART_Rx_Message.DATA.DATA_0;
            CONTROL_System.LED2.LEDx_G = UART_Rx_Message.DATA.DATA_1;
            CONTROL_System.LED2.LEDx_B = UART_Rx_Message.DATA.DATA_2;
            Set_Bright(CONTROL_System.LED2,2);
            TransmitUART();
            break;
        case 3:
            CONTROL_System.LED3.LEDx_R = UART_Rx_Message.DATA.DATA_0;
            CONTROL_System.LED3.LEDx_G = UART_Rx_Message.DATA.DATA_1;
            CONTROL_System.LED3.LEDx_B = UART_Rx_Message.DATA.DATA_2;
            Set_Bright(CONTROL_System.LED3,3);
            TransmitUART();        
            break;
        case 4:
            CONTROL_System.LED4.LEDx_R = UART_Rx_Message.DATA.DATA_0;
            CONTROL_System.LED4.LEDx_G = UART_Rx_Message.DATA.DATA_1;
            CONTROL_System.LED4.LEDx_B = UART_Rx_Message.DATA.DATA_2;
            Set_Bright(CONTROL_System.LED4,4);
            TransmitUART();
            break;
        default:
            TransmitUART();    
            break;
    }
}


/************************ (C) COPYRIGHT FABLIGHT Electronics *****END OF FILE****/
