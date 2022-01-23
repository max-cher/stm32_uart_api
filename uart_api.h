#include "stm32f10x.h"


enum parity_bit{
    none,
    odd,
    even
};

enum stop_bit{
    s05,
    s10,
    s15,
    s20
};





// 2. Setting the baud rate;
void set_uart_baud_rate(USART_TypeDef * uart, u32 baudrate) {
    if(uart == USART1) {    // PCLK2 max 72 MHz
        uart->BRR = (8000000/baudrate);  //(72000000/baudrate);
    } else {                // PCLK1 max 36 MHz
        uart->BRR = (8000000/baudrate); //(36000000/baudrate);
    }
    return;
}


// 3. Setting the parity bit;
void set_uart_parity(USART_TypeDef * uart, int parity) {
    if(parity) {
        uart->CR1 |= (1<<8);
    } else {
        uart->CR1 &= ~(1<<8);
    }
    return;
}


// 4. Setting the duration of the stop bit;
void set_uart_stop(USART_TypeDef * uart, int stop) {
    if(stop == s05) {
        uart->CR2 &= ~(1<<13);
        uart->CR2 |= (1<<12);
    } else if(stop == s10) {
        uart->CR2 &= ~(1<<13);
        uart->CR2 &= ~(1<<12);
    } else if(stop == s15) {
        uart->CR2 |= (1<<13);
        uart->CR2 |= (1<<12);
    } else if(stop == s20) {
        uart->CR2 |= (1<<13);
        uart->CR2 &= ~(1<<12);
    }
    return;
}


// 5. Setting the callback function for the receiver / transmitter;
void set_callback() {
    
    return;
}

// 6. Data array transfer function. Data array will be sent by the transmitter;
void uart_send_array(USART_TypeDef * uart, u8 *data, u8 num_of) {
    for(uint32_t i = 0; i < num_of; i++) {
        while ((uart->SR & USART_SR_TXE) == 0) {}
        uart->DR = data[i];
    }
    return;
}


// 1. Initialization of the module;
void init_uart(USART_TypeDef * uart) {
    if(uart == USART1) {                            // PCLK2 max 72 MHz
        // RX PA10
        // TX PA9
        
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;         // enable port A
        
        GPIOA->CRH &= (~GPIO_CRH_CNF9_0);           // TX
        GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);
        
        GPIOA->CRH &= (~GPIO_CRH_CNF10_0);          // RX
        GPIOA->CRH |= GPIO_CRH_CNF10_1;
        GPIOA->CRH &= (~(GPIO_CRH_MODE10));
        GPIOA->BSRR |= GPIO_ODR_ODR10;
        
        USART1->CR1 = USART_CR1_UE;                 // enable uart
        
        set_uart_baud_rate(uart, 9600);             // set default baudrate
        
        USART1->CR1 |= USART_CR1_TE | USART_CR1_RE ; // enable RX and TX
        USART1->CR2 = 0;
        USART1->CR3 = 0;
        
    } else if(uart == USART2) {                       // PCLK1 max 36 MHz
        // RX PA3
        // TX PA2
        
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        
        
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;          // enable PORT 
        
        GPIOA->CRL &= (~GPIO_CRL_CNF2_0);           // TX
        GPIOA->CRL |= (GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2);
        
        GPIOA->CRL &= (~GPIO_CRL_CNF3_0);          // RX
        GPIOA->CRL |= GPIO_CRL_CNF3_1;
        GPIOA->CRL &= (~(GPIO_CRL_MODE3));
        GPIOA->BSRR |= GPIO_ODR_ODR3;
        
        USART2->CR1 = USART_CR1_UE;                 // enable uart
        
        set_uart_baud_rate(uart, 9600);             // set default baudrate
        
        USART2->CR1 |= USART_CR1_TE | USART_CR1_RE ; // enable RX and TX
        USART2->CR2 = 0;
        USART2->CR3 = 0;
        
    } else if(uart == USART3) {                       // PCLK1 max 36 MHz
        // RX PC11
        // TX PC10
        
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
        
        RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;          // enable PORT 
        
        GPIOC->CRH &= (~GPIO_CRH_CNF10_0);           // TX
        GPIOC->CRH |= (GPIO_CRH_CNF10_1 | GPIO_CRL_MODE2);
        
        GPIOC->CRH &= (~GPIO_CRH_CNF10_0);          // RX
        GPIOC->CRH |= GPIO_CRH_CNF10_1;
        GPIOC->CRH &= (~(GPIO_CRH_MODE10));
        GPIOC->BSRR |= GPIO_ODR_ODR10;
        
        USART3->CR1 = USART_CR1_UE;                 // enable uart
        
        set_uart_baud_rate(uart, 9600);             // set default baudrate
        
        USART3->CR1 |= USART_CR1_TE | USART_CR1_RE ; // enable RX and TX
        USART3->CR2 = 0;
        USART3->CR3 = 0;
        
    }// else if(uart == USART4) {                       // PCLK1 max 36 MHz
    //    RCC->APB1ENR |= RCC_APB1ENR_USART4EN;
    //} else if(uart == USART5) {                       // PCLK1 max 36 MHz
    //    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
    //}
    
    
    
    
    
    return;
}

