
#include "uart_api.h"


void set_uart_baud_rate(USART_TypeDef * uart, u32 baudrate) {
    if(uart == USART1) {    // PCLK2 max 72 MHz
        uart->BRR = (24000000/baudrate);  //(72000000/baudrate);
    } else {                // PCLK1 max 36 MHz
        uart->BRR = (12000000/baudrate); //(36000000/baudrate);
    }
    return;
}


void set_uart_parity(USART_TypeDef * uart, int parity) {
    if(parity) {
        uart->CR1 |= (1<<8);
    } else {
        uart->CR1 &= ~(1<<8);
    }
    return;
}


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


void uart_send_array(USART_TypeDef * uart, char *data, char num_of) {
    for(uint32_t i = 0; i < num_of; i++) {
        while ((uart->SR & USART_SR_TXE) == 0) {}
        uart->DR = data[i];
    }
    return;
}


void uart_send_byte(USART_TypeDef * uart, char data) {
    while ((uart->SR & USART_SR_TXE) == 0) {}
    uart->DR = data;
    return;
}


char uart_receive_byte(USART_TypeDef * uart) {
    char data;
    while ((uart->SR & USART_SR_RXNE) == 0) {}
    data = uart->DR;
    return data;
}


void uart_rx_interrupt_enable(USART_TypeDef * uart) {
    uart->CR1 |= USART_CR1_RXNEIE;
    //uart->CR1 |= USART_CR1_TXEIE;
    
    if(uart == USART1) { 
        NVIC_EnableIRQ (USART1_IRQn);
    } else if(uart == USART2) {
        NVIC_EnableIRQ (USART2_IRQn);
    } else if(uart == USART2) {
        NVIC_EnableIRQ (USART3_IRQn);
    }
    return;
}


void uart_tx_interrupt_enable(USART_TypeDef * uart) {
    //uart->CR1 |= USART_CR1_RXNEIE;
    //uart->CR1 |= USART_CR1_TXEIE;
    
    if(uart == USART1) { 
    //    NVIC_EnableIRQ (USART1_IRQn);
    } else if(uart == USART2) {
    //    NVIC_EnableIRQ (USART2_IRQn);
    } else if(uart == USART2) {
    //    NVIC_EnableIRQ (USART3_IRQn);
    }
    return;
}


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
        
    } else if(uart == UART4) {                       // PCLK1 max 36 MHz
        RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    } else if(uart == UART5) {                       // PCLK1 max 36 MHz
        RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    }
    
    return;
}



void (*handle_byte_uart1_rx)(char byte);
void (*handle_byte_uart2_rx)(char byte);
void (*handle_byte_uart3_rx)(char byte);

void (*handle_byte_uart1_tx)(void);
void (*handle_byte_uart2_tx)(void);
void (*handle_byte_uart3_tx)(void);



void uart_set_callback_rx(USART_TypeDef * uart, void(uart_byte_handler)(char byte)) {
    if(uart == USART1) {
        handle_byte_uart1_rx = uart_byte_handler;
    } else if(uart == USART2) {
        handle_byte_uart2_rx = uart_byte_handler;
    } else if(uart == USART3) {
        handle_byte_uart3_rx = uart_byte_handler;
    }
    uart_rx_interrupt_enable(uart);
    return;
}



void uart_set_callback_tx(USART_TypeDef * uart, void(uart_byte_handler)(void)) {
    if(uart == USART1) {
        handle_byte_uart1_tx = uart_byte_handler;
    } else if(uart == USART2) {
        handle_byte_uart2_tx = uart_byte_handler;
    } else if(uart == USART3) {
        handle_byte_uart3_tx = uart_byte_handler;
    }
    uart_rx_interrupt_enable(uart);
    return;
}


void USART1_IRQHandler(void) {
    
    GPIOC->ODR = 0x00000200;
    char b;
    if((USART1->CR1 & USART_CR1_RXNEIE) && (USART1->SR & USART_SR_RXNE)) {
        b = uart_receive_byte(USART1);
        handle_byte_uart1_rx(b);
    }
    if((USART1->CR1 & USART_CR1_TXEIE) && (USART1->SR & USART_SR_TXE)) {
        
    }
    
    
}

void USART2_IRQHandler(void) {
    GPIOC->ODR = 0x00000100;
    char b;
    if((USART2->CR1 & USART_CR1_RXNEIE) && (USART2->SR & USART_SR_RXNE)) {
        b = uart_receive_byte(USART2);
        handle_byte_uart2_rx(b);
    }
    if((USART2->CR1 & USART_CR1_TXEIE) && (USART2->SR & USART_SR_TXE)) {
        
    }
}

void USART3_IRQHandler(void) {
    GPIOC->ODR = 0x00000100;
    char b;
    if((USART3->CR1 & USART_CR1_RXNEIE) && (USART3->SR & USART_SR_RXNE)) {
        b = uart_receive_byte(USART3);
        handle_byte_uart3_rx(b);
    }
    if((USART3->CR1 & USART_CR1_TXEIE) && (USART3->SR & USART_SR_TXE)) {
        
    }
}

