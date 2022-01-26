#include "stm32f10x.h"
#include "main.h"
#include "uart_api.h"
#include "stm32f10x_it.h"


void uart1_send_byte(char b) {
    uart_send_byte(USART1, b);
}

void uart2_send_byte(char b) {
    uart_send_byte(USART2, b);
}


typedef struct block_s {
    struct block_s *next;
    char data[BLOCK_SIZE];
    char rx_pointer;
    char tx_pointer;
}block_t;

block_t * p_rx;
block_t * p_tx;


void rx_handler(char b) {
    p_rx->data[(u8)(p_rx->rx_pointer)] = b;
    p_rx->rx_pointer++;
    if(p_rx->rx_pointer > BLOCK_SIZE-1) {
        p_rx->rx_pointer = 0;
        p_rx = p_rx->next;
        p_rx->rx_pointer = 0;
    }
    //p_rx->data[(u8)(p_rx->rx_pointer)] = b;
    
    //uart_send_byte(USART2, b);
}


int main(void) {    
    RCC->APB2ENR |= 0x10 | 0x04;                                    /* Enable the GPIOA (bit 2) and GPIOC (bit 8) */
    GPIOC->CRH = 0x11;                                              /* Set GPIOC Pin 8 and Pin 9 to outputs */
    GPIOA->CRL = 0x04;                                              /* Set GPIOA Pin 0 to input floating */
    
    init_uart(USART1);
    init_uart(USART2);
    set_uart_baud_rate(USART1, 9600);
    set_uart_baud_rate(USART2, 115200);
    
    //uart_set_callback_rx(USART1, uart2_send_byte);
    //uart_set_callback_rx(USART2, uart1_send_byte);
    uart_set_callback_rx(USART1, rx_handler);
    
    // ring buffer
    block_t b0;
    block_t b1;
    block_t b2;
    block_t b3;
    
    b0.next = &b1;
    b1.next = &b2;
    b2.next = &b3;
    b3.next = &b0;
    
    b0.rx_pointer = 0;
    b0.tx_pointer = 0;
    p_rx = &b0;
    p_tx = &b0;
    
    
    while(1) {
        
        if(p_tx != p_rx) {
            uart_send_byte(USART2, p_tx->data[(u8)(p_tx->tx_pointer)]);
            p_tx->tx_pointer++;
            if(p_tx->tx_pointer > BLOCK_SIZE-1) {
                p_tx->tx_pointer = 0;
                p_tx = p_tx->next;
                p_tx->tx_pointer = 0;
            }
        }
        
        //ms_delay(50);
        
        //GPIOC->ODR = 0x00000100;
        //ms_delay(100);
        //GPIOC->ODR = 0x00000200;
        //ms_delay(100);
        
        //uart_send_array(USART1, buf, 7);
        
        //uart_send_array(USART2, "Uart2\n\r", 7);
        //uart_send_array(USART1, "Uart1\n\r", 7);
        //uart_send_byte(USART2, 'a');
        
        //byte = uart_receive_byte(USART1);
        //uart_send_byte(USART2, byte);
        
    }
}



void ms_delay(int ms) {
    while(ms-- > 0) {
        volatile int x=5971;
        while(x-- > 0)
            __asm("nop");
    }
}

void delay(void) {
    int i = 100000;                 /* About 1/4 second delay */
    while (i-- > 0) {
        __asm("nop");                 /* This stops it optimising code out */
    }
}

void nmi_handler(void) {
    return;
}

void hardfault_handler(void) {
    return;
}


