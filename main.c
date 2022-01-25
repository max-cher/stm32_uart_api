#include "stm32f10x.h"
#include "main.h"
#include "uart_api.h"
#include "stm32f10x_it.h"





int main(void) {    
    RCC->APB2ENR |= 0x10 | 0x04;                                    /* Enable the GPIOA (bit 2) and GPIOC (bit 8) */
    GPIOC->CRH = 0x11;                                              /* Set GPIOC Pin 8 and Pin 9 to outputs */
    GPIOA->CRL = 0x04;                                              /* Set GPIOA Pin 0 to input floating */
        
        
        
    init_uart(USART1);
    init_uart(USART2);
    set_uart_baud_rate(USART1, 9600);
    set_uart_baud_rate(USART2, 115200);
    
    uart_interrupt_enable(USART1);
    uart_interrupt_enable(USART2);
    
    
    while(1) {
        GPIOC->ODR = 0x00000100;
        ms_delay(50);
        GPIOC->ODR = 0x00000200;
        ms_delay(50);
        
        //uart_send_array(USART1, buf, 7);
        
        //uart_send_array(USART2, "World\n\r", 7);
        
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


