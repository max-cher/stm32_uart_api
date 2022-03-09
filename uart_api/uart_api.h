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
void set_uart_baud_rate(USART_TypeDef * uart, u32 baudrate);


// 3. Setting the parity bit;
void set_uart_parity(USART_TypeDef * uart, int parity);


// 4. Setting the duration of the stop bit;
void set_uart_stop(USART_TypeDef * uart, int stop);


// 6. Data array transfer function. Data array will be sent by the transmitter;
void uart_send_array(USART_TypeDef * uart, char *data, char num_of);


void uart_send_byte(USART_TypeDef * uart, char data);

char uart_receive_byte(USART_TypeDef * uart);

void uart_rx_interrupt_enable(USART_TypeDef * uart);

void uart_tx_interrupt_enable(USART_TypeDef * uart);

// 1. Initialization of the module;
void init_uart(USART_TypeDef * uart);


// callback function protorypes
/*
void (*handle_byte_uart1_rx)(char byte);
void (*handle_byte_uart2_rx)(char byte);
void (*handle_byte_uart3_rx)(char byte);

void (*handle_byte_uart1_tx)(void);
void (*handle_byte_uart2_tx)(void);
void (*handle_byte_uart3_tx)(void);
//*/

// 5. Setting the callback function for the receiver / transmitter;
void uart_set_callback_rx(USART_TypeDef * uart, void(uart_byte_handler)(char byte));

void uart_set_callback_tx(USART_TypeDef * uart, void(uart_byte_handler)(void));


// Interrupt handlers

void USART1_IRQHandler(void);


void USART2_IRQHandler(void);


void USART3_IRQHandler(void);





