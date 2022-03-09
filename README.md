# stm32_uart_api
uart api for stm32 uc
  
  
## Init UART

For initializing uart:
```c
init_uart(USARTx);
```

where USARTx - uart number:  
 * USART1  
 * USART2  
 * USART3  
 * UART4  
 * UART5  

example:  
```c
    init_uart(USART1);
```

Default settings:
 * baudrate = 9600
 * parity bit = no
 * stop bit = 1 bit
 * rx and tx interrupts disabled

  
## Baudrate  

To set baudrate:
```c
set_uart_baud_rate(USARTx, baudrate);
```
example:
```c
set_uart_baud_rate(USART1, 9600);
```







## Requirements  

compile:
* arm-none-eabi-gcc

flash:
* st-flash



## Building

```bash
make  
make flash
```




