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

  
### BaudRate  

To set BaudRate:
```c
set_uart_baud_rate(USARTx, baudrate);
```
example:
```c
set_uart_baud_rate(USART1, 9600);
```

### Parity

To set Parity Bit:
```c
set_uart_parity(USARTx, parity);
```
example:
```c
set_uart_parity(USART1, 0);
```

### Stop Bit

To set Stop Bit:
```c
set_uart_stop(USARTx, stop_bit);
```
where stop_bit:
 * s05 - 0.5 bit,
 * s10 - 1 bit,
 * s15 - 1.5 bit,
 * s20 - 2 bits

example:
```c
set_uart_stop(USART1, s10);
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




