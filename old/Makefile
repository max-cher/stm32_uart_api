#CC      = arm-elf-gcc
#LD      = arm-elf-ld -v
#CP      = arm-elf-objcopy
#OD      = arm-elf-objdump

#STARTUP = startup_stm32f10x.s 
STARTUP = STM32F10x.s 
#ASMSRC = $(STARTUP)



CC		= arm-none-eabi-gcc
LD      = arm-none-eabi-ld -v
CP      = arm-none-eabi-objcopy
OD      = arm-none-eabi-objdump
OBJCOPY=arm-none-eabi-objcopy

#AS      = arm-none-eabi-as

CFLAGS  =  -I./ -c -fno-common -Os -mcpu=cortex-m3 -mthumb -std=c99
LFLAGS  = -Tstm32.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS = -S
#AFLAGS  = -ahls -mapcs-32

SRCS=$(PROJ_NAME).c

PROJ_NAME=main

all: $(PROJ_NAME).bin

clean:
	-rm -f *.lst *.o *.elf *.lst *.bin *.hex

$(PROJ_NAME).bin: $(PROJ_NAME).elf
	@ echo "...copying"
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
#	$(CP) $(CPFLAGS) blinky.elf blinky.bin
#	$(OD) $(ODFLAGS) blinky.elf > blinky.lst

$(PROJ_NAME).elf: $(PROJ_NAME).o stm32f10x_it.o stm32.ld
	@ echo "..linking"
	$(LD) $(LFLAGS) -o $(PROJ_NAME).elf $(PROJ_NAME).o stm32f10x_it.o

$(PROJ_NAME).o: $(PROJ_NAME).c $(PROJ_NAME).h uart_api.h
	@ echo ".compiling"
	$(CC) $(CFLAGS) $(PROJ_NAME).c uart_api.h

#crt.o : $(ASMSRC)
#	$(AS) $(AFLAGS) $(ASMSRC) -o crt.o 

stm32f10x_it.o: stm32f10x_it.c stm32f10x_it.h
	$(CC) $(CFLAGS) stm32f10x_it.c stm32f10x_it.h


install: $(PROJ_NAME).bin
	st-flash write $(PROJ_NAME).bin 0x8000000
