#CC      = arm-elf-gcc
#LD      = arm-elf-ld -v
#CP      = arm-elf-objcopy
#OD      = arm-elf-objdump

CC		= arm-none-eabi-gcc
LD      = arm-none-eabi-ld -v
CP      = arm-none-eabi-objcopy
OD      = arm-none-eabi-objdump
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  =  -I./ -c -fno-common -Os -mcpu=cortex-m3 -mthumb
LFLAGS  = -Tstm32.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS = -S

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

$(PROJ_NAME).elf: $(PROJ_NAME).o stm32.ld
	@ echo "..linking"
	$(LD) $(LFLAGS) -o $(PROJ_NAME).elf $(PROJ_NAME).o

$(PROJ_NAME).o: $(PROJ_NAME).c $(PROJ_NAME).h uart_api.h
	@ echo ".compiling"
	$(CC) $(CFLAGS) $(PROJ_NAME).c uart_api.h

install: $(PROJ_NAME).bin
	st-flash write $(PROJ_NAME).bin 0x8000000
