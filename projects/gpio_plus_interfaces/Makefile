ST_PATH := ../../../SDK
ST_CUBE := STM32CubeF0
ST_SOURCES := $(ST_PATH)/$(ST_CUBE)
INC_PATH = \
       -I$(ST_SOURCES)/Drivers/CMSIS/Include \
       -I$(ST_SOURCES)/Drivers/CMSIS/Device/ST/STM32F0xx/Include \
       -I$(ST_SOURCES)/Drivers/STM32F0xx_HAL_Driver/Inc \
       -I../include
LIB_PATH =  \
       -I$(ST_SOURCES)/Drivers/CMSIS/Lib/GCC

PART = STM32F091xC
TARGET = firmware

# Define the linker script location and chip architecture.
LD_SCRIPT = STM32F091RCTx_FLASH.ld
MCU_SPEC  = cortex-m0

# Toolchain definitions (ARM bare metal defaults)
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OC = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump
OS = arm-none-eabi-size

# Assembly directives.
ASFLAGS += -c
ASFLAGS += -O0
ASFLAGS += -mcpu=$(MCU_SPEC)
ASFLAGS += -mthumb
ASFLAGS += -Wall
# (Set error messages to appear on a single line.)
ASFLAGS += -fmessage-length=0

# C compilation directives
CFLAGS += -mcpu=$(MCU_SPEC)
CFLAGS += -mthumb
CFLAGS += -Wall
CFLAGS += -Wunused-function
CFLAGS += -Wunused-variable
CFLAGS += -g
# (Set error messages to appear on a single line.)
CFLAGS += -fmessage-length=0
# (Set system to ignore semihosted junk)
CFLAGS += --specs=nosys.specs
CFLAGS += $(INC_PATH) -D$(PART)

# Linker directives.
LSCRIPT = ./$(LD_SCRIPT)
LFLAGS += -mcpu=$(MCU_SPEC)
LFLAGS += -mthumb
LFLAGS += -Wall
# LFLAGS += --specs=nosys.specs
LFLAGS += -specs=nano.specs -lc -lnosys
LFLAGS += -nostdlib
LFLAGS += -nostartfiles
LFLAGS += -lgcc
LFLAGS += -T$(LSCRIPT)
LFLAGS += $(LIB_PATH) 

AS_SRC   =  ./startup_stm32f091xc.s
C_SRC    =  ./main.c ./system_stm32f0xx.c ./utils/clock.c ./utils/gpio.c ./utils/uart.c ./utils/i2c.c ./utils/debug.c

OBJS  = $(AS_SRC:.S=.o)
OBJS += $(C_SRC:.c=.o)

.PHONY: all
all: $(TARGET).bin


startup_stm32f091xc.o: startup_stm32f091xc.s
	$(CC) -x assembler-with-cpp $(ASFLAGS) $< -o $@

main.o: main.c
	$(CC) -c $(CFLAGS) $(INC_PATH) $< -o $@

system_stm32f0xx.o: system_stm32f0xx.c
	$(CC) -c $(CFLAGS) $(INC_PATH) $< -o $@

utils/clock.o: utils/clock.c
	$(CC) -c $(CFLAGS) $(INC_PATH) $< -o $@

utils/gpio.o: utils/gpio.c
	$(CC) -c $(CFLAGS) $(INC_PATH) $< -o $@

utils/uart.o: utils/uart.c
	$(CC) -c $(CFLAGS) $(INC_PATH) $< -o $@

utils/i2c.o: utils/i2c.c
	$(CC) -c $(CFLAGS) $(INC_PATH) $< -o $@

utils/debug.o: utils/debug.c
	$(CC) -c $(CFLAGS) $(INC_PATH) $< -o $@

$(TARGET).elf: main.o startup_stm32f091xc.o system_stm32f0xx.o utils/clock.o utils/gpio.o utils/uart.o utils/i2c.o utils/debug.o
	$(CC) $^ $(LFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $< $@
	$(OS) $<

flash: $(TARGET).bin
	st-flash --reset write $< 0x8000000

.PHONY: clean
clean:
	rm -f $(TARGET).elf
	rm -f $(TARGET).bin
	rm -f main.o
	rm -f startup_stm32f091xc.o
	rm -f system_stm32f0xx.o
	rm -f ./utils/*.o