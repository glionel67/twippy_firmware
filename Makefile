# Makefile for stm32f446
PROJECT_NAME = twippy_firmware

### JTAG and environment configuration ###
OPENOCD ?= openocd
OPENOCD_INTERFACE ?= interface/stlink-v2.cfg
OPENOCD_TARGET ?= target/stm32f4x_stlink.cfg
DEBUG = 0# 0 or 1
CPU = cortex-m4
REV = A
VERSION = 1
BIN_DIR = bin
BUILD_DIR = build

### File locations ###
# HAL and CMSIS
HAL_DIR = hal
HAL_INC = $(HAL_DIR)/inc
HAL_SRC = $(HAL_DIR)/src
CMSIS_DIR = CMSIS
CMSIS_INC = $(CMSIS_DIR)/inc
CMSIS_SRC = $(CMSIS_DIR)/src

# FreeRTOS
FREERTOS_DIR = FreeRTOS
FREERTOS_INC = $(FREERTOS_DIR)/inc
FREERTOS_SRC = $(FREERTOS_DIR)/src

# Project
PROJECT_INC = inc
PROJECT_SRC = src

# Tell make to look in that folder if it cannot find a source in the current directory
vpath %.c $(HAL_SRC)
vpath %.c $(CMSIS_SRC)
vpath %.s $(CMSIS_SRC)
vpath %.c $(FREERTOS_SRC)
vpath %.c $(PROJECT_SRC)

### Include files ###
INC_DIRS = $(HAL_INC)
INC_DIRS+= $(CMSIS_INC)
INC_DIRS+= $(FREERTOS_INC)
INC_DIRS+= $(PROJECT_INC)

INCLUDES = $(addprefix -I,$(INC_DIRS))
#$(info $$INCLUDES is [${INCLUDES}])

### Source files ###

# HAL
HAL_MODULES = rcc rcc_ex
HAL_MODULES+= cortex
HAL_MODULES+= pwr pwr_ex
HAL_MODULES+= gpio
#HAL_MODULES+= pcd pcd_ex
HAL_MODULES+= dma
HAL_MODULES+= tim tim_ex
#HAL_MODULES+= i2c i2c_ex
HAL_MODULES+= spi
HAL_MODULES+= uart usart
#HAL_MODULES+= adc
#HAL_MODULES+= flash
#HAL_MODULES+= iwdg
#HAL_MODULES+= nand nor
#HAL_MODULES+= rtc
#HAL_MODULES+= wwdg

HAL_SRCS = stm32f4xx_hal.c
HAL_SRCS+= $(foreach mod,$(HAL_MODULES),stm32f4xx_hal_$(mod).c)

# CMSIS
CMSIS_SRCS = system_stm32f4xx.c
CMSIS_SRCS+= startup_stm32f446xx.s

# FreeRTOS
FREERTOS_SRCS = list.c
FREERTOS_SRCS+= tasks.c
FREERTOS_SRCS+= queue.c
FREERTOS_SRCS+= timers.c
FREERTOS_SRCS+= heap_4.c
FREERTOS_SRCS+= port.c
FREERTOS_SRCS+= freeRtosUtils.c

# PROJECT
PROJECT_SRCS = main.c
PROJECT_SRCS+= stm32f4xx_it.c
PROJECT_SRCS+= gpio.c
PROJECT_SRCS+= uart1.c
PROJECT_SRCS+= uart2.c
#PROJECT_SRCS+= adc.c
PROJECT_SRCS+= encoders.c
#PROJECT_SRCS+= motor.c
#PROJECT_SRCS+= syscall.c
PROJECT_SRCS+= usTimer.c
PROJECT_SRCS+= spi.c
PROJECT_SRCS+= imu.c
PROJECT_SRCS+= mpu9250.c
PROJECT_SRCS+= ahrs.c

SRCS = $(PROJECT_SRCS)
SRCS+= $(HAL_SRCS)
SRCS+= $(CMSIS_SRCS)
SRCS+= $(FREERTOS_SRCS)
#$(info $$SRCS is [${SRCS}])

### Object files ###
OBJS = $(addsuffix .o,$(basename $(SRCS)))
#OBJS+= libarm_math.a
OBJS := $(addprefix $(BUILD_DIR)/,$(OBJS))
#$(info $$OBJS is [${OBJS}])

### Cross-compilation ###
AS = arm-none-eabi-as
AR = arm-none-eabi-ar
CC = arm-none-eabi-gcc
#CXX = arm-none-eabi-g++
#LD = arm-none-eabi-gcc
LD = arm-none-eabi-ld
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
NM = arm-none-eabi-nm
GDB = arm-none-eabi-gdb

# Compiler flags
MCFLAGS = -mcpu=$(CPU) -mthumb -mthumb-interwork -mlittle-endian
MCFLAGS+= -mfloat-abi=hard -mfpu=fpv4-sp-d16

# ST flags
STFLAGS = -DUSE_STDPERIPH_DRIVER
STFLAGS+= -DSTM32F4
STFLAGS+= -DSTM32F4XX
STFLAGS+= -DSTM32F446xx
STFLAGS+= -DUSE_HAL_DRIVER
#STFLAGS+= -DUSE_USB_OTG_FS
STFLAGS+= -DHSE_VALUE=8000000
#STFLAGS+= -DUSE_FULL_ASSERT
STFLAGS+= -DARM_MATH_CM4
#STFLAGS+= -D__FPU_PRESENT=1
#STFLAGS+= -D__FPU_USED=1
STFLAGS+= -D__TARGET_FPU_VFP

CFLAGS = $(MCFLAGS) $(STFLAGS)
ifeq ($(DEBUG), 1)
  CFLAGS+= -O0 -g3 -DDEBUG
else
  CFLAGS+= -Os -g3 -Werror
endif
CFLAGS+= -DBOARD_REV_$(REV) -DVERSION_$(VERSION)
CFLAGS+= $(INCLUDES)
CFLAGS+= -Wall -Wextra -Warray-bounds -Wmissing-braces
#CFLAGS+= -Wno-unused-function -ffreestanding
#CFLAGS+= -Wno-pointer-sign
#CFLAGS+= -Wdouble-promotion # Prevent promoting floats to doubles
#CFLAGS+= -ggdb
CFLAGS+= -fno-strict-aliasing $(C_PROFILE)
#CFLAGS+= -fno-math-errno
#CFLAGS+= -fno-builtin
CFLAGS+= -std=gnu11 #-std=c99
# Compiler flags to generate dependency files:
#CFLAGS+= -MD -MP -MF $(BIN_DIR)/dep/$(@).d -MQ $(@)
#Permits to remove un-used functions and global variables from output file
CFLAGS+= -ffunction-sections -fdata-sections

# Assembler flags
ASFLAGS = $(MCFLAGS) #$(INCLUDES)

# Linker flags
LIBS_INCS =
LIBS_LINK =
LDSCRIPT = STM32F446RETx_FLASH.ld
LDFLAGS = -T$(LDSCRIPT)
LDFLAGS+= $(MCFLAGS)
LDFLAGS+= $(LIBS_INCS)
LDFLAGS+= $(LIBS_LINK)
#LDFLAGS+= -nostartfiles
#LDFLAGS+= --specs=nano.specs -mcpu=cortex-m4 -mthumb
LDFLAGS+= --specs=nosys.specs
#LDFLAGS+= --specs=rdimon.specs
LDFLAGS+= -lm
#LDFLAGS+= -lc
#LDFLAGS+= -Wl,-Map=$(BIN_DIR)/$(PROJECT_NAME).map,--cref,--gc-sections

ELF = $(BIN_DIR)/$(PROJECT_NAME).elf
HEX = $(BIN_DIR)/$(PROJECT_NAME).hex
BIN = $(BIN_DIR)/$(PROJECT_NAME).bin
DFU = $(BIN_DIR)/$(PROJECT_NAME).dfu
MAP = $(BIN_DIR)/$(PROJECT_NAME).map

### Targets ###
#all: $(ELF) $(DFU)
all: $(ELF) $(HEX) $(BIN) $(DFU)

#$(BIN_DIR)/%.elf: $(OBJS)
#	@echo "Building elf"
#	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
#	@echo "elf done"
	
$(ELF): $(OBJS)
	@echo "Building elf\n"
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "elf done"

%.hex: %.elf
	@echo "elf to hex"
	$(OBJCOPY) -O ihex $^ $@
	
%.bin: %.elf
	@echo "elf to bin"
	$(OBJCOPY) $^ -O binary $@

%.dfu: %.bin
	@echo "bin to dfu"
	python2 tools/dfu-convert.py -b 0x8000000:$^ $@

$(BUILD_DIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -o $@ $^

$(BUILD_DIR)/%.o: %.s
	$(CC) -c $(CFLAGS) -o $@ $^

$(BUILD_DIR):
	mkdir -p $@

clean:
	#rm -f $(OBJS)
	find $(BUILD_DIR) -type f -name '*.o' -print0 | xargs -0 -r rm
	rm -f $(ELF) $(HEX) $(BIN) $(DFU) $(MAP)
	@echo "clean done!\n"

dfu:
	dfu-util -d 0483:df11 -a 0 -D $(DFU) -s :leave

reset_and_dfu:
	tools/make/reset-to-dfu.py
	dfu-util -d 0483:df11 -a 0 -D $(DFU) -s :leave

#Flash the stm.
flash:
	st-flash write $(BIN) 0x8000000

flash_dfu:
	dfu-util -a 0 -D $(DFU)

#STM utility targets
halt:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "\$$_TARGETNAME configure -rtos auto"

trace:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -f tools/trace/enable_trace.cfg

gdb: $(PROJECT_NAME).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt" $^

erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c "stm32f4x mass_erase 0" -c shutdown

