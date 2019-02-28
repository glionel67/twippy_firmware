# Makefile for stm32f446
PROJECT_NAME = twippy_firmware

### JTAG and environment configuration ###
OPENOCD ?= openocd
OPENOCD_INTERFACE ?= interface/stlink-v2.cfg
OPENOCD_TARGET ?= target/stm32f4x_stlink.cfg
DEBUG = 0# 0 or 1
CPU = cortex-m4
FPU = fpv4-sp-d16
REV = A
VERSION = 1
BIN_DIR = bin
BUILD_DIR = build

### File locations ###
# HAL
HAL_DIR = hal
HAL_INC = $(HAL_DIR)/inc
HAL_SRC = $(HAL_DIR)/src

# CMSIS
CMSIS_DIR = CMSIS
CMSIS_INC = $(CMSIS_DIR)/inc
CMSIS_SRC = $(CMSIS_DIR)/src

# FreeRTOS
FREERTOS_DIR = FreeRTOS
FREERTOS_INC = $(FREERTOS_DIR)/inc
FREERTOS_SRC = $(FREERTOS_DIR)/src

# Mavlink
MAVLINK_DIR = mavlink
MAVLINK_INC = $(MAVLINK_DIR)
MAVLINK_SRC = $(MAVLINK_DIR)

# Project
PROJECT_INC = inc
PROJECT_SRC = src

# Tell make to look in that folder if it cannot find a source in the current directory
vpath %.c $(HAL_SRC)
vpath %.c $(CMSIS_SRC)
vpath %.s $(CMSIS_SRC)
vpath %.c $(FREERTOS_SRC)
vpath %.c $(PROJECT_SRC)
vpath %.cpp $(PROJECT_SRC)

### Include files ###
INC_DIRS = $(HAL_INC)
INC_DIRS+= $(CMSIS_INC)
INC_DIRS+= $(FREERTOS_INC)
INC_DIRS+= $(MAVLINK_INC)
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
HAL_MODULES+= i2c i2c_ex
HAL_MODULES+= spi
HAL_MODULES+= uart usart
HAL_MODULES+= adc adc_ex
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
PROJECT_SRCS = main_test.c # main.c
PROJECT_SRCS+= stm32f4xx_it.c
PROJECT_SRCS+= gpio.c
PROJECT_SRCS+= uart1.c uart2.c uart3.c
PROJECT_SRCS+= i2c1.c i2c2.c
PROJECT_SRCS+= adc.c
PROJECT_SRCS+= servo.c
PROJECT_SRCS+= led.c
PROJECT_SRCS+= encoder.c
PROJECT_SRCS+= motor.c
PROJECT_SRCS+= motor_control.c
PROJECT_SRCS+= syscalls.c
PROJECT_SRCS+= usTimer.c
PROJECT_SRCS+= spi.c
PROJECT_SRCS+= imu.c
PROJECT_SRCS+= mpu9250.c
PROJECT_SRCS+= ahrs.c
PROJECT_SRCS+= pid_controller.c
PROJECT_SRCS+= balance_control.c
PROJECT_SRCS+= buzzer.c
PROJECT_SRCS+= mavlink_uart.c
#PROJECT_SRCS+= battery.c
#PROJECT_SRCS+= com.c
#PROJECT_SRCS+= led.c
PROJECT_SRCS+= nmea.c
PROJECT_SRCS+= gps.c

SRCS = $(PROJECT_SRCS)
SRCS+= $(HAL_SRCS)
SRCS+= $(CMSIS_SRCS)
SRCS+= $(FREERTOS_SRCS)
#$(info $$SRCS is [${SRCS}])

#CXXSOURCES+=$(shell find -L $(PROJECT_SRC) -name '*.cpp')
CXX_SRCS = mavlink_interface.cpp

### Object files ###
OBJS = $(addsuffix .o,$(basename $(SRCS)))
OBJS+= $(addsuffix .o,$(basename $(CXX_SRCS)))
#OBJS+= libarm_math.a
OBJS := $(addprefix $(BUILD_DIR)/,$(OBJS))
#$(info $$OBJS is [${OBJS}])


### Cross-compilation ###
#PREFIX = arm-none-eabi-
AS = arm-none-eabi-as
AR = arm-none-eabi-ar
CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
LD = arm-none-eabi-ld
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
NM = arm-none-eabi-nm
GDB = arm-none-eabi-gdb

# Compiler flags
MCFLAGS = -mcpu=$(CPU) -mthumb -mthumb-interwork -mfpu=$(FPU)
MCFLAGS+= -mfloat-abi=hard -mlittle-endian

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

# Def flags
DEFS = -DBOARD_REV_$(REV) -DVERSION_$(VERSION)
DEFS+= -Wall -Wextra -Warray-bounds -Wmissing-braces
DEFS+= -fno-strict-aliasing $(C_PROFILE)
DEFS+= -ffunction-sections -fdata-sections
#DEFS+= -Wno-unused-function -ffreestanding
#DEFS+= -Wno-pointer-sign
#DEFS+= -Wdouble-promotion # Prevent promoting floats to doubles
#DEFS+= -fno-math-errno
#DEFS+= -fno-builtin
#DEFS+= -ggdb
ifeq ($(DEBUG), 1)
  DEFS+= -O0 -g3 -DDEBUG
else
  DEFS+= -Os -g3 -Werror
endif

# C flags
CFLAGS = $(MCFLAGS) $(STFLAGS) $(DEFS) $(INCLUDES)
CFLAGS+= -std=gnu11 #-std=c99
# Compiler flags to generate dependency files:
#CFLAGS+= -MD -MP -MF $(BIN_DIR)/dep/$(@).d -MQ $(@)
#Permits to remove un-used functions and global variables from output file

# C++ flags
CXXFLAGS= $(MCFLAGS) $(STFLAGS) $(DEFS) $(INCLUDES)
CXXFLAGS+= -std=c++11

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
LDFLAGS+= --specs=nano.specs
LDFLAGS+= -u_printf_float
#LDFLAGS+=-mcpu=cortex-m4 -mthumb
LDFLAGS+= --specs=nosys.specs
#LDFLAGS+= --specs=rdimon.specs
LDFLAGS+= -lm
LDFLAGS+= -lc
LDFLAGS+= -lnosys 
#LDFLAGS+= -Wl,-Map=$(BIN_DIR)/$(PROJECT_NAME).map,--cref -Wl,--gc-sections

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
	@echo "Building elf...\n"
	#$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	#$(CXX) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	$(CXX) $(LDFLAGS) $(OBJS) -o $@
	@echo "Building elf done!\n"

%.hex: %.elf
	@echo "elf to hex...\n"
	$(OBJCOPY) -O ihex $^ $@
	@echo "elf to hex done!\n"
	
%.bin: %.elf
	@echo "elf to bin...\n"
	$(OBJCOPY) $^ -O binary $@
	@echo "elf to bin done!\n"

%.dfu: %.bin
	@echo "bin to dfu..\n"
	python2 tools/dfu-convert.py -b 0x8000000:$^ $@
	@echo "bin to dfu done!\n"

$(BUILD_DIR)/%.o: %.cpp
	@echo "Compiling C++...\n"
	mkdir -p $(dir $@)
	$(CXX) -c $(CXXFLAGS) -o $@ $^
	@echo "Compiled C++!\n"

$(BUILD_DIR)/%.o: %.c
	@echo "Compiling C...\n"
	mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -o $@ $^
	@echo "Compiled C!\n"

$(BUILD_DIR)/%.o: %.s
	@echo "Assembling...\n"
	$(CC) -c $(CFLAGS) -o $@ $^
	@echo "Assembled!\n"

$(BUILD_DIR):
	mkdir -p $@

clean:
	@echo "Cleaning...\n"
	#rm -f $(OBJS)
	find $(BUILD_DIR) -type f -name '*.o' -print0 | xargs -0 -r rm
	rm -f $(ELF) $(HEX) $(BIN) $(DFU) $(MAP)
	@echo "Clean done!\n"

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

