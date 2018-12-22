MODULES := src \
	src/common \
	src/drv \
	src/hal/STM32F4 \
	src/hal/STM32F4/CMSIS \
	src/third_party/FatFs \
	src/third_party/FreeRTOS \
	src/third_party/simplelink \
	src/third_party/TraceRecorder \
	src/ul

ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

OUTDIR := $(ROOT)/out
OBJDIR := $(OUTDIR)/obj$(patsubst $(abspath $(ROOT))%,%,$(CURDIR))
BINDIR := $(OUTDIR)/bin

ELF := $(BINDIR)/$(notdir $(CURDIR)).elf
BIN := $(BINDIR)/$(notdir $(CURDIR)).bin
MAP := $(OUTDIR)/$(notdir $(CURDIR)).map
LSS := $(OUTDIR)/$(notdir $(CURDIR)).lss

GLOBAL_INC := $(ROOT)/src \
	$(ROOT)/src/hal/STM32F4 \
	$(ROOT)/src/hal/STM32F4/CMSIS/core-support \
	$(ROOT)/src/third_party/FreeRTOS/include \
	$(ROOT)/src/third_party/FreeRTOS/portable/ARM_CM4 \
	$(ROOT)/src/third_party/TraceRecorder/include \
	$(ROOT)/src/third_party/TraceRecorder/streamports/Jlink_RTT/include \
	$(ROOT)/src/third_party/TraceRecorder/config

GLOBAL_DEF := STM32F407xx

GLOBAL_C_CPP_FLAGS := -O0 -g3 \
	-Wall \
	-ffunction-sections -fdata-sections \
	-mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16
	#-mfloat-abi=hard

GLOBAL_CFLAGS := -std=c99

GLOBAL_CPPFLAGS := -std=c++11 \
	-fno-exceptions -fno-rtti \
	-fno-threadsafe-statics -fno-use-cxa-atexit

GLOBAL_AFLAGS :=

LDFLAGS := -Tsrc/hal/STM32F4/STM32F40_41xxx.ld \
	-mcpu=cortex-m4 -mthumb \
	-nostartfiles \
	--specs=nano.specs \
	-Wl,--gc-sections \
	-Wl,-Map="$(MAP)",--cref
	#--specs=rdimon.specs
	#--specs=nosys.specs

CC := arm-none-eabi-gcc
CPP := arm-none-eabi-g++
AS := arm-none-eabi-gcc -x assembler-with-cpp
LD := arm-none-eabi-gcc
GDB := arm-none-eabi-gdb
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE := arm-none-eabi-size

FLASHER := JLink
#FLASHER = openocd
#FLASHER = ST-LINK_CLI

JLINK_PARAM := -device STM32F407VG -if SWD -speed auto

OPENOCD_PARAM := -f interface/stlink-v2.cfg \
	transport select hla_swd \
	-f target/stm32f4x.cfg

OPENOCD_PARAM_DEBUG := $(OPENOCD_PARAM) \
	-c "gdb_port 2331" \
	-c "debug_level 2" \
	-c "set WORKAREASIZE 0x2000" \
	-c "reset_config srst_only"

ifeq ($(OS),Windows_NT)
define MKDIR
	if not exist "$(1)" mkdir "$(1)"
endef
define RMDIR
	if exist "$(1)" rmdir /s /q "$(1)"
endef
define RM
	del /q "$(1)" 2>nul
endef
else
define MKDIR
	mkdir -p $(1)
endef
define RMDIR
	rm -rf $(1)
endef
define RM
	rf -f "$(1)"
endef
endif
