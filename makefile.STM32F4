OUTDIR := out
OBJDIR := $(OUTDIR)/obj
BINDIR := $(OUTDIR)/bin

ELF := $(BINDIR)/$(notdir $(CURDIR)).elf
BIN := $(BINDIR)/$(notdir $(CURDIR)).bin
MAP := $(OUTDIR)/$(notdir $(CURDIR)).map
LSS := $(OUTDIR)/$(notdir $(CURDIR)).lss

INC := src

DEF :=

LIBDIR :=
LIB :=
LINKED_OBJ :=

MODULES := common
MODULES += drv
MODULES += hal/STM32F4
MODULES += third_party/FatFs
MODULES += third_party/FreeRTOS
MODULES += third_party/TraceRecorder
MODULES += ul

#*******************************************************************************
FLASHER := JLink
#FLASHER = openocd
#FLASHER = ST-LINK_CLI

JLINK_PARAM := -device STM32F407VG -if SWD -speed auto

OPENOCD_PARAM := -f interface/stlink-v2.cfg
OPENOCD_PARAM += transport select hla_swd
OPENOCD_PARAM += -f target/stm32f4x.cfg

OPENOCD_PARAM_DEBUG := $(OPENOCD_PARAM)
OPENOCD_PARAM_DEBUG += -c "gdb_port 2331"
OPENOCD_PARAM_DEBUG += -c "debug_level 2"
OPENOCD_PARAM_DEBUG += -c "set WORKAREASIZE 0x2000"
OPENOCD_PARAM_DEBUG += -c "reset_config srst_only"

OS := Windows

CC := arm-none-eabi-gcc
CPP := arm-none-eabi-g++
AS := arm-none-eabi-gcc
LD := arm-none-eabi-gcc
GDB := arm-none-eabi-gdb
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE := arm-none-eabi-size

CFLAGS := -mcpu=cortex-m4
CFLAGS += -mthumb
CFLAGS += -O0 -g3
#CFLAGS += -std=gnu99
CFLAGS += -ffunction-sections -fdata-sections
#CFLAGS += -Wno-packed-bitfield-compat
CFLAGS += -Wundef -Wcast-align
CFLAGS += -mfloat-abi=softfp
CFLAGS += -mfpu=fpv4-sp-d16
#CFLAGS += -mfloat-abi=hard

CPPFLAGS := $(CFLAGS)
CPPFLAGS += -fno-exceptions
CPPFLAGS += -fno-rtti
CPPFLAGS += -funsigned-bitfields
CPPFLAGS += -fshort-enums
CPPFLAGS += -fno-threadsafe-statics
CPPFLAGS += -fno-use-cxa-atexit

AFLAGS := -x assembler-with-cpp

LDFLAGS := -Tsrc/hal/STM32F4/STM32F40_41xxx.ld
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mthumb
LDFLAGS += -nostartfiles
LDFLAGS += --specs=nano.specs
#LDFLAGS += --specs=nosys.specs
#LDFLAGS += --specs=rdimon.specs
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map="$(MAP)",--cref

SRC := main.cpp

-include $(patsubst %, src/%/makefile, $(MODULES))

INC := $(addprefix -I,$(strip $(INC)))
DEF := $(addprefix -D,$(strip $(DEF)))
LIB := $(addprefix -l,$(strip $(LIB)))
LIBDIR := $(addprefix -L,$(strip $(LIBDIR)))
LINKED_OBJ := $(strip $(LINKED_OBJ))

ifdef LIBDIR
LDFLAGS += $(LIBDIR)
endif
ifdef LIB
LDFLAGS += -Wl,--start-group $(LIB) -Wl,--end-group
endif

#*******************************************************************************

OBJ := $(patsubst %.c, $(OBJDIR)/%.o, $(filter %.c,$(SRC)))
OBJ += $(patsubst %.cpp, $(OBJDIR)/%.o, $(filter %.cpp,$(SRC)))
OBJ += $(patsubst %.s, $(OBJDIR)/%.o, $(filter %.s,$(SRC)))

.PHONY: all clean erase flash reset debug check_style

all:
	$(MAKE) --no-print-directory $(ELF)
	$(MAKE) --no-print-directory $(BIN)
	$(MAKE) --no-print-directory $(LSS)
	$(SIZE) $(ELF)

clean:
ifeq ($(OS),Windows)
	if exist "$(OUTDIR)" rmdir /s /q "$(OUTDIR)"
	del /q /f script.jlink 2>nul
endif
ifeq ($(OS),Linux)
	rm -rf $(OUTDIR)
	rm -f script.jlink
endif

erase:
ifeq ($(FLASHER),JLink)
	@echo erase>script.jlink
	@echo q>>script.jlink
	$(FLASHER) $(JLINK_PARAM) -CommanderScript script.jlink
endif
ifeq ($(FLASHER),openocd)
	$(FLASHER) $(OPENOCD_PARAM) -c "init; halt; stm32f4x mass_erase 0; exit"
endif
ifeq ($(FLASHER),ST-LINK_CLI)
	$(FLASHER) -c swd -me
endif

flash:
ifeq ($(FLASHER),JLink)
	@echo r>script.jlink
	@echo loadbin $(BIN), 0 >>script.jlink
	@echo r>>script.jlink
	@echo q>>script.jlink
	$(FLASHER) $(JLINK_PARAM) -CommanderScript script.jlink
endif
ifeq ($(FLASHER),openocd)
	$(FLASHER) $(OPENOCD_PARAM) -c "init; halt; program $(BIN) 0x08000000 \
	verify; reset run; exit"
endif
ifeq ($(FLASHER),ST-LINK_CLI)
	$(FLASHER) -c swd -p $(BIN) 0x08000000 -v -rst
endif

reset:
ifeq ($(FLASHER),JLink)
	@echo r>script.jlink
	@echo q>>script.jlink
	$(FLASHER) $(JLINK_PARAM) -CommanderScript script.jlink
endif
ifeq ($(FLASHER),openocd)
	$(FLASHER) $(OPENOCD_PARAM) -c "init; reset run; exit"
endif
ifeq ($(FLASHER),ST-LINK_CLI)
	$(FLASHER) -Rst
endif

debug:
	$(MAKE) all
	$(MAKE) flash
ifeq ($(FLASHER),JLink)
	jlinkGDBserverCL $(JLINK_PARAM) -vd -singlerun
#	jlinkGDBserverCL $(JLINK_PARAM) -vd -singlerun -rtos GDBServer/RTOSPlugin_FreeRTOS
endif
ifeq ($(FLASHER),openocd)
	$(FLASHER) $(OPENOCD_PARAM_DEBUG)
endif

check_style:
	clang-format -style=file $(addprefix src/,$(SRC))

$(OBJDIR)/%.o: src/%.c
ifeq ($(OS),Windows)
	if not exist "$(@D)" mkdir "$(@D)"
else
	mkdir -p "$(@D)"
endif
	$(CC) $(DEF) $(INC) $(CFLAGS) -c $^ -o $@

$(OBJDIR)/%.o: src/%.cpp
ifeq ($(OS),Windows)
	if not exist "$(@D)" mkdir "$(@D)"
else
	mkdir -p "$(@D)"
endif
	$(CPP) $(DEF) $(INC) $(CFLAGS) $(CPPFLAGS) -c $^ -o $@

$(OBJDIR)/%.o: src/%.s
ifeq ($(OS),Windows)
	if not exist "$(@D)" mkdir "$(@D)"
else
	mkdir -p "$(@D)"
endif
	$(AS) $(DEF) $(INC) $(CFLAGS) $(AFLAGS) -c $^ -o $@

$(ELF): $(OBJ) $(LINKED_OBJ)
ifeq ($(OS),Windows)
	if not exist "$(@D)" mkdir "$(@D)"
else
	mkdir -p "$(@D)"
endif
	$(LD) $(LDFLAGS) $^ -o $@

$(BIN): $(ELF)
ifeq ($(OS),Windows)
	if not exist "$(@D)" mkdir "$(@D)"
else
	mkdir -p "$(@D)"
endif
	$(OBJCOPY) -O binary $< $@

$(LSS): $(ELF)
ifeq ($(OS),Windows)
	if not exist "$(@D)" mkdir "$(@D)"
else
	mkdir -p "$(@D)"
endif
	$(OBJDUMP) -dC $< >> $@
