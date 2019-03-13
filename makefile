include header.mk

OBJ :=
ALL_LIBDIR :=
ALL_LIB :=
ALL_LINKED_OBJ :=

define INCLUDE_MODULE
$(eval include $(module)/makefile)
$(eval OBJ += $(patsubst %.c,$(OBJDIR)/$(module)/%.o,$(filter %.c,$(SRC))) \
	$(patsubst %.cpp,$(OBJDIR)/$(module)/%.o,$(filter %.cpp,$(SRC))) \
	$(patsubst %.s,$(OBJDIR)/$(module)/%.o,$(filter %.s,$(SRC))) \
	$(patsubst %.S,$(OBJDIR)/$(module)/%.o,$(filter %.S,$(SRC))) \
)
$(eval ALL_LIBDIR += $(addprefix -L$(module)/,$(LIBDIR)))
$(eval ALL_LIB += $(addprefix -l,$(LIB)))
$(eval ALL_LINKED_OBJ += $(addprefix $(module)/,$(LINKED_OBJ)))
endef

# Include module and save its SRC.o to OBJ for future linking (repeat for each module)
$(foreach module,$(MODULES),$(eval $(call INCLUDE_MODULE,$(module))))

ALL_LIBDIR := $(strip $(ALL_LIBDIR))
ALL_LIB := $(strip $(ALL_LIB))

define COMPILE_MODULE
	+$(MAKE) -j $(NUMBER_OF_PROCESSORS) --no-print-directory -C $(module)
	
endef

.PHONY: all clean erase flash reset debug

all:
	$(foreach module,$(MODULES),$(call COMPILE_MODULE,$(module)))
	+$(MAKE) -j $(NUMBER_OF_PROCESSORS) --no-print-directory $(ELF)
	+$(MAKE) -j $(NUMBER_OF_PROCESSORS) --no-print-directory $(BIN)
	+$(MAKE) -j $(NUMBER_OF_PROCESSORS) --no-print-directory $(LSS)
	$(SIZE) $(ELF)

clean:
	$(call RMDIR,$(OUTDIR))
	$(call RM,script.jlink)

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
ifeq ($(FLASHER),esptool)
	$(FLASHER) $(ESPTOOL_PARAM) erase_flash
	$(FLASHER) $(ESPTOOL_PARAM) write_flash -fs 512KB -ff 40m -fm qio 0x7C000 src/hal/ESP8266/ESP8266_RTOS_SDK/bin/esp_init_data_default.bin
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
ifeq ($(FLASHER),esptool)
	$(FLASHER) $(ESPTOOL_PARAM) write_flash -fs 512KB -ff 40m -fm qio 0x00000 $(BINDIR)/$(notdir $(CURDIR))-0x00000.bin 0x20000 $(BINDIR)/$(notdir $(CURDIR))-0x20000.bin
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
	jlinkGDBserverCL $(JLINK_PARAM) -strict -vd -nogui -singlerun
endif
ifeq ($(FLASHER),openocd)
	$(FLASHER) $(OPENOCD_PARAM_DEBUG)
endif

check_style:
	clang-format -style=file $(addprefix src/,$(SRC))

$(ELF): $(ALL_LINKED_OBJ) $(OBJ)
	@$(call MKDIR,$(@D))
ifeq ($(ALL_LIB),)
	$(LD) $(LDFLAGS) $^ -o $@
else
	$(LD) $(LDFLAGS) $^ $(ALL_LIBDIR) -Wl,--start-group $(ALL_LIB) -Wl,--end-group -o $@
endif

$(BIN): $(ELF)
	@$(call MKDIR,$(@D))
ifeq ($(FLASHER),esptool)
	esptool elf2image $< -o $(BINDIR)/$(notdir $(CURDIR))-
else
	$(OBJCOPY) -O binary $< $@
endif

$(LSS): $(ELF)
	@$(call MKDIR,$(@D))
	$(OBJDUMP) -dC $< >> $@
