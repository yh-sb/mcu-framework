include config.mk

define INCLUDE_MODULE
$(eval include $(module)/makefile)
$(eval ALL_OBJ += $(patsubst $(OBJDIR)/%.o,$(OBJDIR)/$(module)/%.o,$(OBJ)))
$(eval ALL_LIBDIR += $(addprefix -L$(module)/,$(LIBDIR)))
$(eval ALL_LIB += $(addprefix -l,$(LIB)))
$(eval ALL_LINKED_OBJ += $(addprefix $(module)/,$(LINKED_OBJ)))
endef

define COMPILE_MODULE
@echo --- Compile "$(module)":
@$(MAKE) -j $(NUMBER_OF_PROCESSORS) --no-print-directory -C $(module)

endef

define CLEAN_MODULE
@echo --- Clean "$(module)":
@$(MAKE) -j $(NUMBER_OF_PROCESSORS) --no-print-directory -C $(module) clean

endef

# Collect prerequisites from modules for linkage
$(foreach module,$(MODULES),$(call INCLUDE_MODULE,$(module)))
ALL_LIBDIR := $(strip $(ALL_LIBDIR))
ALL_LIB := $(strip $(ALL_LIB))
ALL_LINKED_OBJ := $(strip $(ALL_LINKED_OBJ))

all:
	$(foreach module,$(MODULES),$(call COMPILE_MODULE,$(module)))
	@$(MAKE) -j $(NUMBER_OF_PROCESSORS) --no-print-directory $(ELF)
	@$(MAKE) -j $(NUMBER_OF_PROCESSORS) --no-print-directory $(BIN)
ifneq ($(FLASHER),esptool)
	@$(MAKE) -j $(NUMBER_OF_PROCESSORS) --no-print-directory $(LSS)
endif
	$(SIZE) $(ELF)

clean:
	$(foreach module,$(MODULES),$(call CLEAN_MODULE,$(module)))
	$(call RMDIR,$(OUTDIR))
ifeq ($(FLASHER),JLink)
	$(call RM,script.jlink)
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
ifeq ($(FLASHER),esptool)
	$(FLASHER) $(ESPTOOL_PARAM) erase_flash
	$(FLASHER) $(ESPTOOL_PARAM) write_flash \
	0xfc000 src/hal/ESP8266/ESP8266_RTOS_SDK/components/esp8266/firmware/esp_init_data_default.bin \
	0xfe000 src/hal/ESP8266/ESP8266_RTOS_SDK/components/esp8266/firmware/blank.bin
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
	$(FLASHER) $(ESPTOOL_PARAM) write_flash \
	0x00000 $(BINDIR)/$(notdir $(CURDIR))-0x00000.bin \
	0x10000 $(BINDIR)/$(notdir $(CURDIR))-0x10000.bin
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
#	jlinkGDBserverCL $(JLINK_PARAM) -strict -vd -nogui -singlerun -rtos GDBServer/RTOSPlugin_FreeRTOS
endif
ifeq ($(FLASHER),openocd)
	$(FLASHER) $(OPENOCD_PARAM_DEBUG)
endif

$(ELF): $(ALL_OBJ) $(ALL_LINKED_OBJ)
	@echo --- Linking "$(BIN)":
	@$(call MKDIR,$(@D))
ifeq ($(ALL_LIB),)
	$(LD) $(LDFLAGS) $^ -o $@
else
	$(LD) $(LDFLAGS) $^ $(ALL_LIBDIR) -Wl,--start-group $(ALL_LIB) -Wl,--end-group -o $@
endif

$(BIN): $(ELF)
ifeq ($(FLASHER),esptool)
	esptool elf2image $< -o $(BINDIR)/$(notdir $(CURDIR))-
else
	$(OBJCOPY) -O binary $< $@
endif

$(LSS): $(ELF)
	$(OBJDUMP) -dC $< >> $@
