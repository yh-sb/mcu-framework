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

LDFLAGS += $(strip $(ALL_LIBDIR))
ALL_LIB := $(strip $(ALL_LIB))

ifdef ALL_LIB
LDFLAGS += -Wl,--start-group $(ALL_LIB) -Wl,--end-group
endif

define COMPILE_MODULE
	+$(MAKE) --no-print-directory -C $(module)
	
endef

.PHONY: all clean erase flash reset debug

all:
	$(foreach module,$(MODULES),$(call COMPILE_MODULE,$(module)))
	+$(MAKE) --no-print-directory $(ELF)
	+$(MAKE) --no-print-directory $(BIN)
	+$(MAKE) --no-print-directory $(LSS)
	$(SIZE) $(ELF)

clean:
	$(call RMDIR,$(OUTDIR))

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

$(ELF): $(ALL_LINKED_OBJ) $(OBJ)
	@$(call MKDIR,$(@D))
	$(LD) $(LDFLAGS) $^ -o $@

$(BIN): $(ELF)
	@$(call MKDIR,$(@D))
	$(OBJCOPY) -O binary $< $@

$(LSS): $(ELF)
	@$(call MKDIR,$(@D))
	$(OBJDUMP) -dC $< >> $@
