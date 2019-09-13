-include ../../../config.mk

SRC := src/nmea/nmea.c \
	src/nmea/parser_static.c \
	src/parsers/gpgga.c \
	src/parsers/gpgll.c \
	src/parsers/gpgsv.c \
	src/parsers/gprmc.c \
	src/parsers/parse.c

INC := ./src/nmea

DEF := ENABLE_GPGGA \
	ENABLE_GPGLL \
	ENABLE_GPGSV \
	ENABLE_GPRMC \
	PARSER_COUNT=4

C_CPP_FLAGS :=
CFLAGS := -Wno-stringop-truncation
CPPFLAGS :=
AFLAGS :=

LIBDIR :=
LIB :=
LINKED_OBJ :=

# -------------------------

OBJ := $(patsubst %.c,$(OBJDIR)/%.o,$(filter %.c,$(SRC))) \
	$(patsubst %.cpp,$(OBJDIR)/%.o,$(filter %.cpp,$(SRC))) \
	$(patsubst %.s,$(OBJDIR)/%.o,$(filter %.s,$(SRC))) \
	$(patsubst %.S,$(OBJDIR)/%.o,$(filter %.S,$(SRC)))

INC := $(addprefix -I,$(GLOBAL_INC) $(INC))
DEF := $(addprefix -D,$(GLOBAL_DEF) $(DEF))
C_CPP_FLAGS := $(strip $(GLOBAL_C_CPP_FLAGS) $(C_CPP_FLAGS))
CFLAGS := $(strip $(GLOBAL_CFLAGS) $(CFLAGS))
CPPFLAGS := $(strip $(GLOBAL_CPPFLAGS) $(CPPFLAGS))
AFLAGS := $(strip $(GLOBAL_AFLAGS) $(AFLAGS))

# Prevent including this part to main makefile
ifneq ($(ROOT),.)

define RENAME_SYMBOLS
$(OBJCOPY) --redefine-sym init=nmea_$(1)_init $(2)
$(OBJCOPY) --redefine-sym parse=nmea_$(1)_parse $(2)
$(OBJCOPY) --redefine-sym set_default=nmea_$(1)_set_default $(2)
$(OBJCOPY) --redefine-sym allocate_data=nmea_$(1)_allocate_data $(2)
$(OBJCOPY) --redefine-sym free_data=nmea_$(1)_free_data $(2)
$(OBJCOPY) --redefine-sym malloc=pvPortMalloc $(2)
$(OBJCOPY) --redefine-sym free=vPortFree $(2)

endef

all: $(OBJ)

clean:
	$(call RMDIR,$(OBJDIR))

# Add .NOTPARALLEL to avoid objcopy collisions in same file during parallel make
.SECONDEXPANSION .NOTPARALLEL:
$(OBJDIR)/%.o: %.c | $$(@D)/.f
	$(CC) $(DEF) $(INC) $(C_CPP_FLAGS) $(CFLAGS) -MMD -MP -c $< -o $@
# Make some redefinitions to avoid changing the libnmea sources
	$(call RENAME_SYMBOLS,$(notdir $*),$@)

$(OBJDIR)/%.o: %.cpp | $$(@D)/.f
	$(CPP) $(DEF) $(INC) $(C_CPP_FLAGS) $(CPPFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR)/%.o: %.s | $$(@D)/.f
	$(AS) $(DEF) $(INC) $(AFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR)/%.o: %.S | $$(@D)/.f
	$(AS) $(DEF) $(INC) $(AFLAGS) -MMD -MP -c $< -o $@

%/.f:
	$(call MKDIR,$(dir $@))

-include $(patsubst %.o,%.d,$(OBJ))

endif
