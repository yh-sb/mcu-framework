OBJ := $(patsubst %.c, $(OBJDIR)/%.o,$(filter %.c,$(SRC))) \
	$(patsubst %.cpp, $(OBJDIR)/%.o,$(filter %.cpp,$(SRC))) \
	$(patsubst %.s, $(OBJDIR)/%.o,$(filter %.s,$(SRC))) \
	$(patsubst %.S, $(OBJDIR)/%.o,$(filter %.S,$(SRC)))

INC := $(addprefix -I,$(strip $(GLOBAL_INC) $(INC)))
DEF := $(addprefix -D,$(strip $(GLOBAL_DEF) $(DEF)))
C_CPP_FLAGS := $(strip $(GLOBAL_C_CPP_FLAGS) $(C_CPP_FLAGS))
CFLAGS := $(strip $(GLOBAL_CFLAGS) $(CFLAGS))
CPPFLAGS := $(strip $(GLOBAL_CPPFLAGS) $(CPPFLAGS))
AFLAGS := $(strip $(GLOBAL_AFLAGS) $(AFLAGS))

.PHONY: all clean

all: $(OBJ)

clean:
	$(call RMDIR,$(OBJDIR))

$(OBJDIR)/%.o: %.c
	@$(call MKDIR,$(@D))
	$(CC) $(DEF) $(INC) $(C_CPP_FLAGS) $(CFLAGS) -c $^ -o $@

$(OBJDIR)/%.o: %.cpp
	@$(call MKDIR,$(@D))
	$(CPP) $(DEF) $(INC) $(C_CPP_FLAGS) $(CPPFLAGS) -c $^ -o $@

$(OBJDIR)/%.o: %.s
	@$(call MKDIR,$(@D))
	$(AS) $(DEF) $(INC) $(AFLAGS) -c $^ -o $@

$(OBJDIR)/%.o: %.S
	@$(call MKDIR,$(@D))
	$(AS) $(DEF) $(INC) $(AFLAGS) -c $^ -o $@
