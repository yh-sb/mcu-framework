-include ../../../../header.mk

# ESP8266_RTOS_SDK v2.x.x (7d4782fcd34fe55d6b403bdd33e5988741e11239)

SRC := atexit.cpp

INC := .\
	./extra_include \
	./include \
	./include/espressif \
	./include/freertos

# for compilation sdk v2
DEF := ICACHE_FLASH \
	MEMLEAK_DEBUG

# for lwip
#DEF += _POSIX_SOURCE \
	LWIP_OPEN_SRC \
	PBUF_RSV_FOR_WLAN \
	EBUF_LWIP

C_CPP_FLAGS :=

# for lwip
#CFLAGS := -fno-aggressive-loop-optimizations \
	-Wno-implicit-function-declaration

CPPFLAGS :=
AFLAGS :=

LIBDIR := lib

LIB := airkiss \
	cirom \
	coap \
	crypto \
	driver \
	espconn \
	espnow \
	freertos \
	gcc \
	hal \
	json \
	lwip \
	main \
	mbedtls \
	mirom \
	mqtt \
	net80211 \
	nopoll \
	openssl \
	phy \
	pp \
	pwm \
	smartconfig \
	spiffs \
	ssc \
	ssl \
	wolfssl \
	wpa \
	wps

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

all: $(OBJ)

clean:
	$(call RMDIR,$(OBJDIR))

.SECONDEXPANSION:
$(OBJDIR)/%.o: %.c | $$(@D)/.f
	$(CC) $(DEF) $(INC) $(C_CPP_FLAGS) $(CFLAGS) -c $^ -o $@

$(OBJDIR)/%.o: %.cpp | $$(@D)/.f
	$(CPP) $(DEF) $(INC) $(C_CPP_FLAGS) $(CPPFLAGS) -c $^ -o $@

$(OBJDIR)/%.o: %.s | $$(@D)/.f
	$(AS) $(DEF) $(INC) $(AFLAGS) -c $^ -o $@

$(OBJDIR)/%.o: %.S | $$(@D)/.f
	$(AS) $(DEF) $(INC) $(AFLAGS) -c $^ -o $@

%/.f:
	$(call MKDIR,$(dir $@))

endif
