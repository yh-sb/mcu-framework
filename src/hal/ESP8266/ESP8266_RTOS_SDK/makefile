-include ../../../../config.mk

SRC := components\freertos\freertos\event_groups.c \
	components\freertos\freertos\list.c \
	components\freertos\freertos\queue.c \
	components\freertos\freertos\stream_buffer.c \
	components\freertos\freertos\tasks.c \
	components\freertos\freertos\timers.c \
	components\freertos\port\esp8266\freertos_hooks.c \
	components\freertos\port\esp8266\impure.c \
	components\freertos\port\esp8266\os_cpu_a.S \
	components\freertos\port\esp8266\panic.c \
	components\freertos\port\esp8266\port.c \
	components\freertos\port\esp8266\xtensa_context.S \
	components\freertos\port\esp8266\xtensa_init.c \
	components\freertos\port\esp8266\xtensa_vectors.S \
	\
	components\esp8266\source\chip_boot.c \
	components\esp8266\source\esp_timer.c \
	components\esp8266\source\esp_wifi_os_adapter.c \
	components\esp8266\source\esp_wifi.c \
	components\esp8266\source\ets_printf.c \
	components\esp8266\source\event_default_handlers.c \
	components\esp8266\source\event_loop.c \
	components\esp8266\source\phy_init.c \
	components\esp8266\source\reset_reason.c \
	components\esp8266\source\startup.c \
	components\esp8266\source\system_api.c \
	components\esp8266\source\task_wdt.c \
	\
	components\heap\src\esp_heap_caps.c \
	components\heap\src\esp_heap_trace.c \
	components\heap\port\esp8266\esp_heap_init.c \
	\
	components\log\log.c \
	\
	components\lwip\apps\dhcpserver\dhcpserver.c \
	components\lwip\lwip\src\api\tcpip.c \
	components\lwip\lwip\src\core\def.c \
	components\lwip\lwip\src\core\dns.c \
	components\lwip\lwip\src\core\inet_chksum.c \
	components\lwip\lwip\src\core\init.c \
	components\lwip\lwip\src\core\ip.c \
	components\lwip\lwip\src\core\memp.c \
	components\lwip\lwip\src\core\netif.c \
	components\lwip\lwip\src\core\pbuf.c \
	components\lwip\lwip\src\core\tcp_in.c \
	components\lwip\lwip\src\core\tcp_out.c \
	components\lwip\lwip\src\core\tcp.c \
	components\lwip\lwip\src\core\timeouts.c \
	components\lwip\lwip\src\core\udp.c \
	components\lwip\lwip\src\core\ipv4\dhcp.c \
	components\lwip\lwip\src\core\ipv4\etharp.c \
	components\lwip\lwip\src\core\ipv4\icmp.c \
	components\lwip\lwip\src\core\ipv4\igmp.c \
	components\lwip\lwip\src\core\ipv4\ip4_addr.c \
	components\lwip\lwip\src\core\ipv4\ip4.c \
	components\lwip\lwip\src\netif\ethernet.c \
	components\lwip\port\esp8266\freertos\heap.c \
	components\lwip\port\esp8266\freertos\sys_arch.c \
	components\lwip\port\esp8266\freertos\udp_sync.c \
	components\lwip\port\esp8266\netif\ethernetif.c \
	\
	components\newlib\newlib\port\esp_malloc.c \
	components\newlib\newlib\port\esp_newlib.c \
	components\newlib\newlib\port\locks.c \
	components\newlib\newlib\port\syscall.c \
	components\newlib\newlib\port\time.c \
	\
	components\nvs_flash\src\nvs_api.cpp \
	components\nvs_flash\src\nvs_item_hash_list.cpp \
	components\nvs_flash\src\nvs_page.cpp \
	components\nvs_flash\src\nvs_pagemanager.cpp \
	components\nvs_flash\src\nvs_storage.cpp \
	components\nvs_flash\src\nvs_types.cpp \
	\
	components\spi_flash\src\partition.c \
	components\spi_flash\src\spi_flash_raw.c \
	components\spi_flash\src\spi_flash.c \
	\
	components\tcpip_adapter\tcpip_adapter_lwip.c \
	\
	components\util\src\crc.c \
	\
	components\wpa_supplicant\src\crypto\aes-internal-enc.c \
	components\wpa_supplicant\src\crypto\aes-wrap.c

INC := . \
	components/bootloader_support/include \
	components/freertos/include \
	components/log/include \
	components/lwip/include/lwip/apps \
	components/lwip/lwip/src/include \
	components/lwip/port/esp8266/include \
	components/newlib/newlib/port/include \
	components/nvs_flash/include \
	components/spi_flash/include \
	components/tcpip_adapter/include \
	components/util/include \
	components/wpa_supplicant/include \
	components/wpa_supplicant/port/include

DEF := ESP_PLATFORM \
	IDF_VER=\"3.1\" \
	__ESP_FILE__=__FILE__ \
	PARTITION_QUEUE_HEADER=\"sys/queue.h\" \
	NVS_CRC_HEADER_FILE=\"crc.h\"

C_CPP_FLAGS := -Wno-error=unused-function \
	-Wno-error=unused-but-set-variable \
	-Wno-error=unused-variable \
	-Wno-error=deprecated-declarations \
	-Wno-unused-parameter \
	-Wno-sign-compare \
	\
	-ffunction-sections \
	-fdata-sections \
	-fstrict-volatile-bitfields \
	\
	-mtext-section-literals \
	-mlongcalls \
	-nostdlib

CFLAGS := -std=gnu99

CPPFLAGS :=
AFLAGS :=

LIBDIR := components/esp8266/lib \
	components/newlib/newlib/lib

LIB := stdc++ \
	gcov \
	c_nano \
	m \
	\
	core \
	espnow \
	gcc \
	hal \
	net80211 \
	phy \
	pp \
	smartconfig \
	ssc \
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
