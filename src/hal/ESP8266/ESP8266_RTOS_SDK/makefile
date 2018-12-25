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

-include ../../../../footer.mk
