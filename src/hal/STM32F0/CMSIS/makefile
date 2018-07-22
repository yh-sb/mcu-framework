-include ../../../../header.mk

SRC := device-support/startup_stm32f072xb.s \
	device-support/system_stm32f0xx.c \
	syscalls.c \
	sys.c

INC := device-support/include

DEF := HSE_VALUE=8000000 \
	ARM_MATH_CM0

C_CPP_FLAGS :=

# Use -std=gnu99 standard to compile syscalls.c without errors
CFLAGS := -std=gnu99
CPPFLAGS :=
AFLAGS :=

LIBDIR := lib
LIB := arm_cortexM0l_math
LINKED_OBJ :=

-include ../../../../footer.mk
