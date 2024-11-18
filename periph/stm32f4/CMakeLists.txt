add_library(periph_stm32f4 STATIC
    src/assert.c
    src/dac_stm32f4.cpp
    src/dma_stm32f4.cpp
    src/exti_stm32f4.cpp
    src/fault_handlers.c
    src/FreeRTOSHooks.c
    src/gpio_stm32f4.cpp
    src/i2c_stm32f4.cpp
    src/pwm_stm32f4.cpp
    src/rtc.cpp
    src/spi_stm32f4.cpp
    src/systick.cpp
    src/timer_stm32f4.cpp
    src/uart_stm32f4.cpp
    src/wdt.cpp
)

target_include_directories(periph_stm32f4
    PUBLIC
        include
)

add_subdirectory(CMSIS)

target_link_libraries(periph_stm32f4
    PUBLIC
        freertos_kernel
    PRIVATE
        periph
        cmsis
)
