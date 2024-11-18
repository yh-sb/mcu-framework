add_library(periph_stm32f0 STATIC
    src/assert.c
    src/dma_stm32f0.cpp
    src/exti_stm32f0.cpp
    src/fault_handlers.c
    src/FreeRTOSHooks.c
    src/gpio_stm32f0.cpp
    src/spi_stm32f0.cpp
    src/systick.cpp
    src/timer_stm32f0.cpp
    src/uart_stm32f0.cpp
    src/wdt.cpp
)

target_include_directories(periph_stm32f0
    PUBLIC
        include
)

add_subdirectory(CMSIS)

target_link_libraries(periph_stm32f0
    PUBLIC
        freertos_kernel
    PRIVATE
        periph
        cmsis
)
