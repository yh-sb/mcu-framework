add_library(cmsis STATIC
    Device/ST/STM32F0xx/Source/startup_stm32f072xb.s
    Device/ST/STM32F0xx/Source/system_stm32f0xx.c
)

target_include_directories(cmsis
    PUBLIC
        Core/Include
        Device/ST/STM32F0xx/Include
)

target_compile_definitions(cmsis
    PRIVATE
        HSE_VALUE=8000000
)
