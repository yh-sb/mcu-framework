add_library(cmsis STATIC
    Device/ST/STM32F1xx/Source/startup_stm32f100xb.s
    Device/ST/STM32F1xx/Source/system_stm32f1xx.c
)

target_include_directories(cmsis
    PUBLIC
        Core/Include
        Device/ST/STM32F1xx/Include
)

target_compile_definitions(cmsis
    PRIVATE
        HSE_VALUE=8000000
)
