add_library(cmsis STATIC
    # sturtup .s file is set in the main CMakeLists.txt
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
