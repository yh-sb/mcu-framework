cmake_minimum_required(VERSION 3.15)

find_program(CMAKE_ASM_COMPILER NAMES arm-none-eabi-gcc)
find_program(CMAKE_C_COMPILER NAMES arm-none-eabi-gcc)
find_program(CMAKE_CXX_COMPILER NAMES arm-none-eabi-g++)
find_program(CMAKE_LINKER NAMES arm-none-eabi-g++)
find_program(CMAKE_OBJCOPY NAMES arm-none-eabi-objcopy)
find_program(CMAKE_SIZE NAMES arm-none-eabi-size)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(stm32f1 C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

add_compile_options(
    -mcpu=cortex-m3
    -mthumb
    -ffunction-sections
    -fdata-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

add_compile_definitions(STM32F100xB)

add_subdirectory(periph)
add_subdirectory(periph/stm32f1)
add_subdirectory(drivers)

set(FREERTOS_PORT GCC_ARM_CM3 CACHE STRING "")
set(FREERTOS_HEAP 4)
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE periph/stm32f1/include)
add_subdirectory(third_party/FreeRTOS-Kernel)

add_subdirectory(third_party/printf)

add_executable(${CMAKE_PROJECT_NAME}
    main.cpp
    periph/stm32f1/src/FreeRTOSHooks.c
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m3
    -Wl,--gc-sections
    -specs=nano.specs
    -specs=nosys.specs
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    -T${CMAKE_CURRENT_SOURCE_DIR}/periph/stm32f1/CMSIS/Device/ST/STM32F1xx/Source/linker/STM32F100XB_FLASH.ld
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    periph
    periph_stm32f1
    drivers
    freertos_kernel
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf "${CMAKE_PROJECT_NAME}.bin"
)
