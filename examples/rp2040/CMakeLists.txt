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

project(rp2040 C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

add_compile_options(
    -mcpu=cortex-m0plus
    -mthumb
    -ffunction-sections
    -fdata-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

add_subdirectory(periph)
add_subdirectory(periph/rp2040)
add_subdirectory(drivers)

# Pico-sdk
set(PICO_PLATFORM rp2040)
set(PICO_DEOPTIMIZED_DEBUG $<CONFIG:Debug>)
add_subdirectory(third_party/pico-sdk)
pico_sdk_init()

# FreeRTOS
set(FREERTOS_PORT GCC_RP2040 CACHE STRING "")
set(FREERTOS_HEAP 4)
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE periph/rp2040/include)
target_compile_definitions(freertos_config INTERFACE FREE_RTOS_KERNEL_SMP=1 LIB_FREERTOS_KERNEL=1)
add_subdirectory(third_party/FreeRTOS-Kernel)
# The two lines below are the fix until PR https://github.com/FreeRTOS/FreeRTOS-Kernel/pull/1197 is merged
target_link_libraries(freertos_kernel_port_headers INTERFACE hardware_sync)
target_link_libraries(freertos_kernel_port PRIVATE pico_multicore)

add_subdirectory(third_party/printf)

add_executable(${CMAKE_PROJECT_NAME}
    main.cpp
    periph/rp2040/src/FreeRTOSHooks.c
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m0plus
    -Wl,--gc-sections
    -specs=nano.specs
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    pico_stdlib
    drivers
    periph
    periph_rp2040
    freertos_kernel
)

set(CMAKE_EXECUTABLE_SUFFIX .elf)
# Generate .uf2, .elf using pico-sdk
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
)

# Flash .uf2 file to RPI-RP2 disk using RP2040 bootloader.
include(debug-probes/uf2-bootloader.cmake)
# If you have JLink connected. Use "flash" target from debug-probes/jlink.cmake with "-device RP2040_M0_0 -if SWD" options.
