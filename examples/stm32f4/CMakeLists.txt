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

project(stm32f4 C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

add_compile_options(
    -mcpu=cortex-m4
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -mthumb
    -ffunction-sections
    -fdata-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

# Set definition for STM32F407VG target
add_compile_definitions(STM32F407xx)

add_subdirectory(periph)
add_subdirectory(periph/stm32f4)
# Set startup file for STM32F407VG target
target_sources(cmsis PRIVATE periph/stm32f4/CMSIS/Device/ST/STM32F4xx/Source/startup_stm32f407xx.s)

add_subdirectory(drivers)

set(FREERTOS_PORT GCC_ARM_CM4F CACHE STRING "")
set(FREERTOS_HEAP 4)
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE periph/stm32f4/include)
add_subdirectory(third_party/FreeRTOS-Kernel)

add_subdirectory(third_party/FatFs)
add_subdirectory(fatfs_diskio)

add_subdirectory(third_party/printf)

add_executable(${CMAKE_PROJECT_NAME}
    main.cpp
    periph/stm32f4/src/FreeRTOSHooks.c
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -Wl,--gc-sections
    -specs=nano.specs
    -specs=nosys.specs
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    -T${CMAKE_CURRENT_SOURCE_DIR}/periph/stm32f4/CMSIS/Device/ST/STM32F4xx/Source/linker/STM32F407VGTx_FLASH.ld
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    periph
    periph_stm32f4
    drivers
    freertos_kernel
    fatfs_diskio
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf "${CMAKE_PROJECT_NAME}.bin"
)

# Add targets for flashing, erasing, resetting and debugging
set(JLINK_PARAMS -device STM32F407VG -if SWD)
include(debug-probes/jlink.cmake)
