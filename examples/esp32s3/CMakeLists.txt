cmake_minimum_required(VERSION 3.15)

find_program(CMAKE_C_COMPILER NAMES xtensa-esp32s3-elf-gcc)
find_program(CMAKE_CXX_COMPILER NAMES xtensa-esp32s3-elf-g++)
find_program(CMAKE_ASM_COMPILER NAMES xtensa-esp32s3-elf-gcc)
find_program(CMAKE_OBJCOPY NAMES xtensa-esp32s3-elf-objcopy)
find_program(CMAKE_SIZE NAMES xtensa-esp32s3-elf-size)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR esp32)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(esp32s3 C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

add_compile_options(
    $<$<NOT:$<COMPILE_LANGUAGE:ASM>>:-mlongcalls>
    -ffunction-sections
    -fdata-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

add_subdirectory(drivers)
add_subdirectory(periph)
add_subdirectory(periph/esp32s3)
add_subdirectory(third_party/printf)

add_executable(${CMAKE_PROJECT_NAME} main.cpp)

# esp-idf
execute_process(COMMAND git -C third_party/esp-idf apply ../../periph/esp32s3/esp-idf-rename-flash-target.patch ERROR_QUIET)
include(third_party/esp-idf/tools/cmake/idf.cmake)
idf_build_process(esp32s3
    COMPONENTS freertos nvs_flash esptool_py esp_wifi
    SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/periph/esp32s3/sdkconfig.defaults"
    SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig"
)
idf_build_executable(${CMAKE_PROJECT_NAME})
# Fix the FreeRTOS include path to avoid the "freertos/" path prefix and expose the ESP_PLATFORM define
target_include_directories(__idf_freertos PUBLIC ${idf_path}/components/freertos/FreeRTOS-Kernel/include/freertos)
target_compile_definitions(__idf_freertos PUBLIC ESP_PLATFORM)
# Alias to esp-idf FreeRTOS kernel, to be consistent with the original FreeRTOS kernel target naming
add_library(freertos_kernel ALIAS __idf_freertos)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -nostdlib
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    periph
    periph_esp32s3
    drivers
    idf::freertos
    idf::nvs_flash
    idf::esp_wifi
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}
)

# Add targets for flashing, erasing, resetting and debugging
set(ESPTOOL_PARAMS --chip esp32s3)
include(debug-probes/esptool.cmake)
