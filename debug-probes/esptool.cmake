if(NOT DEFINED ESPTOOL_PARAMS)
    message(FATAL_ERROR "ESPTOOL_PARAMS is not defined.\n"
        "Example: \"set(ESPTOOL_PARAMS --chip esp32s3)\"")
endif()

add_custom_target(flash
    COMMAND python ${IDF_PATH}/components/esptool_py/esptool/esptool.py
        ${ESPTOOL_PARAMS} --baud 2000000 --before default_reset --after hard_reset write_flash
        0x0 ${CMAKE_BINARY_DIR}/bootloader/bootloader.bin
        0x8000 ${CMAKE_BINARY_DIR}/partition_table/partition-table.bin
        0x10000 ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    VERBATIM
    USES_TERMINAL
)

add_custom_target(erase
    COMMAND python ${IDF_PATH}/components/esptool_py/esptool/esptool.py
        ${ESPTOOL_PARAMS} --before default_reset --after hard_reset erase_flash
    VERBATIM
    USES_TERMINAL
)

add_custom_target(reset
    COMMAND python ${IDF_PATH}/components/esptool_py/esptool/esptool.py ${ESPTOOL_PARAMS} run
    VERBATIM
    USES_TERMINAL
)
