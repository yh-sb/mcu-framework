if(NOT DEFINED JLINK_PARAMS)
    message(FATAL_ERROR "JLINK_PARAMS is not defined.\n"
        "Example: \"set(JLINK_PARAMS -device STM32F407VG -if SWD)\"")
endif()

add_custom_target(flash
    COMMAND echo h>script.jlink
    COMMAND echo loadbin ${CMAKE_PROJECT_NAME}.bin, 0 >>script.jlink
    COMMAND echo r>>script.jlink
    COMMAND echo q>>script.jlink
    COMMAND JLink ${JLINK_PARAMS} -speed auto -CommanderScript script.jlink
    USES_TERMINAL
)

add_custom_target(erase
    COMMAND echo erase>script.jlink
    COMMAND echo q>>script.jlink
    COMMAND JLink ${JLINK_PARAMS} -speed auto -CommanderScript script.jlink
    USES_TERMINAL
)

add_custom_target(reset
    COMMAND echo r>script.jlink
    COMMAND echo q>>script.jlink
    COMMAND JLink ${JLINK_PARAMS} -speed auto -CommanderScript script.jlink
    USES_TERMINAL
)

add_custom_target(debug DEPENDS flash
    COMMAND JLinkGDBServerCL ${JLINK_PARAMS} -speed auto -strict -nogui -singlerun
# -rtos GDBServer/RTOSPlugin_FreeRTOS
    USES_TERMINAL
)
