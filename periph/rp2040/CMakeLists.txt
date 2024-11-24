add_library(periph_rp2040 STATIC
    src/gpio_rp2040.cpp
)

target_include_directories(periph_rp2040
    PUBLIC
        include
)

target_link_libraries(periph_rp2040
    PUBLIC
        freertos_kernel
    PRIVATE
        periph
        hardware_gpio
)
