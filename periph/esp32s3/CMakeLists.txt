add_library(periph_esp32s3 STATIC
    src/gpio_esp32s3.cpp
)

target_include_directories(periph_esp32s3
    PUBLIC
        include
)

target_link_libraries(periph_esp32s3
    PUBLIC
        freertos_kernel
    PRIVATE
        periph
        idf::driver
)
