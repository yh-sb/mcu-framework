add_library(printf STATIC
    printf.c
)

target_include_directories(printf
    PUBLIC
        .
)

target_compile_definitions(printf PRIVATE
    PRINTF_INCLUDE_CONFIG_H
)
