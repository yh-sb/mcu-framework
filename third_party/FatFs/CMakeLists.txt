add_library(fatfs STATIC
    source/ff.c
    source/ffsystem.c
    source/ffunicode.c
)

target_include_directories(fatfs
    PUBLIC
        source
)

target_link_libraries(fatfs
    PRIVATE
        freertos_kernel
)
