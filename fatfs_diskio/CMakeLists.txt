add_library(fatfs_diskio STATIC
    fatfs_diskio.cpp
    fatfs_diskio_dataflash.cpp
    fatfs_diskio_sd.cpp
)

target_include_directories(fatfs_diskio
    PUBLIC
        .
)

target_link_libraries(fatfs_diskio
    PUBLIC
        drivers
        fatfs
)
