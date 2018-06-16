#pragma once

#include "drv/sd/sd.hpp"
#include "ul/fatfs_diskio/fatfs_diskio.hpp"

namespace ul
{
fatfs_diskio_hndlrs_t &fatfs_diskio_get();
}
