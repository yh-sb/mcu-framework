#pragma once

#include <stdint.h>

#include "third_party/FatFs/diskio.h"
#include "drv/sd/sd.hpp"

namespace ul
{
struct fatfs_diskio_t
{
	DSTATUS (*status)(void *ctx);
	DSTATUS (*initialize)(void *ctx);
	DRESULT (*read)(void *ctx, BYTE *buff, DWORD sector, UINT count);
	DRESULT (*write)(void *ctx, const BYTE *buff, DWORD sector, UINT count);
	DRESULT (*ioctl)(void *ctx, BYTE cmd, void *buff);
};

void fatfs_diskio_add(uint8_t pdrv, drv::sd &sd);
}
