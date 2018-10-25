
#include "common/assert.h"

#include "fatfs_diskio.hpp"
#include "fatfs_diskio_sd.hpp"

#include "third_party/FatFs/diskio.h"
#include "third_party/FatFs/ffconf.h"

using namespace ul;

static void *ctx_list[FF_VOLUMES];
static fatfs_diskio_hndlrs_t *hndlrs_list[FF_VOLUMES];

void ul::fatfs_diskio_add(uint8_t pdrv, drv::sd &sd)
{
	ASSERT(pdrv < FF_VOLUMES);
	
	hndlrs_list[pdrv] = &fatfs_diskio_get();
	ctx_list[pdrv] = (void *)&sd;
}

DSTATUS disk_status(BYTE pdrv)
{
	ASSERT(hndlrs_list[pdrv]);
	
	return hndlrs_list[pdrv]->status(ctx_list[pdrv]);
}

DSTATUS disk_initialize(BYTE pdrv)
{
	ASSERT(hndlrs_list[pdrv]);
	
	return hndlrs_list[pdrv]->initialize(ctx_list[pdrv]);
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
	ASSERT(hndlrs_list[pdrv]);
	
	return hndlrs_list[pdrv]->read(ctx_list[pdrv], buff, sector, count);
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
	ASSERT(hndlrs_list[pdrv]);
	
	return hndlrs_list[pdrv]->write(ctx_list[pdrv], buff, sector, count);
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
	ASSERT(hndlrs_list[pdrv]);
	
	return hndlrs_list[pdrv]->ioctl(ctx_list[pdrv], cmd, buff);
}

DWORD get_fattime(void)
{
	return 0;
}
