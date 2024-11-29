
#include <cassert>
#include "fatfs_diskio.hpp"
#include "fatfs_diskio_dataflash.hpp"
#include "fatfs_diskio_sd.hpp"
#include "diskio.h"
#include "ffconf.h"

static void *ctxs[FF_VOLUMES];
static fatfs_diskio_t *diskios[FF_VOLUMES];

void fatfs_diskio_add(uint8_t pdrv, drv::dataflash &df)
{
    assert(pdrv < FF_VOLUMES);
    
    diskios[pdrv] = fatfs_diskio_dataflash();
    ctxs[pdrv] = (void *)&df;
}

void fatfs_diskio_add(uint8_t pdrv, drv::sd &sd)
{
    assert(pdrv < FF_VOLUMES);
    
    diskios[pdrv] = fatfs_diskio_sd();
    ctxs[pdrv] = (void *)&sd;
}

// FatFs handler goes below
DSTATUS disk_status(BYTE pdrv)
{
    assert(diskios[pdrv]);
    
    return diskios[pdrv]->status(ctxs[pdrv]);
}

DSTATUS disk_initialize(BYTE pdrv)
{
    assert(diskios[pdrv]);
    
    return diskios[pdrv]->initialize(ctxs[pdrv]);
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    assert(diskios[pdrv]);
    
    return diskios[pdrv]->read(ctxs[pdrv], buff, sector, count);
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    assert(diskios[pdrv]);
    
    return diskios[pdrv]->write(ctxs[pdrv], buff, sector, count);
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    assert(diskios[pdrv]);
    
    return diskios[pdrv]->ioctl(ctxs[pdrv], cmd, buff);
}

DWORD get_fattime(void)
{
    return 0;
}
