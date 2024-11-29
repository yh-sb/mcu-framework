#include <cassert>
#include "fatfs_diskio_dataflash.hpp"
#include "diskio.h"
#include "drivers/dataflash.hpp"

static DSTATUS status(void *ctx);
static DSTATUS initialize(void *ctx);
static DRESULT read(void *ctx, BYTE *buff, DWORD sector, UINT count);
static DRESULT write(void *ctx, const BYTE *buff, DWORD sector, UINT count);
static DRESULT ioctl(void *ctx, BYTE cmd, void *buff);

fatfs_diskio_t *fatfs_diskio_dataflash()
{
    static fatfs_diskio_t diskio =
    {
        .status = status, .initialize = initialize, .read = read,
        .write = write, .ioctl = ioctl
    };
    
    return &diskio;
}

static DSTATUS status(void *ctx)
{
    drv::dataflash *_df = (drv::dataflash *)ctx;
    
    drv::dataflash::info_t info = _df->info();
    
    return (info.page_size && info.pages) ? STA_NOINIT : 0;
}

static DSTATUS initialize(void *ctx)
{
    drv::sd *_sd = (drv::sd *)ctx;
    
    switch(_sd->init())
    {
        case drv::sd::res::ok: return 0;
        case drv::sd::res::locked: return STA_PROTECT | STA_NOINIT;
        default: return STA_NOINIT;
    }
}

static DRESULT read(void *ctx, BYTE *buff, DWORD sector, UINT count)
{
    drv::dataflash *_df = (drv::dataflash *)ctx;
    
    switch(_df->read(buff, sector, count))
    {
        case drv::dataflash::res::ok: return RES_OK;
        default: return RES_ERROR;
    }
}

static DRESULT write(void *ctx, const BYTE *buff, DWORD sector, UINT count)
{
    drv::dataflash *_df = (drv::dataflash *)ctx;
    
    switch(_df->write((BYTE *)buff, sector, count))
    {
        case drv::dataflash::res::ok: return RES_OK;
        default: return RES_ERROR;
    }
}

static DRESULT ioctl(void *ctx, BYTE cmd, void *buff)
{
    drv::dataflash *_df = (drv::dataflash *)ctx;
    DRESULT res = RES_OK;
    
    switch(cmd)
    {
        case CTRL_SYNC:
            break;
        
        case GET_SECTOR_COUNT:
            *(DWORD *)buff = (DWORD)_df->info().pages;
            break;
        
        case GET_SECTOR_SIZE:
            *(WORD *)buff = (WORD)_df->info().page_size;
            break;
        
        case GET_BLOCK_SIZE:
            *(DWORD *)buff = (DWORD)_df->info().page_size;
            break;
        
        case CTRL_TRIM:
        {
            DWORD start = ((DWORD *)buff)[0];
            DWORD end = ((DWORD *)buff)[1];
            
            if(_df->erase(start, end - start) != drv::dataflash::res::ok)
            {
                res = RES_ERROR;
            }
        }
            break;
        
        default:
            assert(0);
    }
    
    return res;
}
