#include <cassert>
#include "fatfs_diskio_sd.hpp"
#include "diskio.h"
#include "drivers/sd.hpp"

static DSTATUS status(void *ctx);
static DSTATUS initialize(void *ctx);
static DRESULT read(void *ctx, BYTE *buff, DWORD sector, UINT count);
static DRESULT write(void *ctx, const BYTE *buff, DWORD sector, UINT count);
static DRESULT ioctl(void *ctx, BYTE cmd, void *buff);

fatfs_diskio_t *fatfs_diskio_sd()
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
    drv::sd *_sd = (drv::sd *)ctx;
    drv::sd_regs::csd_t csd;
    
    switch(_sd->read_csd(&csd))
    {
        case drv::sd::res::ok: return 0;
        case drv::sd::res::locked: return STA_PROTECT | STA_NOINIT;
        default: return STA_NOINIT;
    }
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
    drv::sd *_sd = (drv::sd *)ctx;
    auto res = drv::sd::res::ok;
    auto card_type = _sd->type();
    
    for(uint32_t i = 0; i < count && (res == drv::sd::res::ok); i++)
    {
        if(card_type == drv::sd::type::sd_v2_hi_capacity)
        {
            res = _sd->read(buff, sector + i);
        }
        else
        {
            res = _sd->read(buff, (sector + i) * SD_BLOCK_SIZE);
        }
    }
    
    switch(res)
    {
        case drv::sd::res::ok: return RES_OK;
        case drv::sd::res::param_err: return RES_PARERR;
        case drv::sd::res::locked: return RES_WRPRT;
        case drv::sd::res::no_response: return RES_NOTRDY;
        default: return RES_ERROR;
    }
}

static DRESULT write(void *ctx, const BYTE *buff, DWORD sector, UINT count)
{
    drv::sd *_sd = (drv::sd *)ctx;
    auto res = drv::sd::res::ok;
    auto card_type = _sd->type();
    
    for(uint32_t i = 0; i < count && (res == drv::sd::res::ok); i++)
    {
        if(card_type == drv::sd::type::sd_v2_hi_capacity)
        {
            res = _sd->write((BYTE *)buff, sector + i);
        }
        else
        {
            res = _sd->write((BYTE *)buff, (sector + i) * SD_BLOCK_SIZE);
        }
    }
    
    switch(res)
    {
        case drv::sd::res::ok: return RES_OK;
        case drv::sd::res::param_err: return RES_PARERR;
        case drv::sd::res::locked: return RES_WRPRT;
        case drv::sd::res::no_response: return RES_NOTRDY;
        default: return RES_ERROR;
    }
}

static DRESULT ioctl(void *ctx, BYTE cmd, void *buff)
{
    drv::sd *_sd = (drv::sd *)ctx;
    DRESULT res = RES_OK;
    
    switch(cmd)
    {
        case CTRL_SYNC:
            break;
        
        case GET_SECTOR_COUNT:
            *(DWORD *)buff = (DWORD)(_sd->capacity() / SD_BLOCK_SIZE);
            break;
        
        case GET_SECTOR_SIZE:
            *(WORD *)buff = (WORD)SD_BLOCK_SIZE;
            break;
        
        case GET_BLOCK_SIZE:
            *(DWORD *)buff = (DWORD)SD_BLOCK_SIZE;
            break;
        
        case CTRL_TRIM:
        {
            DWORD start = ((DWORD *)buff)[0];
            DWORD end = ((DWORD *)buff)[1];
            
            if(_sd->type() == drv::sd::type::sd_v2_hi_capacity)
            {
                start *= SD_BLOCK_SIZE;
                end *= SD_BLOCK_SIZE;
            }
            
            switch(_sd->erase(start, end))
            {
                case drv::sd::res::param_err: res =  RES_PARERR;
                case drv::sd::res::locked: res =  RES_WRPRT;
                case drv::sd::res::no_response: res =  RES_NOTRDY;
                default: res =  RES_ERROR;
            }
        }
            break;
        
        default:
            assert(0);
    }
    
    return res;
}
