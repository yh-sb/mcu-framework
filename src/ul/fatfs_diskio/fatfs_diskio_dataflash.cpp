
#include "common/assert.h"

#include "fatfs_diskio_dataflash.hpp"
#include "third_party/FatFs/diskio.h"
#include "drv/dataflash/dataflash.hpp"

using namespace ul;
using namespace drv;

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
	dataflash *_df = (dataflash *)ctx;
	
	dataflash::info_t info = _df->info();
	
	return (info.page_size && info.pages) ? STA_NOINIT : 0;
}

static DSTATUS initialize(void *ctx)
{
	sd *_sd = (sd *)ctx;
	
	switch(_sd->init())
	{
		case sd::RES_OK: return 0;
		case sd::RES_LOCKED: return STA_PROTECT | STA_NOINIT;
		default: return STA_NOINIT;
	}
}

static DRESULT read(void *ctx, BYTE *buff, DWORD sector, UINT count)
{
	dataflash *_df = (dataflash *)ctx;
	
	switch(_df->read(buff, sector, count))
	{
		case dataflash::RES_OK: return RES_OK;
		default: return RES_ERROR;
	}
}

static DRESULT write(void *ctx, const BYTE *buff, DWORD sector, UINT count)
{
	dataflash *_df = (dataflash *)ctx;
	
	switch(_df->write((BYTE *)buff, sector, count))
	{
		case dataflash::RES_OK: return RES_OK;
		default: return RES_ERROR;
	}
}

static DRESULT ioctl(void *ctx, BYTE cmd, void *buff)
{
	dataflash *_df = (dataflash *)ctx;
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
			
			if(_df->erase(start, end - start) != dataflash::RES_OK)
				res = RES_ERROR;
		}
			break;
		
		default:
			ASSERT(0);
	}
	
	return res;
}
