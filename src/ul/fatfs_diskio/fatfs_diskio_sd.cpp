
#include "common/assert.h"

#include "fatfs_diskio_sd.hpp"
#include "third_party/FatFs/diskio.h"
#include "drv/sd/sd.hpp"

using namespace ul;
using namespace drv;

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
	sd *_sd = (sd *)ctx;
	sd_csd_t csd;
	
	switch(_sd->read_csd(&csd))
	{
		case sd::RES_OK: return 0;
		case sd::RES_LOCKED: return STA_PROTECT | STA_NOINIT;
		default: return STA_NOINIT;
	}
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
	sd *_sd = (sd *)ctx;
	int8_t res = sd::RES_OK;
	sd::type_t card_type = _sd->type();
	
	for(uint32_t i = 0; i < count && (res == sd::RES_OK); i++)
	{
		if(card_type == sd::TYPE_SD_V2_HI_CAPACITY)
			res = _sd->read(buff, sector + i);
		else
			res = _sd->read(buff, (sector + i) * SD_BLOCK_SIZE);
	}
	
	switch(res)
	{
		case sd::RES_OK: return RES_OK;
		case sd::RES_PARAM_ERR: return RES_PARERR;
		case sd::RES_LOCKED: return RES_WRPRT;
		case sd::RES_NO_RESPONSE: return RES_NOTRDY;
		default: return RES_ERROR;
	}
}

static DRESULT write(void *ctx, const BYTE *buff, DWORD sector, UINT count)
{
	sd *_sd = (sd *)ctx;
	int8_t res = RES_OK;
	sd::type_t card_type = _sd->type();
	
	for(uint32_t i = 0; i < count && (res == sd::RES_OK); i++)
	{
		if(card_type == sd::TYPE_SD_V2_HI_CAPACITY)
			res = _sd->write((BYTE *)buff, sector + i);
		else
			res = _sd->write((BYTE *)buff, (sector + i) * SD_BLOCK_SIZE);
	}
	
	switch(res)
	{
		case sd::RES_OK: return RES_OK;
		case sd::RES_PARAM_ERR: return RES_PARERR;
		case sd::RES_LOCKED: return RES_WRPRT;
		case sd::RES_NO_RESPONSE: return RES_NOTRDY;
		default: return RES_ERROR;
	}
}

static DRESULT ioctl(void *ctx, BYTE cmd, void *buff)
{
	sd *_sd = (sd *)ctx;
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
			
			if(_sd->type() == sd::TYPE_SD_V2_HI_CAPACITY)
			{
				start *= SD_BLOCK_SIZE;
				end *= SD_BLOCK_SIZE;
			}
			
			switch(_sd->erase(start, end))
			{
				case sd::RES_PARAM_ERR: res =  RES_PARERR;
				case sd::RES_LOCKED: res =  RES_WRPRT;
				case sd::RES_NO_RESPONSE: res =  RES_NOTRDY;
				default: res =  RES_ERROR;
			}
		}
			break;
		
		default:
			ASSERT(0);
	}
	
	return res;
}
