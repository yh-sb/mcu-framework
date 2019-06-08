#include <string.h>
#include "common/assert.h"
#include "dataflash.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace drv;

constexpr uint8_t dataflash_family_code = 1;
constexpr uint8_t jedec_continuation_code = 0x7F;

// Present since "D" family
constexpr uint8_t cmd_set_page_size_pow2[] = {0x3D, 0x2A, 0x80, 0xA6};
constexpr uint8_t cmd_set_page_size_dataflash[] = {0x3D, 0x2A, 0x80, 0xA7};
constexpr uint8_t cmd_enable_sector_protection[] = {0x3D, 0x2A, 0x7F, 0xA9};
constexpr uint8_t cmd_disable_sector_protection[] = {0x3D, 0x2A, 0x7F, 0x9A};
constexpr uint8_t cmd_enable_sector_lockdown[] = {0x3D, 0x2A, 0x7F, 0x30};
constexpr uint8_t cmd_chip_erase[] = {0xC7, 0x94, 0x80, 0x9A};

enum cmd_t
{
	// Read commands
	CMD_CONTINUOUS_ARRAY_READ                                    = 0xE8, // Legacy command
	CMD_CONTINUOUS_ARRAY_READ_HI_FREQ                            = 0x0B,
	CMD_CONTINUOUS_ARRAY_READ_LOW_FREQ                           = 0x03,
	
	CMD_MAIN_MEMORY_PAGE_READ                                    = 0xD2,
	CMD_BUFFER1_READ                                             = 0xD4,
	CMD_BUFFER2_READ                                             = 0xD6,
	CMD_STATUS_REG_READ                                          = 0xD7,
	
	CMD_SECURITY_REG_READ                                        = 0x77, // Present since "D" family
	CMD_SECURITY_REG_WRITE                                       = 0x9A, // Present since "D" family
	
	// Supported only by new AT45DBxxxD serial (unlike AT45DBxxxB)
	CMD_READ_MANUFACTURER_AND_DEVICE_ID                          = 0x9F,
	
	// Program and erase commands
	CMD_BUFFER1_WRITE                                            = 0x84,
	CMD_BUFFER2_WRITE                                            = 0x87,
	CMD_BUFFER1_TO_MAIN_MEMORY_PAGE_PROGRAM_WITH_BUILTIN_ERASE   = 0x83,
	CMD_BUFFER2_TO_MAIN_MEMORY_PAGE_PROGRAM_WITH_BUILTIN_ERASE   = 0x86,
	CMD_BUFFER1_TO_MAIN_MEMORY_PAGE_PROGRAM_WITHOT_BUILTIN_ERASE = 0x88,
	CMD_BUFFER2_TO_MAIN_MEMORY_PAGE_PROGRAM_WITHOT_BUILTIN_ERASE = 0x89,
	CMD_PAGE_ERASE                                               = 0x81,
	CMD_BLOCK_ERASE                                              = 0x50,
	CMD_MAIN_MEMORY_PAGE_PROGRAM_THROUGH_BUFFER1                 = 0x82,
	CMD_MAIN_MEMORY_PAGE_PROGRAM_THROUGH_BUFFER2                 = 0x85,
	CMD_SECTOR_ERASE                                             = 0x7C, // Present since "D" family
	
	// Additional commands
	CMD_MAIN_MEMORY_PAGE_TO_BUFFER1_TRANSFER                     = 0x53,
	CMD_MAIN_MEMORY_PAGE_TO_BUFFER2_TRANSFER                     = 0x55,
	CMD_MAIN_MEMORY_PAGE_TO_BUFFER1_COMPARE                      = 0x60,
	CMD_MAIN_MEMORY_PAGE_TO_BUFFER2_COMPARE                      = 0x61,
	CMD_AUTO_PAGE_REWRITE_THROUGH_BUFFER1                        = 0x58,
	CMD_AUTO_PAGE_REWRITE_THROUGH_BUFFER2                        = 0x59,
	
	CMD_ENTER_DEEP_POWER_DOWN                                    = 0xB9, // Present since "D" family
	CMD_EXIT_DEEP_POWER_DOWN                                     = 0xAB, // Present since "D" family
};

static const struct
{
	uint8_t cmd;
	/* Timeout in ms:
	    timeout[0] - 256/264 bytes in page
	    timeout[1] - 512/526 bytes in page
	    timeout[2] - 1024/1056 bytes in page
	    timeout[3] - 2048/2112 bytes in page
	*/
	uint16_t timeout[4];
} timeouts[] =
{
	{.cmd = CMD_MAIN_MEMORY_PAGE_PROGRAM_THROUGH_BUFFER1, .timeout = {35, 55, 55, 70}},
	{.cmd = CMD_PAGE_ERASE, .timeout = {25, 50, 45, 60}},
	
	// Specific case. Handled separately
	{.cmd = cmd_chip_erase[0], .timeout = {0, 0, 0, 0}},
};

static uint32_t chip_erase_timeout(dataflash::info_t *info)
{
	switch(info->pages)
	{
		case 512:
			return 2000;
		case 1024:
			return 4000;
		case 2048:
			return 17000;
		case 4096:
			if(info->page_size == 256 || info->page_size == 264)
				return 20000;
			else
				return 40000;
		case 8192:
			if(info->page_size == 512 || info->page_size == 528)
				return 80000;
			else
				return 208000;
		case 16384:
			if(info->page_size == 1024 || info->page_size == 1056)
				return 400000; // Approximate value
			else
				return 800000; // Approximate value
		case 32768:
			return 1600000; // Approximate value
		default:
			ASSERT(0);
			return 0;
	}
}

static uint32_t timeout(uint8_t cmd, dataflash::info_t *info)
{
	if(cmd == cmd_chip_erase[0])
		return chip_erase_timeout(info);

	for(uint8_t i = 0; i < (sizeof(timeouts) / sizeof(timeouts[0])); i++)
	{
		if(cmd != timeouts[i].cmd)
			continue;

		switch(info->page_size)
		{
			case 256:
			case 264: return timeouts[i].timeout[0];
			case 512:
			case 526: return timeouts[i].timeout[1];
			case 1024:
			case 1056: return timeouts[i].timeout[2];
			case 2048:
			case 2112: return timeouts[i].timeout[3];
			default: ASSERT(0); return 0;
		}
	}

	ASSERT(0);
	return 0;
}

dataflash::dataflash(hal::spi &spi, hal::gpio &cs, hal::gpio *wp,
	hal::gpio *rst):
	_spi(spi),
	_cs(cs),
	_wp(wp),
	_rst(rst),
	_info({})
{
	ASSERT(_spi.bit_order() == hal::spi::BIT_ORDER_MSB);
	ASSERT(_cs.mode() == hal::gpio::MODE_DO);
	ASSERT(!_wp || _wp->mode() == hal::gpio::MODE_DO);
	ASSERT(!_rst || _rst->mode() == hal::gpio::MODE_DO);
	
	ASSERT((lock = xSemaphoreCreateMutex()));
}

dataflash::~dataflash()
{
	vSemaphoreDelete(lock);
}

int8_t dataflash::init(page_size_t page_size)
{
	int8_t res = set_page_size(page_size);
	if(res != RES_OK)
		return res;
	
	status_t status = {};
	
	res = read_status(&status);
	if(res != RES_OK)
		return res;
	
	_info = get_info(status.density_code, status.page_size);
	
	return res;
}

int8_t dataflash::read(void *buff, uint16_t page, uint16_t pages)
{
	ASSERT(buff);
	ASSERT(page + pages < _info.pages); // Not enough memory
	ASSERT(pages > 0);
	
	xSemaphoreTake(lock, portMAX_DELAY);
	
	args_t args(page, 0, _info);
	
	int8_t res = cmd_with_read(CMD_CONTINUOUS_ARRAY_READ_HI_FREQ, args, buff,
		_info.page_size * pages, 1);
	
	xSemaphoreGive(lock);
	
	return res;
}

int8_t dataflash::write(void *buff, uint16_t page, uint16_t pages)
{
	ASSERT(buff);
	ASSERT(page + pages < _info.pages); // Not enough memory
	ASSERT(pages > 0);
	
	xSemaphoreTake(lock, portMAX_DELAY);
	
	int res = RES_OK;
	uint8_t cmd = CMD_MAIN_MEMORY_PAGE_PROGRAM_THROUGH_BUFFER1;
	
	for(; pages; pages--, page++, buff = (uint8_t *)buff + _info.page_size)
	{
		args_t args(page, 0, _info);
		
		res = cmd_with_write(cmd, args, buff, _info.page_size,
			timeout(cmd, &_info));
		
		if(res != RES_OK)
			break;
	}
	
	xSemaphoreGive(lock);
	
	return res;
}

int8_t dataflash::erase(uint16_t page, uint16_t pages)
{
	ASSERT(page + pages < _info.pages); // Not enough memory
	ASSERT(pages > 0);
	
	xSemaphoreTake(lock, portMAX_DELAY);
	
	int8_t res;
	
	if (page == 0 && pages == _info.pages)
	{
		args_t args(cmd_chip_erase[1], cmd_chip_erase[2], cmd_chip_erase[3]);
		
		res = cmd(cmd_chip_erase[0], args, timeout(cmd_chip_erase[0], &_info));
	}
	else
	{
		for(; pages; pages--, page++)
		{
			args_t args(page, 0, _info);
			
			res = cmd(CMD_PAGE_ERASE, args, timeout(CMD_PAGE_ERASE, &_info));
			if(res != RES_OK)
				break;
		}
	}
	
	xSemaphoreGive(lock);
	
	return res;
}

int8_t dataflash::jedec(jedec_t *jedec)
{
	ASSERT(jedec);
	
	xSemaphoreTake(lock, portMAX_DELAY);
	
	_cs.set(0);
	
	int8_t res = _spi.write(CMD_READ_MANUFACTURER_AND_DEVICE_ID);
	if(res != RES_OK)
		goto Exit;
	
	for (uint8_t continuation_number = 0; ; continuation_number++)
	{
		// Read first byte of JEDEC (manufacturer_id)
		res = _spi.read(jedec, sizeof(uint8_t));
		if(res != RES_OK)
			goto Exit;
		
		// Skip continuation code
		if (jedec->manufacturer_id != jedec_continuation_code)
			break;
	}
	
	// Read rest part of JEDEC
	res = _spi.read(&jedec->dev_id, sizeof(*jedec) - sizeof(uint8_t));
	if(res != RES_OK)
		goto Exit;
	
	if(!is_jedec_valid(jedec))
		res = RES_UNSUPPORTED_CHIP_ERR;
	
Exit:
	_cs.set(1);
	xSemaphoreGive(lock);
	return res;
}

dataflash::args_t::args_t(uint16_t page, uint16_t byte, info_t info)
{
	uint8_t byte_addr_bits;
	
	switch(info.page_size)
	{
		case 256: byte_addr_bits = 9; break;
		case 264:
		case 512: byte_addr_bits = 10; break;
		case 528:
		case 1024: byte_addr_bits = 11; break;
		case 1056:
		case 2048: byte_addr_bits = 12; break;
		case 2112: byte_addr_bits = 13; break;
		default: ASSERT(0);
	}
	
	// If chip size >= 128 MBit, 4 bytes is required for addressing
	if(info.pages >= 16384)
	{
		arg[0] = page >> (24 - byte_addr_bits);
		arg[1] = page >> (16 - byte_addr_bits);
		arg[2] = page << (byte_addr_bits - 8) | byte >> 8;
		arg[3] = byte;
		size = 4;
	}
	else
	{
		arg[0] = page >> (16 - byte_addr_bits);
		arg[1] = page << (byte_addr_bits - 8) | byte >> 8;
		arg[2] = byte;
		size = 3;
	}
}

dataflash::args_t::args_t(uint8_t arg1, uint8_t arg2, uint8_t arg3)
{
	arg[0] = arg1;
	arg[1] = arg2;
	arg[2] = arg3;
	size = 3;
}

int8_t dataflash::read_status(status_t *status)
{
	_cs.set(0);
	
	int8_t res = _spi.write(CMD_STATUS_REG_READ);
	if(res != RES_OK)
		goto Exit;
	
	res = _spi.read(status, sizeof(*status));
	if(res != RES_OK)
		goto Exit;
	
	if(!is_status_valid(status))
		res = RES_UNSUPPORTED_CHIP_ERR;
	
Exit:
	_cs.set(1);
	return res;
}

bool dataflash::is_status_valid(status_t *status)
{
	status_t all0x00 = {};
	status_t all0xFF;
	memset(&all0xFF, 0xFF, sizeof(all0xFF));
	
	if(!memcmp(status, &all0x00, sizeof(*status)) ||
		!memcmp(status, &all0xFF, sizeof(*status)))
	{
		return false;
	}
	
	switch(status->density_code)
	{
		case DENSITY_CODE_1_MBIT:
		case DENSITY_CODE_2_MBIT:
		case DENSITY_CODE_4_MBIT:
		case DENSITY_CODE_8_MBIT:
		case DENSITY_CODE_16_MBIT:
		case DENSITY_CODE_32_MBIT:
		case DENSITY_CODE_64_MBIT:
		case DENSITY_CODE_128_MBIT:
		case DENSITY_CODE_256_MBIT:
		case DENSITY_CODE_512_MBIT:
			break;
		
		default:
			return false;
	}
	
	return true;
}

bool dataflash::is_jedec_valid(jedec_t *jedec)
{
	jedec_t all0x00 = {};
	jedec_t all0xFF;
	memset(&all0xFF, 0xFF, sizeof(all0xFF));
	
	if(!memcmp(jedec, &all0x00, sizeof(*jedec)) ||
		!memcmp(jedec, &all0xFF, sizeof(*jedec)))
	{
		return false;
	}
	
	if(jedec->dev_id.family_code != dataflash_family_code)
		return false;
	
	return true;
}

dataflash::info_t dataflash::get_info(density_code_t density_code,
	page_size_t page_size)
{
	info_t info = {};
	
	switch(density_code)
	{
		case DENSITY_CODE_1_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 264 : 256;
			info.pages = 512;
			break;
		
		case DENSITY_CODE_2_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 264 : 256;
			info.pages = 1024;
			break;
		
		case DENSITY_CODE_4_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 264 : 256;
			info.pages = 2048;
			break;
		
		case DENSITY_CODE_8_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 264 : 256;
			info.pages = 4096;
			break;
		
		case DENSITY_CODE_16_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 528 : 512;
			info.pages = 4096;
			break;
		
		case DENSITY_CODE_32_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 528 : 512;
			info.pages = 8192;
			break;
		
		case DENSITY_CODE_64_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 1056 : 1024;
			info.pages = 8192;
			break;
		
		case DENSITY_CODE_128_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 1056 : 1024;
			info.pages = 16384;
			break;
		
		case DENSITY_CODE_256_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 2112 : 2048;
			info.pages = 16384;
			break;
		
		case DENSITY_CODE_512_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 2112 : 2048;
			info.pages = 32768;
			break;
	}
	
	return info;
}

dataflash::density_code_t dataflash::get_density_code(info_t *info)
{
	switch(info->pages)
	{
		case 512:
			return DENSITY_CODE_1_MBIT;

		case 1024:
			return DENSITY_CODE_2_MBIT;

		case 2048:
			return DENSITY_CODE_4_MBIT;

		case 4096:
			if(info->page_size == 256 || info->page_size == 264)
				return DENSITY_CODE_8_MBIT;
			else
				return DENSITY_CODE_16_MBIT;

		case 8192:
			if(info->page_size == 512 || info->page_size == 528)
				return DENSITY_CODE_32_MBIT;
			else
				return DENSITY_CODE_64_MBIT;

		case 16384:
			if(info->page_size == 1024 || info->page_size == 1056)
				return DENSITY_CODE_128_MBIT;
			else
				return DENSITY_CODE_256_MBIT;

		case 32768:
			return DENSITY_CODE_512_MBIT;

		default:
			ASSERT(0);
			return (density_code_t)0;
	}
}

int8_t dataflash::cmd(uint8_t cmd, args_t args, uint32_t timeout)
{
	uint8_t cmd_buff[sizeof(cmd) + args.size];
	
	cmd_buff[0] = cmd;
	memcpy(&cmd_buff[1], &args.arg[0], args.size);
	
	int8_t res = _spi.write(cmd_buff, sizeof(cmd_buff), &_cs);
	if(res != RES_OK)
		return RES_SPI_ERR;
	
	if(timeout)
		res = wait_ready(timeout);
	
	return res;
}

int8_t dataflash::cmd_with_write(uint8_t cmd, args_t args, void *buff,
	size_t size, uint32_t timeout)
{
	uint8_t cmd_buff[sizeof(cmd) + args.size];
	
	cmd_buff[0] = cmd;
	memcpy(&cmd_buff[1], &args.arg[0], args.size);
	
	_cs.set(0);
	
	int8_t res = _spi.write(cmd_buff, sizeof(cmd_buff));
	if(res != RES_OK)
		goto Exit;
	
	res = _spi.write(buff, size);
	if(res != RES_OK)
		goto Exit;
	
	_cs.set(1);
	
	if(timeout)
		res = wait_ready(timeout);
	
Exit:
	_cs.set(1);
	return res;
}

int8_t dataflash::cmd_with_read(uint8_t cmd, args_t args, void *buff,
	size_t size, uint8_t dummy_bytes)
{
	uint8_t cmd_buff[sizeof(uint8_t) + args.size + dummy_bytes];
	
	cmd_buff[0] = cmd;
	memcpy(&cmd_buff[1], &args.arg[0], args.size);
	memset(&cmd_buff[1 + args.size], 0xFF, dummy_bytes);
	
	_cs.set(0);
	
	int8_t res = _spi.write(cmd_buff, sizeof(cmd_buff));
	if(res != RES_OK)
		goto Exit;
	
	res = _spi.read(buff, size);
	if(res != RES_OK)
		goto Exit;
	
Exit:
	_cs.set(1);
	return res;
}

int8_t dataflash::wait_ready(uint32_t timeout_ms, status_t *status)
{
	int8_t res;
	status_t _status;
	
	do
	{
		res = read_status(&_status);
		if(res != RES_OK)
		{
			if(status)
				*status = _status;
			return res;
		}
		
		if(_status.ready)
			break;
		
		vTaskDelay(1);
	} while(timeout_ms--);
	
	if(!timeout_ms)
		res = RES_NO_RESPONCE_ERR;
	
	if(status)
		*status = _status;
	
	return res;
}

int8_t dataflash::set_page_size(page_size_t page_size)
{
	const uint8_t *page_size_cmd = (page_size == PAGE_SIZE_DATAFLASH) ?
		cmd_set_page_size_dataflash : cmd_set_page_size_pow2;
	
	args_t args(page_size_cmd[1], page_size_cmd[2], page_size_cmd[3]);
	
	// Do not forget to update _info with new page size
	return cmd(page_size_cmd[0], args, 0);
}
