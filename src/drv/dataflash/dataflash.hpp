#pragma once

#include <stdint.h>

#include "gpio/gpio.hpp"
#include "spi/spi.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
class dataflash
{
	public:
		enum res_t
		{
			RES_OK =  0,
			RES_SPI_ERR = -1,
			RES_ERASE_PORGRAM_ERR = -2,
			RES_UNSUPPORTED_CHIP_ERR = -3,
			RES_NO_RESPONSE_ERR = -4 // Reached timeout for some command
		};
		
		enum page_size_t
		{
			/* Standard DataFlash page size
			   264, 528, 1056 or 2112 bytes
			*/
			PAGE_SIZE_DATAFLASH,
			
			/* "power of 2" (binary) binary page size
			   256, 512, 1024 or 2048 bytes
			*/
			PAGE_SIZE_POW_2
		};
		
		dataflash(hal::spi &spi, hal::gpio &cs, hal::gpio *wp = NULL,
			hal::gpio *rst = NULL);
		~dataflash();
		
		int8_t init(page_size_t page_size = PAGE_SIZE_DATAFLASH);
		
		struct info_t
		{
			uint16_t page_size;
			uint16_t pages;
		};
		info_t info() { return _info; }
		
		int8_t read(void *buff, uint16_t page, uint16_t pages = 1);
		int8_t write(void *buff, uint16_t page, uint16_t pages = 1);
		int8_t erase(uint16_t page, uint16_t pages = 1);
		
#pragma pack(push, 1)
		/* Based on JEDEC publication 106 (JEP106), Manufacturer ID data can
			be comprised of any number of bytes. Some manufacturers may have
			Manufacturer ID codes that are two, three or even four bytes long
			with the first byte(s) in the sequence being 7FH. A system should
			detect code 7FH as a "Continuation Code" and continue to read
			Manufacturer ID bytes. The first non-7FH byte would signify the
			last byte of Manufacturer ID data. For Atmel (and some other
			manufacturers), the Manufacturer ID data is comprised of only
			one byte.
		*/
		//uint8_t jedec_continuation_number;
		
		struct jedec_t
		{
			// JEDEC Assigned Code
			uint8_t manufacturer_id:8;
			
			struct
			{
				/* 00010 - 1 MBit
				   00011 - 2 MBit
				   00100 - 4 MBit
				   00101 - 8 Mbit
				   00110 - 16 Mbit
				   00111 - 32 Mbit
				   01000 - 64 Mbit
				   01001 - 128 Mbit
				   01010 - 256 Mbit
				   01011 - 512 Mbit
				*/
				uint8_t density_code:5;
				
				/* 001 - AT45Dxxx
				   010 - AT26DFxxx
				*/
				uint8_t family_code:3;
				
				uint8_t product_vers_code:5;
				
				/* 000 - 1-bit/Cell Technology (SLC)
				   001 - 2-bit/Cell Technology (MLC)
				*/
				uint8_t mlc_code:3;
			} dev_id;
			
			/* Indicate number of additional bytes with extended info.
			   As indicated in the JEDEC Standard, reading the EDI String Length
			   and any subsequent data is optional.
			*/
			uint8_t extended_dev_info_len:8;
		};
#pragma pack(pop)
		
		int8_t jedec(jedec_t *jedec);
	
	private:
		/* 0   bit allways equal to 0
		   3:1 bits are significant
		   
		   density in Mbits for <= 64 MBit dataflash:
		   2^(significant_bits - 1)
		*/
		enum density_code_t
		{
			DENSITY_CODE_1_MBIT   = (1 << 1) | 0b0001,
			DENSITY_CODE_2_MBIT   = (2 << 1) | 0b0001,
			DENSITY_CODE_4_MBIT   = (3 << 1) | 0b0001,
			DENSITY_CODE_8_MBIT   = (4 << 1) | 0b0001,
			DENSITY_CODE_16_MBIT  = (5 << 1) | 0b0001,
			DENSITY_CODE_32_MBIT  = (6 << 1) | 0b0001,
			DENSITY_CODE_64_MBIT  = (7 << 1) | 0b0001,
			DENSITY_CODE_128_MBIT = 4,
			DENSITY_CODE_256_MBIT = 6,
			DENSITY_CODE_512_MBIT = 8
		};
		
#pragma pack(push, 1)
		struct status_t
		{
			// ----------- 1-st byte of status register
			page_size_t page_size:1;
			
			/* 0 - Sector protection is disabled
			   1 - Sector protection is enabled
			*/
			bool sector_protection:1;
			
			density_code_t density_code:4;
			
			/* 0 - Main memory page data matches buffer data
			   1 - Main memory page data does not match buffer data
			*/
			bool compare_result:1;
			
			/* 0 - Device is busy with an internal operation
			   1 - Device is ready
			*/
			bool ready:1;
			
			// ----------- 2-nd byte of status register (optional)
			/* 0 - No sectors are erase suspended
			   1 - A sector is erase suspended
			*/
			bool erase_suspend:1;
			
			/* 0 - No program operation has been suspended while using Buffer 1
			   1 - A sector is program suspended while using Buffer 1
			*/
			bool program_suspend_status_buff_1:1;
			
			/* 0 - No program operation has been suspended while using Buffer 2
			   1 - A sector is program suspended while using Buffer 2
			*/
			bool program_suspend_status_buff_2:1;
			
			/* 0 - Sector Lockdown command is disabled
			   1 - Sector Lockdown command is enabled
			*/
			bool sector_lockdown:1;
			
			// Reserved for future use
			bool reserved2:1;
			
			/* 0 - Erase or program operation was successful
			   1 - Erase or program error detected
			*/
			bool erase_program_error:1;
			
			// Reserved for future use
			bool reserved1:1;
			
			/* 0 - Device is busy with an internal operation
			   1 - Device is ready
			*/
			bool ready2:1;
		};
#pragma pack(pop)
		struct args_t
		{
			args_t() {};
			args_t(uint16_t page, uint16_t byte, info_t info);
			args_t(uint8_t arg1, uint8_t arg2, uint8_t arg3);
			
			uint8_t arg[4];
			
			// May equal to 4 if chip size >= 128 MBit, otherwise equal to 3
			uint8_t size;
		};
		
		hal::spi &_spi;
		hal::gpio &_cs;
		hal::gpio *_wp;
		hal::gpio *_rst;
		SemaphoreHandle_t api_lock;
		info_t _info;
		
		int8_t read_status(status_t *status);
		bool is_status_valid(status_t *status);
		
		bool is_jedec_valid(jedec_t *jedec);
		
		info_t get_info(density_code_t density_code, page_size_t page_size);
		density_code_t get_density_code(info_t *info);
		
		int8_t cmd(uint8_t cmd, args_t args, uint32_t timeout);
		int8_t cmd_with_write(uint8_t cmd, args_t args, void *buff, size_t size,
			uint32_t timeout);
		int8_t cmd_with_read(uint8_t cmd, args_t args, void *buff, size_t size,
			uint8_t dummy_bytes);
		
		int8_t wait_ready(uint32_t timeout_ms, status_t *status = NULL);
		int8_t set_page_size(page_size_t page_size);
};
}
