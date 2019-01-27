#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "sd_cid_csd_reg.hpp"
#include "gpio/gpio.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
#define SD_BLOCK_SIZE 512

class sd
{
	public:
		enum type_t
		{
			TYPE_UNKNOWN,
			TYPE_SD_V1_X,
			TYPE_SD_V2_STD_CAPACITY, /* SD v2 or later standard capacity */
			TYPE_SD_V2_HI_CAPACITY,  /* SD v2 or later SDHC or SDXC*/
			TYPE_MMC_V3
		};

		enum res_t
		{
			RES_OK               =  0,
			RES_NO_RESPONSE      = -1,
			RES_NO_CARD          = -2, /* cd pin is high (if cd is present) */
			RES_CRC_ERR          = -3,
			RES_UNSUPPORTED_CARD = -4,
			RES_PARAM_ERR        = -5,
			RES_ERASE_ERR        = -6,
			RES_SPI_ERR          = -7,
			RES_READ_ERR         = -8,
			RES_WRITE_ERR        = -9,
			RES_LOCKED           = -10,
		};
		
		sd(hal::gpio *cd = NULL);
		~sd();
		
		int8_t init();
		
		int8_t read(void *buff, uint32_t block_addr);
		int8_t write(void *buff, uint32_t block_addr);
		int8_t erase(uint64_t start_addr, uint64_t end_addr);
		
		type_t type() const { return _info.type; }
		uint64_t capacity() const { return _info.capacity; }
		
		int8_t read_cid(sd_cid_t *cid);
		int8_t read_csd(sd_csd_t *csd);
	
	protected:
		enum cmd_t
		{
			CMD0_GO_IDLE_STATE              = 0,  // Software reset
			CMD1_SEND_OP_COND               = 1,  // Initiate init process
			ACMD41_APP_SEND_OP_COND         = 41, // Initiate init process
			CMD8_SEND_IF_COND               = 8,  // Check voltage range
			CMD9_SEND_CSD                   = 9,  // Read CSD register
			CMD10_SEND_CID                  = 10, // Read CID register
			CMD12_STOP_TRANSMISSION         = 12, // Stop to read data
			CMD16_SET_BLOCKLEN              = 16, // Change R/W block size
			CMD17_READ_SINGLE_BLOCK         = 17, // Read a block
			CMD18_READ_MULTIPLE_BLOCK       = 18, // Read multiple blocks
			ACMD23_SET_WR_BLOCK_ERASE_COUNT = 23, // Define number of blocks to
			                                      // pre-erase with next
			                                      // multi-block write command
			CMD24_WRITE_SINGLE_BLOCK        = 24, // Write a block
			CMD25_WRITE_MULTIPLE_BLOCK      = 25, // Write multiple blocks
			CMD32_ERASE_GROUP_START         = 32, // Set the addr of the first
			                                      // block to be erased
			CMD33_ERASE_GROUP_END           = 33, // Set the addr of the last
			                                      // block to be erased
			CMD38_ERASE                     = 38, // Erases all previously
			                                      // selected blocks
			CMD55_APP_CMD                   = 55, // Leading cmd of ACMD<n> cmd
			CMD58_READ_OCR                  = 58  // Read OCR
		};
		
		enum resp_t
		{
			R1, // Normal responce. 1 byte
			R2, // CID or CSD register. R1 + 16 bytes + 2 bytes CRC
			R3, // Operation conditions register. R1 + 4 bytes. Response for CMD58
			R6, // Published RCA (relative card address). R1 + 4 bytes
			R7  // Card interface condition. R1 + 4 bytes. Response for CMD8
		};
	
	private:
		enum reg_t
		{
			SD_CID_REG,
			//SD_RCA_REG,
			//SD_DSR_REG,
			SD_CSD_REG,
			//SD_SCR_REG,
			//SD_OCR_REG,
			//SD_SSR_REG,
			//SD_CSR_REG,
		};
		
		int8_t read_reg(reg_t reg, void *buff);
		int8_t set_block_size(uint32_t block_size);
		int8_t process_acmd41(bool is_hi_capacity);
		int8_t process_cmd1();
		static res_t check_r1(uint32_t r1);
		
		virtual void select(bool is_selected) = 0;
		virtual int8_t init_sd() = 0;
		virtual void set_speed(uint32_t speed) = 0;
		virtual int8_t send_cmd(cmd_t cmd, uint32_t arg, resp_t resp_type, uint8_t *resp) = 0;
		virtual int8_t read_data(void *data, uint16_t size) = 0;
		virtual int8_t write_data(void *data, uint16_t size) = 0;
		
		hal::gpio *_cd;
		SemaphoreHandle_t api_lock;
		
		struct
		{
			type_t type;
			uint64_t  capacity; // in bytes
		} _info;
};
}
