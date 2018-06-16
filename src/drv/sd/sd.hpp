#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "sd_cid_csd_reg.hpp"
#include "gpio.hpp"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"

namespace drv
{
typedef enum
{
	SD_CARD_UNKNOWN,
	SD_CARD_SD_V1_X,
	SD_CARD_SD_V2_STD_CAPACITY, /* SD v2 or later standard capacity */
	SD_CARD_SD_V2_HI_CAPACITY,  /* SD v2 or later SDHC or SDXC*/
	SD_CARD_MMC_V3
} sd_card_t;

enum sd_err_t
{
	SD_ERR_NONE             =  0,
	SD_ERR_NO_RESPONSE      = -1,
	SD_ERR_NO_CARD          = -2, /* cd pin is high (if cd is present) */
	SD_ERR_CRC              = -3,
	SD_ERR_UNSUPPORTED_CARD = -4,
	SD_ERR_PARAM            = -5,
	SD_ERR_ERASE            = -6,
	SD_ERR_SPI_ERR          = -7,
	SD_ERR_READ_ERR         = -8,
	SD_ERR_WRITE_ERR        = -9,
	SD_ERR_LOCKED           = -10,
};

class sd
{
	public:
		sd(hal::gpio *cd = NULL);
		~sd();
		
		int8_t init();
		
		int8_t read(void *buff, uint32_t block_addr);
		int8_t write(void *buff, uint32_t block_addr);
		int8_t erase(uint64_t start_addr, uint64_t end_addr);
		
		sd_card_t type() const { return _info.type; }
		uint64_t capacity() const { return _info.capacity; }
		
		int8_t read_cid(sd_cid_t *cid);
		int8_t read_csd(sd_csd_t *csd);
	
	protected:
		typedef enum
		{
			/* Software reset
			Arg: no
			Data: no
			Response: R1 */
			CMD0_GO_IDLE_STATE = 0,
			
			/* Initiate init process
			Arg: no
			Data: no
			Response: R1 */
			CMD1_SEND_OP_COND = 1,
			
			/* Only for SDC. Initiate init process
			Arg: Rsv(0)[31], HCS[30], Rsv(0)[29:0]
			Data: no
			Response: R1 */
			ACMD41_APP_SEND_OP_COND = 41,
			
			/* Only for SDS v2. Check voltage range
			Arg: Rsv(0)[31:32], Vdd(1)[11:8], Check pattern(0xAA)[7:0]
			Data: no
			Response: R7 */
			CMD8_SEND_IF_COND = 8,
			
			/* Read CSD register
			Arg: no
			Data: CSD register
			Response: R2 */
			CMD9_SEND_CSD = 9,
			
			/* Read CID register
			Arg: no
			Data: CID register
			Response: R2 */
			CMD10_SEND_CID = 10,
			
			/* Stop to read data
			Arg: no
			Data: no
			Response: R1b */
			CMD12_STOP_TRANSMISSION = 12,
			
			/* Change R/W block size
			Arg: Block length[31:0]
			Data: no
			Response: R1 */
			CMD16_SET_BLOCKLEN = 16,
			
			/* Read a block
			Arg: Address[31:0]
			Data: Block
			Response: R1 */
			CMD17_READ_SINGLE_BLOCK = 17,
			
			/* Read multiple blocks
			Arg: Address[31:0]
			Data: Multiple blocks
			Response: R1 */
			CMD18_READ_MULTIPLE_BLOCK = 18,
			
			/* Only for MMC. Define number of blocks to transfer with next multi-block
			read/write command
			Arg: Number of blocks[15:0]
			Data: no
			Response: R1 */
			CMD23_SET_BLOCK_COUNT = 23,
			
			/* Only for SDS. Define number of blocks to pre-erase with next multi-block
			write command
			Arg: Number of blocks[22:0]
			Data: no
			Response: R1 */
			ACMD23_SET_WR_BLOCK_ERASE_COUNT = 23,
			
			/* Write a block
			Arg: Address[31:0]
			Data: Block
			Response: R1 */
			CMD24_WRITE_SINGLE_BLOCK = 24,
			
			/* Write multiple blocks
			Arg: Address[31:0]
			Data: Multiple blocks
			Response: R1 */
			CMD25_WRITE_MULTIPLE_BLOCK = 25,
			
			/* Leading command of ACMD<n> command
			Arg: no
			Data: no
			Response: R1 */
			CMD55_APP_CMD = 55,
			
			/* Read OCR
			Arg: no
			Data: no
			Response: R3 */
			CMD58_READ_OCR = 58
		} cmd_t;
		
		typedef enum
		{
			R1, // Normal responce. 1 byte
			R2, // CID or CSD register. R1 + 16 bytes + 2 bytes CRC
			R3, // Operation conditions register. R1 + 4 bytes. Response for CMD58
			R6, // Published RCA (relative card address). R1 + 4 bytes
			R7  // Card interface condition. R1 + 4 bytes. Response for CMD8
		} resp_t;
	
	private:
		typedef enum
		{
			SD_CID_REG,
			//SD_RCA_REG,
			//SD_DSR_REG,
			SD_CSD_REG,
			//SD_SCR_REG,
			//SD_OCR_REG,
			//SD_SSR_REG,
			//SD_CSR_REG,
		} sd_reg_t;
		
		int8_t read_reg(sd_reg_t reg, void *buff);
		int8_t set_block_size(uint32_t block_size);
		int8_t process_acmd41(bool is_hi_capacity);
		int8_t process_cmd1();
		
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
			sd_card_t type;
			uint64_t  capacity; // in bytes
		} _info;
};
}
