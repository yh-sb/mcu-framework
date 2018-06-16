#include <stddef.h>
#include <cstring>

#include "common/macros.h"
#include "sd.hpp"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"
#include "third_party/FreeRTOS/include/task.h"

using namespace drv;
using namespace hal;

#define SPI_INIT_BAUD            400000
#define SPI_HISPEED_BAUD         5000000

#define POWER_STABILIZATION_TIME 250 // ms
#define ACMD41_CMD1_RETRY_CNT    1000

#define CMD8_VHS           0x01 // 2.7-3.6 v "Voltage Supplied"
#define CMD8_CHECK_PATTERN 0xAA

#define OCR_CCS_BIT  30 /* Card Capacity Status */
#define OCR_BUSY_BIT 31 /* Card power up status bit */

#define SD_BLOCK_SIZE 512

/* Response 1 description */
typedef enum
{
	R1_IN_IDLE_STATE = 0x01, /* Card is in idle state */
	R1_ERASE_RST     = 0x02, /* Erase sequence was cleared before executing
	                            because an out of erase sequence command was
	                            received */
	R1_ILLEGAL_CMD   = 0x04,
	R1_COM_CRC_ERR   = 0x08,
	R1_ERASE_SEQ_ERR = 0x10, /* Error in the sequence of erase command
	                            occured */
	R1_ADDR_ERR      = 0x20, /* A misaligned address, which did not match the
	                            block length, was used in the command */
	R1_PARAM_ERR     = 0x40, /* The command's argument (e.g. address, block
	                            length) was out of the allowed range for this
	                            card */
	R1_START_BIT     = 0x80  /* Should be always 0 */
} r1_t;

static int8_t check_r1(uint32_t r1);
static uint32_t get_block_count(sd_csd_t *csd);
static void decode_cid(uint8_t *raw_cid, sd_cid_t *cid);
static int8_t decode_csd(uint8_t *raw_csd, sd_csd_t *csd, sd_card_t type);

sd::sd(gpio *cd):
	_cd(cd)
{
	ASSERT(!_cd || _cd->mode() == GPIO_MODE_DI);
	
	memset(&_info, 0, sizeof(_info));
	
	api_lock = xSemaphoreCreateMutex();
	ASSERT(api_lock);
}

sd::~sd()
{
	vSemaphoreDelete(api_lock);
}

int8_t sd::init()
{
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	memset(&_info, 0, sizeof(_info));
	
	int8_t res = SD_ERR_NONE;
	if(_cd && _cd->get())
	{
		res = SD_ERR_NO_CARD;
		goto Exit;
	}
	
	set_speed(SPI_INIT_BAUD);
	// TODO: In fact this delay is 790 us instead of 250000 us
	vTaskDelay(POWER_STABILIZATION_TIME);
	
	init_sd();
	
	select(true);
	
	/* Process CMD0 */
	uint8_t r1;
	res = send_cmd(CMD0_GO_IDLE_STATE, 0, R1, &r1);
	if(res)
		goto Exit;
	
	res = check_r1(r1);
	if(res)
		goto Exit;
	if(!(r1 & R1_IN_IDLE_STATE))
	{
		res = SD_ERR_UNSUPPORTED_CARD;
		goto Exit;
	}
	
	/* Process CMD8 */
	uint16_t cmd8_cond;
	cmd8_cond = (CMD8_VHS << 8) | CMD8_CHECK_PATTERN;
	uint8_t r7[5];
	res = send_cmd(CMD8_SEND_IF_COND, cmd8_cond, R7, r7);
	if(res)
		goto Exit;
	
	uint16_t retry_cnt;
	if(r7[0] & R1_ILLEGAL_CMD)
	{
		/* If the CMD8 is rejected with illigal command error, the card is
		probably SD v1 or MMC v3 */
		
		/* If it is really error (for example crc) - handle it */
		res = check_r1(r7[0]);
		if(res != SD_ERR_UNSUPPORTED_CARD)
			goto Exit;
		
		res = process_acmd41(false);
		if(!res)
		{
			_info.type = SD_CARD_SD_V1_X;
			res = set_block_size(SD_BLOCK_SIZE);
			goto Exit;
		}
		
		/* If the ACMD41 is fail, the card is probably MMC v3 */
		res = process_cmd1();
		if(!res)
		{
			_info.type = SD_CARD_MMC_V3;
			res = set_block_size(SD_BLOCK_SIZE);
		}
		goto Exit;
	}
	/* If the CMD8 is OK, the card is SD v2+ with block or byte addressing type */
	res = check_r1(r7[0]);
	if(res)
		goto Exit;
	
	/* Check CMD8 pattern */
	if(((r7[3] << 8) | r7[4]) != cmd8_cond)
	{
		res = SD_ERR_UNSUPPORTED_CARD;
		goto Exit;
	}
	
	/* Process ACMD41 with hi capacity bit */
	res = process_acmd41(true);
	if(res)
		goto Exit;
	
	/* Process CMD58 to determine SD v2+ with block or byte addressing type */
	uint8_t r3[5];
	res = send_cmd(CMD58_READ_OCR, 0, R3, r3);
	if(res)
		goto Exit;
	
	res = check_r1(r3[0]);
	if(res)
		goto Exit;
	
	uint32_t ocr;
	ocr = (r3[1] << 24) | (r3[2] << 16) | (r3[3] << 8) | r3[4];
	if(ocr & (1 << OCR_CCS_BIT))
	{
		_info.type = SD_CARD_SD_V2_HI_CAPACITY;
		goto Exit;
	}
	_info.type = SD_CARD_SD_V2_STD_CAPACITY;
	
	res = set_block_size(SD_BLOCK_SIZE);
	
Exit:
	if(!res)
	{
		sd_csd_t csd;
		res = read_reg(SD_CSD_REG, &csd);
		if(!res)
			_info.capacity = (uint64_t)get_block_count(&csd) * SD_BLOCK_SIZE;
	}
	
	select(false);
	set_speed(SPI_HISPEED_BAUD);
	xSemaphoreGive(api_lock);
	return res;
}

int8_t sd::read(void *buff, uint32_t block_addr)
{
	ASSERT(buff);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = SD_ERR_NONE;
	if(_cd && _cd->get())
	{
		res = SD_ERR_NO_CARD;
		goto Exit;
	}
	
	select(true);
	
	uint8_t r1;
	res = send_cmd(CMD17_READ_SINGLE_BLOCK, block_addr, R1, &r1);
	if(res)
		goto Exit;
	
	res = check_r1(r1);
	if(res)
		goto Exit;
	
	res = read_data(buff, SD_BLOCK_SIZE);
	if(res)
		goto Exit;
	
Exit:
	select(false);
	xSemaphoreGive(api_lock);
	return res;
}

int8_t sd::write(void *buff, uint32_t block_addr)
{
	ASSERT(buff);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = SD_ERR_NONE;
	if(_cd && _cd->get())
	{
		res = SD_ERR_NO_CARD;
		goto Exit;
	}
	
	select(true);
	
	uint8_t r1;
	res = send_cmd(CMD24_WRITE_SINGLE_BLOCK, block_addr, R1, &r1);
	if(res)
		goto Exit;
	
	res = check_r1(r1);
	if(res)
		goto Exit;
	
	res = write_data(buff, SD_BLOCK_SIZE);
	if(res)
		goto Exit;
	
Exit:
	select(false);
	xSemaphoreGive(api_lock);
	return res;
}

int8_t sd::erase(uint64_t start_addr, uint64_t end_addr)
{
	
}

int8_t sd::read_cid(sd_cid_t *cid)
{
	uint8_t r2[17];
	int8_t res = send_cmd(CMD10_SEND_CID, 0, R2, r2);
	if(res)
		return res;
	
	res = check_r1(r2[0]);
	if(res)
		return res;
int8_t sd::read_cid(sd_cid_t *cid)
{
	ASSERT(cid);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = SD_ERR_NONE;
	if(_cd && _cd->get())
	{
		res = SD_ERR_NO_CARD;
		goto Exit;
	}
	
	select(true);
	
	res = read_reg(SD_CID_REG, cid);
	
Exit:
	select(false);
	xSemaphoreGive(api_lock);
	return res;
}

int8_t sd::read_csd(sd_csd_t *csd)
{
	ASSERT(csd);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = SD_ERR_NONE;
	if(_cd && _cd->get())
	{
		res = SD_ERR_NO_CARD;
		goto Exit;
	}
	
	select(true);
	
	res = read_reg(SD_CSD_REG, csd);
	
Exit:
	select(false);
	xSemaphoreGive(api_lock);
	return res;
}

int8_t sd::read_reg(sd_reg_t reg, void *buff)
{
	int8_t res;
	uint8_t r2[17];
	
	switch(reg)
	{
		case SD_CID_REG:
			res = send_cmd(CMD10_SEND_CID, 0, R2, r2);
			if(res)
				return res;
			
			res = check_r1(r2[0]);
			if(res)
				return res;
			
			decode_cid(&r2[1], (sd_cid_t *)buff);
			break;
		
		case SD_CSD_REG:
			res = send_cmd(CMD9_SEND_CSD, 0, R2, r2);
			if(res)
				return res;
			
			res = check_r1(r2[0]);
			if(res)
				return res;
			
			res = decode_csd(&r2[1], (sd_csd_t *)buff, _info.type);
			if(res)
				memset(buff, 0, sizeof(sd_csd_t));
			break;
		
		default:
			ASSERT(0);
			return SD_ERR_PARAM;
	}
	
	return res;
}

int8_t sd::set_block_size(uint32_t block_size)
{
	/* Process CMD16 to force block size to 512 bytes to work with FAT file
	system */
	uint8_t r1;
	int8_t res = send_cmd(CMD16_SET_BLOCKLEN, block_size, R1, &r1);
	if(res)
		return res;
	
	return check_r1(r1);
}

int8_t sd::process_acmd41(bool is_hi_capacity)
{
	int8_t res;
	uint8_t r1;
	uint16_t retry_cnt = ACMD41_CMD1_RETRY_CNT;
	while(retry_cnt--)
	{
		res = send_cmd(CMD55_APP_CMD, 0, R1, &r1);
		if(res)
			goto Exit;
		
		res = check_r1(r1);
		if(res)
			goto Exit;
		
		res = send_cmd(ACMD41_APP_SEND_OP_COND, is_hi_capacity ? 0x40000000 : 0,
			R1, &r1);
		if(res)
			goto Exit;
		
		res = check_r1(r1);
		if(res)
			goto Exit;
		
		if(!(r1 & R1_IN_IDLE_STATE))
			break;
		
		vTaskDelay(1);
	}
	if(!retry_cnt)
		res = SD_ERR_NO_RESPONSE;
	
Exit:
	return res;
}

int8_t sd::process_cmd1()
{
	int8_t res;
	uint8_t r1;
	uint16_t retry_cnt = ACMD41_CMD1_RETRY_CNT;
	while(retry_cnt--)
	{
		res = send_cmd(CMD1_SEND_OP_COND, 0, R1, &r1);
		if(res)
			goto Exit;
		
		res = check_r1(r1);
		if(res)
			goto Exit;
		
		if(!(r1 & R1_IN_IDLE_STATE))
			break;
		
		vTaskDelay(1);
	}
	if(!retry_cnt)
		res = SD_ERR_NO_RESPONSE;
	
Exit:
	return res;
}

static int8_t check_r1(uint32_t r1)
{
	int8_t res = SD_ERR_NONE;
	
	if(r1 & R1_COM_CRC_ERR)
		res = SD_ERR_CRC;
	else if((r1 & R1_ERASE_RST) || (r1 & R1_ERASE_SEQ_ERR))
		res = SD_ERR_ERASE;
	else if((r1 & R1_PARAM_ERR) || (r1 & R1_ADDR_ERR))
		res = SD_ERR_PARAM;
	else if(r1 & R1_ILLEGAL_CMD)
		res = SD_ERR_UNSUPPORTED_CARD;
	
	return res;
}

static uint32_t calc_block_size(uint8_t *raw_csd, sd_card_t type)
{
	if(type == SD_CARD_SD_V2_STD_CAPACITY || type == SD_CARD_SD_V2_HI_CAPACITY)
	{
		return (16UL << (raw_csd[10] >> 4));
	}
	else if(type == SD_CARD_SD_V1_X)
	{
		return ((((raw_csd[10] & 63) << 1) + ((uint16_t)(raw_csd[11] & 128) >> 7) + 1) << ((raw_csd[13] >> 6) - 1));
	}
	else if(type == SD_CARD_MMC_V3)
	{
		return (((uint16_t)((raw_csd[10] & 124) >> 2) + 1) * (((raw_csd[11] & 3) << 3) + ((raw_csd[11] & 224) >> 5) + 1));
	}
	else
		return 0;
}

static uint32_t get_block_count(sd_csd_t *csd)
{
	uint32_t res;
	switch(csd->v1.csd_struct)
	{
		case SD_CSD_STRUCT_VER_1_0:
			res = (csd->v1.dev_size + 1) * (1 << (csd->v1.dev_size_mul + 2));
			res = (res * (1 << csd->v1.max_read_block_len)) / SD_BLOCK_SIZE;
			break;
		
		case SD_CSD_STRUCT_VER_2_0:
			res = (csd->v1.dev_size + 1) * 1024;
			break;
		
		default:
			res = 0;
	}
	
	return res;
}

static void decode_cid(uint8_t *raw_cid, sd_cid_t *cid)
{
	cid->manufact_id = (sd_manufact_id_t)raw_cid[0];
	memcpy(cid->oem_app_id, &raw_cid[1], sizeof(cid->oem_app_id));
	memcpy(cid->prod_name, &raw_cid[3], sizeof(cid->prod_name));
	cid->prod_rev = raw_cid[8];
	cid->prod_sn = (raw_cid[9] << 24) | (raw_cid[10] << 16) |
		(raw_cid[11] << 8) | raw_cid[12];
	cid->manufact_date = (raw_cid[13] << 8) | raw_cid[14];
	cid->crc7 = raw_cid[15] & 0xFE;
}

static int8_t decode_csd(uint8_t *raw_csd, sd_csd_t *csd, sd_card_t type)
{
	uint8_t csd_struct = raw_csd[0] >> 6;
	
	// TODO: need to rewrite the following code more understandable way
	if(csd_struct == SD_CSD_STRUCT_VER_1_0 && (type == SD_CARD_SD_V1_X ||
		type == SD_CARD_SD_V2_STD_CAPACITY))
	{
		csd->v1.csd_struct = (sd_csd_struct_t)csd_struct;
		csd->v1.taac = raw_csd[1];
		csd->v1.nsac = raw_csd[2];
		csd->v1.max_transfer_rate = raw_csd[3];
		csd->v1.cmd_classes = (raw_csd[4] << 4) | (raw_csd[5] >> 4);
		csd->v1.max_read_block_len = (sd_csd_max_read_block_len_t)(raw_csd[5] & 0x0F);
		csd->v1.partial_block_read_en = raw_csd[6] >> 7;
		csd->v1.write_block_misalign = (raw_csd[6] & 0x40) >> 6;
		csd->v1.read_block_misalign = (raw_csd[6] & 0x20) >> 5;
		csd->v1.dsr_impl = (raw_csd[6] & 0x10) >> 4;
		csd->v1.dev_size = ((raw_csd[6] & 0x03) << 10) | (raw_csd[7] << 2) | (raw_csd[8] >> 6);
		csd->v1.max_read_curr_vdd_min = (sd_csd_max_curr_vdd_min_t)((raw_csd[8] & 0x38) >> 3);
		csd->v1.max_read_curr_vdd_max = (sd_csd_max_curr_vdd_max_t)(raw_csd[8] & 0x07);
		csd->v1.max_write_curr_vdd_min = (sd_csd_max_curr_vdd_min_t)(raw_csd[9] >> 5);
		csd->v1.max_write_curr_vdd_max = (sd_csd_max_curr_vdd_max_t)((raw_csd[9] & 0x1C) >> 2);
		csd->v1.dev_size_mul = (sd_csd_dev_size_mul_t)(((raw_csd[9] & 0x03) << 1) | (raw_csd[10] >> 7));
		csd->v1.erase_single_block_en = (raw_csd[10] & 0x40) >> 6;
		csd->v1.erase_sector_size = ((raw_csd[10] & 0x3F) << 1) | (raw_csd[11] >> 7);
		csd->v1.write_protect_group_size = raw_csd[11] & 0x7F;
		csd->v1.write_protect_group_en = raw_csd[12] >> 7;
		csd->v1.r2w_factor = (sd_csd_r2w_factor_t)((raw_csd[12] & 0x1C) >> 2);
		csd->v1.max_write_block_len = (sd_csd_max_write_block_len_t)(((raw_csd[12] & 0x03) << 2) | (raw_csd[13] >> 6));
		csd->v1.partial_block_write_en = (raw_csd[13] & 0x20) >> 5;
		csd->v1.file_format_grp = raw_csd[14] >> 7;
		csd->v1.copy_flag = (raw_csd[14] & 0x40) >> 6;
		csd->v1.perm_write_protection = (raw_csd[14] & 0x20) >> 5;
		csd->v1.temp_write_protection = (raw_csd[14] & 0x10) >> 4;
		csd->v1.file_format = (sd_csd_file_format_t)((raw_csd[14] & 0x0C) >> 2);
		csd->v1.crc7 = raw_csd[15] & 0xFE;
	}
	else if(csd_struct == SD_CSD_STRUCT_VER_2_0 &&
		type == SD_CARD_SD_V2_HI_CAPACITY)
	{
		csd->v2.csd_struct = (sd_csd_struct_t)csd_struct;
		csd->v2.taac = raw_csd[1];
		csd->v2.nsac = raw_csd[2];
		csd->v2.max_transfer_rate = raw_csd[3];
		csd->v2.cmd_classes = (raw_csd[4] << 4) | (raw_csd[5] >> 4);
		csd->v2.max_read_block_len = (sd_csd_max_read_block_len_t)(raw_csd[5] & 0x0F);
		csd->v2.partial_block_read_en = raw_csd[6] >> 7;
		csd->v2.write_block_misalign = (raw_csd[6] & 0x40) >> 6;
		csd->v2.read_block_misalign = (raw_csd[6] & 0x20) >> 5;
		csd->v2.dsr_impl = (raw_csd[6] & 0x10) >> 4;
		csd->v2.dev_size = ((raw_csd[7] >> 2) << 16) | (raw_csd[8] << 8) | raw_csd[9];
		csd->v2.erase_single_block_en = (raw_csd[10] & 0x40) >> 6;
		csd->v2.erase_sector_size = ((raw_csd[10] & 0x3F) << 1) | (raw_csd[11] >> 7);
		csd->v2.write_protect_group_size = raw_csd[11] & 0x7F;
		csd->v2.write_protect_group_en = raw_csd[12] >> 7;
		csd->v2.r2w_factor = (sd_csd_r2w_factor_t)((raw_csd[12] & 0x1C) >> 2);
		csd->v2.max_write_block_len = (sd_csd_max_write_block_len_t)(((raw_csd[12] & 0x03) << 2) | (raw_csd[13] >> 6));
		csd->v2.partial_block_write_en = (raw_csd[13] & 0x20) >> 5;
		csd->v2.file_format_grp = raw_csd[14] >> 7;
		csd->v2.copy_flag = (raw_csd[14] & 0x40) >> 6;
		csd->v2.perm_write_protection = (raw_csd[14] & 0x20) >> 5;
		csd->v2.temp_write_protection = (raw_csd[14] & 0x10) >> 4;
		csd->v2.file_format = (sd_csd_file_format_t)((raw_csd[14] & 0x0C) >> 2);
		csd->v2.crc7 = raw_csd[15] & 0xFE;
	}
	else if((csd_struct == SD_CSD_STRUCT_VER_1_0 ||
		csd_struct == SD_CSD_STRUCT_VER_2_0) && type == SD_CARD_MMC_V3)
	{
		csd->mmc.csd_struct = (sd_csd_struct_t)csd_struct;
		csd->mmc.spec_ver = (sd_csd_spec_ver_t)((raw_csd[0] & 0x3C) >> 2);
		csd->mmc.taac = raw_csd[1];
		csd->mmc.nsac = raw_csd[2];
		csd->mmc.max_transfer_rate = raw_csd[3];
		csd->mmc.cmd_classes = (raw_csd[4] << 4) | (raw_csd[5] >> 4);
		csd->mmc.max_read_block_len = (sd_csd_max_read_block_len_t)(raw_csd[5] & 0x0F);
		csd->mmc.partial_block_read_en = raw_csd[6] >> 7;
		csd->mmc.write_block_misalign = (raw_csd[6] & 0x40) >> 6;
		csd->mmc.read_block_misalign = (raw_csd[6] & 0x20) >> 5;
		csd->mmc.dsr_impl = (raw_csd[6] & 0x10) >> 4;
		csd->mmc.dev_size = ((raw_csd[6] & 0x03) << 10) | (raw_csd[7] << 2) | (raw_csd[8] >> 6);
		csd->mmc.max_read_curr_vdd_min = (sd_csd_max_curr_vdd_min_t)((raw_csd[8] & 0x38) >> 3);
		csd->mmc.max_read_curr_vdd_max = (sd_csd_max_curr_vdd_max_t)(raw_csd[8] & 0x07);
		csd->mmc.max_write_curr_vdd_min = (sd_csd_max_curr_vdd_min_t)(raw_csd[9] >> 5);
		csd->mmc.max_write_curr_vdd_max = (sd_csd_max_curr_vdd_max_t)((raw_csd[9] & 0x1C) >> 2);
		csd->mmc.dev_size_mul = (sd_csd_dev_size_mul_t)(((raw_csd[9] & 0x03) << 1) | (raw_csd[10] >> 7));
		csd->mmc.erase_sector_size = (raw_csd[10] & 0x7C) >> 2;
		csd->mmc.erase_group_size = ((raw_csd[10] & 0x03) << 3) | (raw_csd[11] >> 5);
		csd->mmc.write_protect_group_size = raw_csd[11] & 0x1F;
		csd->mmc.write_protect_group_en = raw_csd[12] >> 7;
		csd->mmc.default_eec = (raw_csd[12] & 0x60) >> 5;
		csd->mmc.r2w_factor = (sd_csd_r2w_factor_t)((raw_csd[12] & 0x1C) >> 2);
		csd->mmc.max_write_block_len = (sd_csd_max_write_block_len_t)(((raw_csd[12] & 0x03) << 2) | (raw_csd[13] >> 6));
		csd->mmc.partial_block_write_en = (raw_csd[13] & 0x20) >> 5;
		csd->mmc.file_format_grp = raw_csd[14] >> 7;
		csd->mmc.copy_flag = (raw_csd[14] & 0x40) >> 6;
		csd->mmc.perm_write_protection = (raw_csd[14] & 0x20) >> 5;
		csd->mmc.temp_write_protection = (raw_csd[14] & 0x10) >> 4;
		csd->mmc.file_format = (sd_csd_file_format_t)((raw_csd[14] & 0x0C) >> 2);
		csd->mmc.eec = raw_csd[14] & 0x03;
		csd->mmc.crc7 = raw_csd[15] & 0xFE;
	}
	else
		return SD_ERR_UNSUPPORTED_CARD;
	
	return SD_ERR_NONE;
}
