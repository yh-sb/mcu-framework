#pragma once

#include "sd_cid_csd_reg.hpp"
#include "periph/gpio.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
#define SD_BLOCK_SIZE 512

class sd
{
public:
    enum class res : int8_t
    {
        ok               =  0,
        no_response      = -1,
        no_card          = -2, // cd pin is high (if cd is present)
        crc_err          = -3,
        unsupported_card = -4,
        param_err        = -5,
        erase_err        = -6,
        spi_err          = -7,
        read_err         = -8,
        write_err        = -9,
        locked           = -10,
    };
    
    enum class type : uint8_t
    {
        unknown,
        sd_v1_x,
        sd_v2_std_capacity, // SD v2 or later standard capacity
        sd_v2_hi_capacity,  // SD v2 or later SDHC or SDXC
        mmc_v3
    };
    
    sd(periph::gpio *cd = nullptr);
    ~sd();
    
    enum res init();
    
    enum res read(void *buff, uint32_t block_addr);
    enum res write(void *buff, uint32_t block_addr);
    enum res erase(uint64_t start_addr, uint64_t end_addr);
    
    enum type type() const { return info.type; }
    uint64_t capacity() const { return info.capacity; }
    
    enum res read_cid(drv::sd_regs::cid_t *cid);
    enum res read_csd(drv::sd_regs::csd_t *csd);
    
    // Delete copy constructor and copy assignment operator
    sd(const sd&) = delete;
    sd& operator=(const sd&) = delete;
    
    // Delete move constructor and move assignment operator
    sd(sd&&) = delete;
    sd& operator=(sd&&) = delete;
    
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
        ACMD23_SET_WR_BLOCK_ERASE_COUNT = 23, // Define number of blocks to pre-erase with next multi-block write command
        CMD24_WRITE_SINGLE_BLOCK        = 24, // Write a block
        CMD25_WRITE_MULTIPLE_BLOCK      = 25, // Write multiple blocks
        CMD32_ERASE_GROUP_START         = 32, // Set the addr of the first block to be erased
        CMD33_ERASE_GROUP_END           = 33, // Set the addr of the last block to be erased
        CMD38_ERASE                     = 38, // Erases all previously selected blocks
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
    
    enum res read_reg(reg_t reg, void *buff);
    enum res set_block_size(uint32_t block_size);
    enum res process_acmd41(bool is_hi_capacity);
    enum res process_cmd1();
    static enum res check_r1(uint32_t r1);
    
    virtual void select(bool is_selected) = 0;
    virtual enum res init_sd() = 0;
    virtual void set_speed(uint32_t speed) = 0;
    virtual enum res send_cmd(cmd_t cmd, uint32_t arg, resp_t resp_type, uint8_t *resp) = 0;
    virtual enum res read_data(void *data, uint16_t size) = 0;
    virtual enum res write_data(void *data, uint16_t size) = 0;
    
    periph::gpio *cd;
    SemaphoreHandle_t api_lock;
    
    struct
    {
        enum type type;
        uint64_t  capacity; // in bytes
    } info;
};
} // namespace drv
