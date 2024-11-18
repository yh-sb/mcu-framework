#pragma once

#include <cstdint>
#include <cstdbool>

namespace drv::sd_regs
{
/* Unofficial list of manufacturer IDs description */
enum manufact_id_t
{
    SD_MANUFACT_ID_PANASONIC = 1,
    SD_MANUFACT_ID_TOSHIBA   = 2,
    SD_MANUFACT_ID_SANDISK   = 3,
    SD_MANUFACT_ID_RITEK     = 6,
    SD_MANUFACT_ID_ATP       = 9,
    SD_MANUFACT_ID_KINGMAX   = 19,
    SD_MANUFACT_ID_DYNACARD  = 25,
    SD_MANUFACT_ID_PQI       = 26,
    SD_MANUFACT_ID_SAMSUNG   = 27,
    SD_MANUFACT_ID_ADATA     = 29,
    SD_MANUFACT_ID_PHISON    = 39,
    SD_MANUFACT_ID_BARUN     = 40,
    SD_MANUFACT_ID_STEC      = 81,
    SD_MANUFACT_ID_SWISSBIT  = 93,
    SD_MANUFACT_ID_NETLIST   = 97,
    SD_MANUFACT_ID_CACTUS    = 99,
    SD_MANUFACT_ID_JIAELEC   = 116,
    SD_MANUFACT_ID_JANGTAY   = 130,
    SD_MANUFACT_ID_NETCOM    = 131,
    SD_MANUFACT_ID_STRONTIUM = 133,
};

/* Card identification register */
struct cid_t
{
    manufact_id_t manufact_id;   /* ManufacturerID */
    char             oem_app_id[2]; /* OEM/Application ID */
    char             prod_name[5];  /* Product name (ASCII string) */
    uint8_t          prod_rev;      /* Product revision (example: 0x47 means 4.7) */
    uint32_t         prod_sn;       /* Product serial number */
    uint16_t         manufact_date; /* Manufacturing date (year + month) */
    uint8_t          crc7;
};

enum csd_struct_t
{
    SD_CSD_STRUCT_VER_1_0 = 0,
    SD_CSD_STRUCT_VER_2_0 = 1
};

// Bits definitions for "cmd_classes" csd field
enum csd_card_cmd_class_t
{
    SD_CSD_CARD_CMD_CLASS_0  = (1 << 0),
    SD_CSD_CARD_CMD_CLASS_1  = (1 << 1),
    SD_CSD_CARD_CMD_CLASS_2  = (1 << 2),
    SD_CSD_CARD_CMD_CLASS_3  = (1 << 3),
    SD_CSD_CARD_CMD_CLASS_4  = (1 << 4),
    SD_CSD_CARD_CMD_CLASS_5  = (1 << 5),
    SD_CSD_CARD_CMD_CLASS_6  = (1 << 6),
    SD_CSD_CARD_CMD_CLASS_7  = (1 << 7),
    SD_CSD_CARD_CMD_CLASS_8  = (1 << 8),
    SD_CSD_CARD_CMD_CLASS_9  = (1 << 9),
    SD_CSD_CARD_CMD_CLASS_10 = (1 << 10),
    SD_CSD_CARD_CMD_CLASS_11 = (1 << 11)
};

enum csd_dev_size_mul_t
{
    SD_CSD_DEV_SIZE_MUL_4   = 0,
    SD_CSD_DEV_SIZE_MUL_8   = 1,
    SD_CSD_DEV_SIZE_MUL_16  = 2,
    SD_CSD_DEV_SIZE_MUL_32  = 3,
    SD_CSD_DEV_SIZE_MUL_64  = 4,
    SD_CSD_DEV_SIZE_MUL_128 = 5,
    SD_CSD_DEV_SIZE_MUL_256 = 6,
    SD_CSD_DEV_SIZE_MUL_512 = 7
};

enum csd_max_read_block_len_t
{
    SD_CSD_MAX_READ_BLOCK_LEN_512  = 9,
    SD_CSD_MAX_READ_BLOCK_LEN_1024 = 10,
    SD_CSD_MAX_READ_BLOCK_LEN_2048 = 11
};

enum csd_max_curr_vdd_min_t
{
    SD_CSD_MAX_CURR_VDD_MIN_05_MA  = 0,
    SD_CSD_MAX_CURR_VDD_MIN_1_MA   = 1,
    SD_CSD_MAX_CURR_VDD_MIN_5_MA   = 2,
    SD_CSD_MAX_CURR_VDD_MIN_10_MA  = 3,
    SD_CSD_MAX_CURR_VDD_MIN_25_MA  = 4,
    SD_CSD_MAX_CURR_VDD_MIN_35_MA  = 5,
    SD_CSD_MAX_CURR_VDD_MIN_60_MA  = 6,
    SD_CSD_MAX_CURR_VDD_MIN_100_MA = 7
};

enum csd_max_curr_vdd_max_t
{
    SD_CSD_MAX_CURR_VDD_MAX_1_MA   = 0,
    SD_CSD_MAX_CURR_VDD_MAX_5_MA   = 1,
    SD_CSD_MAX_CURR_VDD_MAX_10_MA  = 2,
    SD_CSD_MAX_CURR_VDD_MAX_25_MA  = 3,
    SD_CSD_MAX_CURR_VDD_MAX_35_MA  = 4,
    SD_CSD_MAX_CURR_VDD_MAX_45_MA  = 5,
    SD_CSD_MAX_CURR_VDD_MAX_80_MA  = 6,
    SD_CSD_MAX_CURR_VDD_MAX_200_MA = 7
};

enum csd_r2w_factor_t
{
    SD_CSD_R2W_FACTOR_1  = 0,
    SD_CSD_R2W_FACTOR_2  = 1, // Write half as fast as read
    SD_CSD_R2W_FACTOR_4  = 2,
    SD_CSD_R2W_FACTOR_8  = 3,
    SD_CSD_R2W_FACTOR_16 = 4,
    SD_CSD_R2W_FACTOR_32 = 5,
};

enum csd_max_write_block_len_t
{
    SD_CSD_MAX_WRITE_BLOCK_LEN_512  = 9,
    SD_CSD_MAX_WRITE_BLOCK_LEN_1024 = 10,
    SD_CSD_MAX_WRITE_BLOCK_LEN_2048 = 11
};

enum csd_file_format_t
{
    SD_CSD_FILE_FORMAT_PARTITION_TABLE  = 0,
    SD_CSD_FILE_FORMAT_BOOT_SECTOR_ONLY = 1,
    SD_CSD_FILE_FORMAT_UNIVERSAL        = 2,
    SD_CSD_FILE_FORMAT_OTHER            = 3
};

enum csd_spec_ver_t
{
    SD_CSD_SPEC_VER_MMC_V10_12 = 0, // MultiMediaCard protocol version 1.0-1.2
    SD_CSD_SPEC_VER_MMC_V14    = 1, // MultiMediaCard protocol version 1.4
    SD_CSD_SPEC_VER_MMC_V20_22 = 2, // MultiMediaCard protocol version 2.0-2.2
};

// Map of CSD v1.0 register (SD card v1.01-1.10 and v2.00 std capacity card)
struct csd_v1_t
{
    csd_struct_t              csd_struct;               // CSD structure (1.0 or 1.1). 2 bits
    uint8_t                   taac;                     // Data read access-time 1. 8 bits
    uint8_t                   nsac;                     // Data read access-time 2 in CLK cycles. 8 bits
    uint8_t                   max_transfer_rate;        // Max bus clock frequency. 8 bits
    uint16_t                  cmd_classes;              // Card command classes. 12 bits
    csd_max_read_block_len_t  max_read_block_len;       // Max read data block length. 4 bits
    bool                      partial_block_read_en;    // Partial blocks for read allowed
    bool                      write_block_misalign;     // Write block misalignment
    bool                      read_block_misalign;      // Read block misalignment
    bool                      dsr_impl;                 // DSR implemented
    uint16_t                  dev_size;                 // Device Size. 12 bits
    csd_max_curr_vdd_min_t    max_read_curr_vdd_min;    // Max read current at the minimal power supply. 3 bits
    csd_max_curr_vdd_max_t    max_read_curr_vdd_max;    // Max read current at the maximal power supply. 3 bits
    csd_max_curr_vdd_min_t    max_write_curr_vdd_min;   // Max write current at the minimal power supply. 3 bits
    csd_max_curr_vdd_max_t    max_write_curr_vdd_max;   // Max read current at the maximal power supply. 3 bits
    csd_dev_size_mul_t        dev_size_mul;             // Device size multiplier. 3 bits
    bool                      erase_single_block_en;    // Erase single block enable
    uint8_t                   erase_sector_size;        // Erase sector size. 7 bits
    uint8_t                   write_protect_group_size; // Write protect group size. 7 bits
    bool                      write_protect_group_en;   // Write protect group enable
    csd_r2w_factor_t          r2w_factor;               // Write speed factor. 3 bits
    csd_max_write_block_len_t max_write_block_len;      // Max write data block length. 4 bits
    bool                      partial_block_write_en;   // Partial blocks for write allowed
    bool                      file_format_grp;          // File format group
    bool                      copy_flag;                // Copy flag (OTP)
    bool                      perm_write_protection;    // Permanent write protection
    bool                      temp_write_protection;    // Temporary write protection
    csd_file_format_t         file_format;              // File format. 2 bits
    uint8_t                   crc7;                     // 7 bits
};

// Map of CSD v2.0 register (SD card v2.00 high and extended capacity)
struct sd_csd_v2_t
{
    csd_struct_t              csd_struct;               // CSD structure (1.0 or 1.1). 2 bits
    uint8_t                   taac;                     // Data read access-time 1. 8 bits
    uint8_t                   nsac;                     // Data read access-time 2 in CLK cycles. 8 bits
    uint8_t                   max_transfer_rate;        // Max bus clock frequency. 8 bits
    uint16_t                  cmd_classes;              // Card command classes. 12 bits
    csd_max_read_block_len_t  max_read_block_len;       // Max read data block length. 4 bits
    bool                      partial_block_read_en;    // Partial blocks for read allowed
    bool                      write_block_misalign;     // Write block misalignment
    bool                      read_block_misalign;      // Read block misalignment
    bool                      dsr_impl;                 // DSR implemented
    uint32_t                  dev_size;                 // Device Size. 22 bits
    bool                      erase_single_block_en;    // Erase single block enable
    uint8_t                   erase_sector_size;        // Erase sector size. 7 bits
    uint8_t                   write_protect_group_size; // Write protect group size. 7 bits
    bool                      write_protect_group_en;   // Write protect group enable
    csd_r2w_factor_t          r2w_factor;               // Write speed factor. 3 bits
    csd_max_write_block_len_t max_write_block_len;      // Max write data block length. 4 bits
    bool                      partial_block_write_en;   // Partial blocks for write allowed
    bool                      file_format_grp;          // File format group
    bool                      copy_flag;                // Copy flag (OTP)
    bool                      perm_write_protection;    // Permanent write protection
    bool                      temp_write_protection;    // Temporary write protection
    csd_file_format_t         file_format;              // File format. 2 bits
    uint8_t                   crc7;                     // 7 bits
};

// Map of CSD MMC register
struct sd_csd_mmc_t
{
    csd_struct_t              csd_struct;               // CSD structure (1.0 or 1.1). 2 bits
    csd_spec_ver_t            spec_ver;                 // MMC specification version (1.0-1.2, 1.4, 2.0-2.2). 4 bits
    uint8_t                   taac;                     // Data read access-time 1. 8 bits
    uint8_t                   nsac;                     // Data read access-time 2 in CLK cycles. 8 bits
    uint8_t                   max_transfer_rate;        // Max bus clock frequency. 8 bits
    uint16_t                  cmd_classes;              // Card command classes. 12 bits
    csd_max_read_block_len_t  max_read_block_len;       // Max read data block length. 4 bits
    bool                      partial_block_read_en;    // Partial blocks for read allowed
    bool                      write_block_misalign;     // Write block misalignment
    bool                      read_block_misalign;      // Read block misalignment
    bool                      dsr_impl;                 // DSR implemented
    uint16_t                  dev_size;                 // Device Size. 12 bits
    csd_max_curr_vdd_min_t    max_read_curr_vdd_min;    // Max read current at the minimal power supply. 3 bits
    csd_max_curr_vdd_max_t    max_read_curr_vdd_max;    // Max read current at the maximal power supply. 3 bits
    csd_max_curr_vdd_min_t    max_write_curr_vdd_min;   // Max write current at the minimal power supply. 3 bits
    csd_max_curr_vdd_max_t    max_write_curr_vdd_max;   // Max read current at the maximal power supply. 3 bits
    csd_dev_size_mul_t        dev_size_mul;             // Device size multiplier. 3 bits
    uint8_t                   erase_sector_size;        // Erase sector size. 5 bits
    uint8_t                   erase_group_size;         // Erase group size. 5 bits
    uint8_t                   write_protect_group_size; // Write protect group size. 5 bits
    bool                      write_protect_group_en;   // Write protect group enable
    uint8_t                   default_eec;              // Default error correction code. 2 bits
    csd_r2w_factor_t          r2w_factor;               // Read to write speed factor. 3 bits
    csd_max_write_block_len_t max_write_block_len;      // Max write data block length. 4 bits
    bool                      partial_block_write_en;   // Partial blocks for write allowed
    bool                      file_format_grp;          // File format group
    bool                      copy_flag;                // Copy flag (OTP)
    bool                      perm_write_protection;    // Permanent write protection
    bool                      temp_write_protection;    // Temporary write protection
    csd_file_format_t         file_format;              // File format. 2 bits
    uint8_t                   eec;                      // Error correction code. 2 bits
    uint8_t                   crc7;                     // 7 bits
};

/* Card specific data register */
union csd_t
{
    csd_v1_t  v1;
    sd_csd_v2_t  v2;
    sd_csd_mmc_t mmc;
};
} // namespace drv::sd
