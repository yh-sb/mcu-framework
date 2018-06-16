#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace hal
{
typedef enum
{
	RTC_CLK_LSI,
	RTC_CLK_LSE
} rtc_clk_t;

typedef struct
{
	uint8_t year;
	uint8_t mon;
	uint8_t day;
	uint8_t wday;
	uint8_t h;
	uint8_t m;
	uint8_t s;
} rtc_time_t;

/**
 * @brief      RTC initialization
 *
 * @param[in]  clk   Clock source for RTC
 *
 * @return     0 in case of success, otherwise - negative value
 */
int8_t rtc_init(rtc_clk_t clk);

/**
 * @brief         Get RTC time
 *
 * @param[in,out] time  Pointer to the hal_rtc_time_t struct
 */
void rtc_get_time(rtc_time_t *time);

/**
 * @brief      Set RTC time
 *
 * @param[in]  time  Pointer to the hal_rtc_time_t struct
 *
 * @return     0 in case of success, otherwise - negative value
 */
int8_t rtc_set_time(rtc_time_t *time);

/**
 * @brief      Write the array into the backup registers
 *
 * @param[in]  addr  Offset in backup registers
 * @param[in]  buff  Pointer to the buffer
 * @param[in]  size  Size of the buffer, which should be written
 */
void rtc_bckp_write(uint8_t addr, uint8_t *buff, uint8_t size);

/**
 * @brief         Read array from backup registers
 *
 * @param[in]     addr  Offset in backup registers
 * @param[in,out] buff  Pointer to the buffer
 * @param[in]     size  Size of the buffer, which should be filled
 */
void rtc_bckp_read(uint8_t addr, uint8_t *buff, uint8_t size);
}
