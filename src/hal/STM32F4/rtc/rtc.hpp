#pragma once

#include <stdint.h>
#include <stddef.h>

namespace hal
{
class rtc
{
	public:
		enum clk_t
		{
			CLK_LSI,
			CLK_LSE
		};
		
		struct time_t
		{
			uint8_t year;
			uint8_t month;
			uint8_t day;
			uint8_t wday;
			uint8_t h;
			uint8_t m;
			uint8_t s;
		};
		
		/**
		 * @brief      RTC initialization
		 *
		 * @param[in]  clk   Clock source for RTC
		 *
		 * @return     0 in case of success, otherwise - negative value
		 */
		static int8_t init(clk_t clk);
		
		/**
		 * @brief         Get RTC time
		 *
		 * @param[in,out] time  Pointer to the time_t struct
		 */
		static void get(time_t *time);
		
		/**
		 * @brief      Set RTC time
		 *
		 * @param[in]  time  Pointer to the time_t struct
		 *
		 * @return     0 in case of success, otherwise - negative value
		 */
		static int8_t set(time_t *time);
		
		/**
		 * @brief      Write the array into the backup registers
		 *
		 * @param[in]  addr  Offset in backup registers
		 * @param[in]  buff  Pointer to the buffer
		 * @param[in]  size  Size of the buffer, which should be written
		 */
		static void bckp_write(uint8_t addr, void *buff, size_t size);
		
		/**
		 * @brief         Read array from backup registers
		 *
		 * @param[in]     addr  Offset in backup registers
		 * @param[in,out] buff  Pointer to the buffer
		 * @param[in]     size  Size of the buffer, which should be filled
		 */
		static void bckp_read(uint8_t addr, void *buff, size_t size);
	
	private:
		rtc() {}
};
};
