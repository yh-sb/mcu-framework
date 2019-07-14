#pragma once

#include <stdint.h>
#include <stddef.h>
#include <time.h>

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
		
		/**
		 * @brief      RTC initialization
		 *
		 * @param[in]  clk   Clock source for RTC
		 *
		 * @return     0 in case of success, otherwise - negative value
		 */
		static int8_t init(clk_t clk);
		
		/**
		 * @brief            Get RTC time
		 * 
		 * @return struct tm struct with current time
		 */
		static struct tm get();
		
		/**
		 * @brief         Set RTC time
		 * 
		 * @param time    struct with time
		 * @return int8_t 0 in case of success, otherwise - negative value
		 */
		static int8_t set(struct tm time);
		
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
		static bool is_valid(struct tm &time);
		rtc() {}
};
};
