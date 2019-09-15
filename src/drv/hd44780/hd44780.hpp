#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "gpio/gpio.hpp"
#include "tim/tim.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

namespace drv
{
class hd44780
{
	public:
		hd44780(hal::gpio &rs, hal::gpio &rw, hal::gpio &e, hal::gpio &db4,
			hal::gpio &db5, hal::gpio &db6, hal::gpio &db7, hal::tim &tim);
		
		~hd44780();
		
		void init();
		
		/**
		 * @brief Print formatted text
		 * 
		 * @param ddram_addr DDRAM address to which text will be writed.
		 *                   Example for 4 line display: 0 - 1st line,
		 *                   64 - 2nd line, 20 - 3rd line, 84 - 4th line
		 * @param format A string that specifies the format of the output
		 * @param ... Arguments used by format string
		 * @return uint8_t New DDRAM address after writing
		 */
		uint8_t print(uint8_t ddram_addr, const char *format, ...);
		
		/**
		 * @brief Print one byte
		 * 
		 * @param ddram_addr DDRAM address to which text will be writed
		 * @param byte ASCII symbol
		 * @return uint8_t New DDRAM address after writing
		 */
		uint8_t print(uint8_t ddram_addr, char byte);
		
		uint8_t ddram_addr();
		
		void write_cgram(uint8_t buff[8][8]);
		void read_cgram(uint8_t buff[8][8]);
		
		void clear();
		
	private:
		hal::gpio &_rs;
		hal::gpio &_rw;
		hal::gpio &_e;
		hal::gpio *_db[4];
		hal::tim &_tim;
		TaskHandle_t task;
		SemaphoreHandle_t api_lock;
		
		enum write_t
		{
			CMD,
			DATA
		};
		
		void write_4bit(uint8_t half_byte);
		void write(write_t type, uint8_t byte);
		
		uint8_t read_4bit();
		uint8_t read_bf_and_ddram_addr();
		
		static void tim_cb(hal::tim *tim, void *ctx);
		void delay(uint32_t us);
};
}
