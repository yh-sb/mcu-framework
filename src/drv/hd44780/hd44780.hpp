#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gpio/gpio.hpp"
#include "tim/tim.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
class hd44780
{
	public:
		hd44780(hal::gpio &rs, hal::gpio &rw, hal::gpio &e, hal::gpio &db4,
			hal::gpio &db5, hal::gpio &db6, hal::gpio &db7, hal::tim &tim);
		
		~hd44780();
		
		void init();
		
		void print(const char *str);
		void print(char byte);
		
		void ddram_addr(uint8_t addr);
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
		
		SemaphoreHandle_t _lock;
		
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
