#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gpio.hpp"
#include "tim.hpp"

namespace drv
{
class hd44780
{
	public:
		hd44780(hal::gpio &e, hal::gpio &rw, hal::gpio &rs, hal::gpio &db4,
			hal::gpio &db5, hal::gpio &db6, hal::gpio &db7, hal::tim &tim);
		
		~hd44780();
		
		void init();
		
		void print(const char *str);
		void print(char byte);
		
		void x(uint8_t x);
		void y(uint8_t y);
		
		uint8_t x() const { return _x; }
		uint8_t y() const { return _y; }
		
	private:
		hal::gpio &_e;
		hal::gpio &_rw;
		hal::gpio &_rs;
		hal::gpio *_db[4];
		hal::tim &_tim;
		
		uint8_t _x;
		uint8_t _y;
		
		/*
		typedef enum
		{
			CMD_CLEAR_DISPLAY                  = 1 << 0,
			CMD_RETRNU_HOME                    = 1 << 1,
			CMD_SET_CURSOR_SHIFT               = (1 << 2) | (1 << 1),
			CMD_ON_OFF_DISPLAY_SHIFT           = (1 << 2) | (1 << 0),
			CMD_ON_OFF_DISPLAY                 = (1 << 3) | (1 << 2),
			CMD_ON_OFF_CURSOR                  = (1 << 3) | (1 << 1),
			CMD_ON_OFF_CURSOR_BLINK            = (1 << 3) | (1 << 0),
			CMD_ON_OFF_CURSOR_OR_DISPLAY_SHIFT = 
			CMD_ON_OFF_CURSOR_OR_DISPLAY_SHIFT = 
		} cmd_t;
		*/
		
		enum write_t
		{
			CMD,
			DATA
		};
		
		void write_4bit(uint8_t half_byte);
		void write(write_t type, uint8_t byte);
		void delay(uint32_t us);
};
}
