#pragma once

#include <stdint.h>

namespace hal
{
class wdt
{
	public:
		static void init(uint16_t ms);
		static void on();
		static void reload();
	
	private:
	wdt() {}
};
}
