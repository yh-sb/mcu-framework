#pragma once

#include <stdint.h>

#include "gpio.hpp"

namespace hal
{
class dac
{
	public:
		enum class periph
		{
			DAC_1,
			DAC_2
		};
		
		enum class align
		{
			R_8,
			R_12,
			L_12
		};
		
		dac(periph periph, align align, gpio &gpio);
		~dac();
		
		void set(uint16_t val) const;
		void set(float val) const;
		uint16_t get() const;
	
	private:
		periph _periph;
		align _align;
		gpio &_gpio;
};
}
