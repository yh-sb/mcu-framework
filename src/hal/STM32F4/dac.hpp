#pragma once

#include <stdint.h>

#include "gpio.hpp"

namespace hal
{
class dac
{
	public:
		enum class dac_t
		{
			DAC_1,
			DAC_2
		};
		
		enum class align_t
		{
			ALIGN_8_R,
			ALIGN_12_R,
			ALIGN_12_L
		};
		
		dac(dac_t dac, align_t align, gpio &gpio);
		~dac();
		
		void set(uint16_t val) const;
		void set(float val) const;
		uint16_t get() const;
	
	private:
		dac_t _dac;
		align_t _align;
		gpio &_gpio;
};
}
