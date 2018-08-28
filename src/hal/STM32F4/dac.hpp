#pragma once

#include <stdint.h>

#include "gpio.hpp"

namespace hal
{
typedef enum
{
	DAC_1,
	DAC_2,
	DAC_END
} dac_t;

typedef enum
{
	DAC_ALIGN_8_R,
	DAC_ALIGN_12_R,
	DAC_ALIGN_12_L
} dac_align_t;

class dac
{
	public:
		dac(dac_t dac, dac_align_t align, gpio &gpio);
		~dac();
		
		void set(uint16_t val) const;
		void set(float val) const;
		uint16_t get() const;
	
	private:
		dac_t _dac;
		dac_align_t _align;
		gpio &_gpio;
};
}
