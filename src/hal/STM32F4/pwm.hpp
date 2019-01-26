#pragma once

#include <stdint.h>

#include "tim.hpp"
#include "gpio.hpp"

namespace hal
{
class pwm
{
	public:
		enum ch_t
		{
			CH_1,
			CH_2,
			CH_3,
			CH_4,
			CH_END
		};

		enum mode_t
		{
			MODE_INVERTED,
			MODE_NONINVERTED
		};
		
		pwm(tim::tim_t tim, ch_t ch, mode_t mode, gpio &gpio);
		~pwm();
		
		void freq(uint32_t freq);
		uint32_t freq() const { return _freq; }
		void duty(uint8_t duty);
		uint8_t duty() const { return _duty; }
		
		void start() const;
		void stop() const;
	
	private:
		tim::tim_t _tim;
		ch_t _ch;
		uint32_t _freq;
		uint8_t _duty;
		mode_t _mode;
		gpio &_gpio;
};
}
