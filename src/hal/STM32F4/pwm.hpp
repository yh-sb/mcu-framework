#pragma once

#include <stdint.h>

#include "hal/STM32F4/tim.hpp"
#include "hal/STM32F4/gpio.hpp"

namespace hal
{
typedef enum
{
	PWM_CH_1,
	PWM_CH_2,
	PWM_CH_3,
	PWM_CH_4,
	PWM_CH_END
} pwm_ch_t;

typedef enum
{
	PWM_MODE_INVERTED,
	PWM_MODE_NONINVERTED
} pwm_mode_t;

class pwm
{
	public:
		pwm(tim_t tim, pwm_ch_t ch, pwm_mode_t mode, gpio &gpio);
		~pwm();
		
		void freq(uint32_t freq);
		uint32_t freq() const { return _freq; }
		void duty(uint8_t duty);
		uint8_t duty() const { return _duty; }
		
		void start() const;
		void stop() const;
	
	private:
		tim_t _tim;
		pwm_ch_t _ch;
		uint32_t _freq;
		uint8_t _duty;
		pwm_mode_t _mode;
		gpio &_gpio;
};
}
