#pragma once

#include <stdint.h>

#include "gpio.hpp"

namespace hal { class exti; }
// For internal use only! (called from ISR)
extern "C" void exti_irq_hndlr(hal::exti *obj);

namespace hal
{
class exti
{
	public:
		enum class trigger
		{
			RISING,
			FALLING,
			BOTH
		};
		
		typedef void (*exti_cb_t)(exti *exti, void *ctx);
		
		exti(gpio &gpio, enum trigger trigger);
		~exti();
		
		void cb(exti_cb_t cb, void *ctx);
		
		void on();
		void off();
		
		void trigger(enum trigger trigger);
		enum trigger trigger() const { return _trigger; }
		
		exti &operator = (const exti &);
	
	private:
		gpio &_gpio;
		enum trigger _trigger;
		void *_ctx;
		exti_cb_t _cb;
		friend void ::exti_irq_hndlr(exti *obj);
};
}
