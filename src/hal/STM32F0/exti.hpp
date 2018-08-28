#pragma once

#include <stdint.h>

#include "gpio.hpp"

namespace hal { class exti; }
// For internal use only! (called from ISR)
extern "C" void exti_irq_hndlr(hal::exti *obj);

namespace hal
{
typedef enum
{
	EXTI_TRIGGER_RISING,
	EXTI_TRIGGER_FALLING,
	EXTI_TRIGGER_BOTH
} exti_trigger_t;

typedef void (*exti_cb_t)(exti *exti, void *ctx);

class exti
{
	public:
		exti(gpio &gpio, exti_trigger_t trigger);
		~exti();
		
		void on(exti_cb_t cb, void *ctx);
		void off();
		void trigger(exti_trigger_t trigger);
		exti_trigger_t trigger() const { return _trigger; }
		
		exti &operator = (const exti &);
	
	private:
		gpio &_gpio;
		exti_trigger_t _trigger;
		void *_ctx;
		exti_cb_t _cb;
		friend void ::exti_irq_hndlr(exti *obj);
};
}
