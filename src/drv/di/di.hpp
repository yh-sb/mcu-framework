#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gpio/gpio.hpp"

namespace drv
{
class di
{
	typedef void (*di_cb_t)(di *di, bool state, void *ctx);
	
	public:
		di(hal::gpio &gpio, uint16_t threshold, bool default_state);
		~di();
		
		void cb(di_cb_t cb, void *ctx) { _cb = cb; _ctx = ctx; };
		void poll();
		uint16_t threshold() const { return _threshold; }
		void threshold(uint16_t threshold) { _threshold = threshold; }
		
	private:
		void jitter_filter();
		
		hal::gpio &_gpio;
		uint16_t _threshold;
		bool _state;
		uint16_t _cnt;
		void *_ctx;
		di_cb_t _cb;
};
}
