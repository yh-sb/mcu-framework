#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gpio.hpp"

namespace drv
{
class enc
{
	typedef void (*enc_cb_t)(enc *enc, int8_t diff, void *ctx);
	
	public:
		enc(hal::gpio &a, hal::gpio &b);
		~enc();
		
		void cb(enc_cb_t cb, void *ctx) { _cb = cb; _ctx = ctx; };
		void poll();
		
	private:
		hal::gpio &_a;
		hal::gpio &_b;
		uint8_t prev_state;
		uint8_t prev_prev_state;
		void *_ctx;
		enc_cb_t _cb;
};
}
