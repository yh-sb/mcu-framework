#pragma once

#include <stdint.h>

#include "gpio/gpio.hpp"
#include "tim/tim.hpp"
#include "exti/exti.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace drv
{
class singlewire
{
	public:
		enum res_t
		{
			OK      =  0,
			NODEV   = -1,
			DEVERR  = -2,
			READERR = -3,
			BUSY    = -4
		};
		
		singlewire(hal::gpio &gpio, hal::tim &tim, hal::exti &exti);
		~singlewire();
		
		int8_t read(uint8_t *buff, uint16_t size);
	
	private:
		hal::gpio &_gpio;
		hal::tim &_tim;
		hal::exti &_exti;
		TaskHandle_t task;
		int8_t res;
		
		struct
		{
			uint8_t state;
			uint8_t *buff;
			uint16_t size;
			uint16_t byte;
			uint8_t bit;
		} fsm;
		void fsm_start(uint8_t *buff, uint16_t size);
		void fsm_run(bool is_tim_expired);
		
		static void tim_cb(hal::tim *tim, void *ctx);
		static void exti_cb(hal::exti *exti, void *ctx);
};
}
