#include <stddef.h>

#include "common/assert.h"
#include "tim.hpp"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/timer_struct.h"
#include "FreeRTOS.h"

using namespace hal;

static void tim_hw_cb(void *arg);
static void calc_clk(tim::tim_t tim, uint32_t us, uint8_t *div, uint32_t *load);

tim::tim(tim_t tim):
	_tim(tim),
	_us(0),
	_ctx(NULL),
	_cb(NULL)
{
	// ESP8266 has only one hw timer
	ASSERT(tim == TIM_1);
	
	_xt_isr_mask(1 << ETS_FRC_TIMER1_INUM);
	_xt_isr_attach(ETS_FRC_TIMER1_INUM, tim_hw_cb, this);
	TM1_EDGE_INT_ENABLE();
	_xt_isr_unmask(1 << ETS_FRC_TIMER1_INUM);
}

tim::~tim()
{
	frc1.ctrl.en = 0;
	frc1.ctrl.val = 0;
	_xt_isr_mask(1 << ETS_FRC_TIMER1_INUM);
	TM1_EDGE_INT_DISABLE();
	_xt_isr_attach(ETS_FRC_TIMER1_INUM, NULL, NULL);
}

void tim::cb(cb_t cb, void *ctx)
{
	_xt_isr_mask(1 << ETS_FRC_TIMER1_INUM);
	_cb = cb;
	_ctx = ctx;
	_xt_isr_unmask(1 << ETS_FRC_TIMER1_INUM);
}

void tim::us(uint32_t us)
{
	ASSERT(us > 0);
	
	_us = us;
	
	uint8_t div;
	uint32_t load;
	calc_clk(_tim, _us, &div, &load);
	
	_xt_isr_mask(1 << ETS_FRC_TIMER1_INUM);
	frc1.ctrl.val = 0;
	frc1.ctrl.div = div;
	frc1.load.data = load;
	_xt_isr_unmask(1 << ETS_FRC_TIMER1_INUM);
}

void tim::start(bool is_cyclic)
{
	ASSERT(_us > 0);
	ASSERT(_cb);
	/* This action allowed only when TIM is disabled */
	ASSERT(!frc1.ctrl.en);
	
	_xt_isr_mask(1 << ETS_FRC_TIMER1_INUM);
	frc1.ctrl.reload = is_cyclic;
	frc1.ctrl.intr_type = 0; // TIMER_EDGE_INT
	frc1.ctrl.en = 1;
	_xt_isr_unmask(1 << ETS_FRC_TIMER1_INUM);
}

void tim::stop()
{
	_xt_isr_mask(1 << ETS_FRC_TIMER1_INUM);
	frc1.ctrl.en = 0;
	frc1.ctrl.val = 0;
	_xt_isr_unmask(1 << ETS_FRC_TIMER1_INUM);
}

bool tim::is_expired() const
{
	return !frc1.ctrl.en;
}

static void calc_clk(tim::tim_t tim, uint32_t us, uint8_t *div, uint32_t *load)
{
	constexpr uint32_t max_load = 0x7FFFFF;
	constexpr uint8_t div_step = 4;
	enum
	{
		TIMER_CLKDIV_1 = 0,
		TIMER_CLKDIV_16 = div_step,
		TIMER_CLKDIV_256 = div_step * 2
	};
	
	*div = TIMER_CLKDIV_1;
	*load = us * (APB_CLK_FREQ / 1000000);
	
	while(*load > max_load)
	{
		*div += div_step;
		*load >>= div_step;
		ASSERT(*div <= TIMER_CLKDIV_256);
	}
	ASSERT(*load);
}

extern "C" void tim_irq_hndlr(hal::tim *obj)
{
	if(obj->_cb)
		obj->_cb(obj, obj->_ctx);
}

static void tim_hw_cb(void* arg)
{
	if(!frc1.ctrl.reload)
		frc1.ctrl.en = 0;
	
	tim_irq_hndlr((tim *)arg);
}
