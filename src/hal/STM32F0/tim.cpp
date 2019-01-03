#include <stddef.h>

#include "common/assert.h"

#include "hal/STM32F0/tim.hpp"
#include "hal/STM32F0/rcc.hpp"

#include "hal/STM32F0/CMSIS/device-support/include/stm32f0xx.h"

using namespace hal;

#define IRQ_PRIORITY 1
#define MAX_RESOL 0xFFFF

static TIM_TypeDef *const tim_list[TIM_END] =
{
	TIM1,
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	TIM2,
#else
	NULL,
#endif
	TIM3, NULL, NULL,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM6,
#else
	NULL,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM7,
#else
	NULL,
#endif
	NULL, NULL, NULL, NULL, NULL, NULL, TIM14,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM15,
#else
	NULL,
#endif
	TIM16,
	TIM17
};

static uint32_t const rcc_list[TIM_END] =
{
	RCC_APB2ENR_TIM1EN,
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	RCC_APB1ENR_TIM2EN,
#else
	0,
#endif
	RCC_APB1ENR_TIM3EN,
	0,
	0,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB1ENR_TIM6EN,
#else
	0,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB1ENR_TIM7EN,
#else
	0,
#endif
	0, 0, 0, 0, 0, 0, RCC_APB1ENR_TIM14EN,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB2ENR_TIM15EN,
#else
	0,
#endif
	RCC_APB2ENR_TIM16EN, RCC_APB2ENR_TIM17EN
};

static volatile uint32_t *const rcc_bus_list[TIM_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, NULL, NULL, &RCC->APB1ENR,
	&RCC->APB1ENR, NULL, NULL, NULL, NULL, NULL, NULL, &RCC->APB1ENR,
	&RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR
};

static rcc_src_t const rcc_src_list[TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1,
	static_cast<rcc_src_t>(0), static_cast<rcc_src_t>(0),
	RCC_SRC_APB1, RCC_SRC_APB1,
	static_cast<rcc_src_t>(0), static_cast<rcc_src_t>(0),
	static_cast<rcc_src_t>(0), static_cast<rcc_src_t>(0),
	static_cast<rcc_src_t>(0), static_cast<rcc_src_t>(0),
	RCC_SRC_APB1, RCC_SRC_APB2, RCC_SRC_APB2, RCC_SRC_APB2
};

static IRQn_Type const irq_list[TIM_END] =
{
	TIM1_CC_IRQn,
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	TIM2_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	TIM3_IRQn, static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM6_DAC_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM7_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0), TIM14_IRQn,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM15_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	TIM16_IRQn,
	TIM17_IRQn
};

static tim *obj_list[TIM_END];

static void calc_clk(tim_t tim, uint32_t us, uint16_t *presc,
	uint16_t *reload);

tim::tim(tim_t tim):
	_tim(tim),
	_us(0),
	_ctx(NULL),
	_cb(NULL)
{
	ASSERT(tim < TIM_END && tim_list[_tim]);
	
	*rcc_bus_list[_tim] |= rcc_list[_tim];
	
	obj_list[_tim] = this;
	
	/* Enable interrupt */
	tim_list[_tim]->DIER |= TIM_DIER_UIE;
	
	NVIC_SetPriority(irq_list[_tim], IRQ_PRIORITY);
	NVIC_EnableIRQ(irq_list[_tim]);
}

tim::~tim()
{
	
}

void tim::cb(tim_cb_t cb, void *ctx)
{
	_cb = cb;
	_ctx = ctx;
}

void tim::us(uint32_t us)
{
	ASSERT(us > 0);
	
	_us = us;
	uint16_t presc = 0;
	uint16_t reload = 0;
	calc_clk(_tim, _us, &presc, &reload);
	
	tim_list[_tim]->PSC = presc;
	tim_list[_tim]->ARR = reload;
	
	// Update ARR, PSC and clear CNT register
	tim_list[_tim]->EGR = TIM_EGR_UG;
}

void tim::start(bool is_cyclic)
{
	ASSERT(_us > 0);
	ASSERT(_cb);
	/* This action allowed only when TIM is disabled */
	ASSERT(!(tim_list[_tim]->CR1 & TIM_CR1_CEN));
	
	if(is_cyclic)
		tim_list[_tim]->CR1 &= ~TIM_CR1_OPM;
	else
		tim_list[_tim]->CR1 |= TIM_CR1_OPM;
	
	tim_list[_tim]->CNT = 0;
	tim_list[_tim]->CR1 |= TIM_CR1_CEN;
}

void tim::stop()
{
	tim_list[_tim]->CR1 &= ~TIM_CR1_CEN;
	tim_list[_tim]->CNT = 0;
	tim_list[_tim]->SR &= ~TIM_SR_UIF;
}

bool tim::is_expired() const
{
	return !static_cast<bool>(tim_list[_tim]->CR1 & TIM_CR1_CEN);
}

static void calc_clk(tim_t tim, uint32_t us, uint16_t *presc,
	uint16_t *reload)
{
	uint32_t clk_freq = rcc_get_freq(rcc_src_list[tim]);
	/* If APBx prescaller no equal to 1, TIMx prescaller multiplies by 2 */
	if(clk_freq != rcc_get_freq(RCC_SRC_AHB))
		clk_freq *= 2;
	
	uint32_t tmp_presc = 0;
	uint32_t tmp_reload = us * (clk_freq / 1000000);
	if(tmp_reload <= MAX_RESOL)
		tmp_presc = 1;
	else
	{
		tmp_presc = ((tmp_reload + (MAX_RESOL / 2)) / MAX_RESOL) + 1;
		tmp_reload /= tmp_presc;
	}
	
	ASSERT(tmp_presc <= MAX_RESOL);
	ASSERT(tmp_reload <= MAX_RESOL);
	
	*presc = (uint16_t)(tmp_presc - 1);
	*reload = (uint16_t)(tmp_reload - 1);
}

extern "C" void tim_irq_hndlr(hal::tim *obj)
{
	TIM_TypeDef *tim_reg = tim_list[obj->_tim];
	
	if((tim_reg->DIER & TIM_DIER_UIE) && (tim_reg->SR & TIM_SR_UIF))
		tim_reg->SR &= ~TIM_SR_UIF;
	else if((tim_reg->DIER & TIM_DIER_CC1IE) && (tim_reg->SR & TIM_SR_CC1IF))
		tim_reg->SR &= ~TIM_SR_CC1IF;
	else if((tim_reg->DIER & TIM_DIER_CC2IE) && (tim_reg->SR & TIM_SR_CC2IF))
		tim_reg->SR &= ~TIM_SR_CC2IF;
	else if((tim_reg->DIER & TIM_DIER_CC3IE) && (tim_reg->SR & TIM_SR_CC3IF))
		tim_reg->SR &= ~TIM_SR_CC3IF;
	else if((tim_reg->DIER & TIM_DIER_CC4IE) && (tim_reg->SR & TIM_SR_CC4IF))
		tim_reg->SR &= ~TIM_SR_CC4IF;
	else
		return;
	
	if(obj->_cb)
	{
		obj->_cb(obj, obj->_ctx);
		
		if(tim_reg->CR1 & TIM_CR1_OPM)
			obj->_cb = NULL;
	}
}

extern "C" void TIM1_CC_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_1]);
}

#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
extern "C" void TIM2_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_2]);
}
#endif

extern "C" void TIM3_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_3]);
}

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F070xB)
extern "C" void TIM6_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_6]);
}
#elif defined(STM32F051x8) || defined(STM32F058xx) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
extern "C" void TIM6_DAC_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_6]);
}
#endif

#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
void TIM7_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_7]);
}
#endif

extern "C" void TIM14_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_14]);
}

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
extern "C" void TIM15_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_15]);
}
#endif

extern "C" void TIM16_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_16]);
}

extern "C" void TIM17_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_17]);
}
