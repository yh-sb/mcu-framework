#include <stddef.h>

#include "common/assert.h"

#include "tim.hpp"
#include "rcc.hpp"

#include "CMSIS/device-support/include/stm32f1xx.h"
#include "CMSIS/core-support/core_cm3.h"

using namespace hal;

#define IRQ_PRIORITY 1
#define MAX_16BIT 0xFFFF

static TIM_TypeDef *const tim_list[tim::TIM_END] =
{
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F103x6) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM1,
#else
	NULL,
#endif
	TIM2,
	TIM3,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM4,
#else
	NULL,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	TIM5,
#else
	NULL,
#endif
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xE) || \
	defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM6, TIM7,
#else
	NULL, NULL,
#endif
#if defined(STM32F103xE) || defined(STM32F103xG)
	TIM8,
#else
	NULL,
#endif
#if defined(STM32F101xG) || defined(STM32F103xG)
	TIM9, TIM10, TIM11,
#else
	NULL, NULL, NULL,
#endif
#if defined(STM32F100xE) || defined(STM32F101xG) || defined(STM32F103xG)
	TIM12, TIM13, TIM14,
#else
	NULL, NULL, NULL,
#endif
#if defined(STM32F100xB) || defined(STM32F100xE)
	TIM15, TIM16, TIM17
#else
	NULL, NULL, NULL
#endif
};

static uint32_t const rcc_list[tim::TIM_END] =
{
	RCC_APB2ENR_TIM1EN,
	RCC_APB1ENR_TIM2EN,
	RCC_APB1ENR_TIM3EN,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	RCC_APB1ENR_TIM4EN,
#else
	0,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	RCC_APB1ENR_TIM5EN,
#else
	0,
#endif
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xE) || \
	defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	RCC_APB1ENR_TIM6EN, RCC_APB1ENR_TIM7EN,
#else
	0, 0,
#endif
#if defined(STM32F103xE) || defined(STM32F103xG)
	RCC_APB2ENR_TIM8EN,
#else
	0,
#endif
#if defined(STM32F101xG) || defined(STM32F103xG)
	RCC_APB2ENR_TIM9EN, RCC_APB2ENR_TIM10EN, RCC_APB2ENR_TIM11EN,
#else
	0, 0, 0,
#endif
#if defined(STM32F100xE) || defined(STM32F101xG) || defined(STM32F103xG)
	RCC_APB1ENR_TIM12EN, RCC_APB1ENR_TIM13EN, RCC_APB1ENR_TIM14EN,
#else
	0, 0, 0,
#endif
#if defined(STM32F100xB) || defined(STM32F100xE)
	RCC_APB2ENR_TIM15EN, RCC_APB2ENR_TIM16EN, RCC_APB2ENR_TIM17EN
#else
	0, 0, 0
#endif
};

static volatile uint32_t *const rcc_bus_list[tim::TIM_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB2ENR, &RCC->APB2ENR,
	&RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR,
	&RCC->APB2ENR, &RCC->APB2ENR
};

static rcc_src_t const rcc_src_list[tim::TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB2, RCC_SRC_APB2,
	RCC_SRC_APB2, RCC_SRC_APB2, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB2,
	RCC_SRC_APB2, RCC_SRC_APB2
};

static IRQn_Type const irq_list[tim::TIM_END] =
{
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F103x6) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM1_CC_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	TIM2_IRQn, TIM3_IRQn,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM4_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	TIM5_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xE) || \
	defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM6_IRQn, TIM7_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F103xE) || defined(STM32F103xG)
	TIM8_CC_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F101xG) || defined(STM32F103xG)
	TIM9_IRQn, TIM10_IRQn, TIM11_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xG) || defined(STM32F103xG)
	TIM12_IRQn, TIM13_IRQn, TIM14_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xG) || \
	defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || \
	defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
	TIM1_BRK_TIM15_IRQn, TIM1_UP_TIM16_IRQn, TIM1_TRG_COM_TIM17_IRQn
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0)
#endif
};

static tim *obj_list[tim::TIM_END];

static void calc_clk(tim::tim_t tim, uint32_t us, uint16_t *psc, uint16_t *arr);

tim::tim(tim_t tim):
	_tim(tim),
	_us(0),
	_ctx(NULL),
	_cb(NULL)
{
	ASSERT(tim < TIM_END && tim_list[_tim]);
	
	*rcc_bus_list[_tim] |= rcc_list[_tim];
	
	obj_list[_tim] = this;
	
	/* Allow that only counter overflow/underflow generates irq
	   (avoid irq generation when set UG bit)
	*/
	tim_list[_tim]->CR1 |= TIM_CR1_URS;
	
	// Enable interrupt
	tim_list[_tim]->DIER |= TIM_DIER_UIE;
	
	NVIC_SetPriority(irq_list[_tim], IRQ_PRIORITY);
	NVIC_EnableIRQ(irq_list[_tim]);
}

tim::~tim()
{
	
}

void tim::cb(cb_t cb, void *ctx)
{
	_cb = cb;
	_ctx = ctx;
}

void tim::us(uint32_t us)
{
	ASSERT(us > 0);
	
	_us = us;
	uint16_t psc, arr;
	calc_clk(_tim, _us, &psc, &arr);
	
	tim_list[_tim]->PSC = psc;
	tim_list[_tim]->ARR = arr;
	
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

static void calc_clk(tim::tim_t tim, uint32_t us, uint16_t *psc, uint16_t *arr)
{
	uint32_t clk_freq = rcc_get_freq(rcc_src_list[tim]);
	/* If APBx prescaller no equal to 1, TIMx prescaller multiplies by 2 */
	if(clk_freq != rcc_get_freq(RCC_SRC_AHB))
		clk_freq *= 2;
	
	uint32_t tmp_psc = 1;
	uint32_t tmp_arr = us * (clk_freq / 1000000);
	
	if(tmp_arr > MAX_16BIT)
	{
		// tmp_arr is too big for ARR register (16 bit), increase the prescaler
		tmp_psc = ((tmp_arr + (MAX_16BIT / 2)) / MAX_16BIT) + 1;
		tmp_arr /= tmp_psc;
	}
	
	ASSERT(tmp_psc <= MAX_16BIT);
	ASSERT(tmp_arr <= MAX_16BIT);
	
	*psc = (uint16_t)(tmp_psc - 1);
	*arr = (uint16_t)(tmp_arr - 1);
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
		obj->_cb(obj, obj->_ctx);
}

#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F103x6) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void TIM1_CC_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_1]);
}
#endif

extern "C" void TIM2_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_2]);
}

extern "C" void TIM3_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_3]);
}

#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void TIM4_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_4]);
}
#endif

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
extern "C" void TIM5_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_5]);
}
#endif

#if defined(STM32F100xB) || defined(STM32F100xE)
extern "C" void TIM6_DAC_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_6]);
}

extern "C" void TIM7_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_7]);
}
#elif defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || \
	defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void TIM6_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_6]);
}

extern "C" void TIM7_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_7]);
}
#endif

#if defined(STM32F103xE) || defined(STM32F103xG)
extern "C" void TIM8_CC_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_8]);
}
#endif

#if defined(STM32F101xG)
extern "C" void TIM9_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_9]);
}

extern "C" void TIM10_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_10]);
}

extern "C" void TIM11_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_11]);
}
#elif defined(STM32F103xG)
extern "C" void TIM1_BRK_TIM9_IRQHandler(void)
{
	if((TIM1->DIER & TIM_DIER_BIE) && (TIM1->SR & TIM_SR_BIF))
		tim_irq_hndlr(obj_list[tim::TIM_1]);
	else
		tim_irq_hndlr(obj_list[tim::TIM_9]);
}

extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	if((TIM1->DIER & TIM_DIER_UIE) && (TIM1->SR & TIM_SR_UIF))
		tim_irq_hndlr(obj_list[tim::TIM_1]);
	else
		tim_irq_hndlr(obj_list[tim::TIM_10]);
}

extern "C" void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	if(((TIM1->DIER & TIM_DIER_COMIE) && (TIM1->SR & TIM_SR_COMIF)) ||
		((TIM1->DIER & TIM_DIER_TIE) && (TIM1->SR & TIM_SR_TIF)))
	{
		tim_irq_hndlr(obj_list[tim::TIM_1]);
	}
	else
		tim_irq_hndlr(obj_list[tim::TIM_11]);
}
#endif

#if defined(STM32F100xE) || defined(STM32F101xG)
extern "C" void TIM12_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_12]);
}

extern "C" void TIM13_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_13]);
}

extern "C" void TIM14_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[tim::TIM_14]);
}
#elif defined(STM32F103xG)
extern "C" void TIM8_BRK_TIM12_IRQHandler(void)
{
	if((TIM8->DIER & TIM_DIER_BIE) && (TIM8->SR & TIM_SR_BIF))
		tim_irq_hndlr(obj_list[tim::TIM_8]);
	else
		tim_irq_hndlr(obj_list[tim::TIM_12]);
}

extern "C" void TIM8_UP_TIM13_IRQHandler(void)
{
	if((TIM8->DIER & TIM_DIER_UIE) && (TIM8->SR & TIM_SR_UIF))
		tim_irq_hndlr(obj_list[tim::TIM_8]);
	else
		tim_irq_hndlr(obj_list[tim::TIM_13]);
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	if(((TIM8->DIER & TIM_DIER_COMIE) && (TIM8->SR & TIM_SR_COMIF)) ||
		((TIM8->DIER & TIM_DIER_TIE) && (TIM8->SR & TIM_SR_TIF)))
	{
		tim_irq_hndlr(obj_list[tim::TIM_8]);
	}
	else
		tim_irq_hndlr(obj_list[tim::TIM_14]);
}
#endif

#if defined(STM32F100xB) || defined(STM32F100xE)
extern "C" void TIM1_BRK_TIM15_IRQHandler(void)
{
	if((TIM1->DIER & TIM_DIER_BIE) && (TIM1->SR & TIM_SR_BIF))
		tim_irq_hndlr(obj_list[tim::TIM_1]);
	else
		tim_irq_hndlr(obj_list[tim::TIM_15]);
}

extern "C" void TIM1_UP_TIM16_IRQHandler(void)
{
	if((TIM1->DIER & TIM_DIER_UIE) && (TIM1->SR & TIM_SR_UIF))
		tim_irq_hndlr(obj_list[tim::TIM_1]);
	else
		tim_irq_hndlr(obj_list[tim::TIM_16]);
}

extern "C" void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
	if(((TIM1->DIER & TIM_DIER_COMIE) && (TIM1->SR & TIM_SR_COMIF)) ||
		((TIM1->DIER & TIM_DIER_TIE) && (TIM1->SR & TIM_SR_TIF)))
	{
		tim_irq_hndlr(obj_list[tim::TIM_1]);
	}
	else
		tim_irq_hndlr(obj_list[tim::TIM_17]);
}
#endif
