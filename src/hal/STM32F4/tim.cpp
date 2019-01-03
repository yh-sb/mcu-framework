#include <stddef.h>

#include "common/assert.h"

#include "hal/STM32F4/tim.hpp"
#include "hal/STM32F4/rcc.hpp"

#include "hal/STM32F4/CMSIS/device-support/include/stm32f4xx.h"
#include "hal/STM32F4/CMSIS/core-support/core_cm4.h"

using namespace hal;

#define IRQ_PRIORITY 1
#define MAX_16BIT 0xFFFF

static TIM_TypeDef *const tim_list[TIM_END] =
{
	TIM1,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM2, TIM3, TIM4,
#else
	NULL, NULL, NULL,
#endif
	TIM5,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F410Cx) || \
	defined(STM32F410Rx) || defined(STM32F410Tx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM6,
#else
	NULL,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM7, TIM8,
#else
	NULL, NULL,
#endif
	TIM9,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM10,
#else
	NULL,
#endif
	TIM11,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM12, TIM13, TIM14
#else
	NULL, NULL, NULL
#endif
};

static uint32_t const reset_list[TIM_END] =
{
	RCC_APB2RSTR_TIM1RST,  RCC_APB1RSTR_TIM2RST,  RCC_APB1RSTR_TIM3RST,
	RCC_APB1RSTR_TIM4RST,  RCC_APB1RSTR_TIM5RST,  RCC_APB1RSTR_TIM6RST,
	RCC_APB1RSTR_TIM7RST,  RCC_APB2RSTR_TIM9RST,  RCC_APB2RSTR_TIM9RST,
	RCC_APB2RSTR_TIM10RST, RCC_APB2RSTR_TIM11RST, RCC_APB1RSTR_TIM12RST,
	RCC_APB1RSTR_TIM13RST, RCC_APB1RSTR_TIM14RST
};

static uint32_t const rcc_list[TIM_END] =
{
	RCC_APB2ENR_TIM1EN,  RCC_APB1ENR_TIM2EN,  RCC_APB1ENR_TIM3EN,
	RCC_APB1ENR_TIM4EN,  RCC_APB1ENR_TIM5EN,  RCC_APB1ENR_TIM6EN,
	RCC_APB1ENR_TIM7EN,  RCC_APB2ENR_TIM8EN,  RCC_APB2ENR_TIM9EN,
	RCC_APB2ENR_TIM10EN, RCC_APB2ENR_TIM11EN, RCC_APB1ENR_TIM12EN,
	RCC_APB1ENR_TIM13EN, RCC_APB1ENR_TIM14EN
};

static volatile uint32_t *reset_addr_list[TIM_END] =
{
	&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR,
	&RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR,
	&RCC->APB1RSTR, &RCC->APB2RSTR, &RCC->APB2RSTR,
	&RCC->APB2RSTR, &RCC->APB2RSTR, &RCC->APB1RSTR,
	&RCC->APB1RSTR, &RCC->APB1RSTR
};

static volatile uint32_t *const rcc_bus_list[TIM_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB2ENR, &RCC->APB2ENR,
	&RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB1ENR
};

static rcc_src_t const rcc_src_list[TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB2, RCC_SRC_APB2,
	RCC_SRC_APB2, RCC_SRC_APB2, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1
};

static IRQn_Type const irq_list[TIM_END] =
{
	TIM1_CC_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM2_IRQn, TIM3_IRQn, TIM4_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0),
#endif
	TIM5_IRQn,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F410Cx) || \
	defined(STM32F410Rx) || defined(STM32F410Tx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM6_DAC_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM7_IRQn, TIM8_CC_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
	TIM1_BRK_TIM9_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM1_UP_TIM10_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	TIM1_TRG_COM_TIM11_IRQn,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM8_BRK_TIM12_IRQn, TIM8_UP_TIM13_IRQn, TIM8_TRG_COM_TIM14_IRQn
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0)
#endif
};

static tim *obj_list[TIM_END];

static void calc_clk(tim_t tim, uint32_t us, uint16_t *psc, uint16_t *arr);

tim::tim(tim_t tim):
	_tim(tim),
	_us(0),
	_ctx(NULL),
	_cb(NULL)
{
	ASSERT(tim < TIM_END && tim_list[_tim]);
	
	*rcc_bus_list[_tim] |= rcc_list[_tim];
	
	*reset_addr_list[_tim] |= reset_list[_tim];
	*reset_addr_list[_tim] &= ~reset_list[_tim];
	
	obj_list[_tim] = this;
	
	// Enable interrupt
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

static void calc_clk(tim_t tim, uint32_t us, uint16_t *psc, uint16_t *arr)
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

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
extern "C" void TIM2_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_2]);
}

extern "C" void TIM3_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_3]);
}

extern "C" void TIM4_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_4]);
}
#endif

extern "C" void TIM5_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_5]);
}

#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F410Cx) || \
	defined(STM32F410Rx) || defined(STM32F410Tx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
extern "C" void TIM6_DAC_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_6]);
}
#endif

#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
extern "C" void TIM7_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_7]);
}

extern "C" void TIM8_CC_IRQHandler(void)
{
	tim_irq_hndlr(obj_list[TIM_8]);
}
#endif

extern "C" void TIM1_BRK_TIM9_IRQHandler(void)
{
	if((TIM1->DIER & TIM_DIER_BIE) && (TIM1->SR & TIM_SR_BIF))
		tim_irq_hndlr(obj_list[TIM_1]);
	else
		tim_irq_hndlr(obj_list[TIM_9]);
}

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	if((TIM1->DIER & TIM_DIER_UIE) && (TIM1->SR & TIM_SR_UIF))
		tim_irq_hndlr(obj_list[TIM_1]);
	else
		tim_irq_hndlr(obj_list[TIM_10]);
}
#endif

extern "C" void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	if(((TIM1->DIER & TIM_DIER_COMIE) && (TIM1->SR & TIM_SR_COMIF)) ||
		((TIM1->DIER & TIM_DIER_TIE) && (TIM1->SR & TIM_SR_TIF)))
	{
		tim_irq_hndlr(obj_list[TIM_1]);
	}
	else
		tim_irq_hndlr(obj_list[TIM_11]);
}

#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
extern "C" void TIM8_BRK_TIM12_IRQHandler(void)
{
	if((TIM8->DIER & TIM_DIER_BIE) && (TIM8->SR & TIM_SR_BIF))
		tim_irq_hndlr(obj_list[TIM_8]);
	else
		tim_irq_hndlr(obj_list[TIM_12]);
}

extern "C" void TIM8_UP_TIM13_IRQHandler(void)
{
	if((TIM8->DIER & TIM_DIER_UIE) && (TIM8->SR & TIM_SR_UIF))
		tim_irq_hndlr(obj_list[TIM_8]);
	else
		tim_irq_hndlr(obj_list[TIM_13]);
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	if(((TIM8->DIER & TIM_DIER_COMIE) && (TIM8->SR & TIM_SR_COMIF)) ||
		((TIM8->DIER & TIM_DIER_TIE) && (TIM8->SR & TIM_SR_TIF)))
	{
		tim_irq_hndlr(obj_list[TIM_8]);
	}
	else
		tim_irq_hndlr(obj_list[TIM_14]);
}
#endif
