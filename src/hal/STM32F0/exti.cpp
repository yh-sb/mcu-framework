#include <stddef.h>

#include "common/assert.h"

#include "hal/STM32F0/exti.hpp"
#include "hal/STM32F0/gpio.hpp"

#include "hal/STM32F0/CMSIS/device-support/include/stm32f0xx.h"
#include "hal/STM32F0/CMSIS/core-support/core_cm0.h"

using namespace hal;

#define IRQ_PRIORITY 3
#define GPIO_AF_15_EVENTOUT 0x0F

static IRQn_Type const irq_list[PIN_QTY] =
{
	EXTI0_1_IRQn, EXTI0_1_IRQn, EXTI2_3_IRQn,
	EXTI2_3_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
	EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
	EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
	EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
	EXTI4_15_IRQn
};

static GPIO_TypeDef *const gpio_list[PORT_QTY] =
{
	GPIOA, GPIOB, GPIOC,
#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
	defined(STM32F051x8) || defined(STM32F058xx) || defined(STM32F070x6) || \
	defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
	GPIOD,
#else
	NULL,
#endif
#if defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	GPIOE,
#else
	NULL,
#endif
	GPIOF
};

static uint32_t const port_list[PORT_QTY] =
{
	SYSCFG_EXTICR1_EXTI0_PA, SYSCFG_EXTICR1_EXTI0_PB, SYSCFG_EXTICR1_EXTI0_PC,
#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
	defined(STM32F051x8) || defined(STM32F058xx) || defined(STM32F070x6) || \
	defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
	SYSCFG_EXTICR1_EXTI0_PD,
#else
	0,
#endif
#if defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	SYSCFG_EXTICR1_EXTI0_PE,
#else
	0,
#endif
	SYSCFG_EXTICR1_EXTI0_PF
};

static uint8_t const src_offset_list[PIN_QTY] =
{
	0, 4, 8,
	0, 4, 8,
	0, 4, 8,
	0, 4, 8
};

static exti *obj_list[PIN_QTY];

static void gpio_af_init(gpio &gpio);

exti::exti(gpio &gpio, edge edge):
	_gpio(gpio),
	_edge(edge),
	_ctx(NULL),
	_cb(NULL)
{
	ASSERT(_gpio.mode() == gpio::mode::DI);
	
	gpio_af_init(_gpio);
	
	/* Enable clock */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	/* Setup EXTI line source */
	uint8_t pin = _gpio.pin();
	uint8_t exti_src = src_offset_list[pin];
	SYSCFG->EXTICR[pin / 2] &= ~(0x0F << exti_src);
	SYSCFG->EXTICR[pin / 2] |= port_list[_gpio.port()] << exti_src;
	
	uint32_t line_bit = 1 << pin;
	/* Setup EXTI mask regs */
	EXTI->EMR &= ~line_bit;
	EXTI->IMR &= ~line_bit;
	
	/* Setup EXTI trigger */
	EXTI->RTSR &= ~line_bit;
	EXTI->FTSR &= ~line_bit;
	if(_edge == edge::RISING)
		EXTI->RTSR |= line_bit;
	else if(_edge == edge::FALLING)
		EXTI->FTSR |= line_bit;
	else
	{
		EXTI->RTSR |= line_bit;
		EXTI->FTSR |= line_bit;
	}
	
	obj_list[pin] = this;
	
	NVIC_ClearPendingIRQ(irq_list[pin]);
	NVIC_SetPriority(irq_list[pin], IRQ_PRIORITY);
	NVIC_EnableIRQ(irq_list[pin]);
}

exti::~exti()
{
	
}

void exti::cb(exti_cb_t cb, void *ctx)
{
	_cb = cb;
	_ctx = ctx;
}

void exti::on()
{
	ASSERT(_cb);
	
	uint8_t pin = _gpio.pin();
	uint32_t line_bit = 1 << pin;
	/* Clear EXTI line pending bits */
	EXTI->PR |= line_bit;
	/* Setup EXTI line configuration */
	EXTI->IMR |= line_bit;
	
	NVIC_ClearPendingIRQ(irq_list[pin]);
}

void exti::off()
{
	/* Clear EXTI line configuration */
	EXTI->IMR &= ~(1 << _gpio.pin());
}

void exti::trigger(edge edge)
{
	_edge = edge;
	uint32_t line_bit = 1 << _gpio.pin();
	if(_edge == edge::RISING)
		EXTI->RTSR |= line_bit;
	else if(_edge == edge::FALLING)
		EXTI->FTSR |= line_bit;
	else
	{
		EXTI->RTSR |= line_bit;
		EXTI->FTSR |= line_bit;
	}
}

static void gpio_af_init(gpio &gpio)
{
	GPIO_TypeDef *gpio_reg = gpio_list[gpio.port()];
	
	uint8_t pin = gpio.pin();
	if(pin < 8)
	{
		gpio_reg->AFR[0] &= ~(0x0F << (pin * 4));
		gpio_reg->AFR[0] |= GPIO_AF_15_EVENTOUT << (pin * 4);
	}
	else
	{
		gpio_reg->AFR[1] &= ~(0x0F << ((pin - 8) * 4));
		gpio_reg->AFR[1] |= GPIO_AF_15_EVENTOUT << ((pin - 8) * 4);
	}
}

extern "C" void exti_irq_hndlr(hal::exti *obj)
{
	/* Clear EXTI line pending bits */
	EXTI->PR = 1 << obj->_gpio.pin();
	
	if(obj->_cb)
		obj->_cb(obj, obj->_ctx);
}

extern "C" void EXTI0_1_IRQHandler(void)
{
	uint32_t pending_bit = EXTI->PR;
	
	if(pending_bit & (1 << 0))
		exti_irq_hndlr(obj_list[0]);
	else if(pending_bit & (1 << 1))
		exti_irq_hndlr(obj_list[1]);
}

extern "C" void EXTI2_3_IRQHandler(void)
{
	uint32_t pending_bit = EXTI->PR;
	
	if(pending_bit & (1 << 2))
		exti_irq_hndlr(obj_list[2]);
	else if(pending_bit & (1 << 3))
		exti_irq_hndlr(obj_list[3]);
}

extern "C" void EXTI4_15_IRQHandler(void)
{
	uint32_t pending_bit = EXTI->PR;
	
	for(uint8_t i = 4; i <= 15; i++)
	{
		if(pending_bit & (1 << i))
		{
			exti_irq_hndlr(obj_list[i]);
			break;
		}
	}
}
