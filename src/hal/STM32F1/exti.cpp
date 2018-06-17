#include <stddef.h>

#include "common/macros.h"

#include "hal/STM32F1/exti.hpp"
#include "hal/STM32F1/gpio.hpp"

#include "hal/STM32F1/CMSIS/device-support/include/stm32f1xx.h"

using namespace hal;

#define IRQ_PRIORITY 3

static IRQn_Type const irq_list[PIN_QTY] =
{
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn,
	EXTI3_IRQn, EXTI4_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn
};

static GPIO_TypeDef *const gpio_list[PORT_QTY] =
{
	GPIOA, GPIOB, GPIOC, GPIOD,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xB) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	GPIOE,
#else
	NULL,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG)
	GPIOF, GPIOG
#else
	NULL, NULL
#endif
};

static uint32_t const port_list[PORT_QTY] =
{
	AFIO_EXTICR1_EXTI0_PA, AFIO_EXTICR1_EXTI0_PB, AFIO_EXTICR1_EXTI0_PC,
	AFIO_EXTICR1_EXTI0_PD, AFIO_EXTICR1_EXTI0_PE, AFIO_EXTICR1_EXTI0_PF,
	AFIO_EXTICR1_EXTI0_PG
};

static uint8_t const src_offset_list[PIN_QTY] =
{
	0, 4, 8,
	0, 4, 8,
	0, 4, 8,
	0, 4, 8
};

static exti *obj_list[PIN_QTY];

exti::exti(gpio &gpio, exti_trigger_t trigger):
	_gpio(gpio),
	_trigger(trigger),
	_ctx(NULL),
	_cb(NULL)
{
	ASSERT(_gpio.mode() == GPIO_MODE_DI);
	
	/* Enable clock */
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	/* Setup EXTI line source */
	uint8_t pin = _gpio.pin();
	uint8_t exti_src = src_offset_list[pin];
	AFIO->EXTICR[pin / 2] &= ~(0x0F << exti_src);
	AFIO->EXTICR[pin / 2] |= port_list[_gpio.port()] << exti_src;
	
	uint32_t line_bit = 1 << pin;
	/* Setup EXTI mask regs */
	EXTI->EMR &= ~line_bit;
	EXTI->IMR &= ~line_bit;
	
	/* Setup EXTI trigger */
	EXTI->RTSR &= ~line_bit;
	EXTI->FTSR &= ~line_bit;
	if(_trigger == EXTI_TRIGGER_RISING)
		EXTI->RTSR |= line_bit;
	else if(_trigger == EXTI_TRIGGER_FALLING)
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

void exti::on(exti_cb_t cb, void *ctx)
{
	ASSERT(cb);
	
	_cb = cb;
	_ctx = ctx;
	
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
	_cb = NULL;
	_ctx = NULL;
	/* Clear EXTI line configuration */
	EXTI->IMR &= ~(1 << _gpio.pin());
}

void exti::trigger(exti_trigger_t trigger)
{
	_trigger = trigger;
	uint32_t line_bit = 1 << _gpio.pin();
	if(_trigger == EXTI_TRIGGER_RISING)
		EXTI->RTSR |= line_bit;
	else if(_trigger == EXTI_TRIGGER_FALLING)
		EXTI->FTSR |= line_bit;
	else
	{
		EXTI->RTSR |= line_bit;
		EXTI->FTSR |= line_bit;
	}
}

extern "C" void exti_irq_hndlr(hal::exti *obj)
{
	/* Clear EXTI line pending bits */
	EXTI->PR = 1 << obj->_gpio.pin();
	
	if(obj->_cb)
		obj->_cb(obj, obj->_ctx);
}

extern "C" void EXTI0_IRQHandler(void)
{
	exti_irq_hndlr(obj_list[0]);
}

extern "C" void EXTI1_IRQHandler(void)
{
	exti_irq_hndlr(obj_list[1]);
}

extern "C" void EXTI2_IRQHandler(void)
{
	exti_irq_hndlr(obj_list[2]);
}

extern "C" void EXTI3_IRQHandler(void)
{
	exti_irq_hndlr(obj_list[3]);
}

extern "C" void EXTI4_IRQHandler(void)
{
	exti_irq_hndlr(obj_list[4]);
}

extern "C" void EXTI9_5_IRQHandler(void)
{
	uint32_t pending_bit = EXTI->PR;
	
	for(uint8_t i = 5; i <= 9; i++)
	{
		if(pending_bit & (1 << i))
		{
			exti_irq_hndlr(obj_list[i]);
			break;
		}
	}
}

extern "C" void EXTI15_10_IRQHandler(void)
{
	uint32_t pending_bit = EXTI->PR;
	
	for(uint8_t i = 10; i <= 15; i++)
	{
		if(pending_bit & (1 << i))
		{
			exti_irq_hndlr(obj_list[i]);
			break;
		}
	}
}
