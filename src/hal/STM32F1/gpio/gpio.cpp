#include <stdint.h>
#include <stddef.h>

#include "common/assert.h"
#include "gpio.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"

using namespace hal;

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

static uint32_t const rcc_list[PORT_QTY] =
{
	RCC_APB2ENR_IOPAEN, RCC_APB2ENR_IOPBEN, RCC_APB2ENR_IOPCEN,
	RCC_APB2ENR_IOPDEN, RCC_APB2ENR_IOPEEN,
#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
	RCC_APB2ENR_IOPFEN, RCC_APB2ENR_IOPGEN
#else
	0, 0
#endif
};

gpio::gpio(uint8_t port, uint8_t pin, mode_t mode, bool state):
	_port(port),
	_pin(pin),
	_mode(mode)
{
	ASSERT(_port < PORT_QTY && gpio_list[_port]);
	ASSERT(_pin < PIN_QTY);
	ASSERT(_mode <= MODE_AF);
	
	RCC->APB2ENR |= rcc_list[_port];
	
	gpio::mode(_mode, state);
}

gpio::~gpio()
{
	GPIO_TypeDef *gpio = gpio_list[_port];
	
	if(_pin < 8)
	{
		/* No pull-up, no pull-down */
		gpio->CRL &= ~(GPIO_CRL_MODE0 << (_pin * 4));
		/* Analog mode */
		gpio->CRL &= ~(GPIO_CRL_CNF0 << (_pin * 4));
	}
	else
	{
		/* No pull-up, no pull-down */
		gpio->CRH &= ~(GPIO_CRL_MODE0 << ((_pin - 8) * 4));
		/* Analog mode */
		gpio->CRH &= ~(GPIO_CRL_CNF0 << ((_pin - 8) * 4));
	}
}

void gpio::set(bool state) const
{
	ASSERT(_mode == MODE_DO || _mode == MODE_OD);
	
	gpio_list[_port]->BSRR = 1 << (state ? _pin : _pin + 16);
}

bool gpio::get() const
{
	ASSERT(_mode != MODE_AN && _mode != MODE_AF);
	
	return (bool)(gpio_list[_port]->IDR & (1 << _pin));
}

void gpio::toggle() const
{
	ASSERT(_mode == MODE_DO || _mode == MODE_OD);
	
	gpio_list[_port]->ODR ^= 1 << _pin;
}

void gpio::mode(mode_t mode, bool state)
{
	ASSERT(mode <= MODE_AF);
	
	_mode = mode;
	GPIO_TypeDef *gpio = gpio_list[_port];
	
	if(_pin < 8)
	{
		/* Input mode */
		gpio->CRL &= ~(GPIO_CRL_MODE0 << (_pin * 4));
		/* Analog mode */
		gpio->CRL &= ~(GPIO_CRL_CNF0 << (_pin * 4));
	}
	else
	{
		/* Input mode */
		gpio->CRH &= ~(GPIO_CRL_MODE0 << ((_pin - 8) * 4));
		/* Analog mode */
		gpio->CRH &= ~(GPIO_CRL_CNF0 << ((_pin - 8) * 4));
	}
	
	switch(_mode)
	{
		case MODE_DO:
			if(_pin < 8)
				gpio->CRL |= GPIO_CRL_MODE0 << (_pin * 4);
			else
				gpio->CRH |= GPIO_CRL_MODE0 << ((_pin - 8) * 4);
			break;
		
		case MODE_OD:
			if(_pin < 8)
			{
				gpio->CRL |= GPIO_CRL_CNF0_0 << (_pin * 4);
				gpio->CRL |= GPIO_CRL_MODE0 << (_pin * 4);
			}
			else
			{
				gpio->CRH |= GPIO_CRL_CNF0_0 << ((_pin - 8) * 4);
				gpio->CRH |= GPIO_CRL_MODE0 << ((_pin - 8) * 4);
			}
			break;
		
		case MODE_DI:
			if(_pin < 8)
				gpio->CRL |= GPIO_CRL_CNF0_1 << (_pin * 4);
			else
				gpio->CRH |= GPIO_CRL_CNF0_1 << ((_pin - 8) * 4);
			
			if(state)
				/* Pull-up */
				gpio->ODR |= GPIO_ODR_ODR0 << _pin;
			else
				/* Pull-down */
				gpio->ODR &= ~(GPIO_ODR_ODR0 << _pin);
			break;
		
		case MODE_AN:
			/* Analog mode has already enabled */
			break;
		
		case MODE_AF:
			if(_pin < 8)
			{
				gpio->CRL |= GPIO_CRL_CNF0_1 << (_pin * 4);
				gpio->CRL |= GPIO_CRL_MODE0 << (_pin * 4);
			}
			else
			{
				gpio->CRH |= GPIO_CRL_CNF0_1 << ((_pin - 8) * 4);
				gpio->CRH |= GPIO_CRL_MODE0 << ((_pin - 8) * 4);
			}
			break;
	}
	
	/* Setup default state */
	if(_mode == MODE_DO || _mode == MODE_OD)
		gpio->BSRR =  1 << (state ? _pin : _pin + 16);
}
