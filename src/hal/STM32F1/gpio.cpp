#include <stdint.h>
#include <stddef.h>

#include "common/macros.h"

#include "hal/STM32F1/gpio.hpp"

#include "hal/STM32F1/CMSIS/device-support/include/stm32f1xx.h"

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

gpio::gpio(uint8_t port, uint8_t pin, gpio_mode_t mode, bool state):
	_port(port),
	_pin(pin),
	_mode(mode)
{
	ASSERT(_port < PORT_QTY && gpio_list[_port]);
	ASSERT(_pin < PIN_QTY);
	
	GPIO_TypeDef *gpio = gpio_list[_port];
	
	RCC->APB2ENR |= rcc_list[_port];
	
	if(_pin < 8)
	{
		/* Switch pin to input */
		gpio->CRL &= ~(GPIO_CRL_MODE0 << (_pin * 4));
		
		/* Switch pin to analog mode */
		gpio->CRL &= ~(GPIO_CRL_CNF0 << (_pin * 4));
	}
	else
	{
		/* Switch pin to input */
		gpio->CRH &= ~(GPIO_CRL_MODE0 << ((_pin - 8) * 4));
		
		/* Switch pin to analog mode */
		gpio->CRH &= ~(GPIO_CRL_CNF0 << ((_pin - 8) * 4));
	}
	
	switch(_mode)
	{
		case GPIO_MODE_DO:
			if(_pin < 8)
				gpio->CRL |= GPIO_CRL_MODE0 << (_pin * 4);
			else
				gpio->CRH |= GPIO_CRL_MODE0 << ((_pin - 8) * 4);
			break;
		
		case GPIO_MODE_OD:
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
		
		case GPIO_MODE_DI:
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
		
		case GPIO_MODE_AN:
			/* Analog mode has already enabled */
			break;
		
		case GPIO_MODE_AF:
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
	if(_mode == GPIO_MODE_DO || _mode == GPIO_MODE_OD)
	{
		if(state)
			gpio->BSRR |= (1 << _pin);
		else
			gpio->BRR |= (1 << _pin);
	}
}

gpio::~gpio()
{
	GPIO_TypeDef *gpio = gpio_list[_port];
	
	if(_pin < 8)
	{
		/* Switch pin to input */
		gpio->CRL &= ~(GPIO_CRL_MODE0 << (_pin * 4));
		
		/* Switch pin to analog mode */
		gpio->CRL &= ~(GPIO_CRL_CNF0 << (_pin * 4));
	}
	else
	{
		/* Switch pin to input */
		gpio->CRH &= ~(GPIO_CRL_MODE0 << ((_pin - 8) * 4));
		
		/* Switch pin to analog mode */
		gpio->CRH &= ~(GPIO_CRL_CNF0 << ((_pin - 8) * 4));
	}
}

void gpio::set(bool state) const
{
	ASSERT(_mode == GPIO_MODE_DO || _mode == GPIO_MODE_OD);
	
	if(state)
		gpio_list[_port]->BSRR |= (1 << _pin);
	else
		gpio_list[_port]->BRR |= (1 << _pin);
}

bool gpio::get() const
{
	ASSERT(_mode != GPIO_MODE_AN && _mode != GPIO_MODE_AF);
	
	return (bool)(gpio_list[_port]->IDR & (1 << _pin));
}

void gpio::toggle() const
{
	ASSERT(_mode == GPIO_MODE_DO || _mode == GPIO_MODE_OD);
	
	gpio_list[_port]->ODR ^= 1 << _pin;
}
