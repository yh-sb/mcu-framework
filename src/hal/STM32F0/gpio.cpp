#include <stdint.h>
#include <stddef.h>

#include "common/assert.h"

#include "hal/STM32F0/gpio.hpp"

#include "hal/STM32F0/CMSIS/device-support/include/stm32f0xx.h"

using namespace hal;

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

static uint32_t const rcc_list[PORT_QTY] =
{
	RCC_AHBENR_GPIOAEN, RCC_AHBENR_GPIOBEN, RCC_AHBENR_GPIOCEN,
#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
	defined(STM32F051x8) || defined(STM32F058xx) || defined(STM32F070x6) || \
	defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
	RCC_AHBENR_GPIODEN,
#else
	0,
#endif
#if defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	RCC_AHBENR_GPIOEEN,
#else
	0,
#endif
	RCC_AHBENR_GPIOFEN
};

gpio::gpio(uint8_t port, uint8_t pin, mode_t mode, bool state):
	_port(port),
	_pin(pin),
	_mode(mode)
{
	ASSERT(_port < PORT_QTY && gpio_list[_port]);
	ASSERT(_pin < PIN_QTY);
	ASSERT(_mode <= MODE_AF);
	
	RCC->AHBENR |= rcc_list[_port];
	
	gpio::mode(_mode, state);
}

gpio::~gpio()
{
	GPIO_TypeDef *gpio = gpio_list[_port];
	/* No pull-up, no pull-down */
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (_pin * 2));
	/* Analog mode */
	gpio->MODER |= GPIO_MODER_MODER0 << (_pin * 2);
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
	
	/* Input mode */
	gpio->MODER &= ~(GPIO_MODER_MODER0 << (_pin * 2));
	
	/* No pull-up, no pull-down */
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (_pin * 2));
	
	/* Set very high speed */
	gpio->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR0 << (_pin * 2);
	
	switch(_mode)
	{
		case MODE_DO:
			/* Digital output type */
			gpio->MODER |= GPIO_MODER_MODER0_0 << (_pin * 2);
			/* Push-pull type */
			gpio->OTYPER &= ~(GPIO_OTYPER_OT_0 << _pin);
			break;
		
		case MODE_OD:
			/* Digital output type */
			gpio->MODER |= GPIO_MODER_MODER0_0 << (_pin * 2);
			/* Open drain type */
			gpio->OTYPER |= GPIO_OTYPER_OT_0 << _pin;
			break;
		
		case MODE_DI:
			/* Pull-up or pull-down */
			if(state)
				gpio->PUPDR |= GPIO_PUPDR_PUPDR0_0 << (_pin * 2);
			else
				gpio->PUPDR |= GPIO_PUPDR_PUPDR0_1 << (_pin * 2);
			break;
		
		case MODE_AN:
			/* Analog mode */
			gpio->MODER |= GPIO_MODER_MODER0 << (_pin * 2);
			break;
		
		case MODE_AF:
			/* Alternate function mode */
			gpio->MODER |= GPIO_MODER_MODER0_1 << (_pin * 2);
			/* Modification of AFR register should be done during periph init */
			break;
	}
	
	/* Setup default state */
	if(_mode == MODE_DO || _mode == MODE_OD)
		gpio->BSRR =  1 << (state ? _pin : _pin + 16);
}