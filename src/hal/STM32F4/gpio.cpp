#include <stdint.h>
#include <stddef.h>

#include "common/macros.h"

#include "hal/STM32F4/gpio.hpp"

#include "hal/STM32F4/CMSIS/device-support/include/stm32f4xx.h"

using namespace hal;

static GPIO_TypeDef *const gpio_list[PORT_QTY] =
{
	GPIOA, GPIOB, GPIOC,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	GPIOD, GPIOE,
#else
	NULL, NULL,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	GPIOF, GPIOG,
#else
	NULL, NULL,
#endif
	GPIOH,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || \
	defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	GPIOI,
#else
	NULL,
#endif
#if defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	GPIOJ, GPIOK
#else
	NULL, NULL
#endif
};

static uint32_t const rcc_list[PORT_QTY] =
{
	RCC_AHB1ENR_GPIOAEN, RCC_AHB1ENR_GPIOBEN, RCC_AHB1ENR_GPIOCEN,
#if defined(STM32F401xC) || defined(STM32F411xE) || defined(STM32F401xE) || \
	defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F446xx) || defined(STM32F405xx) || defined(STM32F407xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F427xx) || \
	defined(STM32F429xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_AHB1ENR_GPIODEN, RCC_AHB1ENR_GPIOEEN,
#else
	0, 0,
#endif
#if defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F446xx) || defined(STM32F405xx) || defined(STM32F407xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F427xx) || \
	defined(STM32F429xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_AHB1ENR_GPIOFEN, RCC_AHB1ENR_GPIOGEN,
#else
	0, 0,
#endif
	RCC_AHB1ENR_GPIOHEN,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || \
	defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_AHB1ENR_GPIOIEN,
#else
	0,
#endif
#if defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	RCC_AHB1ENR_GPIOJEN, RCC_AHB1ENR_GPIOKEN
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
	
	RCC->AHB1ENR |= rcc_list[_port];
	
	/* Input mode */
	gpio->MODER &= ~(GPIO_MODER_MODE0 << (_pin * 2));
	
	/* No pull-up, no pull-down */
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (_pin * 2));
	
	/* Set very high speed */
	gpio->OSPEEDR |= GPIO_OSPEEDR_OSPEED0 << (_pin * 2);
	
	switch(_mode)
	{
		case GPIO_MODE_DO:
			/* Digital output type */
			gpio->MODER |= GPIO_MODER_MODE0_0 << (_pin * 2);
			/* Push-pull type */
			gpio->OTYPER &= ~(GPIO_OTYPER_OT0 << _pin);
			break;
		
		case GPIO_MODE_OD:
			/* Digital output type */
			gpio->MODER |= GPIO_MODER_MODE0_0 << (_pin * 2);
			/* Open drain type */
			gpio->OTYPER |= GPIO_OTYPER_OT0 << _pin;
			break;
		
		case GPIO_MODE_DI:
			/* Pull-up or pull-down */
			if(state)
				gpio->PUPDR |= GPIO_PUPDR_PUPD0_0 << (_pin * 2);
			else
				gpio->PUPDR |= GPIO_PUPDR_PUPD0_1 << (_pin * 2);
			break;
		
		case GPIO_MODE_AN:
			/* Analog mode */
			gpio->MODER |= GPIO_MODER_MODE0 << (_pin * 2);
			break;
		
		case GPIO_MODE_AF:
			/* Alternate function mode */
			gpio->MODER |= GPIO_MODER_MODE0_1 << (_pin * 2);
			/* Modification of AFR register should be done during periph init */
			break;
	}
	
	/* Setup default state */
	if(_mode == GPIO_MODE_DO || _mode == GPIO_MODE_OD)
		gpio->BSRR =  1 << (state ? _pin : _pin + 16);
}

gpio::~gpio()
{
	GPIO_TypeDef *gpio = gpio_list[_port];
	/* No pull-up, no pull-down */
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (_pin * 2));
	/* Analog mode */
	gpio->MODER |= GPIO_MODER_MODE0 << (_pin * 2);
}

void gpio::set(bool state) const
{
	ASSERT(_mode == GPIO_MODE_DO || _mode == GPIO_MODE_OD);
	
	gpio_list[_port]->BSRR = 1 << (state ? _pin : _pin + 16);
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
