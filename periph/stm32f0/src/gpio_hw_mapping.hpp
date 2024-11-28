#pragma once

#include "periph/gpio_stm32f0.hpp"
#include "stm32f0xx.h"

namespace periph::gpio_hw_mapping
{
constexpr auto ports = 6; // GPIOF - the last gpio port
constexpr auto pins = 16; // The total number of pins in port

constexpr GPIO_TypeDef *const gpio[ports] =
{
    GPIOA, GPIOB, GPIOC,
#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
    defined(STM32F051x8) || defined(STM32F058xx) || defined(STM32F070x6) || \
    defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
    defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
    GPIOD,
#else
    nullptr,
#endif
#if defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
    defined(STM32F091xC) || defined(STM32F098xx)
    GPIOE,
#else
    nullptr,
#endif
    GPIOF
};

constexpr uint32_t rcc_en[ports] =
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
} // namespace periph::gpio_hw_mapping
