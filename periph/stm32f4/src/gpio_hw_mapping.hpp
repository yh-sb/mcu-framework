#pragma once

#include "periph/gpio_stm32f4.hpp"
#include "stm32f4xx.h"

namespace periph::gpio_hw_mapping
{
constexpr auto ports = 11; // GPIOK - the last gpio port
constexpr auto pins = 16; // The total number of pins

constexpr GPIO_TypeDef *const gpio[ports] =
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
    nullptr, nullptr,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
    defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
    defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
    GPIOF, GPIOG,
#else
    nullptr, nullptr,
#endif
    GPIOH,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || \
    defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
    defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F469xx) || \
    defined(STM32F479xx)
    GPIOI,
#else
    nullptr,
#endif
#if defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
    defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
    GPIOJ, GPIOK
#else
    nullptr, nullptr
#endif
};

constexpr uint32_t rcc_en[ports] =
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
} // namespace periph::gpio_hw_mapping
