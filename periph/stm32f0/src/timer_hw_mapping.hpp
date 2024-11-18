#pragma once

#include "periph/timer_stm32f0.hpp"
#include "rcc.hpp"
#include "stm32f0xx.h"

namespace periph::timer_hw_mapping
{
constexpr TIM_TypeDef *const timer[timer_stm32f0::timers] =
{
    TIM1,
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
    defined(STM32F091xC) || defined(STM32F098xx)
    TIM2,
#else
    nullptr,
#endif
    TIM3, nullptr, nullptr,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
    defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    TIM6,
#else
    nullptr,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    TIM7,
#else
    nullptr,
#endif
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, TIM14,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
    defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    TIM15,
#else
    nullptr,
#endif
    TIM16,
    TIM17
};

constexpr uint32_t rcc_en[timer_stm32f0::timers] =
{
    RCC_APB2ENR_TIM1EN,
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
    defined(STM32F091xC) || defined(STM32F098xx)
    RCC_APB1ENR_TIM2EN,
#else
    0,
#endif
    RCC_APB1ENR_TIM3EN,
    0,
    0,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
    defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    RCC_APB1ENR_TIM6EN,
#else
    0,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    RCC_APB1ENR_TIM7EN,
#else
    0,
#endif
    0, 0, 0, 0, 0, 0, RCC_APB1ENR_TIM14EN,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
    defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    RCC_APB2ENR_TIM15EN,
#else
    0,
#endif
    RCC_APB2ENR_TIM16EN, RCC_APB2ENR_TIM17EN
};

constexpr volatile uint32_t *rcc_en_reg[timer_stm32f0::timers] =
{
    &RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, nullptr, nullptr, &RCC->APB1ENR,
    &RCC->APB1ENR, nullptr, nullptr, nullptr, nullptr, nullptr,
    nullptr, &RCC->APB1ENR, &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR
};

constexpr rcc::clk_source rcc_src[timer_stm32f0::timers] =
{
    rcc::clk_source::apb2,
    rcc::clk_source::apb1,
    rcc::clk_source::apb1,
    static_cast<rcc::clk_source>(0),
    static_cast<rcc::clk_source>(0),
    rcc::clk_source::apb1,
    rcc::clk_source::apb1,
    static_cast<rcc::clk_source>(0),
    static_cast<rcc::clk_source>(0),
    static_cast<rcc::clk_source>(0),
    static_cast<rcc::clk_source>(0),
    static_cast<rcc::clk_source>(0),
    static_cast<rcc::clk_source>(0),
    rcc::clk_source::apb1,
    rcc::clk_source::apb2,
    rcc::clk_source::apb2,
    rcc::clk_source::apb2
};

constexpr IRQn_Type irqn[timer_stm32f0::timers] =
{
    TIM1_CC_IRQn,
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
    defined(STM32F091xC) || defined(STM32F098xx)
    TIM2_IRQn,
#else
    static_cast<IRQn_Type>(0),
#endif
    TIM3_IRQn, static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
    defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    TIM6_DAC_IRQn,
#else
    static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    TIM7_IRQn,
#else
    static_cast<IRQn_Type>(0),
#endif
    static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
    static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
    static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0), TIM14_IRQn,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
    defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    TIM15_IRQn,
#else
    static_cast<IRQn_Type>(0),
#endif
    TIM16_IRQn, TIM17_IRQn
};
} // namespace periph::timer_hw_mapping
