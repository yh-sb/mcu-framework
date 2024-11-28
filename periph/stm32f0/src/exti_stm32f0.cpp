#include <cassert>
#include "periph/exti_stm32f0.hpp"
#include "gpio_hw_mapping.hpp"
#include "stm32f0xx.h"
#include "core_cm0.h"

using namespace periph;

static exti_stm32f0 *obj_list[gpio_hw_mapping::pins];

constexpr IRQn_Type irqn[gpio_hw_mapping::pins] =
{
    EXTI0_1_IRQn, EXTI0_1_IRQn, EXTI2_3_IRQn, EXTI2_3_IRQn, EXTI4_15_IRQn,
    EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
    EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
    EXTI4_15_IRQn
};

exti_stm32f0::exti_stm32f0(gpio_stm32f0 &gpio, enum trigger trigger):
    gpio(gpio),
    _trigger(trigger)
{
    assert(gpio.mode() == gpio::mode::digital_input);
    
    // Enable clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    
    uint8_t pin = gpio.pin();
    
    // Setup EXTI line source
    uint8_t exti_offset = (pin % SYSCFG_EXTICR1_EXTI1_Pos) * SYSCFG_EXTICR1_EXTI1_Pos;
    SYSCFG->EXTICR[pin / SYSCFG_EXTICR1_EXTI1_Pos] &= ~(SYSCFG_EXTICR1_EXTI0 << exti_offset);
    SYSCFG->EXTICR[pin / SYSCFG_EXTICR1_EXTI1_Pos] |= static_cast<uint8_t>(gpio.port()) << exti_offset;
    
    uint32_t line_bit = 1 << pin;
    // Setup EXTI mask regs
    EXTI->EMR &= ~line_bit;
    EXTI->IMR &= ~line_bit;
    
    // Setup EXTI trigger
    EXTI->RTSR |= line_bit;
    EXTI->FTSR |= line_bit;
    if(_trigger == trigger::rising)
    {
        EXTI->FTSR &= ~line_bit;
    }
    else if(_trigger == trigger::falling)
    {
        EXTI->RTSR &= ~line_bit;
    }
    
    obj_list[pin] = this;
    
    NVIC_ClearPendingIRQ(irqn[pin]);
    NVIC_SetPriority(irqn[pin], 2);
    NVIC_EnableIRQ(irqn[pin]);
}

exti_stm32f0::~exti_stm32f0()
{
    uint8_t pin = gpio.pin();
    
    NVIC_DisableIRQ(irqn[pin]);
    EXTI->IMR &= ~(1 << pin);
    
    uint8_t exti_offset = (pin % SYSCFG_EXTICR1_EXTI1_Pos) * SYSCFG_EXTICR1_EXTI1_Pos;
    SYSCFG->EXTICR[pin / SYSCFG_EXTICR1_EXTI1_Pos] &= ~(SYSCFG_EXTICR1_EXTI0 << exti_offset);
    
    obj_list[pin] = nullptr;
}

void exti_stm32f0::set_callback(std::function<void()> on_interrupt)
{
    this->on_interrupt = std::move(on_interrupt);
}

void exti_stm32f0::enable()
{
    assert(on_interrupt);
    
    uint8_t pin = gpio.pin();
    
    EXTI->PR |= 1 << pin;
    EXTI->IMR |= 1 << pin;
    
    NVIC_ClearPendingIRQ(irqn[pin]);
}

void exti_stm32f0::disable()
{
    EXTI->IMR &= ~(1 << gpio.pin());
}

void exti_stm32f0::trigger(enum trigger trigger)
{
    _trigger = trigger;
    uint32_t line_bit = 1 << gpio.pin();
    
    EXTI->RTSR |= line_bit;
    EXTI->FTSR |= line_bit;
    if(_trigger == trigger::rising)
    {
        EXTI->FTSR &= ~line_bit;
    }
    else if(_trigger == trigger::falling)
    {
        EXTI->RTSR &= ~line_bit;
    }
}

extern "C" void exti_irq_hndlr(periph::exti_stm32f0 *obj)
{
    EXTI->PR = 1 << obj->gpio.pin();
    
    if(obj->on_interrupt)
    {
        obj->on_interrupt();
    }
}

extern "C" void EXTI0_1_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;
    
    if(pr & EXTI_PR_PR0)
    {
        exti_irq_hndlr(obj_list[EXTI_PR_PR0_Pos]);
    }
    else if(pr & EXTI_PR_PR1)
    {
        exti_irq_hndlr(obj_list[EXTI_PR_PR1_Pos]);
    }
}

extern "C" void EXTI2_3_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;
    
    if(pr & EXTI_PR_PR2)
    {
        exti_irq_hndlr(obj_list[EXTI_PR_PR2_Pos]);
    }
    else if(pr & EXTI_PR_PR3)
    {
        exti_irq_hndlr(obj_list[EXTI_PR_PR3_Pos]);
    }
}

extern "C" void EXTI4_15_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;
    
    for(uint8_t i = EXTI_PR_PR4_Pos; i < EXTI_PR_PR16_Pos; i++)
    {
        if(pr & (1 << i))
        {
            exti_irq_hndlr(obj_list[i]);
            break;
        }
    }
}
