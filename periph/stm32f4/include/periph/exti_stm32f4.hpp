#pragma once

#include "periph/exti.hpp"
#include "gpio_stm32f4.hpp"

namespace periph { class exti_stm32f4; }
// For internal use only! (called from ISR)
extern "C" void exti_irq_hndlr(periph::exti_stm32f4 *obj);

namespace periph
{
class exti_stm32f4 : public exti
{
public:
    exti_stm32f4(gpio_stm32f4 &gpio, enum trigger trigger = trigger::both);
    ~exti_stm32f4();
    
    void set_callback(std::function<void()> on_interrupt) final;
    
    void on() final;
    void off() final;
    
    void trigger(enum trigger trigger) final;
    enum trigger trigger() final { return _trigger; }
    
private:
    enum trigger _trigger;
    gpio_stm32f4 &gpio;
    std::function<void()> on_interrupt;
    friend void ::exti_irq_hndlr(exti_stm32f4 *obj);
};
}
