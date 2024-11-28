#pragma once

#include "periph/exti.hpp"
#include "gpio_stm32f1.hpp"

namespace periph { class exti_stm32f1; }
// For internal use only! (called from ISR)
extern "C" void exti_irq_hndlr(periph::exti_stm32f1 *obj);

namespace periph
{
class exti_stm32f1 : public exti
{
public:
    /**
     * @brief  Construct exti (external interrupt) object
     * 
     * @param  gpio    GPIO pin to use as interrupt source
     * @param  trigger Trigger type
     */
    exti_stm32f1(gpio_stm32f1 &gpio, enum trigger trigger = trigger::both);
    ~exti_stm32f1();
    
    void set_callback(std::function<void()> on_interrupt) final;
    
    void enable() final;
    void disable() final;
    
    void trigger(enum trigger trigger) final;
    enum trigger trigger() final { return _trigger; }
    
private:
    enum trigger _trigger;
    gpio_stm32f1 &gpio;
    std::function<void()> on_interrupt;
    friend void ::exti_irq_hndlr(exti_stm32f1 *obj);
};
} // namespace periph
