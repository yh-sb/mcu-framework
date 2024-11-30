#pragma once

#include "periph/pwm.hpp"
#include "gpio_stm32f1.hpp"

namespace periph
{
class pwm_stm32f1 : public pwm
{
public:
    enum class mode : uint8_t
    {
        noninverted,
        inverted
    };
    
    static constexpr uint8_t pwm_channels = 4; // The total number of pwm channels
    
    /**
     * @brief  Construct pwm (pulse-width modulation) object
     * 
     * @param  timer   The timer instance to be used:
     *                 - 1:  Advanced-control timer TIM1
     *                 - 2:  General-purpose timer TIM2
     *                 - 3:  General-purpose timer TIM3
     *                 - 4:  General-purpose timer TIM4
     *                 - 5:  General-purpose timer TIM5
     *                 - 8:  Advanced-control timer TIM8
     *                 - 9:  General-purpose timer TIM9
     *                 - 10: General-purpose timer TIM10
     *                 - 11: General-purpose timer TIM11
     *                 - 12: General-purpose timer TIM12
     *                 - 13: General-purpose timer TIM13
     *                 - 14: General-purpose timer TIM14
     *                 - 15: General-purpose timer TIM15
     *                 - 16: General-purpose timer TIM16
     *                 - 17: General-purpose timer TIM17
     * @param  channel The PWM channel to be used. Can be 1 to 4
     * @param  gpio    GPIO pin to be used as PWM output
     * @param  mode    PWM mode
     */
    pwm_stm32f1(uint8_t timer, uint8_t channel, gpio_stm32f1 &gpio, enum mode mode = mode::noninverted);
    ~pwm_stm32f1();
    
    void frequency(uint32_t frequency) final;
    
    uint32_t frequency() const final { return freq; }
    
    void duty_cycle(uint8_t duty_cycle) final;
    
    uint8_t duty_cycle() const final { return _duty_cycle; }
    
    void start() final;
    void stop() final;
    
    // Delete copy constructor and copy assignment operator
    pwm_stm32f1(const pwm_stm32f1&) = delete;
    pwm_stm32f1& operator=(const pwm_stm32f1&) = delete;
    
    // Delete move constructor and move assignment operator
    pwm_stm32f1(pwm_stm32f1&&) = delete;
    pwm_stm32f1& operator=(pwm_stm32f1&&) = delete;
    
private:
    uint8_t tim;
    uint8_t ch;
    uint32_t freq;
    uint8_t _duty_cycle;
    mode mode;
    gpio_stm32f1 &gpio;
    static void calc_frequency(uint8_t timer, uint32_t frequency, uint16_t &presc, uint16_t &reload);
    static uint16_t calc_ccr(uint8_t timer, uint8_t duty_cycle);
};
} // namespace periph
