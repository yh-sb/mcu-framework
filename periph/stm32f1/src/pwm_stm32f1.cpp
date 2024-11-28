#include <limits>
#include <cassert>
#include "periph/pwm_stm32f1.hpp"
#include "periph/timer_stm32f1.hpp"
#include "timer_hw_mapping.hpp"
#include "rcc.hpp"
#include "stm32f1xx.h"

using namespace periph;

constexpr uint8_t const max_channels[timer_stm32f1::timers] =
{
    4, 4, 4, 4, 4, 0,
    0, 4, 2, 1, 1, 2,
    1, 1, 2, 2, 2 // TODO: Clarify the number of channels for different MCUs (100xx vs 103xx)
};

constexpr uint32_t ccmr[pwm_stm32f1::pwm_channels][static_cast<uint8_t>(pwm_stm32f1::mode::inverted) + 1] = 
{
    {TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1, TIM_CCMR1_OC1M},
    {TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1, TIM_CCMR1_OC2M},
    {TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1, TIM_CCMR2_OC3M},
    {TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1, TIM_CCMR2_OC4M}
};

pwm_stm32f1::pwm_stm32f1(uint8_t timer, uint8_t channel, gpio_stm32f1 &gpio, enum mode mode):
    freq(0),
    _duty_cycle(0),
    mode(mode),
    gpio(gpio)
{
    assert(timer >= 1 && timer <= timer_stm32f1::timers && timer_hw_mapping::timer[timer - 1]);
    assert(channel >= 1 && channel <= pwm_channels);
    assert(channel <= max_channels[timer - 1]);
    assert(gpio.mode() == gpio::mode::alternate_function);
    
    tim = timer - 1;
    ch = channel - 1;
    
    *timer_hw_mapping::rcc_en_reg[tim] |= timer_hw_mapping::rcc_en[tim];
    
    TIM_TypeDef *tim_reg = timer_hw_mapping::timer[tim];
    
    // Enable PWM output
    tim_reg->CCER |= TIM_CCER_CC1E << (ch * TIM_CCER_CC2E_Pos);
    
    switch(ch)
    {
        case 0:
            tim_reg->CCMR1 &= ~TIM_CCMR1_OC1M;
            tim_reg->CCMR1 |= ccmr[ch][static_cast<uint8_t>(mode)];
            break;
        
        case 1:
            tim_reg->CCMR1 &= ~TIM_CCMR1_OC2M;
            tim_reg->CCMR1 |= ccmr[ch][static_cast<uint8_t>(mode)];
            break;
        
        case 2:
            tim_reg->CCMR2 &= ~TIM_CCMR2_OC3M;
            tim_reg->CCMR2 |= ccmr[ch][static_cast<uint8_t>(mode)];
            break;
        
        case 3:
            tim_reg->CCMR2 &= ~TIM_CCMR2_OC4M;
            tim_reg->CCMR2 |= ccmr[ch][static_cast<uint8_t>(mode)];
            break;
        
        default: assert(0);
    }
    
    // Enable output for advanced timers (TIM1, TIM8): RM0090 chapter 17.4.18 (page 575)
    if(tim == 0 || tim == 7)
    {
        tim_reg->BDTR |= TIM_BDTR_MOE;
    }
}

pwm_stm32f1::~pwm_stm32f1()
{
    timer_hw_mapping::timer[tim]->CR1 &= ~TIM_CR1_CEN;
    *timer_hw_mapping::rcc_en_reg[tim] &= ~timer_hw_mapping::rcc_en[tim];
}

void pwm_stm32f1::frequency(uint32_t frequency)
{
    assert(frequency > 0);
    
    freq = frequency;
    uint16_t presc, reload;
    calc_frequency(tim, freq, presc, reload);
    
    timer_hw_mapping::timer[tim]->PSC = presc;
    timer_hw_mapping::timer[tim]->ARR = reload;
}

void pwm_stm32f1::duty_cycle(uint8_t duty_cycle)
{
    assert(duty_cycle <= 100);
    
    _duty_cycle = duty_cycle;
    uint16_t ccr = calc_ccr(tim, _duty_cycle);
    switch(ch)
    {
        case 0: timer_hw_mapping::timer[tim]->CCR1 = ccr; break;
        case 1: timer_hw_mapping::timer[tim]->CCR2 = ccr; break;
        case 2: timer_hw_mapping::timer[tim]->CCR3 = ccr; break;
        case 3: timer_hw_mapping::timer[tim]->CCR4 = ccr; break;
        default: assert(0);
    }
}

void pwm_stm32f1::start()
{
    assert(freq > 0);
    
    timer_hw_mapping::timer[tim]->CR1 &= ~TIM_CR1_OPM;
    timer_hw_mapping::timer[tim]->CR1 |= TIM_CR1_CEN;
}

void pwm_stm32f1::stop()
{
    timer_hw_mapping::timer[tim]->CR1 &= ~TIM_CR1_CEN;
}

void pwm_stm32f1::calc_frequency(uint8_t timer, uint32_t freq, uint16_t &presc, uint16_t &reload)
{
    uint32_t clk_freq = rcc::frequency(timer_hw_mapping::rcc_src[timer]);
    
    // If APBx prescaller more then 1, TIMx prescaller multiplies by 2
    if(clk_freq != rcc::frequency(rcc::clk_source::ahb))
    {
        clk_freq *= 2;
    }
    
    /* Increase timer clock frequency or use timer with higher clock frequency
    to pass this assert */
    assert(freq <= clk_freq);
    
    uint32_t tmp_presc = 0;
    uint32_t tmp_reload = (clk_freq + (freq / 2)) / freq;
    constexpr auto tim_max_resol = std::numeric_limits<uint16_t>::max();
    
    if(tmp_reload <= tim_max_resol)
    {
        tmp_presc = 1;
    }
    else
    {
        tmp_presc = ((tmp_reload + (tim_max_resol / 2)) / tim_max_resol) + 1;
        tmp_reload /= tmp_presc;
    }
    
    /* Minimum value for correct duty cycle setup (in percent). Increase timer
    clock frequency or use timer with higher clock frequency to pass this
    assert */
    assert(tmp_reload > 100);
    
    assert(tmp_presc <= tim_max_resol);
    assert(tmp_reload <= tim_max_resol);
    
    presc = tmp_presc - 1;
    reload = tmp_reload - 1;
}

uint16_t pwm_stm32f1::calc_ccr(uint8_t timer, uint8_t duty_cycle)
{
    uint16_t ccr = 0;
    
    if(duty_cycle > 0)
    {
        ccr = (timer_hw_mapping::timer[timer]->ARR * duty_cycle) / 100;
        ccr += 1;
    }
    
    return ccr;
}
