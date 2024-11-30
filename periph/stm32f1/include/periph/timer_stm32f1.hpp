#pragma once

#include "periph/timer.hpp"

namespace periph { class timer_stm32f1; }
// For internal use only! (called from ISR)
extern "C" void tim_irq_hndlr(periph::timer_stm32f1 *obj);

namespace periph
{
class timer_stm32f1 : public timer
{
public:
    static constexpr uint8_t timers = 17; // The total number of timers
    
    /**
     * @brief  Construct timer object
     * 
     * @param  timer The timer instance to be used:
     *               - 1:  Advanced-control timer TIM1
     *               - 2:  General-purpose timer TIM2
     *               - 3:  General-purpose timer TIM3
     *               - 4:  General-purpose timer TIM4
     *               - 5:  General-purpose timer TIM5
     *               - 6:  Basic timer TIM6
     *               - 7:  Basic timer TIM7
     *               - 8:  Advanced-control timer TIM8
     *               - 9:  General-purpose timer TIM9
     *               - 10: General-purpose timer TIM10
     *               - 11: General-purpose timer TIM11
     *               - 12: General-purpose timer TIM12
     *               - 13: General-purpose timer TIM13
     *               - 14: General-purpose timer TIM14
     *               - 15: General-purpose timer TIM15
     *               - 16: General-purpose timer TIM16
     *               - 17: General-purpose timer TIM17
     */
    timer_stm32f1(uint8_t timer);
    ~timer_stm32f1();
    
    void set_callback(std::function<void()> on_timeout) final;
    
    void timeout(std::chrono::microseconds timeout) final;
    
    std::chrono::microseconds timeout() const final { return _timeout; }
    
    void start(bool is_cyclic = false) final;
    void stop() final;
    
    bool is_expired() const final;
    
    // Delete copy constructor and copy assignment operator
    timer_stm32f1(const timer_stm32f1&) = delete;
    timer_stm32f1& operator=(const timer_stm32f1&) = delete;
    
    // Delete move constructor and move assignment operator
    timer_stm32f1(timer_stm32f1&&) = delete;
    timer_stm32f1& operator=(timer_stm32f1&&) = delete;
    
private:
    uint8_t tim;
    std::chrono::microseconds _timeout;
    std::function<void()> on_timeout;
    static void calc_clk(uint8_t tim, uint32_t usec, uint16_t &psc, uint16_t &arr);
    friend void ::tim_irq_hndlr(timer_stm32f1 *obj);
};
} // namespace periph
