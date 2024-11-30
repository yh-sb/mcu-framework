#pragma once

#include "periph/timer.hpp"

namespace periph { class timer_stm32f0; }
// For internal use only! (called from ISR)
extern "C" void tim_irq_hndlr(periph::timer_stm32f0 *obj);

namespace periph
{
class timer_stm32f0 : public timer
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
     * 
     *               - 6:  Basic timer TIM6
     *               - 7:  Basic timer TIM7
     * 
     *               - 14: General-purpose timer TIM14
     *               - 15: General-purpose timer TIM15
     *               - 16: General-purpose timer TIM16
     *               - 17: General-purpose timer TIM17
     */
    timer_stm32f0(uint8_t timer);
    ~timer_stm32f0();
    
    void set_callback(std::function<void()> on_timeout) final;
    
    void timeout(std::chrono::microseconds timeout) final;
    
    std::chrono::microseconds timeout() const final { return _timeout; }
    
    void start(bool is_cyclic = false) final;
    void stop() final;
    
    bool is_expired() const final;
    
    // Delete copy constructor and copy assignment operator
    timer_stm32f0(const timer_stm32f0&) = delete;
    timer_stm32f0& operator=(const timer_stm32f0&) = delete;
    
    // Delete move constructor and move assignment operator
    timer_stm32f0(timer_stm32f0&&) = delete;
    timer_stm32f0& operator=(timer_stm32f0&&) = delete;
    
private:
    uint8_t tim;
    std::chrono::microseconds _timeout;
    std::function<void()> on_timeout;
    static void calc_clk(uint8_t tim, uint32_t usec, uint16_t &psc, uint16_t &arr);
    friend void ::tim_irq_hndlr(timer_stm32f0 *obj);
};
} // namespace periph
