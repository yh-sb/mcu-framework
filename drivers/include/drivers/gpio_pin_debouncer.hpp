#pragma once

#include <chrono>
#include "periph/gpio.hpp"

namespace drv
{
class gpio_pin_debouncer
{
public:
    gpio_pin_debouncer(periph::gpio &gpio, std::chrono::milliseconds debounce_timeout = std::chrono::milliseconds(50),
        bool default_state = true);
    ~gpio_pin_debouncer();
    
    /**
     * @brief Poll for new state. Should be called with 1 ms period
     * 
     * @param new_state Reference to new gpio pin state after debouncing.
     * @return true  Switch of state has happened, check new_state and handle it.
     * @return false No switch has happened.
     */
    bool poll_1ms(bool &new_state);
    
    std::chrono::milliseconds debounce_timeout() const { return _debounce_timeout; }
    void debounce_timeout(std::chrono::milliseconds debounce_timeout) { _debounce_timeout = debounce_timeout; }
    
    // Delete copy constructor and copy assignment operator
    gpio_pin_debouncer(const gpio_pin_debouncer&) = delete;
    gpio_pin_debouncer& operator=(const gpio_pin_debouncer&) = delete;
    
    // Delete move constructor and move assignment operator
    gpio_pin_debouncer(gpio_pin_debouncer&&) = delete;
    gpio_pin_debouncer& operator=(gpio_pin_debouncer&&) = delete;
    
private:
    bool get_filtered_state();
    
    periph::gpio &gpio;
    std::chrono::milliseconds _debounce_timeout;
    bool state;
    size_t cnt;
};
} // namespace drv
