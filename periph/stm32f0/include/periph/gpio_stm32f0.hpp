#pragma once

#include "periph/gpio.hpp"

namespace periph
{
class gpio_stm32f0 : public gpio
{
public:
    enum class port : uint8_t
    {
        a, b, c, d, e, f
    };
    
    /**
     * @brief  Construct gpio (general-purpose input/output) object
     * 
     * @param  port  GPIO port
     * @param  pin   GPIO pin. Can be 0 to 15
     * @param  mode  GPIO mode
     * @param  state Initial state of the pin
     */
    gpio_stm32f0(enum port port, uint8_t pin, enum mode mode, bool state = false);
    ~gpio_stm32f0();
    
    void set(bool state) final;
    void toggle() final;
    void mode(enum mode mode, bool state = false) final;
    
    bool get() const final;
    enum mode mode() const final { return _mode; }
    enum port port() const { return _port; }
    uint8_t pin() const final { return _pin; }
    
    // Delete copy constructor and copy assignment operator
    gpio_stm32f0(const gpio_stm32f0&) = delete;
    gpio_stm32f0& operator=(const gpio_stm32f0&) = delete;
    
    // Delete move constructor and move assignment operator
    gpio_stm32f0(gpio_stm32f0&&) = delete;
    gpio_stm32f0& operator=(gpio_stm32f0&&) = delete;
    
private:
    enum port _port;
    uint8_t _pin;
    enum mode _mode;
};
} // namespace periph
