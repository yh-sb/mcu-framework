#pragma once

#include "periph/gpio.hpp"

namespace periph
{
class gpio_stm32f4 : public gpio
{
public:
    enum class port : uint8_t
    {
        a, b, c, d, e, f, g, h, i, j, k
    };
    
    /**
     * @brief  Construct gpio (general-purpose input/output) object
     * 
     * @param  port   GPIO port
     * @param  pin    GPIO pin. Can be 0 to 15
     * @param  mode   GPIO mode
     * @param  state  Initial state of the pin
     */
    gpio_stm32f4(enum port port, uint8_t pin, enum mode mode, bool state = false);
    ~gpio_stm32f4();
    
    void set(bool state) final;
    void toggle() final;
    void mode(enum mode mode, bool state = false) final;
    
    bool get() const final;
    enum mode mode() const final { return _mode; }
    enum port port() const { return _port; }
    uint8_t pin() const final { return _pin; }
    
    // Delete copy constructor and copy assignment operator
    gpio_stm32f4(const gpio_stm32f4&) = delete;
    gpio_stm32f4& operator=(const gpio_stm32f4&) = delete;
    
    // Delete move constructor and move assignment operator
    gpio_stm32f4(gpio_stm32f4&&) = delete;
    gpio_stm32f4& operator=(gpio_stm32f4&&) = delete;
    
private:
    enum port _port;
    uint8_t _pin;
    enum mode _mode;
};
} // namespace periph
