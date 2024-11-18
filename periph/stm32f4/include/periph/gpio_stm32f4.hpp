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
    
    gpio_stm32f4(port port, uint8_t pin, enum mode mode, bool state = false);
    ~gpio_stm32f4();
    
    void set(bool state) final;
    void toggle() final;
    void mode(enum mode mode, bool state = false) final;
    
    bool get() const final;
    enum mode mode() const final { return _mode; }
    enum port port() const { return _port; }
    uint8_t pin() const final { return _pin; }
    
private:
    enum port _port;
    uint8_t _pin;
    enum mode _mode;
};
} // namespace periph
