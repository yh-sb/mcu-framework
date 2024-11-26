#pragma once

#include "periph/gpio.hpp"

namespace periph
{
class gpio_esp32s3 : public gpio
{
public:
    gpio_esp32s3(uint8_t pin, enum mode mode, bool state = false);
    ~gpio_esp32s3();
    
    void set(bool state) final;
    void toggle() final;
    void mode(enum mode mode, bool state = false) final;
    
    bool get() const final;
    enum mode mode() const final { return _mode; }
    uint8_t pin() const final { return _pin; }
    
private:
    uint8_t _pin;
    enum mode _mode;
};
} // namespace periph
