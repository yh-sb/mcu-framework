#pragma once

#include "periph/gpio.hpp"

namespace periph
{
class gpio_rp2040 : public gpio
{
public:
    gpio_rp2040(uint8_t pin, enum mode mode, bool state = false);
    ~gpio_rp2040();
    
    void set(bool state) final;
    void toggle() final;
    void mode(enum mode mode, bool state = false) final;
    
    bool get() const final;
    enum mode mode() const final { return _mode; }
    uint8_t pin() const final { return _pin; }
    
    // Delete copy constructor and copy assignment operator
    gpio_rp2040(const gpio_rp2040&) = delete;
    gpio_rp2040& operator=(const gpio_rp2040&) = delete;
    
    // Delete move constructor and move assignment operator
    gpio_rp2040(gpio_rp2040&&) = delete;
    gpio_rp2040& operator=(gpio_rp2040&&) = delete;
    
private:
    uint8_t _pin;
    enum mode _mode;
};
} // namespace periph
