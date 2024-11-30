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
    
    // Delete copy constructor and copy assignment operator
    gpio_esp32s3(const gpio_esp32s3&) = delete;
    gpio_esp32s3& operator=(const gpio_esp32s3&) = delete;
    
    // Delete move constructor and move assignment operator
    gpio_esp32s3(gpio_esp32s3&&) = delete;
    gpio_esp32s3& operator=(gpio_esp32s3&&) = delete;
    
private:
    uint8_t _pin;
    enum mode _mode;
};
} // namespace periph
