#pragma once

#include <functional>
#include "periph/gpio.hpp"

namespace drv
{
class encoder
{
public:
    encoder(periph::gpio &a, periph::gpio &b);
    ~encoder();
    
    void set_callback(std::function<void(int8_t diff)> on_change);
    void poll();
    
private:
    periph::gpio &a, &b;
    uint8_t prev_state;
    uint8_t prev_prev_state;
    std::function<void(int8_t diff)> on_change;
};
} // namespace drv
