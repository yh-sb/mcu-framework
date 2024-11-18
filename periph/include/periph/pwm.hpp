#pragma once

#include <cstdint>

namespace periph
{
class pwm
{
public:
    pwm() = default;
    virtual ~pwm() = default;
    
    virtual void frequency(uint32_t frequency) = 0;
    
    virtual uint32_t frequency() const = 0;
    
    virtual void duty_cycle(uint8_t duty_cycle) = 0;
    
    virtual uint8_t duty_cycle() const = 0;
    
    virtual void start() = 0;
    
    virtual void stop() = 0;
};
} // namespace periph
