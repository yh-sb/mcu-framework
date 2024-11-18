#pragma once

#include <cstdint>

namespace periph
{
class dac
{
public:
    dac() = default;
    virtual ~dac() = default;
    
    virtual void set(uint16_t code) = 0;
    
    virtual void set(float voltage) = 0;
    
    virtual uint16_t get() const = 0;
};
} // namespace periph
