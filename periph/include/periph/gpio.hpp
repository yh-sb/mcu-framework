#pragma once

#include <cstdint>

namespace periph
{
class gpio
{
public:
    enum class mode : uint8_t
    {
        digital_output,
        open_drain,
        digital_input,
        analog,
        alternate_function
    };
    
    gpio() = default;
    virtual ~gpio() = default;
    
    virtual void set(bool state) = 0;
    
    virtual void toggle() = 0;
    
    virtual void mode(mode mode, bool state = false) = 0;
    
    virtual bool get() const = 0;
    
    virtual enum mode mode() const = 0;
    
    virtual uint8_t pin() const = 0;
};
} // namespace periph
