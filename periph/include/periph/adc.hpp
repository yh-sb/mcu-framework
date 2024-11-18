#pragma once

#include <cstdint>
#include <functional>

namespace periph
{
class adc_onetime
{
public:
    adc_onetime() = default;
    virtual ~adc_onetime() = default;
    
    virtual double read() = 0;
};

class adc_cyclic
{
public:
    adc_cyclic() = default;
    virtual ~adc_cyclic() = default;
    
    virtual void frequency(uint32_t frequency) = 0;
    virtual uint32_t frequency() const = 0;
    
    virtual void set_callback(uint8_t channel, std::function<void(double voltage)> on_value) = 0;
    
    virtual void start() = 0;
    virtual void stop() = 0;
};
} // namespace periph
