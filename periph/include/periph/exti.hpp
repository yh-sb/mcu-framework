#pragma once

#include <cstdint>
#include <functional>

namespace periph
{
class exti
{
public:
    enum class trigger : uint8_t
    {
        rising,
        falling,
        both
    };
    
    exti() = default;
    virtual ~exti() = default;
    
    virtual void set_callback(std::function<void()> on_interrupt) = 0;
    
    virtual void enable() = 0;
    
    virtual void disable() = 0;
    
    virtual void trigger(trigger trigger) = 0;
    
    virtual enum trigger trigger() = 0;
};
} // namespace periph
