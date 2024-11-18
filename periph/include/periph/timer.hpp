#pragma once

#include <cstdint>
#include <functional>
#include <chrono>

namespace periph
{
class timer
{
public:
    timer() = default;
    virtual ~timer() = default;
    
    virtual void set_callback(std::function<void()> on_timeout) = 0;
    
    virtual void timeout(std::chrono::microseconds timeout) = 0;
    
    virtual std::chrono::microseconds timeout() const = 0;
    
    virtual void start(bool is_cyclic = false) = 0;
    
    virtual void stop() = 0;
    
    virtual bool is_expired() const = 0;
};
} // namespace periph
