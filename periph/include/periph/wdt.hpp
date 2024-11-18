#pragma once

#include <cstdint>
#include <chrono>

namespace periph
{
class wdt
{
public:
    static void timeout(std::chrono::milliseconds timeout);
    
    static void start();
    
    static void reload();
    
private:
    wdt() {}
};
} // namespace periph
