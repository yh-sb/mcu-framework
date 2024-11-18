#pragma once

#include <cstdint>
#include <ctime>
#include <functional>

namespace periph
{
class rtc
{
public:
    enum class clk_source : uint8_t
    {
        internal,
        external
    };
    
    enum class res : int8_t
    {
        ok    =  0,
        error = -1
    };
    
    static res init(clk_source clk);
    
    static std::tm get();
    
    static res set(std::tm &tm);
    
    static void bckp_write(uint8_t addr, const void *buff, size_t size);
    
    static void bckp_read(uint8_t addr, void *buff, size_t size);
    
    static void set_alarm_callback(std::function<void(const std::tm &)> on_alarm);
    static void set_alarm(const std::tm &tm);
    
    static bool is_valid(const std::tm &tm);
    
private:
    rtc() {};
};
} // namespace periph
