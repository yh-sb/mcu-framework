#pragma once

#include <cstdint>
#include <chrono>

namespace periph
{
class uart
{
public:
    enum class res : int8_t
    {
        ok           =  0,
        read_timeout = -1,
        read_error   = -2,
        write_error  = -3
    };
    
    uart() = default;
    virtual ~uart() = default;
    
    virtual void baudrate(uint32_t baudrate) = 0;
    
    virtual uint32_t baudrate() const = 0;
    
    virtual res write(const void *buff, uint16_t size) = 0;
    
    virtual res read(void *buff, uint16_t *size,
        std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) = 0;
    
    virtual res write_read(const void *write_buff, uint16_t write_size, void *read_buff,
        uint16_t *read_size, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) = 0;
};
} // namespace periph
