#pragma once

#include <cstdint>

namespace periph
{
class i2c
{
public:
    enum class res : int8_t
    {
        ok          =  0,
        no_ack      = -1,
        write_error = -2,
        read_error  = -3
    };
    
    i2c() = default;
    virtual ~i2c() = default;
    
    virtual void baudrate(uint32_t baudrate) = 0;
    
    virtual uint32_t baudrate() const = 0;
    
    virtual res write_read(uint16_t address, const void *write_buff, uint16_t write_size,
        void *read_buff, uint16_t read_size) = 0;
};
} // namespace periph
