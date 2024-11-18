#pragma once

#include <cstdint>
#include "periph/gpio.hpp"

namespace periph
{
class spi
{
public:
    enum class cpol : uint8_t
    {
        low, // Clock is low when inactive
        high // Clock is high when inactive
    };
    
    enum class cpha : uint8_t
    {
        leading, // Data is valid on clock leading edge
        trailing // Data is valid on clock trailing edge
    };
    
    enum class bit_order : uint8_t
    {
        msb,
        lsb
    };
    
    enum class res : int8_t
    {
        ok    =  0,
        error = -1
    };
    
    spi() = default;
    virtual ~spi() = default;
    
    virtual void baudrate(uint32_t baudrate) = 0;
    
    virtual uint32_t baudrate() const = 0;
    
    virtual void cpol(cpol cpol) = 0;
    
    virtual enum cpol cpol() const = 0;
    
    virtual void cpha(cpha cpha) = 0;
    
    virtual enum cpha cpha() const = 0;
    
    virtual void bit_order(bit_order bit_order) = 0;
    
    virtual enum bit_order bit_order() const = 0;
    
    virtual res write(const void *buff, uint16_t size, gpio *cs = nullptr) = 0;
    
    virtual res write(uint8_t byte, gpio *cs = nullptr) = 0;
    
    virtual res read(void *buff, uint16_t size, gpio *cs = nullptr) = 0;
    
    virtual res write_read(const void *write_buff, void *read_buff, uint16_t size, gpio *cs = nullptr) = 0;
};

// Chip select helper class. Automatically put cs pin high in destructor
class spi_cs
{
public:
    spi_cs(gpio &cs):
        cs(cs)
    {
        cs.set(0);
    }
    
    ~spi_cs()
    {
        cs.set(1);
    }
    
private:
    gpio &cs;
};
} // namespace periph
