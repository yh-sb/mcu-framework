#pragma once

#include "periph/uart.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
class onewire
{
public:
    enum class res : int8_t
    {
        ok           =  0,
        line_busy    = -1,
        no_device    = -2,
        write_error  = -3,
        read_error   = -4
    };
    
    onewire(periph::uart &uart);
    ~onewire();
    
    enum res write(uint64_t rom, uint8_t write_byte);
    enum res write(uint64_t rom, void *write_buff, uint16_t write_size);
    enum res read(uint64_t rom, void *read_buff, uint16_t read_size);
    enum res write_read(uint64_t rom, void *write_buff, uint16_t write_size,
        void *read_buff, uint16_t read_size);
    enum res read_rom(uint64_t &rom);
    //enum res search(uint64_t *rom_list, size_t *rom_list_size);
    
    // Delete copy constructor and copy assignment operator
    onewire(const onewire&) = delete;
    onewire& operator=(const onewire&) = delete;
    
    // Delete move constructor and move assignment operator
    onewire(onewire&&) = delete;
    onewire& operator=(onewire&&) = delete;
    
private:
    enum res do_reset();
    enum res write_buff(void *buff, uint8_t size);
    enum res read_buff(void *buff, uint8_t size);
    enum res write_byte(uint8_t byte);
    enum res read_byte(uint8_t *byte);
    
    periph::uart &uart;
    SemaphoreHandle_t api_lock;
};
} // namespace drv
