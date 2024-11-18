#include <cassert>
#include <cstring>
#include "drivers/onewire.hpp"

using namespace drv;

constexpr auto read_wait_timeout = std::chrono::milliseconds(5);

enum cmd
{
    search_rom   = 0xF0,
    alarm_search = 0xEC, // Search ROM with alarm
    read_rom     = 0x33, // Read ROM of 1wire device. Only if single device is present on the 1wire bus
    match_rom    = 0x55, // Select specific 1wire device by it's ROM
    skip_rom     = 0xCC  // Send/receive data to all devices on the bus
};

onewire::onewire(periph::uart &uart):
    uart(uart)
{
    assert(api_lock = xSemaphoreCreateMutex());
}

onewire::~onewire()
{
    xSemaphoreGive(api_lock);
    vSemaphoreDelete(api_lock);
}

enum onewire::res onewire::write(uint64_t rom, uint8_t write_byte)
{
    return onewire::write(rom, &write_byte, 1);
}

enum onewire::res onewire::write(uint64_t rom, void *write_buff, uint16_t write_size)
{
    assert(write_buff);
    assert(write_size > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    auto res = do_reset();
    if(res != res::ok)
    {
        goto Exit;
    }
    
    if(!rom)
    {
        res = write_byte(cmd::skip_rom);
    }
    else
    {
        uint8_t rom_buff[9];
        rom_buff[0] = cmd::match_rom;
        for(uint8_t i = 1; i < sizeof(rom_buff); i++, rom = rom >> 8)
        {
            rom_buff[i] = rom;
        }
        res = onewire::write_buff(rom_buff, sizeof(rom_buff));
    }
    if(res != res::ok)
    {
        goto Exit;
    }
    
    res = onewire::write_buff(write_buff, write_size);
    
Exit:
    xSemaphoreGive(api_lock);
    return res;
}

enum onewire::res onewire::read(uint64_t rom, void *read_buff, uint16_t read_size)
{
    assert(read_buff);
    assert(read_size > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    auto res = do_reset();
    if(res != res::ok)
    {
        goto Exit;
    }
    
    if(!rom)
    {
        res = write_byte(cmd::skip_rom);
    }
    else
    {
        uint8_t rom_buff[9];
        rom_buff[0] = cmd::match_rom;
        for(uint8_t i = 1; i < sizeof(rom_buff); i++, rom = rom >> 8)
        {
            rom_buff[i] = rom;
        }
        res = write_buff(rom_buff, sizeof(rom_buff));
    }
    if(res != res::ok)
    {
        goto Exit;
    }
    
    res = onewire::read_buff(read_buff, read_size);
    
Exit:
    xSemaphoreGive(api_lock);
    return res;
}

enum onewire::res onewire::write_read(uint64_t rom, void *write_buff, uint16_t write_size,
    void *read_buff, uint16_t read_size)
{
    assert(write_buff);
    assert(write_size > 0);
    assert(read_buff);
    assert(read_size > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    auto res = do_reset();
    if(res != res::ok)
    {
        goto Exit;
    }
    
    if(!rom)
    {
        res = write_byte(cmd::skip_rom);
    }
    else
    {
        uint8_t rom_buff[9];
        rom_buff[0] = cmd::match_rom;
        for(uint8_t i = 1; i < sizeof(rom_buff); i++, rom = rom >> 8)
        {
            rom_buff[i] = rom;
        }
        res = onewire::write_buff(rom_buff, sizeof(rom_buff));
    }
    if(res != res::ok)
    {
        goto Exit;
    }
    
    if(write_buff)
    {
        res = onewire::write_buff(write_buff, write_size);
        if(res != res::ok)
        {
            goto Exit;
        }
    }
    
    if(read_buff)
    {
        res = onewire::read_buff(read_buff, read_size);
    }
    
Exit:
    xSemaphoreGive(api_lock);
    return res;
}

enum onewire::res onewire::read_rom(uint64_t &rom)
{
    assert(rom);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    auto res = do_reset();
    if(res != res::ok)
    {
        goto Exit;
    }
    
    res = write_byte(cmd::read_rom);
    if(res != res::ok)
    {
        goto Exit;
    }
    
    uint8_t rx_buff[8];
    res = read_buff(rx_buff, 8);
    if(res != res::ok)
    {
        goto Exit;
    }
    
    rom = 0;
    for(uint8_t i = 0; i < sizeof(rom); i++)
    {
        rom = (rom << 8) | rx_buff[7 - i];
    }
    
Exit:
    xSemaphoreGive(api_lock);
    return res;
}

/*enum onewire::res onewire::search(uint64_t *rom_list, size_t *rom_list_size)
{
}*/

enum onewire::res onewire::do_reset()
{
    uint8_t write_buff = 0xF0, read_buff = 0x00;
    uint16_t size = sizeof(read_buff);
    
    if(uart.baudrate() != 9600)
    {
        uart.baudrate(9600);
    }
    
    auto res = res::ok;
    if(uart.write(&write_buff, 1) != periph::uart::res::ok)
    {
        res = res::write_error;
        goto Exit;
    }
    
    switch(uart.read(&read_buff, &size, read_wait_timeout))
    {
        case periph::uart::res::read_timeout: res = res::no_device; break;
        case periph::uart::res::read_error: res = res::read_error; break;
        case periph::uart::res::write_error: res = res::write_error; break;
        default:
            if(size != sizeof(read_buff))
            {
                res = res::read_error;
            }
            else if(read_buff == 0x00)
            {
                res = res::line_busy;
            }
    }
    
Exit:
    uart.baudrate(115200);
    return res;
}

enum onewire::res onewire::write_buff(void *buff, uint8_t size)
{
    auto res = res::ok;
    
    for(uint8_t i = 0; (i < size) && (res == res::ok); i++)
    {
        res = write_byte(static_cast<uint8_t *>(buff)[i]);
    }
    
    return res;
}

enum onewire::res onewire::read_buff(void *buff, uint8_t size)
{
    auto res = res::ok;
    
    for(uint8_t i = 0; (i < size) && (res == res::ok); i++)
    {
        res = read_byte(&static_cast<uint8_t *>(buff)[i]);
    }
    
    return res;
}

enum onewire::res onewire::write_byte(uint8_t byte)
{
    uint8_t write_buff[8], read_buff[8];
    uint16_t size = sizeof(read_buff);
    
    for(uint8_t i = 0; i < size; i++)
    {
        write_buff[i] = ((byte >> i) & 1) ? 0xFF : 0x00;
    }
    
    auto res = res::ok;
    switch(uart.write_read(write_buff, size, read_buff, &size, read_wait_timeout))
    {
        case periph::uart::res::read_timeout: res = res::no_device; break;
        case periph::uart::res::read_error: res = res::read_error; break;
        case periph::uart::res::write_error: res = res::write_error; break;
        default:
            if(size != sizeof(read_buff))
            {
                res = res::read_error;
            }
            else if(read_buff == 0x00)
            {
                res = res::line_busy;
            }
    }
    
    if(res == res::ok)
    {
        if(memcmp(write_buff, read_buff, sizeof(write_buff)))
        {
            res = res::line_busy;
        }
    }
    
    return res;
}

enum onewire::res onewire::read_byte(uint8_t *byte)
{
    uint8_t write_buff[8], read_buff[8];
    uint16_t size = sizeof(read_buff);
    
    memset(write_buff, 0xFF, sizeof(write_buff));
    
    auto res = res::ok;
    switch(uart.write_read(write_buff, size, read_buff, &size, read_wait_timeout))
    {
        case periph::uart::res::read_timeout: res = res::no_device; break;
        case periph::uart::res::read_error: res = res::read_error; break;
        case periph::uart::res::write_error: res = res::write_error; break;
        default:
            if(size != sizeof(read_buff))
            {
                res = res::read_error;
            }
            else if(read_buff == 0x00)
            {
                res = res::line_busy;
            }
    }
    
    if(res == res::ok)
    {
        *byte = 0;
        for(uint8_t i = 0; i < size; i++)
        {
            *byte |= (read_buff[i] == 0xFF) << i;
        }
    }
    
    return res;
}
