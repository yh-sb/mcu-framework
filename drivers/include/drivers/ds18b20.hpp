#pragma once

#include "drivers/onewire.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
class ds18b20
{
public:
    enum class res : int8_t
    {
        ok        =  0,
        no_device = -1,
        crc_error = -2,
        line_busy = -3
    };
    
    enum class resolution : uint8_t
    {
        _9_bit,  /* 0,5    °C,  94 ms */
        _10_bit, /* 0,25   °C, 188 ms */
        _11_bit, /* 0,125  °C, 375 ms */
        _12_bit  /* 0,0625 °C, 750 ms. Default */
    };
    
    ds18b20(drv::onewire &onewire);
    ~ds18b20();
    
    enum res get_temperature(uint64_t rom, float &temperature);
    
    enum res set_resolution(uint64_t rom, enum resolution resolution);  // 10 ms
    enum res get_resolution(uint64_t rom, enum resolution &resolution); // 6  ms
    
    enum res set_high_temperature_alarm(uint64_t rom, uint8_t temperature);  // 10 ms
    enum res get_high_temperature_alarm(uint64_t rom, uint8_t &temperature); // 6  ms
    
    enum res set_low_temperature_alarm(uint64_t rom, uint8_t temperature);  // 10 ms
    enum res get_low_temperature_alarm(uint64_t rom, uint8_t &temperature); // 6  ms
    
    enum res write_eeprom(uint64_t rom);   // 1  ms
    enum res restore_eeprom(uint64_t rom); // 1  ms
    
    // Delete copy constructor and copy assignment operator
    ds18b20(const ds18b20&) = delete;
    ds18b20& operator=(const ds18b20&) = delete;
    
    // Delete move constructor and move assignment operator
    ds18b20(ds18b20&&) = delete;
    ds18b20& operator=(ds18b20&&) = delete;
    
private:
    #pragma pack(push, 1)
    struct scratchpad_t
    {
        uint8_t temp_lsb;    // Byte 0: Temperature LSB
        uint8_t temp_msb;    // Byte 1: Temperature MSB
        uint8_t th;          // Byte 2: High alarm temperature (TH)
        uint8_t tl;          // Byte 3: Low alarm temperature (TL)
        uint8_t config;      // Byte 4: Configuration register
        uint8_t reserved[3]; // Bytes 5-7: Reserved
        uint8_t crc;         // Byte 8: CRC
    };
    #pragma pack(pop)
    
    enum res write_scratchpad(uint64_t rom, const scratchpad_t &scratchpad);
    enum res read_scratchpad(uint64_t rom, scratchpad_t &scratchpad);
    static uint8_t calc_crc(void *buff, uint8_t size);
    static float calc_temperature(uint8_t temp_lsb, uint8_t temp_msb, uint8_t conf);
    static enum res convert_result(onewire::res res);
    
    drv::onewire &onewire;
    SemaphoreHandle_t api_lock;
    enum resolution resolution;
};
} // namespace drv
