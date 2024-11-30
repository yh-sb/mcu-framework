#pragma once

#include "drivers/singlewire.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
class dht
{
public:
    enum class res : int8_t
    {
        ok           =  0,
        no_device    = -1,
        device_error = -2,
        line_busy    = -3,
        crc_error    = -4
    };
    
    enum class device : uint8_t
    {
        dht11 = 0,
        dht22 = 1
    };
    
    dht(drv::singlewire &singlewire, enum device dht_device);
    ~dht();
    
    struct value_t
    {
        uint16_t rh_x10;
        int16_t t_x10;
    };
    
    enum res read(value_t &value);
    
    // Delete copy constructor and copy assignment operator
    dht(const dht&) = delete;
    dht& operator=(const dht&) = delete;
    
    // Delete move constructor and move assignment operator
    dht(dht&&) = delete;
    dht& operator=(dht&&) = delete;
    
private:
    drv::singlewire &singlewire;
    enum device dht_device;
    SemaphoreHandle_t api_lock;
    TickType_t recent_time;
    bool is_first_measurement;
    
    void wait_ready();
    static bool is_crc_valid(const uint8_t *buff);
    static value_t parce_data(const uint8_t *buff);
};
} // namespace drv
