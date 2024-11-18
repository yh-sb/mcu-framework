#include <cassert>
#include "FreeRTOS.h"
#include "task.h"
#include "drivers/dht.hpp"

using namespace drv;

static const uint16_t timeout[] = {1200, 2200};

// Byte map of response from DHTxx
enum
{
    RH_INT,
    RH_DEC,
    T_INT,
    T_DEC,
    CHECKSUM,
    BYTES_TOTAL // Total number of bytes in response
};

dht::dht(drv::singlewire &singlewire, enum device dht_device):
    singlewire(singlewire),
    dht_device(dht_device),
    recent_time(xTaskGetTickCount()),
    is_first_measurement(true)
{
    assert(api_lock = xSemaphoreCreateMutex());
}

dht::~dht()
{
    xSemaphoreGive(api_lock);
    vSemaphoreDelete(api_lock);
}

enum dht::res dht::read(value_t &value)
{
    wait_ready();
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    res res = res::ok;
    uint8_t buff[BYTES_TOTAL];
    switch(singlewire.read(buff, sizeof(buff)))
    {
        case singlewire::res::ok: break;
        case singlewire::res::no_device: res = res::no_device; goto Exit;
        case singlewire::res::line_busy: res = res::line_busy; goto Exit;
        default: res = res::device_error; goto Exit;
    }
    
    if(!is_crc_valid(buff))
    {
        res = res::crc_error;
        goto Exit;
    }
    
    value = parce_data(buff);
    
Exit:
    xSemaphoreGive(api_lock);
    return res;
}

void dht::wait_ready(void)
{
    if(!is_first_measurement)
    {
        xTaskDelayUntil(&recent_time, timeout[static_cast<uint8_t>(dht_device)]);
    }
    else
    {
        xTaskDelayUntil(&recent_time, timeout[static_cast<uint8_t>(dht_device)] * 2);
        is_first_measurement = false;
    }
}

bool dht::is_crc_valid(const uint8_t *buff)
{
    return buff[CHECKSUM] == buff[RH_INT] + buff[RH_DEC] + buff[T_INT] + buff[T_DEC];
}

dht::value_t dht::parce_data(const uint8_t *buff)
{
    value_t val =
    {
        .rh_x10 = (uint16_t)((buff[RH_INT] * 10) + buff[RH_DEC]),
        .t_x10 = (int16_t)((buff[T_INT] * 10) + buff[T_DEC])
    };
    
    return val;
}
