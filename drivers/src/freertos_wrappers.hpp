#pragma once

#include "FreeRTOS.h"
#include "semphr.h"

namespace freertos
{
// Take semaphore and release automatically in destructor
class semaphore_take
{
public:
    semaphore_take(const SemaphoreHandle_t &semaphore,
        TickType_t ticks_to_wait):
        _semaphore(semaphore)
    {
        is_taken = xSemaphoreTake(_semaphore, ticks_to_wait) == pdTRUE;
    }
    
    ~semaphore_take()
    {
        xSemaphoreGive(_semaphore);
    }
    
    bool is_taken;
    
    // Delete copy constructor and copy assignment operator
    semaphore_take(const semaphore_take&) = delete;
    semaphore_take& operator=(const semaphore_take&) = delete;
    
    // Delete move constructor and move assignment operator
    semaphore_take(semaphore_take&&) = delete;
    semaphore_take& operator=(semaphore_take&&) = delete;
    
private:
    const SemaphoreHandle_t &_semaphore;
};
} // namespace freertos
