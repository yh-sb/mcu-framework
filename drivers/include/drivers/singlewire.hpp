#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "periph/gpio.hpp"
#include "periph/timer.hpp"
#include "periph/exti.hpp"

namespace drv
{
class singlewire
{
public:
    enum class res : int8_t
    {
        ok           =  0,
        no_device    = -1,
        device_error = -2,
        read_error   = -3,
        line_busy    = -4
    };
    
    singlewire(periph::gpio &gpio, periph::timer &timer, periph::exti &exti);
    ~singlewire();
    
    enum res read(uint8_t *buff, uint16_t size);
    
    // Delete copy constructor and copy assignment operator
    singlewire(const singlewire&) = delete;
    singlewire& operator=(const singlewire&) = delete;
    
    // Delete move constructor and move assignment operator
    singlewire(singlewire&&) = delete;
    singlewire& operator=(singlewire&&) = delete;
    
private:
    periph::gpio &gpio;
    periph::timer &timer;
    periph::exti &exti;
    TaskHandle_t task;
    res res;
    
    enum state
    {
        request,
        wait_response_start,
        wait_response_end,
        wait_bit_start_low,
        wait_bit_start_high,
        wait_bit_check
    };
    static constexpr std::chrono::microseconds timeouts[singlewire::state::wait_bit_check + 1] =
    {
        std::chrono::microseconds(18000), // request
        std::chrono::microseconds(40),    // wait_response_start
        std::chrono::microseconds(95),    // wait_response_end
        std::chrono::microseconds(95),    // wait_bit_start_low
        
        /* Normally this timeout should be 50 us, but DHT22 keeps the data line low
        for 67 us after each byte. Therefore, increase this timeout to avoid res::read_error
        */
        std::chrono::microseconds(67),    // wait_bit_start_high
        std::chrono::microseconds(35)     // wait_bit_check
    };
    
    struct
    {
        enum state state;
        uint8_t *buff;
        uint16_t size;
        uint16_t byte;
        uint8_t bit;
    } fsm;
    void fsm_start(uint8_t *buff, uint16_t size);
    void fsm_run(bool is_timer_expired);
};
} // namespace drv
