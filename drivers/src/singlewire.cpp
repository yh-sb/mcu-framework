#include <cassert>
#include "drivers/singlewire.hpp"

using namespace drv;

singlewire::singlewire(periph::gpio &gpio, periph::timer &timer, periph::exti &exti):
    gpio(gpio),
    timer(timer),
    exti(exti)
{
    assert(gpio.mode() == periph::gpio::mode::open_drain);
    
    this->timer.set_callback([this](){ fsm_run(true); });
    this->exti.set_callback([this](){ fsm_run(false); });
}

singlewire::~singlewire()
{
    timer.stop();
    exti.disable();
}

enum singlewire::res singlewire::read(uint8_t *buff, uint16_t size)
{
    if(!gpio.get())
    {
        return res::line_busy;
    }
    
    task = xTaskGetCurrentTaskHandle();
    fsm_start(buff, size);
    
    // Task will be unlocked from fsm when data reception will be finished
    ulTaskNotifyTake(true, portMAX_DELAY);
    
    return res;
}

void singlewire::fsm_start(uint8_t *buff, uint16_t size)
{
    fsm.buff = buff;
    fsm.size = size;
    
    fsm.state = state::request;
    fsm.byte = 0;
    fsm.bit = 7;
    
    gpio.set(0);
    timer.timeout(std::chrono::microseconds(timeouts[state::request]));
    timer.start();
}

void singlewire::fsm_run(bool is_timer_expired)
{
    switch(fsm.state)
    {
        case state::request:
            fsm.state = state::wait_response_start;
            gpio.set(1);
            exti.trigger(periph::exti::trigger::falling);
            exti.enable();
            timer.timeout(std::chrono::microseconds(timeouts[state::wait_response_start]));
            timer.start();
            break;
        
        case state::wait_response_start:
            if(is_timer_expired)
            {
                exti.disable();
                res = res::no_device;
                goto Exit;
            }
            timer.stop();
            
            fsm.state = state::wait_response_end;
            exti.trigger(periph::exti::trigger::rising);
            timer.timeout(std::chrono::microseconds(timeouts[state::wait_response_end]));
            timer.start();
            break;
        
        case state::wait_response_end:
            if(is_timer_expired)
            {
                exti.disable();
                res = res::device_error;
                goto Exit;
            }
            timer.stop();
            
            fsm.state = state::wait_bit_start_low;
            exti.trigger(periph::exti::trigger::falling);
            timer.timeout(std::chrono::microseconds(timeouts[state::wait_bit_start_low]));
            timer.start();
            break;
        
        case state::wait_bit_start_low:
            if(is_timer_expired)
            {
                exti.disable();
                res = res::read_error;
                goto Exit;
            }
            timer.stop();
            
            fsm.state = state::wait_bit_start_high;
            exti.trigger(periph::exti::trigger::rising);
            timer.timeout(std::chrono::microseconds(timeouts[state::wait_bit_start_high]));
            timer.start();
            break;
        
        case state::wait_bit_start_high:
            if(is_timer_expired)
            {
                exti.disable();
                res = res::read_error;
                goto Exit;
            }
            timer.stop();
            
            fsm.state = state::wait_bit_check;
            exti.trigger(periph::exti::trigger::falling);
            timer.timeout(std::chrono::microseconds(timeouts[state::wait_bit_check]));
            timer.start();
            break;
        
        case state::wait_bit_check:
            if(is_timer_expired)
            {
                fsm.buff[fsm.byte] |= 1 << fsm.bit;
                
                fsm.state = state::wait_bit_start_low;
                exti.trigger(periph::exti::trigger::falling);
                timer.timeout(std::chrono::microseconds(timeouts[state::wait_bit_check]));
                timer.start();
            }
            else
            {
                timer.stop();
                fsm.buff[fsm.byte] &= ~(1 << fsm.bit);
                
                fsm.state = state::wait_bit_start_high;
                exti.trigger(periph::exti::trigger::rising);
                timer.timeout(std::chrono::microseconds(timeouts[state::wait_bit_check] +
                    timeouts[state::wait_bit_start_high]));
                timer.start();
            }
            
            if(fsm.bit > 0)
            {
                fsm.bit--;
            }
            else
            {
                // Start reading next byte
                fsm.byte++;
                fsm.bit = 7;
                
                if(fsm.byte == fsm.size)
                {
                    timer.stop();
                    exti.disable();
                    res = res::ok;
                    goto Exit;
                }
            }
            break;
    }
    return;
    
Exit:
    BaseType_t hi_task_woken = 0;
    vTaskNotifyGiveFromISR(task, &hi_task_woken);
    portYIELD_FROM_ISR(hi_task_woken);
}
