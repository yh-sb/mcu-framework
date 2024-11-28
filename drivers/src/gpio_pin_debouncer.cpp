#include <cassert>
#include "drivers/gpio_pin_debouncer.hpp"

using namespace drv;

gpio_pin_debouncer::gpio_pin_debouncer(periph::gpio &gpio, std::chrono::milliseconds debounce_timeout,
    bool default_state):
    gpio(gpio),
    _debounce_timeout(debounce_timeout),
    state(default_state),
    cnt(0)
{
    assert(gpio.mode() == periph::gpio::mode::digital_input);
}

gpio_pin_debouncer::~gpio_pin_debouncer()
{
}

bool gpio_pin_debouncer::poll_1ms(bool &new_state)
{
    bool prev_state = state;
    new_state = get_filtered_state();
    
    return prev_state != new_state;
}

bool gpio_pin_debouncer::get_filtered_state()
{
    if(gpio.get())
    {
        if(cnt < _debounce_timeout.count())
        {
            cnt++;
        }
        else
        {
            state = true;
        }
    }
    else
    {
        if(cnt > 0)
        {
            cnt--;
        }
        else
        {
            state = false;
        }
    }
    
    return state;
}
